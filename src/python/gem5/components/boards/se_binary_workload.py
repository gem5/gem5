# Copyright (c) 2021 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from .abstract_board import AbstractBoard
from ...resources.resource import (
    FileResource,
    AbstractResource,
    BinaryResource,
    CheckpointResource,
    SimpointResource,
)
from gem5.utils.simpoint import SimPoint

from m5.objects import SEWorkload, Process

from typing import Optional, List, Union
from m5.util import warn
from pathlib import Path


class SEBinaryWorkload:
    """
    This class is used to enable simple Syscall-Execution (SE) mode execution
    of a binary.

    For this to function correctly the SEBinaryWorkload class should be added
    as a superclass to a board (i.e., something that inherits from
    AbstractBoard).

    **Important Notes:** At present this implementation is limited. A single
    process is added to all cores as the workload. Therefore, despite allowing
    for multi-core setups, multi-program workloads are not presently supported.
    """

    def set_se_binary_workload(
        self,
        binary: BinaryResource,
        exit_on_work_items: bool = True,
        stdin_file: Optional[FileResource] = None,
        stdout_file: Optional[Path] = None,
        stderr_file: Optional[Path] = None,
        arguments: List[str] = [],
        checkpoint: Optional[Union[Path, CheckpointResource]] = None,
    ) -> None:
        """Set up the system to run a specific binary.

        **Limitations**
        * Only supports single threaded applications.
        * Dynamically linked executables are partially supported when the host
          ISA and the simulated ISA are the same.

        :param binary: The resource encapsulating the binary to be run.
        :param exit_on_work_items: Whether the simulation should exit on work
        items. True by default.
        :param stdin_file: The input file for the binary
        :param arguments: The input arguments for the binary
        :param checkpoint: The checkpoint directory. Used to restore the
        simulation to that checkpoint.
        """

        # We assume this this is in a multiple-inheritance setup with an
        # Abstract board. This function will not work otherwise.
        assert isinstance(self, AbstractBoard)

        # If we are setting a workload of this type, we need to run as a
        # SE-mode simulation.
        self._set_fullsystem(False)

        binary_path = binary.get_local_path()
        self.workload = SEWorkload.init_compatible(binary_path)

        process = Process()
        process.executable = binary_path
        process.cmd = [binary_path] + arguments
        if stdin_file is not None:
            process.input = stdin_file.get_local_path()
        if stdout_file is not None:
            process.output = stdout_file.as_posix()
        if stderr_file is not None:
            process.errout = stderr_file.as_posix()

        for core in self.get_processor().get_cores():
            core.set_workload(process)

        # Set whether to exit on work items for the se_workload
        self.exit_on_work_items = exit_on_work_items

        # Here we set `self._checkpoint_dir`. This is then used by the
        # Simulator module to setup checkpoints.
        if checkpoint:
            if isinstance(checkpoint, Path):
                self._checkpoint = checkpoint
            elif isinstance(checkpoint, AbstractResource):
                self._checkpoint_dir = Path(checkpoint.get_local_path())
            else:
                raise Exception(
                    "The checkpoint_dir must be None, Path, or "
                    "AbstractResource."
                )

    def set_se_simpoint_workload(
        self,
        binary: BinaryResource,
        arguments: List[str] = [],
        simpoint: Union[SimpointResource, SimPoint] = None,
        checkpoint: Optional[Union[Path, CheckpointResource]] = None,
    ) -> None:
        """Set up the system to run a SimPoint workload.

        **Limitations**
        * Only supports single threaded applications.
        * Dynamically linked executables are partially supported when the host
          ISA and the simulated ISA are the same.

        **Warning:** SimPoints only works with one core

        :param binary: The resource encapsulating the binary to be run.
        :param arguments: The input arguments for the binary
        :param simpoint: The SimPoint object or Resource that contains the list of
        SimPoints starting instructions, the list of weights, and the SimPoints
        interval
        :param checkpoint: The checkpoint directory. Used to restore the
        simulation to that checkpoint.
        """

        # convert input to SimPoint if necessary
        if isinstance(simpoint, SimpointResource):
            self._simpoint_object = SimPoint(simpoint)
        else:
            assert isinstance(simpoint, SimPoint)
            self._simpoint_object = simpoint

        if self.get_processor().get_num_cores() > 1:
            warn("SimPoints only works with one core")
        self.get_processor().get_cores()[0]._set_simpoint(
            inst_starts=self._simpoint_object.get_simpoint_start_insts(),
            board_initialized=False,
        )

        # Call set_se_binary_workload after SimPoint setup is complete
        self.set_se_binary_workload(
            binary=binary,
            arguments=arguments,
            checkpoint=checkpoint,
        )

    def get_simpoint(self) -> SimPoint:
        """
        Returns the SimPoint object set. If no SimPoint object has been set an
        exception is thrown.
        """
        if getattr(self, "_simpoint_object", None):
            return self._simpoint_object
        raise Exception("This board does not have a simpoint set.")
