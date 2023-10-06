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
from pathlib import Path
from typing import List
from typing import Optional
from typing import Union

from gem5.resources.elfie import ELFieInfo
from gem5.resources.looppoint import Looppoint
from m5.objects import Process
from m5.objects import SEWorkload
from m5.util import warn

from ...resources.resource import AbstractResource
from ...resources.resource import BinaryResource
from ...resources.resource import CheckpointResource
from ...resources.resource import FileResource
from ...resources.resource import SimpointDirectoryResource
from ...resources.resource import SimpointResource
from ..processors.switchable_processor import SwitchableProcessor
from .abstract_board import AbstractBoard


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
        env_list: Optional[List[str]] = None,
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
        :param stdout_file: The output file for the binary
        :param stderr_file: The error output file for the binary
        :param env_list: The environment variables defined for the binary
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
        if env_list is not None:
            process.env = env_list

        if isinstance(self.get_processor(), SwitchableProcessor):
            # This is a hack to get switchable processors working correctly in
            # SE mode. The "get_cores" API for processors only gets the current
            # switched-in cores and, in most cases, this is what the script
            # required. In the case there are switched-out cores via the
            # SwitchableProcessor, we sometimes need to apply things to ALL
            # cores (switched-in or switched-out). In this case we have an
            # `__all_cores` function. Here we must apply the process to every
            # core.
            #
            # A better API for this which avoids `isinstance` checks would be
            # welcome.
            for core in self.get_processor()._all_cores():
                core.set_workload(process)
        else:
            for core in self.get_processor().get_cores():
                core.set_workload(process)

        # Set whether to exit on work items for the se_workload
        self.exit_on_work_items = exit_on_work_items

        # Here we set `self._checkpoint`. This is then used by the
        # Simulator module to setup checkpoints.
        if checkpoint:
            if isinstance(checkpoint, Path):
                self._checkpoint = checkpoint
            elif isinstance(checkpoint, AbstractResource):
                self._checkpoint = Path(checkpoint.get_local_path())
            else:
                raise Exception(
                    "The checkpoint must be None, Path, or "
                    "AbstractResource."
                )

    def set_se_simpoint_workload(
        self,
        binary: BinaryResource,
        arguments: List[str] = [],
        simpoint: SimpointResource = None,
        checkpoint: Optional[Union[Path, CheckpointResource]] = None,
    ) -> None:
        """Set up the system to run a SimPoint workload.

        **Limitations**
        * Only supports single threaded applications.
        * Dynamically linked executables are partially supported when the host
          ISA and the simulated ISA are the same.

        **Warning:** Simpoints only works with one core

        :param binary: The resource encapsulating the binary to be run.
        :param arguments: The input arguments for the binary
        :param simpoint: The SimpointResource that contains the list of
        SimPoints starting instructions, the list of weights, and the SimPoints
        interval
        :param checkpoint: The checkpoint directory. Used to restore the
        simulation to that checkpoint.
        """

        self._simpoint_resource = simpoint

        if self.get_processor().get_num_cores() > 1:
            warn("SimPoints only works with one core")
        self.get_processor().get_cores()[0]._set_simpoint(
            inst_starts=self._simpoint_resource.get_simpoint_start_insts(),
            board_initialized=False,
        )

        # Call set_se_binary_workload after SimPoint setup is complete
        self.set_se_binary_workload(
            binary=binary,
            arguments=arguments,
            checkpoint=checkpoint,
        )

    def get_simpoint(self) -> SimpointResource:
        """
        Returns the SimpointResorce object set. If no SimpointResource object
        has been set an exception is thrown.
        """
        if getattr(self, "_simpoint_resource", None):
            return self._simpoint_resource
        raise Exception("This board does not have a simpoint set.")

    def set_se_looppoint_workload(
        self,
        binary: AbstractResource,
        looppoint: Looppoint,
        arguments: List[str] = [],
        checkpoint: Optional[Union[Path, AbstractResource]] = None,
        region_id: Optional[Union[int, str]] = None,
    ) -> None:
        """Set up the system to run a LoopPoint workload.

        **Limitations**
        * Dynamically linked executables are partially supported when the host
          ISA and the simulated ISA are the same.

        :param binary: The resource encapsulating the binary to be run.
        :param looppoint: The LoopPoint object that contain all the information
        gather from the LoopPoint files and a LoopPointManager that will raise
        exit events for LoopPoints
        :param arguments: The input arguments for the binary
        :param region_id: If set, will only load the Looppoint region
        corresponding to that ID.
        """

        assert isinstance(looppoint, Looppoint)
        self._looppoint_object = looppoint
        if region_id:
            self._looppoint_object.set_target_region_id(region_id=region_id)
        self._looppoint_object.setup_processor(self.get_processor())

        # Call set_se_binary_workload after LoopPoint setup is complete
        self.set_se_binary_workload(
            binary=binary,
            arguments=arguments,
            checkpoint=checkpoint,
        )

    def set_se_elfie_workload(
        self,
        elfie: AbstractResource,
        elfie_info: ELFieInfo,
        arguments: List[str] = [],
        checkpoint: Optional[Union[Path, AbstractResource]] = None,
    ) -> None:
        """Set up the system to run a ELFie workload.

        **Limitations**
        * Dynamically linked executables are partially supported when the host
          ISA and the simulated ISA are the same.

        :param elfie: The resource encapsulating the binary elfie to be run.
        :param elfie_info: The ELFieInfo object that contain all the
        information for the ELFie
        :param arguments: The input arguments for the binary
        """

        assert isinstance(elfie_info, ELFieInfo)
        self._elfie_info_object = elfie_info

        self._elfie_info_object.setup_processor(self.get_processor())

        # Call set_se_binary_workload after LoopPoint setup is complete
        self.set_se_binary_workload(
            binary=elfie,
            arguments=arguments,
            checkpoint=checkpoint,
        )

    def get_looppoint(self) -> Looppoint:
        """
        Returns the LoopPoint object set. If no LoopPoint object has been set
        an exception is thrown.
        """
        if getattr(self, "_looppoint_object", None):
            return self._looppoint_object
        raise Exception("This board does not have a looppoint set.")
