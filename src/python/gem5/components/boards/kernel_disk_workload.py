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

from abc import abstractmethod

from .abstract_board import AbstractBoard
from ...resources.resource import AbstractResource

from typing import List, Optional, Union
import os
from pathlib import Path

import m5


class KernelDiskWorkload:
    """
    The purpose of this abstract class is to enable a full-system boot
    consisting of of a kernel which will then load a disk image.

    For this to function correctly, the KernelDiskWorkload class should be
    added as a superclass to a board and the abstract methods implemented.
    E.g.:

    ```
    class X86Board(AbstractBoard, KernelDiskWorkload):
        ...
        @overrides(KernelDiskWorkload)
        def get_default_kernel_args(self) -> List[str]:
            return [
                "earlyprintk=ttyS0",
                "console=ttyS0",
                "lpj=7999923",
                "root={root_value}",
            ]
        ...
    ```

    Notes
    -----

    * This assumes only one disk is set.
    * This assumes the Linux kernel is used.
    """

    @abstractmethod
    def get_default_kernel_args(self) -> List[str]:
        """
        Returns a default list of arguments for the workload kernel. We assume
        the following strings may be used as placeholders, to be replaced when
        `set_kernel_disk_workload` is executed:

        * `{root_value}` : set to `get_default_kernel_root_val()`.

        :returns: A default list of arguments for the workload kernel.
        """
        raise NotImplementedError

    @abstractmethod
    def get_disk_device(self) -> str:
        """
        Get the disk device, e.g., "/dev/sda", where the disk image is placed.

        :returns: The disk device.
        """
        raise NotImplementedError

    @abstractmethod
    def _add_disk_to_board(self, disk_image: AbstractResource) -> None:
        """
        Sets the configuration needed to add the disk image to the board.

        **Note:** This will be executed at the end of the
        `set_kernel_disk_workload` function.

        :param disk_image: The disk image to add to the system.
        """
        raise NotImplementedError

    def get_disk_root_partition(
        cls, disk_image: AbstractResource
    ) -> Optional[str]:
        """
        Obtains the root partition of a disk image by inspecting the resource's
        metadata.

        :returns: The disk image's root partition.
        """
        try:
            return disk_image.get_metadata()["additional_metadata"][
                "root_partition"
            ]
        except KeyError:
            return None

    def get_default_kernel_root_val(self, disk_image: AbstractResource) -> str:
        """
        Get the default kernel root value to be passed to the kernel. This is
        determined by the value implemented in the `get_disk_device()`
        function, and the disk image partition, obtained from
        `get_disk_root_partition()`


        :param disk_image: The disk image to be added to the system.
        :returns: The default value for the 'root' argument to be passed to the
        kernel.
        """
        return self.get_disk_device() + (
            self.get_disk_root_partition(disk_image) or ""
        )

    def set_kernel_disk_workload(
        self,
        kernel: AbstractResource,
        disk_image: AbstractResource,
        bootloader: Optional[AbstractResource] = None,
        readfile: Optional[str] = None,
        readfile_contents: Optional[str] = None,
        kernel_args: Optional[List[str]] = None,
        exit_on_work_items: bool = True,
        checkpoint: Optional[Union[Path, AbstractResource]] = None,
    ) -> None:
        """
        This function allows the setting of a full-system run with a Kernel
        and a disk image.

        :param kernel: The kernel to boot.
        :param disk_image: The disk image to mount.
        :param bootloader: The current implementation of the ARM board requires
        three resources to operate -- kernel, disk image, and, a bootloader.
        :param readfile: An optional parameter stating the file to be read by
        by `m5 readfile`.
        :param readfile_contents: An optional parameter stating the contents of
        the readfile file. If set with `readfile`, the contents of `readfile`
        will be overwritten with `readfile_contents`, otherwise a new file will
        be created with the value of `readfile_contents`.
        :param kernel_args: An optional parameter for setting arguments to be
        passed to the kernel. By default set to `get_default_kernel_args()`.
        :param exit_on_work_items: Whether the simulation should exit on work
        items. True by default.
        :param checkpoint: The checkpoint directory. Used to restore the
        simulation to that checkpoint.
        """

        # We assume this this is in a multiple-inheritance setup with an
        # Abstract board. This function will not work otherwise.
        assert isinstance(self, AbstractBoard)

        # If we are setting a workload of this type, we need to run as a
        # full-system simulation.
        self._set_fullsystem(True)

        # Set the kernel to use.
        self.workload.object_file = kernel.get_local_path()

        # Set the arguments to be passed to the kernel.
        self.workload.command_line = (
            " ".join(kernel_args or self.get_default_kernel_args())
        ).format(
            root_value=self.get_default_kernel_root_val(disk_image=disk_image)
        )

        # Setting the bootloader information for ARM board. The current
        # implementation of the ArmBoard class expects a boot loader file to be
        # provided along with the kernel and the disk image.

        if bootloader is not None:
            self._bootloader = [bootloader.get_local_path()]

        # Set the readfile.
        if readfile:
            self.readfile = readfile
        elif readfile_contents:
            self.readfile = os.path.join(m5.options.outdir, "readfile")

        # Add the contents to the readfile, if specified.
        if readfile_contents:
            file = open(self.readfile, "w+")
            file.write(readfile_contents)
            file.close()

        self._add_disk_to_board(disk_image=disk_image)

        # Set whether to exit on work items.
        self.exit_on_work_items = exit_on_work_items

        # Here we set `self._checkpoint_dir`. This is then used by the
        # Simulator module to setup checkpoints.
        if checkpoint:
            if isinstance(checkpoint, Path):
                self._checkpoint = checkpoint
            elif isinstance(checkpoint, AbstractResource):
                self._checkpoint = Path(checkpoint.get_local_path())
            else:
                # The checkpoint_dir must be None, Path, Or AbstractResource.
                raise Exception(
                    "Checkpoints must be passed as a Path or an "
                    "AbstractResource."
                )
