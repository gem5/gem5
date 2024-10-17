# Copyright (c) 2024 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import base64
import os
from typing import (
    List,
    Optional,
)

from ...components.boards.abstract_board import AbstractBoard
from ...components.boards.kernel_disk_workload import KernelDiskWorkload
from ...components.boards.x86_board import X86Board
from ...components.cachehierarchies.abstract_cache_hierarchy import (
    AbstractCacheHierarchy,
)
from ...components.devices.gpus.amdgpu import BaseViperGPU
from ...components.memory.abstract_memory_system import AbstractMemorySystem
from ...components.processors.abstract_processor import AbstractProcessor
from ...utils.override import overrides


class ViperBoard(X86Board):
    """
    A derivative of X86Board capable of full system simulation for X86 with a
    GPU device. Provides all the functionality of the X86Board with helper
    methods specific to booting a disk with GPU libraries installed.
    """

    def __init__(
        self,
        clk_freq: str,
        processor: AbstractProcessor,
        memory: AbstractMemorySystem,
        cache_hierarchy: AbstractCacheHierarchy,
        gpus: Optional[List[BaseViperGPU]] = None,
    ) -> None:
        super().__init__(
            clk_freq=clk_freq,
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )
        print("Viper board __init__ was called here now")

        self._gpus = gpus

    def get_devices(self):
        return self._gpus

    @overrides(AbstractBoard)
    def _connect_things(self) -> None:
        print("Viper board connect things was called here now")
        super()._connect_things()

        if self._gpus is not None:
            for gpu in self._gpus:
                gpu.connectGPU(self)

    @overrides(KernelDiskWorkload)
    def get_disk_device(self):
        return "/dev/sda"

    @overrides(KernelDiskWorkload)
    def get_default_kernel_args(self) -> List[str]:
        # The regular parameters used with gem5 plus (1) fbdev_emulation=0
        # to disable having to implement this functionality, (2) blacklist
        # amdgpu because we need to copy the VBIOS into memory first, and (3)
        # blacklist psmouse as amdgpu driver adds new mouse commands which
        # gem5 does not implement and they do not seem to be documented.
        return [
            "earlyprintk=ttyS0",
            "console=ttyS0",
            "lpj=7999923",
            "root={root_value}",
            "drm_kms_helper.fbdev_emulation=0",
            "modprobe.blacklist=amdgpu",
            "modprobe.blacklist=psmouse",
        ]

    # Replicate the capability of the old GPUFS config, which embed a binary
    # application or script into a bash script setting up the environment and
    # loading the GPU driver.
    def make_gpu_app(self, gpu: BaseViperGPU, app: str, debug: bool = False):
        driver_load_command = gpu.get_driver_command(debug=debug)

        with open(os.path.abspath(app), "rb") as binfile:
            encodedBin = base64.b64encode(binfile.read()).decode()

        application_command = (
            f'echo "{encodedBin}" | base64 -d > myapp\n'
            "chmod +x myapp\n"
            "./myapp {}\n"
            "/sbin/m5 exit\n"
        )

        return driver_load_command + application_command
