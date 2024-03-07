# Copyright (c) 2023 The Regents of the University of California
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

from typing import List

from m5.util import warn, panic

from ...components.processors.cpu_types import CPUTypes
from ...components.boards.x86_board import X86Board
from ...components.memory.single_channel import SingleChannelDDR4_2400
from ...components.processors.simple_processor import SimpleProcessor
from .cpu_cache_hierarchy import (
    ViperCPUCacheHierarchy,
)
from .gpus import Vega10GPU
from ...coherence_protocol import CoherenceProtocol
from ...isas import ISA
from ...utils.requires import requires
from ...utils.override import overrides
from ...components.boards.kernel_disk_workload import KernelDiskWorkload

from m5.objects import (
    Addr,
    AddrRange,
    AMDGPUDevice,
)


class X86ViperBoard(X86Board):
    """


    Example
    -------

    An example of using the X86ViperBoard can be found in
    `<>`.

    To run:

    ```
    scons build/VEGA_X86/gem5.opt -j`nproc`
    ./build/VEGA_X86/gem5.opt <>
    ```

    """

    def __init__(self, gpu_model: str):
        """
        GPU model can be one of the following: Vega10, MI200
        """

        requires(
            isa_required=ISA.X86,
            coherence_protocol_required=CoherenceProtocol.GPU_VIPER,
        )

        memory = SingleChannelDDR4_2400(size="3GB")

        # Supported CPUs: Atomic and KVM
        processor = SimpleProcessor(
            cpu_type=CPUTypes.KVM, isa=ISA.X86, num_cores=1
        )
        cache_hierarchy = ViperCPUCacheHierarchy(
            l1d_size="32kB",
            l1d_assoc=8,
            l1i_size="32kB",
            l1i_assoc=8,
            l2_size="1MB",
            l2_assoc=16,
        )

        print(cache_hierarchy.get_ruby())

        super().__init__(
            clk_freq="3GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

        gpu_device = AMDGPUDevice(pci_func=0, pci_dev=8, pci_bus=0)
        self.pc.south_bridge.gpu = gpu_device
        mmio_trace = "/home/mporemba/gem5-resources/src/gpu-fs/vega_mmio.log"
        rom = ""  # This isn't really used, rom is on the disk
        self.gpu = Vega10GPU(8, gpu_device, mmio_trace, rom)

    # I think this isn't needed?  Not good with python classes
    # The shadow rom is no longer necessary which is what was here
    @overrides(X86Board)
    def _setup_memory_ranges(self):
        super()._setup_memory_ranges()

    def _set_mmio_file(self, file_name: str):
        mmio_md5 = hashlib.md5(open(file_name, "rb").read()).hexdigest()
        if mmio_md5 != "c4ff3326ae8a036e329b8b595c83bd6d":
            panic("MMIO file does not match gem5 resources")

    @overrides(KernelDiskWorkload)
    def get_default_kernel_args(self) -> List[str]:
        return [
            "earlyprintk=ttyS0",
            "console=ttyS0,9600",
            "lpj=7999923",
            "root=/dev/sda1",
            "drm_kms_helper.fbdev_emulation=0",
            "modprobe.blacklist=amdgpu",
            "modprobe.blacklist=psmouse",
        ]
