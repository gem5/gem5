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
    Sequence,
    Tuple,
)

from m5.objects import X86E820Entry
from m5.params import (
    AddrRange,
    Port,
)
from m5.util import warn

from ...components.boards.abstract_board import AbstractBoard
from ...components.boards.abstract_system_board import AbstractSystemBoard
from ...components.boards.kernel_disk_workload import KernelDiskWorkload
from ...components.boards.x86_board import X86Board
from ...components.cachehierarchies.abstract_cache_hierarchy import (
    AbstractCacheHierarchy,
)
from ...components.devices.gpus.amdgpu import BaseViperGPU
from ...components.memory.abstract_memory_system import AbstractMemorySystem
from ...components.memory.single_channel import SingleChannelDDR4_2400
from ...components.processors.abstract_processor import AbstractProcessor
from ...utils.override import overrides


class ViperBoard(X86Board):
    """
    A derivative of X86Board capable of full system simulation for X86 with a
    GPU device. Provides all the functionality of the X86Board with helper
    methods specific to booting a disk with GPU libraries installed, enables
    AVX on supported CPU models (KVM only right now) for PyTorch, a workaround
    for X86Board 3GiB memory limitation, and helper methods to create a
    temporary readfile which reads in a binary blob into the simulator.

    **Limitations**
    * Only KVM and ATOMIC CPU models can be used due to issue CPU Ruby protocol
    * 3GiB limit workaround is a VIPER specific hack which silently creates an
      additional memory controller with 128MiB of additional memory.
    * This board should not be used to measure CPU performance.
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

        # Low memory for CPU side which allows the OS to boot. The rest of
        # main memory is then mapped above 4GiB. The memory type does not
        # matter as it is on the CPU side and should not be used much.
        low_mem_size = "128MiB"
        self.low_mem = SingleChannelDDR4_2400(size=low_mem_size)
        self.low_mem.mem_ctrl[0].dram.range = AddrRange(
            start=0x0, size=low_mem_size
        )

        self.gpus = gpus

    @overrides(AbstractCacheHierarchy)
    def get_coherence_protocol(self):
        return CoherenceProtocol.GPU_VIPER

    @overrides(AbstractBoard)
    def get_devices(self):
        return self.gpus

    @overrides(AbstractBoard)
    def _connect_things(self) -> None:
        super()._connect_things()

        if self.gpus is not None:
            for gpu in self.gpus:
                gpu.connectGPU(self)

        self.workload.enable_osxsave = 1
        # These values are taken from a real CPU and are further explained here:
        # https://sandpile.org/x86/cpuid.htm#level_0000_000Dh
        avx_extended_state = [
            0x00000007,
            0x00000340,
            0x00000000,
            0x00000340,
            0x00000000,
            0x00000340,
            0x00000000,
            0x00000000,
            0x00000100,
            0x00000240,
            0x00000000,
            0x00000040,
            0x00000000,
            0x00000000,
            0x00000000,
            0x00000000,
        ]

        # This modifies the default value for ECX only (4th in this array).
        # See: https://sandpile.org/x86/cpuid.htm#level_0000_0001h
        # Enables AVX, OSXSAVE, XSAVE, POPCNT, SSE4.2, SSE4.1, CMPXCHG16B,
        # and FMA.
        avx_cpu_features = [0x00020F51, 0x00000805, 0xEFDBFBFF, 0x1C803209]
        for core_wrapper in self.get_processor().get_cores():
            if not core_wrapper.is_kvm_core():
                warn("AVX is only supported with KVM cores")
            core = core_wrapper.get_simobject()
            for isa in core.isa:
                isa.vendor_string = "AuthenticAMD"
                isa.ExtendedState = avx_extended_state
                isa.FamilyModelStepping = avx_cpu_features

        # By default stdlib creates multiple event queues whenever KVM CPU is
        # used. Multiple event queues break many things in the GPU model
        # including m5 commands, simulation exits before all kernels launched,
        # and more. Disable them until multiple event queues are more stable.
        if len(self.get_processor().get_cores()) == 1:
            for core_wrapper in self.get_processor().get_cores():
                if core_wrapper.is_kvm_core():
                    core = core_wrapper.get_simobject()
                    core.eventq_index = 0

        # The System() object in gem5 has a memories parameter which defaults
        # to Self.all. This will collect *all* AbstractMemories and connect to
        # the CPU side. To avoid this we manually assign the memories param to
        # the CPU side memories. We need the MemInterface which is called dram
        # in the MemCtrl class even though it might not be modelling dram.
        self.memories = (
            self.memory.get_mem_interfaces()
            + self.low_mem.get_mem_interfaces()
        )

    def get_pci_host(self):
        return self.pc.pci_host

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

    def get_low_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        return [(ctrl.dram.range, ctrl.port) for ctrl in self.low_mem.mem_ctrl]

    @overrides(AbstractSystemBoard)
    def _setup_memory_ranges(self):
        # This is exactly the same code as x86_board.py but with the memory
        # size >3GiB exception removed.
        memory = self.get_memory()

        data_range = AddrRange(start=0x100000000, size=memory.get_size())
        memory.set_memory_range([data_range])

        # Add the address range for the IO
        self.mem_ranges = [
            data_range,  # All data
            AddrRange(0xC0000000, size=0x100000),  # For I/0
        ]

    @overrides(X86Board)
    def _setup_io_devices(self):
        # Call the base class which handles many more things and then
        # overwrite the e820 table for our memory ranges.
        super()._setup_io_devices()

        entries = [
            # Mark the first megabyte of memory as reserved
            X86E820Entry(addr=0, size="639KiB", range_type=1),
            X86E820Entry(addr=0x9FC00, size="385KiB", range_type=2),
            # Kernel memory
            X86E820Entry(
                addr=0x100000,
                size=f"{self.low_mem.mem_ctrl[0].dram.range.size() - 0x100000:d}B",
                range_type=1,
            ),
        ]

        # Map the real main memory above 4GiB
        entries.append(
            X86E820Entry(
                addr=0x100000000,
                size=f"{self.mem_ranges[0].size()}B",
                range_type=1,
            )
        )

        # Reserve the last 16KiB of the 32-bit address space for m5ops
        entries.append(
            X86E820Entry(addr=0xFFFF0000, size="64KiB", range_type=2)
        )

        self.workload.e820_table.entries = entries

    @overrides(KernelDiskWorkload)
    def _set_readfile_contents(self, readfile_contents):
        """In the case of a GPU workload, we need to load the GPU driver first.
        This method will prepend the driver load command to the
        readfile_contents. Note that this assumes that all of the gpus are the
        same, if there's more that one GPU.
        """

        if self.get_devices() is None:
            warn("No GPU devices. Not loading GPU driver.")
            return super()._set_readfile_contents(readfile_contents)

        driver_load_command = self.get_devices()[0].get_driver_command()
        contents = driver_load_command + "\n" + readfile_contents

        return super()._set_readfile_contents(contents)

    # Replicate the capability of the old GPUFS config, which embed a binary
    # application or script into a bash script setting up the environment and
    # loading the GPU driver.
    def make_gpu_app(self, gpu: BaseViperGPU, app: str, opts: str):
        with open(os.path.abspath(app), "rb") as binfile:
            encodedBin = base64.b64encode(binfile.read()).decode()

        application_command = (
            f'echo "{encodedBin}" | base64 -d > myapp\n'
            "chmod +x myapp\n"
            f"./myapp {opts}\n"
            "sleep 20\n"
            "/sbin/m5 exit\n"
        )

        return application_command
