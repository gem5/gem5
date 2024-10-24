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

from m5.objects import (
    AMDGPUDevice,
    SubSystem,
)

from ....components.boards.abstract_board import AbstractBoard
from ....prebuilt.viper.gpu_cache_hierarchy import ViperGPUCacheHierarchy
from .viper_shader import ViperShader


class BaseViperGPU(SubSystem):
    _base_pci_dev = 8
    _gpu_count = 0
    _my_id = 0

    @classmethod
    def next_pci_dev(cls):
        cls._gpu_count += 1  # Use count for this particular type
        return cls._base_pci_dev + cls._gpu_count - 1

    @classmethod
    def get_gpu_count(cls):
        return cls._gpu_count

    def __init__(self):
        # Setup various PCI related parameters
        self._my_id = self.get_gpu_count()
        pci_dev = self.next_pci_dev()

        device = AMDGPUDevice(pci_func=0, pci_dev=pci_dev, pci_bus=0)
        self._device = device

    def set_shader(self, shader: ViperShader):
        self._shader = shader

    def get_cpu_dma_ports(self):
        return self._shader.get_cpu_dma_ports()

    def connectGPU(self, board: AbstractBoard) -> None:
        # Connect a CPU pointer. This is only used for SE mode. Any CPU will
        # work, so pick assuming there is at least one
        cpus = board.get_processor()
        self._shader.set_cpu_pointer(cpus.cores[0].core)

        # Connect all PIO buses
        self._shader.connect_iobus(board.get_io_bus())

        # The System() object in gem5 has a memories parameter which defaults
        # to Self.all. This will collect *all* AbstractMemories and connect to
        # the CPU side. To avoid this we manually assign the memories param to
        # the CPU side memories. We need the MemInterface which is called dram
        # in the MemCtrl class even though it might not be modelling dram.
        memory = board.get_memory()
        cpu_abs_mems = [mem.dram for mem in memory.get_memory_controllers()]
        board.memories = cpu_abs_mems

        # Make the cache hierarchy. This will create an independent RubySystem
        # class containing only the GPU caches with no network connection to
        # the CPU cache hierarchy.
        self._device.gpu_caches = ViperGPUCacheHierarchy(
            tcp_size=self._tcp_size,
            tcp_assoc=self._tcp_assoc,
            sqc_size=self._sqc_size,
            sqc_assoc=self._sqc_assoc,
            scalar_size=self._scalar_size,
            scalar_assoc=self._scalar_assoc,
            tcc_size=self._tcc_size,
            tcc_assoc=self._tcc_assoc,
            tcc_count=self._tcc_count,
            cu_per_sqc=self._cu_per_sqc,
            num_memory_channels=self._num_memory_channels,
            cache_line_size=self._cache_line_size,
            shader=self._shader,
        )

        # Collect GPU memory controllers created in the GPU cache hierarchy.
        # First assign them as a child to the device so the SimObject unproxy.
        # The device requires the memories parameter to be set as the system
        # pointer required by the AbstractMemory class is set by AMDGPUDevice.
        self._device.mem_ctrls = self._device.gpu_caches.get_mem_ctrls()
        gpu_abs_mems = [mem.dram for mem in self._device.mem_ctrls]
        self._device.memories = gpu_abs_mems

        # Finally attach to the board. PciDevices default to Parent.any for the
        # PciHost parameter. To make sure this is found we need to connect to
        # board.pc or a child of board.pc. Historically we place this in the
        # south bridge.
        board.pc.south_bridge.gpu_shader = self._shader

        # This is cosmetic so the device shows as board.pc.south_bridge.gpu###
        # instead of board.pc.south_bridge.gpu_shader.CUs.l1_tlb.gpu_device.
        gpu_name = f"gpu{self._my_id}"
        self._device.set_parent(board.pc.south_bridge, gpu_name)


# A scaled down MI210-like device. Defaults to ~1/4th of an MI210.
class MI210(BaseViperGPU):
    def __init__(
        self,
        num_cus: int = 32,
        cu_per_sqc: int = 4,
        tcp_size: str = "16KiB",
        tcp_assoc: int = 16,
        sqc_size: str = "32KiB",
        sqc_assoc: int = 8,
        scalar_size: str = "32KiB",
        scalar_assoc: int = 8,
        tcc_size: str = "256KiB",
        tcc_assoc: int = 16,
        tcc_count: int = 8,
        num_memory_channels: int = 8,
        cache_line_size: int = 64,
    ):
        super().__init__()

        self._cu_per_sqc = cu_per_sqc
        self._tcp_size = tcp_size
        self._tcp_assoc = tcp_assoc
        self._sqc_size = sqc_size
        self._sqc_assoc = sqc_assoc
        self._scalar_size = scalar_size
        self._scalar_assoc = scalar_assoc
        self._tcc_size = tcc_size
        self._tcc_assoc = tcc_assoc
        self._tcc_count = tcc_count
        self._num_memory_channels = num_memory_channels
        self._cache_line_size = cache_line_size

        self._device.device_name = "MI200"

        self._device.DeviceID = 0x740F
        self._device.SubsystemVendorID = 0x1002
        self._device.SubsystemID = 0x0C34

        # Setup device-specific address ranges for various SoC components.
        shader = ViperShader(
            self._my_id, num_cus, cache_line_size, self._device
        )
        self.set_shader(shader)

        # Setup the SDMA engines depending on device. The MMIO base addresses
        # can be found in the driver code under:
        # include/asic_reg/sdmaX/sdmaX_Y_Z_offset.h
        num_sdmas = 5
        sdma_bases = [0x4980, 0x6180, 0x78000, 0x79000, 0x7A000]
        sdma_sizes = [0x1000] * 5

        self._device.sdmas = shader._create_sdmas(sdma_bases, sdma_sizes)

        # Setup the Command Processor's PM4 engines.
        pm4_starts = [0xC000]
        pm4_ends = [0xD000]

        self._device.pm4_pkt_procs = shader._create_pm4s(pm4_starts, pm4_ends)

    def get_driver_command(self, debug: bool = False):
        debug_commands = "dmesg -n8\n" if debug else ""

        driver_load_command = (
            "export LD_LIBRARY_PATH=/opt/rocm/lib:$LD_LIBRARY_PATH\n"
            "export HSA_ENABLE_INTERRUPT=0\n"
            "export HCC_AMDGPU_TARGET=gfx90a\n"
            f"{debug_commands}\n"
            "dd if=/root/roms/mi200.rom of=/dev/mem bs=1k seek=768 count=128\n"
            "if [ ! -f /lib/modules/`uname -r`/updates/dkms/amdgpu.ko ]; then\n"
            '    echo "ERROR: Missing DKMS package for kernel `uname -r`. Exiting gem5."\n'
            "    /sbin/m5 exit\n"
            "fi\n"
            "modprobe -v amdgpu ip_block_mask=0x6f ppfeaturemask=0 dpm=0 audio=0 ras_enable=0\n"
        )

        return driver_load_command


# Defaults to a single "XCD" (i.e., 1/8th of a full MI300X).
class MI300X(BaseViperGPU):
    def __init__(
        self,
        num_cus: int = 40,
        cu_per_sqc: int = 4,
        tcp_size: str = "16KiB",
        tcp_assoc: int = 16,
        sqc_size: str = "32KiB",
        sqc_assoc: int = 8,
        scalar_size: str = "32KiB",
        scalar_assoc: int = 8,
        tcc_size: str = "256KiB",
        tcc_assoc: int = 16,
        tcc_count: int = 16,
        num_memory_channels: int = 16,
        cache_line_size: int = 64,
    ):
        super().__init__()

        self._cu_per_sqc = cu_per_sqc
        self._tcp_size = tcp_size
        self._tcp_assoc = tcp_assoc
        self._sqc_size = sqc_size
        self._sqc_assoc = sqc_assoc
        self._scalar_size = scalar_size
        self._scalar_assoc = scalar_assoc
        self._tcc_size = tcc_size
        self._tcc_assoc = tcc_assoc
        self._tcc_count = tcc_count
        self._num_memory_channels = num_memory_channels
        self._cache_line_size = cache_line_size

        self._device.device_name = "MI300X"

        self._device.DeviceID = 0x740F
        self._device.SubsystemVendorID = 0x1002
        self._device.SubsystemID = 0x0C34

        # Setup device-specific address ranges for various SoC components.
        shader = ViperShader(
            self._my_id, num_cus, cache_line_size, self._device
        )
        self.set_shader(shader)

        # These currently use MI200 values until the MI300X bios is released.
        num_sdmas = 5
        sdma_bases = [0x4980, 0x6180, 0x78000, 0x79000, 0x7A000]
        sdma_sizes = [0x1000] * 5

        self._device.sdmas = shader._create_sdmas(sdma_bases, sdma_sizes)

        # Setup the Command Processor's PM4 engines.
        pm4_starts = [0xC000]
        pm4_ends = [0xD000]

        self._device.pm4_pkt_procs = shader._create_pm4s(pm4_starts, pm4_ends)

    def get_driver_command(self, debug: bool = False):
        debug_commands = "dmesg -n8\n" if debug else ""

        driver_load_command = (
            "export LD_LIBRARY_PATH=/opt/rocm/lib:$LD_LIBRARY_PATH\n"
            "export HSA_ENABLE_INTERRUPT=0\n"
            "export HCC_AMDGPU_TARGET=gfx942\n"
            'export HSA_OVERRIDE_GFX_VERSION="9.4.2"\n'
            f"{debug_commands}\n"
            "dd if=/root/roms/mi200.rom of=/dev/mem bs=1k seek=768 count=128\n"
            "if [ ! -f /lib/modules/`uname -r`/updates/dkms/amdgpu.ko ]; then\n"
            '    echo "ERROR: Missing DKMS package for kernel `uname -r`. Exiting gem5."\n'
            "    /sbin/m5 exit\n"
            "fi\n"
            "modprobe -v amdgpu ip_block_mask=0x6f ppfeaturemask=0 dpm=0 audio=0 ras_enable=0\n"
        )

        return driver_load_command
