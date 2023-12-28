# Copyright (c) 2021 Advanced Micro Devices, Inc.
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

from m5.objects.ClockedObject import ClockedObject
from m5.objects.Device import (
    DmaDevice,
    DmaVirtDevice,
)
from m5.objects.PciDevice import (
    PciDevice,
    PciLegacyIoBar,
    PciMemBar,
    PciMemUpperBar,
)
from m5.params import *
from m5.proxy import *


# PCI device model for an AMD Vega 10 based GPU. The PCI codes and BARs
# correspond to a Vega Frontier Edition hardware device. None of the PCI
# related values in this class should be changed.
#
# This class requires a ROM binary and an MMIO trace to initialize the
# device registers and memory. It is intended only to be used in full-system
# simulation under Linux where the amdgpu driver is modprobed.
class AMDGPUDevice(PciDevice):
    type = "AMDGPUDevice"
    cxx_header = "dev/amdgpu/amdgpu_device.hh"
    cxx_class = "gem5::AMDGPUDevice"

    # Human readable name for device ID
    device_name = Param.String("Vega10", "Codename for device")

    # IDs for AMD Vega 10
    VendorID = 0x1002
    DeviceID = 0x6863
    # Command 0x3 never gets sent indicating IO and Mem bars are enabled. Hard
    # code the command here and deal unassigned BARs on C++ side.
    Command = 0x3
    Status = 0x0280
    Revision = 0x0
    ClassCode = 0x03
    SubClassCode = 0x00
    ProgIF = 0x00

    # Use max possible BAR size for Vega 10. We can override with driver param
    BAR0 = PciMemBar(size="16GiB")
    BAR1 = PciMemUpperBar()
    BAR2 = PciMemBar(size="2MiB")
    BAR3 = PciMemUpperBar()
    BAR4 = PciLegacyIoBar(addr=0xF000, size="256B")
    BAR5 = PciMemBar(size="512KiB")

    InterruptLine = 14
    InterruptPin = 2
    ExpansionROM = 0

    rom_binary = Param.String("ROM binary dumped from hardware")
    trace_file = Param.String("MMIO trace collected on hardware")
    checkpoint_before_mmios = Param.Bool(
        False, "Take a checkpoint before the device begins sending MMIOs"
    )

    # SDMA engines. There are a different number depending on device,
    # therefore an array is used.
    sdmas = VectorParam.SDMAEngine("All SDMA Engines")

    # The cp is needed here to handle certain packets the device may receive.
    # The config script should not create a new cp here but rather assign the
    # same cp that is assigned to the Shader SimObject.
    cp = Param.GPUCommandProcessor(NULL, "Command Processor")
    pm4_pkt_proc = Param.PM4PacketProcessor("PM4 Packet Processor")
    memory_manager = Param.AMDGPUMemoryManager("GPU Memory Manager")
    memories = VectorParam.AbstractMemory([], "All memories in the device")
    device_ih = Param.AMDGPUInterruptHandler("GPU Interrupt handler")


class SDMAEngine(DmaVirtDevice):
    type = "SDMAEngine"
    cxx_header = "dev/amdgpu/sdma_engine.hh"
    cxx_class = "gem5::SDMAEngine"

    mmio_base = Param.Addr(0x0, "Base MMIO Address")
    mmio_size = Param.Addr(0x800, "Size of MMIO range")

    gpu_device = Param.AMDGPUDevice(NULL, "GPU Controller")
    walker = Param.VegaPagetableWalker("Page table walker")


class PM4PacketProcessor(DmaVirtDevice):
    type = "PM4PacketProcessor"
    cxx_header = "dev/amdgpu/pm4_packet_processor.hh"
    cxx_class = "gem5::PM4PacketProcessor"


class AMDGPUMemoryManager(ClockedObject):
    type = "AMDGPUMemoryManager"
    cxx_header = "dev/amdgpu/memory_manager.hh"
    cxx_class = "gem5::AMDGPUMemoryManager"

    port = RequestPort("Memory Port to access VRAM (device memory)")
    system = Param.System(Parent.any, "System the dGPU belongs to")


class AMDGPUInterruptHandler(DmaDevice):
    type = "AMDGPUInterruptHandler"
    cxx_header = "dev/amdgpu/interrupt_handler.hh"
    cxx_class = "gem5::AMDGPUInterruptHandler"


class AMDGPUSystemHub(DmaDevice):
    type = "AMDGPUSystemHub"
    cxx_class = "gem5::AMDGPUSystemHub"
    cxx_header = "dev/amdgpu/system_hub.hh"
