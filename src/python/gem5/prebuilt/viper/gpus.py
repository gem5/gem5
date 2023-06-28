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

"""
Parameters: 
- number of compute units


"""

from pathlib import Path
from typing import List

from m5.objects import (
    Shader,
    ComputeUnit,
    Wavefront,
    SimplePoolManager,
    VectorRegisterFile,
    ScalarRegisterFile,
    RegisterManager,
    LdsState,
    BaseCPU,
    AMDGPUDevice,
    AMDGPUInterruptHandler,
    GPUDispatcher,
    GPUCommandProcessor,
    HSAPacketProcessor,
    VegaPagetableWalker,
    AMDGPUSystemHub,
    PM4PacketProcessor,
    AMDGPUMemoryManager,
    SDMAEngine,
    VegaGPUTLB,
    TLBCoalescer,
)


class ViperGPU(Shader):
    def __init__(self, num_cu: int, device: AMDGPUDevice, mmio_trace: Path, rom: Path):
        """

        This is the GPU, not the PCI device. However, this object does have
        functions to set up the PCI device.
        """
        super().__init__()
        self.timing = True
        # used to track the (many, many) DMA ports
        self._dma_ports = []

        # VIPER GPU protocol implements release consistency at GPU side. So,
        # we make their writes visible to the global memory and should read
        # from global memory during kernel boundary. The pipeline initiates
        # (or do not initiate) the acquire/release operation depending on
        # these impl_kern_launch_rel and impl_kern_end_rel flags. The flag=true
        # means pipeline initiates a acquire/release operation at kernel launch/end
        # VIPER protocol is write-through based, and thus only impl_kern_launch_acq
        # needs to set.
        self.impl_kern_launch_acq = True
        self.impl_kern_end_rel = False

        self.CUs = [
            ViperCU(device)
            for _ in range(num_cu)
        ]

        self._create_tlbs()

        self.trace_file = mmio_trace
        self.rom_binary = rom

        self._setup_device(device)

        # This arbitrary address is something in the X86 I/O hole
        hsapp_gpu_map_paddr = 0xE00000000
        self.dispatcher = GPUDispatcher()
        self.gpu_cmd_processor = GPUCommandProcessor(
            hsapp = HSAPacketProcessor(
                pioAddr=hsapp_gpu_map_paddr,
                numHWQueues=10,
                walker=VegaPagetableWalker(),
            ),
            dispatcher=self.dispatcher,
            walker=VegaPagetableWalker(),
        )
        self._dma_ports.append(self.gpu_cmd_processor.hsapp.dma)
        self._dma_ports.append(self.gpu_cmd_processor.hsapp.walker.dma)
        self._dma_ports.append(self.gpu_cmd_processor.walker.dma)
        self._dma_ports.append(self.gpu_cmd_processor.dma)

        self.system_hub = AMDGPUSystemHub()
        self._dma_ports.append(self.system_hub.dma)

    def set_cpu_pointer(self, cpu: BaseCPU):
        """ Set the CPU pointer for the CUs.
        """
        for cu in self.CUs:
            cu.cpu_pointer = cpu
    
    def _setup_device(self, device: AMDGPUDevice):
        """ Set the device type info on the device connected to the south 
            bridge.
        """
        self._device = device

        device.cp = self.gpu_cmd_processor
        device.device_ih = AMDGPUInterruptHandler()
        self._dma_ports.append(device.device_ih.dma)

        # Setup PM4 packet processor
        device.pm4_pkt_proc = PM4PacketProcessor()
        self._dma_ports.append(device.pm4_pkt_proc.dma)

        # GPU data path
        device.memory_manager = AMDGPUMemoryManager()
        self._dma_ports.append(device.memory_manager.dma)

        self._dma_ports.append(device.dma)
    
    def _create_sdmas(self, num_sdmas: int, sdma_bases: List[int], sdma_sizes: List[int]):
        """ Create the SDMA engines.
        """
        sdmas = [
            SDMAEngine(
                walker = VegaPagetableWalker(),
                mmio_base=sdma_bases[i],
                mmio_size=sdma_sizes[i],
            )
            for i in range(num_sdmas)
        ]

        for sdma in sdmas:
            self._dma_ports.append(sdma.dma)
            self._dma_ports.append(sdma.walker.dma)

        return sdmas
    
    def _get_dma_ports(self):
        return self._dma_ports

    def _create_tlbs(self):
        """ Connect per-CU TLBs to the L2/L3 TLBs
        """
        self.l2_tlb = VegaGPUTLB(
            gpu_device = self._device,
            size = 4096,
            assoc = 32,
            hitLatency = 69,
            missLatency1 = 750, ## ????
            missLatency2 = 750, ## ????
            maxOutstandingRequests = 64,
        )
        self.l2_coalescer = TLBCoalescer(
            probesPerCycle = 2,
            tlb_level = 2,
            coalescingWindow = 1,
            disableCoalescing = False, 
        )
        self.l2_tlb.cpu_side_ports = self.l2_coalescer.mem_side_ports  # May need [0]?

        for cu in self.CUs:
            for port in cu.get_tlb_ports():
                self.l2_coalescer.cpu_side_ports = port
        
        self.l3_tlb = VegaGPUTLB(
            gpu_device = self._device,
            size = 8192,
            assoc = 32,
            hitLatency = 150,
            missLatency1 = 750, ## ????
            missLatency2 = 750, ## ????
            maxOutstandingRequests = 64,
        )
        self.l3_coalescer = TLBCoalescer(
            probesPerCycle = 2,
            tlb_level = 3,
            coalescingWindow = 1,
            disableCoalescing = False, 
        )
        self.l3_tlb.cpu_side_ports = self.l3_coalescer.mem_side_ports  # May need [0]?
        self.l3_coalescer.cpu_side_ports = self.l2_tlb.mem_side_ports

        self._dma_ports.append(self.l3_tlb.walker.dma)
    
    def connect_iobus(self, iobus: BaseXBar):
        """ Connect the GPU devices to the IO bus.
        """
        self.gpu_cmd_processor.pio = iobus.mem_side_ports
        self.gpu_cmd_processor.hsapp.pio = iobus.mem_side_ports
        self.system_hub.pio = iobus.mem_side_ports
        self._device.pio = iobus.mem_side_ports
        for sdma in self._device.sdmas:
            sdma.pio = iobus.mem_side_ports
        self._device.device_ih.pio = iobus.mem_side_ports
        self._device.pm4_pkt_proc.pio = iobus.mem_side_ports

class MI100GPU(ViperGPU):
    def setup_device(self, device: AMDGPUDevice):
        super().setup_device(device)
        device.DeviceID = 0x738C
        device.SubsystemVendorID = 0x1002
        device.SubsystemID = 0x0C34

        # Setup the SDMA engines depending on device. The MMIO base addresses
        # can be found in the driver code under:
        # include/asic_reg/sdmaX/sdmaX_Y_Z_offset.h
        num_sdmas = 8
        sdma_bases = [
            0x4980,
            0x6180,
            0x78000,
            0x79000,
            0x7A000,
            0x7B000,
            0x7C000,
            0x7D000,
        ]
        sdma_sizes = [0x1000] * 8

        device.sdmas = self._create_sdmas(num_sdmas, sdma_bases, sdma_sizes)

class Vega10GPU(ViperGPU):
    def setup_device(self, device: AMDGPUDevice):
        super().setup_device(device)
        device.DeviceID = 0x6863

        # Setup the SDMA engines depending on device. The MMIO base addresses
        # can be found in the driver code under:
        # include/asic_reg/sdmaX/sdmaX_Y_Z_offset.h
        num_sdmas = 2
        sdma_bases = [0x4980, 0x5180]
        sdma_sizes = [0x800] * 2

        device.sdmas = self._create_sdmas(num_sdmas, sdma_bases, sdma_sizes)

class ViperCU(ComputeUnit):

    _next_id = 0
    @classmethod
    def _get_next_id(cls):
        cls._next_id += 1
        return cls._next_id - 1

    def __init__(self, device: AMDGPUDevice):
        super().__init__()
        self._device = device

        self.cu_id = self._get_next_id()
        self.localDataStore = LdsState()
        self.simds_per_cu = 4
        self.n_wf = 10

        self.wavefronts = [
            Wavefront(simdId=j, wf_slot_id=k)
            for j in range(self.simds_per_cu) for k in range(self.n_wf)
        ]

        self.vector_register_file = [
            VectorRegisterFile(simd_id=i)
            for i in range(self.simds_per_cu)
        ]

        self.scalar_register_file = [
            ScalarRegisterFile(simd_id=i)
            for i in range(self.simds_per_cu)
        ]

        self.register_manager = RegisterManager(
            vrf_pool_managers=[
                SimplePoolManager()
                for _ in range(self.simds_per_cu)
            ],
            srf_pool_managers=[
                SimplePoolManager()
                for _ in range(self.simds_per_cu)
            ]
        )

        self.ldsPort = self.ldsBus.cpu_side_port
        self.ldsPort.mem_side_port = self.localDataStore.cuPort

        self._create_tlbs()
    
    def _create_tlbs(self):
        self.l1_tlb = VegaGPUTLB(
            gpu_device = self._device,
            size = 32,
            assoc = 32,
            hitLatency = 1,
            missLatency1 = 750, ## ????
            missLatency2 = 750, ## ????
            maxOutstandingRequests = 64,
        )
        self.l1_coalescer = TLBCoalescer(
            probesPerCycle = 2,
            tlb_level = 1,
            coalescingWindow = 1,
            disableCoalescing = False, 
        )
        self.l1_tlb.cpu_side_ports = self.l1_coalescer.mem_side_ports  # May need [0]?
        self.translation_port = self.l1_coalescer.cpu_side_ports

        self.sqc_tlb = VegaGPUTLB( # the I-TLB
            gpu_device = self._device,
            size = 32,
            assoc = 32,
            hitLatency = 1,
            missLatency1 = 750, ## ????
            missLatency2 = 750, ## ????
            maxOutstandingRequests = 64,
        )
        self.sqc_coalescer = TLBCoalescer(
            probesPerCycle = 2,
            tlb_level = 1,
            coalescingWindow = 1,
            disableCoalescing = False, 
        )
        self.sqc_tlb.cpu_side_ports = self.sqc_coalescer.mem_side_ports
        self.sqc_tlb_port = self.sqc_coalescer.cpu_side_ports

        self.scalar_tlb = VegaGPUTLB( # the Scalar D-TLB
            size = 32,
            assoc = 32,
            hitLatency = 1,
            missLatency1 = 750, ## ????
            missLatency2 = 750, ## ????
            maxOutstandingRequests = 64,
        )
        self.scalar_coalescer = TLBCoalescer(
            probesPerCycle = 2,
            tlb_level = 1,
            coalescingWindow = 1,
            disableCoalescing = False, 
        )
        self.scalar_tlb.cpu_side_ports = self.scalar_coalescer.mem_side_ports
        self.scalar_tlb_port = self.scalar_coalescer.cpu_side_ports

    def get_tlb_ports(self):
        return [
            self.l1_tlb.mem_side_ports,
            self.sqc_tlb.mem_side_ports,
            self.scalar_tlb.mem_side_ports,
        ]