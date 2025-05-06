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

from typing import List

from m5.objects import (
    AddrRange,
    AMDGPUDevice,
    AMDGPUInterruptHandler,
    AMDGPUMemoryManager,
    AMDGPUSystemHub,
    BaseCPU,
    BaseXBar,
    ComputeUnit,
    DynPoolManager,
    GPUCommandProcessor,
    GPUDispatcher,
    HSAPacketProcessor,
    LdsState,
    PciLegacyIoBar,
    PM4PacketProcessor,
    RegisterFileCache,
    RegisterManager,
    ScalarRegisterFile,
    SDMAEngine,
    Shader,
    VectorRegisterFile,
    VegaGPUTLB,
    VegaPagetableWalker,
    VegaTLBCoalescer,
    Wavefront,
)


class ViperCU(ComputeUnit):
    def __init__(self, cu_id: int, device: AMDGPUDevice):
        """ComputeUnit object of a gfx9-like compute unit."""
        super().__init__()
        self._device = device

        self.cu_id = cu_id

        # Use in multiple places. Define variables to change once.
        self._vreg_file_size = 2048
        self._sreg_file_size = 2048

        # Latencies, etc. use defaults in src/gpu-compute/GPU.py.
        self.num_SIMDs = 4
        self.n_wf = 8

        self.localDataStore = LdsState(
            size=65536,
        )

        self.wavefronts = [
            Wavefront(simdId=j, wf_slot_id=k)
            for j in range(self.num_SIMDs)
            for k in range(self.n_wf)
        ]

        self.vector_register_file = [
            VectorRegisterFile(simd_id=i, num_regs=self._vreg_file_size)
            for i in range(self.num_SIMDs)
        ]

        self.scalar_register_file = [
            ScalarRegisterFile(simd_id=i, num_regs=self._sreg_file_size)
            for i in range(self.num_SIMDs)
        ]

        self.register_file_cache = [
            RegisterFileCache(simd_id=i) for i in range(self.num_SIMDs)
        ]

        self.register_manager = RegisterManager(
            policy="static",
            vrf_pool_managers=[
                DynPoolManager(pool_size=self._vreg_file_size, min_alloc=4)
                for _ in range(self.num_SIMDs)
            ],
            srf_pool_managers=[
                DynPoolManager(pool_size=self._sreg_file_size, min_alloc=4)
                for _ in range(self.num_SIMDs)
            ],
        )

        self.ldsPort = self.ldsBus.cpu_side_port
        self.ldsBus.mem_side_port = self.localDataStore.cuPort

        self._create_tlbs()

    def _create_tlbs(self):
        # Vector memory TLB
        self.l1_tlb = VegaGPUTLB(
            gpu_device=self._device,
            size=64,
            assoc=64,
            hitLatency=1,
            missLatency1=750,
            missLatency2=750,
            maxOutstandingReqs=64,
        )

        self.l1_coalescer = VegaTLBCoalescer(tlb_level=1)

        self.translation_port = self.l1_coalescer.cpu_side_ports
        self.l1_coalescer.mem_side_ports = self.l1_tlb.cpu_side_ports

        # Scalar memory TLB
        self.scalar_tlb = VegaGPUTLB(
            gpu_device=self._device,
            size=64,
            assoc=64,
            hitLatency=1,
            missLatency1=750,
            missLatency2=750,
            maxOutstandingReqs=64,
        )

        self.scalar_coalescer = VegaTLBCoalescer(tlb_level=1)

        self.scalar_tlb_port = self.scalar_coalescer.cpu_side_ports
        self.scalar_coalescer.mem_side_ports = self.scalar_tlb.cpu_side_ports

        # Instruction memory TLB
        self.sqc_tlb = VegaGPUTLB(
            gpu_device=self._device,
            size=64,
            assoc=64,
            hitLatency=1,
            missLatency1=750,
            missLatency2=750,
            maxOutstandingReqs=64,
        )

        self.sqc_coalescer = VegaTLBCoalescer(tlb_level=1)

        self.sqc_tlb_port = self.sqc_coalescer.cpu_side_ports
        self.sqc_coalescer.mem_side_ports = self.sqc_tlb.cpu_side_ports

    def get_tlb_ports(self):
        return [
            self.l1_tlb.mem_side_ports,
            self.sqc_tlb.mem_side_ports,
            self.scalar_tlb.mem_side_ports,
        ]


class ViperShader(Shader):
    def __init__(
        self,
        shader_id: int,
        num_cus: int,
        cache_line_size: int,
        device: AMDGPUDevice,
    ):
        """
        The shader defines something the represents a single software visible
        GPU (e.g., a graphics card, a chiplet on a GPU, etc.).
        """
        super().__init__()

        self._shader_id = shader_id
        self._cache_line_size = cache_line_size
        self._device = device

        self.n_wf = 8
        self.timing = True
        # used to track the (many, many) DMA ports
        self._cpu_dma_ports = []
        self._gpu_dma_ports = []

        # VIPER GPU protocol implements release consistency at GPU side. So,
        # we make their writes visible to the global memory and should read
        # from global memory during kernal boundary. The pipeline initiates
        # (or do not initiate) the acquire/release operation depending on
        # these impl_kern_launch_rel and impl_kern_end_rel flags. The flag=true
        # means pipeline initiates a acquire/release operation at kernel launch/end
        # VIPER protocol is write-through based, and thus only impl_kern_launch_acq
        # needs to set.
        self.impl_kern_launch_acq = True
        self.impl_kern_end_rel = False

        # Attach compute units to GPU
        self.CUs = [ViperCU(idx, device) for idx in range(num_cus)]

        self._create_tlbs(device)

        # This arbitrary address is something in the X86 I/O hole
        hsapp_gpu_map_paddr = 0xE00000000
        self.dispatcher = GPUDispatcher()
        self.gpu_cmd_proc = GPUCommandProcessor(
            hsapp=HSAPacketProcessor(
                pioAddr=hsapp_gpu_map_paddr,
                numHWQueues=10,
                walker=VegaPagetableWalker(),
            ),
            dispatcher=self.dispatcher,
            walker=VegaPagetableWalker(),
        )
        self._cpu_dma_ports.append(self.gpu_cmd_proc.hsapp.dma)
        self._cpu_dma_ports.append(self.gpu_cmd_proc.dma)

        self._gpu_dma_ports.append(self.gpu_cmd_proc.hsapp.walker.port)
        self._gpu_dma_ports.append(self.gpu_cmd_proc.walker.port)

        self.system_hub = AMDGPUSystemHub()
        self._cpu_dma_ports.append(self.system_hub.dma)

        self._setup_device(device)

    def get_compute_units(self):
        return self.CUs

    def _setup_device(self, device: AMDGPUDevice):
        """Set the device type info on the device connected via PCI."""
        device.cp = self.gpu_cmd_proc
        device.device_ih = AMDGPUInterruptHandler()
        self._cpu_dma_ports.append(device.device_ih.dma)

        # GPU data path
        device.memory_manager = AMDGPUMemoryManager(
            cache_line_size=self._cache_line_size,
        )
        self._gpu_dma_ports.append(device.memory_manager.port)

        self._cpu_dma_ports.append(device.dma)

        # Use the gem5 default of 0x280 OR'd  with 0x10 which tells Linux there is
        # a PCI capabilities list to travse.
        device.Status = 0x0290

        # The PCI capabilities are like a linked list. The list has a memory
        # offset and a capability type ID read by the OS. Make the first
        # capability at 0x80 and set the PXCAP (PCI express) capability to
        # that address. Mark the type ID as PCI express.
        # We leave the next ID of PXCAP blank to end the list.
        device.PXCAPBaseOffset = 0x80
        device.CapabilityPtr = 0x80
        device.PXCAPCapId = 0x10

        # Set bits 7 and 8 in the second PCIe device capabilities register which
        # reports support for PCIe atomics for 32 and 64 bits respectively.
        # Bit 9 for 128-bit compare and swap is not set because the amdgpu driver
        # does not check this.
        device.PXCAPDevCap2 = 0x00000180

        # Set bit 6 to enable atomic requestor, meaning this device can request
        # atomics from other PCI devices.
        device.PXCAPDevCtrl2 = 0x0040

        # If there are multiple GPUs in the system, make sure the VBIOS region
        # and the legacy IO bar do not overlap with the ranges from other GPUs.
        if self._shader_id != 0:
            device.ExpansionROM = 0xD0000000 + (0x20000 * self._shader_id)
            bar4_addr = 0xF000 + (0x100 * self._shader_id)
            device.BAR4 = PciLegacyIoBar(addr=bar4_addr, size="256B")

    def _create_pm4s(self, pm4_starts: List[int], pm4_ends: List[int]):
        """Create PM4 packet processors."""
        num_pm4s = len(pm4_starts)

        pm4_procs = [
            PM4PacketProcessor(
                ip_id=i,
                mmio_range=AddrRange(start=pm4_starts[i], end=pm4_ends[i]),
            )
            for i in range(num_pm4s)
        ]

        for pm4_proc in pm4_procs:
            self._cpu_dma_ports.append(pm4_proc.dma)

        return pm4_procs

    def _create_sdmas(self, sdma_bases: List[int], sdma_sizes: List[int]):
        """Create the SDMA engines."""
        num_sdmas = len(sdma_bases)

        sdmas = [
            SDMAEngine(
                walker=VegaPagetableWalker(),
                mmio_base=sdma_bases[i],
                mmio_size=sdma_sizes[i],
            )
            for i in range(num_sdmas)
        ]

        for sdma in sdmas:
            self._cpu_dma_ports.append(sdma.dma)
            self._gpu_dma_ports.append(sdma.walker.port)

        return sdmas

    def get_cpu_dma_ports(self):
        return self._cpu_dma_ports

    def get_gpu_dma_ports(self):
        return self._gpu_dma_ports

    def _create_tlbs(self, device: AMDGPUDevice):
        """Connect per-CU TLBs to the L2/L3 TLBs"""
        self.l2_tlb = VegaGPUTLB(
            gpu_device=device,
            size=4096,
            assoc=64,
            hitLatency=69,
            missLatency1=750,
            missLatency2=750,
            maxOutstandingReqs=64,
        )

        self.l2_coalescer = VegaTLBCoalescer(tlb_level=2)

        self.l3_tlb = VegaGPUTLB(
            gpu_device=device,
            size=8192,
            assoc=64,
            hitLatency=150,
            missLatency1=750,
            missLatency2=750,
            maxOutstandingReqs=64,
        )

        self.l3_coalescer = VegaTLBCoalescer(tlb_level=3)

        # Port flow: [L1s] -> L2 coalescer -> L2 tlb -> L3 coalescer -> L3 tlb
        for cu in self.CUs:
            for port in cu.get_tlb_ports():
                self.l2_coalescer.cpu_side_ports = port
        self.l2_coalescer.mem_side_ports = self.l2_tlb.cpu_side_ports
        self.l2_tlb.mem_side_ports = self.l3_coalescer.cpu_side_ports
        self.l3_coalescer.mem_side_ports = self.l3_tlb.cpu_side_ports

        self._gpu_dma_ports.append(self.l3_tlb.walker.port)

    def connect_iobus(self, iobus: BaseXBar):
        """Connect the GPU objects to the IO bus."""
        self.gpu_cmd_proc.pio = iobus.mem_side_ports
        self.gpu_cmd_proc.hsapp.pio = iobus.mem_side_ports
        self.system_hub.pio = iobus.mem_side_ports
        self._device.pio = iobus.mem_side_ports
        self._device.device_ih.pio = iobus.mem_side_ports
        for sdma in self._device.sdmas:
            sdma.pio = iobus.mem_side_ports
        for pm4_proc in self._device.pm4_pkt_procs:
            pm4_proc.pio = iobus.mem_side_ports

    def set_cpu_pointer(self, cpu: BaseCPU):
        """Set the CPU pointer for the Shader."""
        self.cpu_pointer = cpu
