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

from system.amdgpu import *

from m5.util import panic

from common.Benchmarks import *
from common.FSConfig import *
from common import GPUTLBConfig
from common import Simulation
from ruby import Ruby

from example.gpufs.Disjoint_VIPER import *


def makeGpuFSSystem(args):
    # Boot options are standard gem5 options plus:
    # - Framebuffer device emulation 0 to reduce driver code paths.
    # - Blacklist amdgpu as it cannot (currently) load in KVM CPU.
    # - Blacklist psmouse as amdgpu driver adds proprietary commands that
    #   cause gem5 to panic.
    boot_options = [
        "earlyprintk=ttyS0",
        "console=ttyS0,9600",
        "lpj=7999923",
        "root=/dev/sda1",
        "drm_kms_helper.fbdev_emulation=0",
        "modprobe.blacklist=amdgpu",
        "modprobe.blacklist=psmouse",
    ]
    cmdline = " ".join(boot_options)

    if MemorySize(args.mem_size) < MemorySize("2GB"):
        panic("Need at least 2GB of system memory to load amdgpu module")

    # Use the common FSConfig to setup a Linux X86 System
    (TestCPUClass, test_mem_mode) = Simulation.getCPUClass(args.cpu_type)
    if test_mem_mode == "atomic":
        test_mem_mode = "atomic_noncaching"
    disks = [args.disk_image]
    if args.second_disk is not None:
        disks.extend([args.second_disk])
    bm = SysConfig(disks=disks, mem=args.mem_size)
    system = makeLinuxX86System(
        test_mem_mode, args.num_cpus, bm, True, cmdline=cmdline
    )
    system.workload.object_file = binary(args.kernel)

    # Set the cache line size for the entire system.
    system.cache_line_size = args.cacheline_size

    # Create a top-level voltage and clock domain.
    system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)
    system.clk_domain = SrcClockDomain(
        clock=args.sys_clock, voltage_domain=system.voltage_domain
    )

    # Create a CPU voltage and clock domain.
    system.cpu_voltage_domain = VoltageDomain()
    system.cpu_clk_domain = SrcClockDomain(
        clock=args.cpu_clock, voltage_domain=system.cpu_voltage_domain
    )

    # Setup VGA ROM region
    system.shadow_rom_ranges = [AddrRange(0xC0000, size=Addr("128kB"))]

    # Create specified number of CPUs. GPUFS really only needs one.
    system.cpu = [
        TestCPUClass(clk_domain=system.cpu_clk_domain, cpu_id=i)
        for i in range(args.num_cpus)
    ]
    if ObjectList.is_kvm_cpu(TestCPUClass):
        system.kvm_vm = KvmVM()

    # Create AMDGPU and attach to southbridge
    shader = createGPU(system, args)
    connectGPU(system, args)

    # The shader core will be whatever is after the CPU cores are accounted for
    shader_idx = args.num_cpus
    system.cpu.append(shader)

    # This arbitrary address is something in the X86 I/O hole
    hsapp_gpu_map_paddr = 0xE00000000
    hsapp_pt_walker = VegaPagetableWalker()
    gpu_hsapp = HSAPacketProcessor(
        pioAddr=hsapp_gpu_map_paddr,
        numHWQueues=args.num_hw_queues,
        walker=hsapp_pt_walker,
    )
    dispatcher = GPUDispatcher()
    cp_pt_walker = VegaPagetableWalker()
    gpu_cmd_proc = GPUCommandProcessor(
        hsapp=gpu_hsapp, dispatcher=dispatcher, walker=cp_pt_walker
    )
    shader.dispatcher = dispatcher
    shader.gpu_cmd_proc = gpu_cmd_proc

    system.pc.south_bridge.gpu.cp = gpu_cmd_proc

    # GPU Interrupt Handler
    device_ih = AMDGPUInterruptHandler()
    system.pc.south_bridge.gpu.device_ih = device_ih

    # Setup the SDMA engines depending on device. The MMIO base addresses
    # can be found in the driver code under:
    # include/asic_reg/sdmaX/sdmaX_Y_Z_offset.h
    num_sdmas = 2
    sdma_bases = []
    sdma_sizes = []
    if args.gpu_device == "Vega10":
        num_sdmas = 2
        sdma_bases = [0x4980, 0x5180]
        sdma_sizes = [0x800] * 2
    elif args.gpu_device == "MI100":
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
    else:
        m5.util.panic(f"Unknown GPU device {args.gpu_device}")

    sdma_pt_walkers = []
    sdma_engines = []
    for sdma_idx in range(num_sdmas):
        sdma_pt_walker = VegaPagetableWalker()
        sdma_engine = SDMAEngine(
            walker=sdma_pt_walker,
            mmio_base=sdma_bases[sdma_idx],
            mmio_size=sdma_sizes[sdma_idx],
        )
        sdma_pt_walkers.append(sdma_pt_walker)
        sdma_engines.append(sdma_engine)

    system.pc.south_bridge.gpu.sdmas = sdma_engines

    # Setup PM4 packet processor
    pm4_pkt_proc = PM4PacketProcessor()
    system.pc.south_bridge.gpu.pm4_pkt_proc = pm4_pkt_proc

    # GPU data path
    gpu_mem_mgr = AMDGPUMemoryManager()
    system.pc.south_bridge.gpu.memory_manager = gpu_mem_mgr

    # CPU data path (SystemHub)
    system_hub = AMDGPUSystemHub()
    shader.system_hub = system_hub

    # GPU, HSAPP, and GPUCommandProc are DMA devices
    system._dma_ports.append(gpu_hsapp)
    system._dma_ports.append(gpu_cmd_proc)
    system._dma_ports.append(system.pc.south_bridge.gpu)
    for sdma in sdma_engines:
        system._dma_ports.append(sdma)
    system._dma_ports.append(device_ih)
    system._dma_ports.append(pm4_pkt_proc)
    system._dma_ports.append(system_hub)
    system._dma_ports.append(gpu_mem_mgr)
    system._dma_ports.append(hsapp_pt_walker)
    system._dma_ports.append(cp_pt_walker)
    for sdma_pt_walker in sdma_pt_walkers:
        system._dma_ports.append(sdma_pt_walker)

    gpu_hsapp.pio = system.iobus.mem_side_ports
    gpu_cmd_proc.pio = system.iobus.mem_side_ports
    system.pc.south_bridge.gpu.pio = system.iobus.mem_side_ports
    for sdma in sdma_engines:
        sdma.pio = system.iobus.mem_side_ports
    device_ih.pio = system.iobus.mem_side_ports
    pm4_pkt_proc.pio = system.iobus.mem_side_ports
    system_hub.pio = system.iobus.mem_side_ports

    # Full system needs special TLBs for SQC, Scalar, and vector data ports
    args.full_system = True
    GPUTLBConfig.config_tlb_hierarchy(
        args, system, shader_idx, system.pc.south_bridge.gpu, True
    )

    # Create Ruby system using disjoint VIPER topology
    system.ruby = Disjoint_VIPER()
    system.ruby.create(args, system, system.iobus, system._dma_ports)

    # Create a seperate clock domain for Ruby
    system.ruby.clk_domain = SrcClockDomain(
        clock=args.ruby_clock, voltage_domain=system.voltage_domain
    )

    for (i, cpu) in enumerate(system.cpu):
        # Break once we reach the shader "CPU"
        if i == args.num_cpus:
            break

        #
        # Tie the cpu ports to the correct ruby system ports
        #
        cpu.clk_domain = system.cpu_clk_domain
        cpu.createThreads()
        cpu.createInterruptController()

        system.ruby._cpu_ports[i].connectCpuPorts(cpu)

        for j in range(len(system.cpu[i].isa)):
            system.cpu[i].isa[j].vendor_string = "AuthenticAMD"

    if args.host_parallel:
        # To get the KVM CPUs to run on different host CPUs, specify a
        # different event queue for each CPU.  The last CPU is a GPU
        # shader and should be skipped.
        for i, cpu in enumerate(system.cpu[:-1]):
            for obj in cpu.descendants():
                obj.eventq_index = 0
            cpu.eventq_index = i + 1

    gpu_port_idx = (
        len(system.ruby._cpu_ports)
        - args.num_compute_units
        - args.num_sqc
        - args.num_scalar_cache
    )
    gpu_port_idx = gpu_port_idx - args.num_cp * 2

    # Connect token ports. For this we need to search through the list of all
    # sequencers, since the TCP coalescers will not necessarily be first. Only
    # TCP coalescers use a token port for back pressure.
    token_port_idx = 0
    for i in range(len(system.ruby._cpu_ports)):
        if isinstance(system.ruby._cpu_ports[i], VIPERCoalescer):
            system.cpu[shader_idx].CUs[
                token_port_idx
            ].gmTokenPort = system.ruby._cpu_ports[i].gmTokenPort
            token_port_idx += 1

    wavefront_size = args.wf_size
    for i in range(args.num_compute_units):
        # The pipeline issues wavefront_size number of uncoalesced requests
        # in one GPU issue cycle. Hence wavefront_size mem ports.
        for j in range(wavefront_size):
            system.cpu[shader_idx].CUs[i].memory_port[
                j
            ] = system.ruby._cpu_ports[gpu_port_idx].in_ports[j]
        gpu_port_idx += 1

    for i in range(args.num_compute_units):
        if i > 0 and not i % args.cu_per_sqc:
            gpu_port_idx += 1
        system.cpu[shader_idx].CUs[i].sqc_port = system.ruby._cpu_ports[
            gpu_port_idx
        ].in_ports
    gpu_port_idx = gpu_port_idx + 1

    for i in range(args.num_compute_units):
        if i > 0 and not i % args.cu_per_scalar_cache:
            gpu_port_idx += 1
        system.cpu[shader_idx].CUs[i].scalar_port = system.ruby._cpu_ports[
            gpu_port_idx
        ].in_ports
    gpu_port_idx = gpu_port_idx + 1

    return system
