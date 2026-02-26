# Copyright (c) 2016 Georgia Institute of Technology
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
#
# Author: Tushar Krishna

import argparse
import os
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath

addToPath("../")

from common import Options
from ruby import Ruby

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)

parser = argparse.ArgumentParser()
Options.addNoISAOptions(parser)

parser.add_argument(
    "--temporal-traffic",
    type=str,
    default="by_default",
    help="Temporal traffic type and arguments",
)

parser.add_argument(
    "--synthetic",
    default="uniform_random",
    choices=[
        "uniform_random",
        "tornado",
        "bit_complement",
        "bit_reverse",
        "bit_rotation",
        "neighbor",
        "shuffle",
        "transpose",
    ],
)

parser.add_argument(
    "-i",
    "--injectionrate",
    type=float,
    default=0.1,
    metavar="I",
    help="Injection rate in packets per cycle per node. \
                        Takes decimal value between 0 to 1 (eg. 0.225). \
                        Number of digits after 0 depends upon --precision.",
)

parser.add_argument(
    "--precision",
    type=int,
    default=3,
    help="Number of digits of precision after decimal point\
                        for injection rate",
)

parser.add_argument(
    "--sim-cycles", type=int, default=1000, help="Number of simulation cycles"
)

parser.add_argument(
    "--num-packets-max",
    type=int,
    default=-1,
    help="Stop injecting after --num-packets-max.\
                        Set to -1 to disable.",
)

parser.add_argument(
    "--single-sender-id",
    type=int,
    default=-1,
    help="Only inject from this sender.\
                        Set to -1 to disable.",
)

parser.add_argument(
    "--single-dest-id",
    type=int,
    default=-1,
    help="Only send to this destination.\
                        Set to -1 to disable.",
)

parser.add_argument(
    "--inj-vnet",
    type=int,
    default=-1,
    choices=[-1, 0, 1, 2],
    help="Only inject in this vnet (0, 1 or 2).\
                        0 and 1 are 1-flit, 2 is 5-flit.\
                        Set to -1 to inject randomly in all vnets.",
)

parser.add_argument(
    "--buffers-per-data-vc",
    type=int,
    default=4,
    choices=[1, 2, 3, 4, 5],
    help="The size of each data vc in terms of \
                        no of flits.",
)

parser.add_argument(
    "--response-limit",
    type=int,
    default=5000,
    help="the simulation will stop if \
            here is no progress for that many cycles",
)
'''
parser.add_argument(
    "--simple-physical-channels",
    action="store_true",
    default=False,
    help="""Simplenetwork links uses a seperate physical channel
            for each virtual network""",
)
'''
parser.add_argument(
    "--fast-boot",
    action="store_true",
    help="Use Kvm for fast boot and checkpoint os state",
)
parser.add_argument(
    "--restore-from",
    type=str,
    default="",
    help="Path to checkpoint directory to restore from",
)
#
# Add the ruby specific and protocol specific options
#
Ruby.define_options(parser)

args = parser.parse_args()
print(f"args.cacheline_size:{args.cacheline_size}")
"""
args.num_cpus=64
args.num_dirs=64
args.network="garnet"
args.topology="Mesh_XY"
args.inj_vnet=-1
args.router_latency=5
args.link_latency=1
args.mesh_rows=8
args.sim_cycles=1000000
args.num_packets_max=-1
args.single_sender_id=-1
args.single_dest_id=-1
args.link_width_bits=128
args.vcs_per_vnet=16
args.garnet_deadlock_threshold=5000
args.routing_algorithm=1
args.mem_size="512MiB"
args.synthetic="uniform_random"
args.injection_rate=0.01
"""
"""
inj_rates=[
        0.025, 0.03, 0.03, 0.005, 0.005, 0.03, 0.03, 0.03,
        0.03, 0.005, 0.03, 0.005, 0.03, 0.03, 0.03, 0.03,
        0.03, 0.03, 0.005, 0.005, 0.005, 0.005, 0.03, 0.03,
        0.005, 0.03, 0.005, 0.005, 0.005, 0.005, 0.03, 0.005,
        0.005, 0.03, 0.005, 0.005, 0.005, 0.005, 0.03, 0.005,
        0.03, 0.005, 0.005, 0.005, 0.005, 0.005, 0.03, 0.03,
        0.03, 0.005, 0.03, 0.005, 0.03, 0.03, 0.005, 0.03,
        0.03, 0.03, 0.03, 0.005, 0.005, 0.03, 0.005, 0.03,
        ]"""
inj_rates = [
    0.025,
    0.03,
    0.03,
    0.005,
    0.005,
    0.03,
    0.025,
    0.020,
    0.03,
    0.005,
    0.03,
    0.005,
    0.03,
    0.03,
    0.03,
    0.03,
    0.04,
    0.03,
    0.005,
    0.005,
    0.005,
    0.005,
    0.03,
    0.03,
    0.005,
    0.03,
    0.005,
    0.005,
    0.005,
    0.005,
    0.03,
    0.005,
    0.005,
    0.03,
    0.005,
    0.005,
    0.005,
    0.005,
    0.025,
    0.005,
    0.03,
    0.005,
    0.005,
    0.005,
    0.005,
    0.005,
    0.04,
    0.02,
    0.03,
    0.005,
    0.025,
    0.005,
    0.04,
    0.03,
    0.005,
    0.03,
    0.025,
    0.03,
    0.025,
    0.001,
    0.005,
    0.03,
    0.005,
    0.04,
]
import random

random.seed(23)
x_n = [random.random() for i in range(args.num_cpus)]

# temporal_traffic=['liebovitch,'+str(random.random())+','+'1.5'+\
# ','+'1.0125'+','+'0.25'+','+'0.5' for i in range(args.num_cpus)]

# temporal_traffic=["liebovitch,"+str(random.random())+","+"1.5"+\
# ","+"1.0125"+","+"0.25"+","+"0.5"+"," for i in range(args.num_cpus)]

# temporal_traffic=["intermittency_map,"+str((random.random()/2)+0.07)+\
# ","+"0.0001"+","+"0.7"+","+"2"+"," for i in range(args.num_cpus)]
# high injection

# temporal_traffic=["intermittency_map,"+str((random.random()/2)+0.001)+\
# ","+"0.0001"+","+"0.9"+","+"2"+"," for i in range(args.num_cpus)]

# temporal_traffic=["bernoulli_shift"+","+str(x_n[i])+\
# ","+str(inj_rates[i]) for i in range(args.num_cpus)]
# inj_rates=[0.001 if(i%2==0) else 0.03 for i in range(args.num_cpus)]

# temporal_traffic=["intermittency_map,"+str((random.random()/2)+0.09)+\
# ","+"0.0001"+","+"0.9"+","+"2"+"," for i in range(args.num_cpus)]
# high injection
temporal_traffic = [
    "intermittency_map,"
    + str((random.random() / 2) + 0.001)
    + ","
    + "0.00001"
    + ","
    + "0.9"
    + ","
    + "3"
    + ","
    for i in range(args.num_cpus)
]
# temporal_traffic=["intermittency_map,"+str((random.random()/2)+0.07)+\
# ","+"0.00001"+","+"0.9"+","+"3"+"," for i in range(args.num_cpus)]
"""
cpus = [
#    MyGarnetSyntheticTraffic(
    #GarnetSyntheticTraffic(
    #GarnetSyntheticTrafficV2(
    #GarnetSyntheticTrafficV4(temp_traffic_type=args.temporal_traffic,
    #GarnetSyntheticTrafficV4(temp_traffic_args=args.temporal_traffic,
    GarnetSyntheticTrafficV4(
    temp_traffic_args=temporal_traffic[args.num_cpus-1-i],
        num_packets_max=args.num_packets_max,
        single_sender=args.single_sender_id,
        single_dest=args.single_dest_id,
        sim_cycles=args.sim_cycles,
        traffic_type=args.synthetic,
        #inj_rate=args.injectionrate,
        inj_rate=inj_rates[args.num_cpus-1-i],
        inj_vnet=args.inj_vnet,
        precision=args.precision,
        num_dest=args.num_dirs,
        response_limit=args.response_limit,
    )
    for i in range(args.num_cpus)
]
"""
FS_var = True


# cpus=[BaseMinorCPU() for i in range(args.num_cpus)]
# cpus=[BaseTimingSimpleCPU() for i in range(args.num_cpus)]
# cpus=[X86CPU() for i in range(args.num_cpus)]

# buildEnv["PROTOCOL"]='MESI_Two_Level';

# buildEnv["PROTOCOL"]='MESI_Three_Level';  args.num_clusters=1;
# args.l0i_size='32KiB'; args.l0d_size='32KiB'; args.l0i_assoc=4;
# args.l0d_assoc=4;
args.network = "garnet"
# args.ruby=True;
args.num_cpus = 20
# args.num_dirs=2;    args.topology='Mesh_XY';
# args.num_dirs=4;    args.topology='MeshDirCorners_XY';
args.num_dirs = 4
args.topology = "Mesh_XY"
args.mesh_rows = 4
args.num_l2caches = 20

if args.fast_boot:
    args.ruby = False
    system = System(
        mem_ranges=[AddrRange(args.mem_size)], mem_mode="atomic_noncaching"
    )
    system.kvm_vm = KvmVM()
    cpus = [X86KvmCPU(cpu_id=i) for i in range(args.num_cpus)]
    print(f"Inside args.fast_boot\nargs:{args}")
elif args.restore_from:
    args.ruby = True
    system = System(mem_ranges=[AddrRange(args.mem_size)], mem_mode="timing")
    cpus = [X86O3CPU(cpu_id=i) for i in range(args.num_cpus)]
    print(f"Inside args.restore_from\nargs:{args}")
system.cpu = cpus


# cpus=[TimingSimpleCPU() for i in range(args.num_cpus)]
# cpus=[X86TimingSimpleCPU() for i in range(args.num_cpus)]
# cpus=[X86MinorCPU() for i in range(args.num_cpus)]
# cpus=[X86O3CPU() for i in range(args.num_cpus)]
# cpus=[X86KvmCPU() for i in range(args.num_cpus)]

print(f"cpus:\n{cpus}")
print(f"buildEnv[PROTOCOL]:{buildEnv['PROTOCOL']}")
print(f"----buildEnv----:{buildEnv}")
print("----args.num_dirs:", args.num_dirs)
print(f"----args----:{args}")

# create the desired simulated system
# system = System(cpu=cpus, mem_ranges=[AddrRange(args.mem_size)])


# Create a top-level voltage domain and clock domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)
print(f"----system.memories----\n{system.memories}")
print(f"----system.system_port----\n{system.system_port}")
for mem in system.memories:
    print(f"----mem----\n{mem}")
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)

for i, cpu in enumerate(system.cpu):
    cpu.clk_domain = system.clk_domain
    cpu.createThreads()
    cpu.createInterruptController()
    if args.fast_boot:
        for obj in cpu.descendants():
            obj.eventq_index = 0
        cpu.eventq_index = 0

"""
for i,cpu in enumerate(system.cpu):
    if(cpu.type[-6:]=="KvmCPU"):
        for obj in cpu.descendants():
            obj.eventq_index=0
        cpu.eventq_index = i+1
        system.kvm_vm = KvmVM()
        print("inside")
    print(f"cpu.type:{cpu.type}")
"""
# Ruby.create_system(args, False, system)
# Ruby.create_system(args, FS_var, system)

if FS_var:
    from m5.objects import Pc

    system.pc = Pc()
    print(f"type(system.workload):{type(system.workload)}")
    system.workload = X86FsLinux()
    print(f"type(system.workload):{type(system.workload)}")
    system.m5ops_base = 0xFFFF0000
    system.iobus = IOXBar()

    # Set up all of the I/O.
    # this is mostly copy paste form x86board.py

    # Constants similar to x86_traits.hh
    IO_address_space_base = 0x8000000000000000
    pci_config_address_space_base = 0xC000000000000000
    interrupts_address_space_base = 0xA000000000000000
    APIC_range_size = 1 << 12

    # system.pc.attachIO(system.iobus, [system.pc.south_bridge.ide.dma])

    # print(f"system:\n{system.ruby.type}")
    # if system.ruby.type=="RubySystem":
    # if args.ruby:
    if args.ruby and not args.fast_boot:
        # system.pc.attachIO(system.get_io_bus(), \
        # [system.pc.south_bridge.ide.dma])
        # Ruby.create_system(args, FS_var, system)
        Ruby.create_system(
            args,
            FS_var,
            system,
            system.iobus,
            [system.pc.south_bridge.ide.dma],
        )  # ,
        #    system.iobus.mem_side_ports])
        system.pc.attachIO(system.iobus, [system.pc.south_bridge.ide.dma])
        # Create a seperate clock domain for Ruby
        system.ruby.clk_domain = SrcClockDomain(
            clock=args.ruby_clock, voltage_domain=system.voltage_domain
        )
        # system.ruby.connectIOPorts(system.pc.south_bridge.ide.dma)
        # system.ruby.connectIOPorts(system.iobus)
        # print("Ruby system ports:", [port.name() \
        # for port in system.ruby._cpu_ports])
        # print("Ruby DMA ports:", [port.name() \
        # for port in system.ruby._dma_ports])
        # system.ruby._dma_ports.append(system.pc.south_bridge.ide.dma)
        # system.ruby._dma_ports = [system.pc.south_bridge.ide.dma]

    else:
        system.membus = SystemXBar(width=64)

        system.bridge = Bridge(delay="50ns")
        system.bridge.mem_side_port = system.iobus.cpu_side_ports
        system.bridge.cpu_side_port = system.membus.mem_side_ports
        # Constants similar to x86_traits.hh
        IO_address_space_base = 0x8000000000000000
        pci_config_address_space_base = 0xC000000000000000
        interrupts_address_space_base = 0xA000000000000000
        APIC_range_size = 1 << 12

        system.bridge.ranges = [
            AddrRange(0xC0000000, 0xFFFF0000),
            AddrRange(
                IO_address_space_base, interrupts_address_space_base - 1
            ),
            AddrRange(pci_config_address_space_base, Addr.max),
        ]

        system.apicbridge = Bridge(delay="50ns")
        system.apicbridge.cpu_side_port = system.iobus.mem_side_ports
        system.apicbridge.mem_side_port = (
            # system.membus.mem_side_ports
            system.membus.cpu_side_ports
        )
        system.apicbridge.ranges = [
            AddrRange(
                interrupts_address_space_base,
                interrupts_address_space_base
                + len(system.cpu) * APIC_range_size
                - 1,
            )
        ]
        system.pc.attachIO(system.iobus)

    # Add in a bios information structure
    system.workload.smbios_table.structures = [X86SMBiosBiosInformation()]

    # Set up the Intel MP table
    base_entries = []
    ext_entries = []
    # Updated the X86 board with MADT entries
    madt_entries = []
    for i in range(len(system.cpu)):
        bp = X86IntelMPProcessor(
            local_apic_id=i,
            local_apic_version=0x14,
            enable=True,
            bootstrap=(i == 0),
        )
        base_entries.append(bp)
        lapic = X86ACPIMadtLAPIC(acpi_processor_id=i, apic_id=i, flags=1)
        madt_entries.append(lapic)

    io_apic = X86IntelMPIOAPIC(
        id=len(system.cpu),
        version=0x11,
        enable=True,
        address=0xFEC00000,
    )

    system.pc.south_bridge.io_apic.apic_id = io_apic.id
    base_entries.append(io_apic)
    madt_entries.append(
        X86ACPIMadtIOAPIC(id=io_apic.id, address=io_apic.address, int_base=0)
    )

    pci_bus = X86IntelMPBus(bus_id=0, bus_type="PCI   ")
    base_entries.append(pci_bus)
    isa_bus = X86IntelMPBus(bus_id=1, bus_type="ISA   ")
    base_entries.append(isa_bus)
    connect_busses = X86IntelMPBusHierarchy(
        bus_id=1, subtractive_decode=True, parent_bus=0
    )
    ext_entries.append(connect_busses)

    pci_dev4_inta = X86IntelMPIOIntAssignment(
        interrupt_type="INT",
        polarity="ConformPolarity",
        trigger="ConformTrigger",
        source_bus_id=0,
        source_bus_irq=0 + (4 << 2),
        dest_io_apic_id=io_apic.id,
        dest_io_apic_intin=16,
    )
    base_entries.append(pci_dev4_inta)
    pci_dev4_inta_madt = X86ACPIMadtIntSourceOverride(
        bus_source=pci_dev4_inta.source_bus_id,
        irq_source=pci_dev4_inta.source_bus_irq,
        sys_int=pci_dev4_inta.dest_io_apic_intin,
        flags=0,
    )
    madt_entries.append(pci_dev4_inta_madt)

    def assignISAInt(irq, apicPin):
        assign_8259_to_apic = X86IntelMPIOIntAssignment(
            interrupt_type="ExtInt",
            polarity="ConformPolarity",
            trigger="ConformTrigger",
            source_bus_id=1,
            source_bus_irq=irq,
            dest_io_apic_id=io_apic.id,
            dest_io_apic_intin=0,
        )
        base_entries.append(assign_8259_to_apic)

        assign_to_apic = X86IntelMPIOIntAssignment(
            interrupt_type="INT",
            polarity="ConformPolarity",
            trigger="ConformTrigger",
            source_bus_id=1,
            source_bus_irq=irq,
            dest_io_apic_id=io_apic.id,
            dest_io_apic_intin=apicPin,
        )
        base_entries.append(assign_to_apic)
        # acpi
        assign_to_apic_acpi = X86ACPIMadtIntSourceOverride(
            bus_source=1, irq_source=irq, sys_int=apicPin, flags=0
        )
        madt_entries.append(assign_to_apic_acpi)

    assignISAInt(0, 2)
    assignISAInt(1, 1)

    for i in range(3, 15):
        assignISAInt(i, i)

    system.workload.intel_mp_table.base_entries = base_entries
    system.workload.intel_mp_table.ext_entries = ext_entries

    madt = X86ACPIMadt(
        local_apic_address=0, records=madt_entries, oem_id="madt"
    )

    system.workload.acpi_description_table_pointer.rsdt.entries.append(madt)
    system.workload.acpi_description_table_pointer.xsdt.entries.append(madt)
    system.workload.acpi_description_table_pointer.oem_id = "gem5"
    system.workload.acpi_description_table_pointer.rsdt.oem_id = "gem5"
    system.workload.acpi_description_table_pointer.xsdt.oem_id = "gem5"

    entries = [
        # Mark the first megabyte of memory as reserved
        X86E820Entry(addr=0, size="639KiB", range_type=1),
        X86E820Entry(addr=0x9FC00, size="385KiB", range_type=2),
        # Mark the rest of physical memory as available
        X86E820Entry(
            addr=0x100000,
            size=f"{system.mem_ranges[0].size() - 0x100000:d}B",
            range_type=1,
        ),
    ]

    # Reserve the last 16KiB of the 32-bit address space for m5ops
    entries.append(X86E820Entry(addr=0xFFFF0000, size="64KiB", range_type=2))

    system.workload.e820_table.entries = entries


"""
i = 0
for ruby_port in system.ruby._cpu_ports:
    #
    # Tie the cpu test ports to the ruby cpu port
    #
    cpus[i].test = ruby_port.in_ports
    i += 1
"""
if args.ruby and not args.fast_boot:
    for i, cpu in enumerate(cpus):
        # cpu.clk_domain=system.clk_domain
        # cpu.createThreads()
        ruby_port = system.ruby._cpu_ports[i]
        # cpu.createInterruptController()
        ruby_port.connectCpuPorts(cpu)

binary = [
    "/home/docker_share/gem5_mar15/materials/02-using-gem5/\
            03-running-in-gem5/02-annotate-this-x86"
    for i in range(len(cpus))
]
arguments = [[i] for i in range(len(cpus))]
# these are arguments to processes
# pass the kernel to be used
from gem5.resources.resource import (
    DiskImageResource,
    KernelResource,
)

# system.workload.object_file=KernelResource('/home/docker_share/\
# gem5_mar20/materials/02-using-gem5/07-full-system/\
# fs-granular/disk-image/vmlinux-x86-ubuntu')
kernel_used = KernelResource(
    "/home/docker_share/gem5_mar20/materials/02-using-gem5/\
            07-full-system/fs-granular/disk-image/vmlinux-x86-ubuntu"
)
system.workload.object_file = kernel_used.get_local_path()
disk_image = DiskImageResource(
    "/home/docker_share/gem5_mar20/materials/02-using-gem5/\
            07-full-system/fs-granular/\
            disk-image-ubuntu-24-04-borrowed/x86-ubuntu-24-04-gapbs"
)
disk_device = "/dev/hda"
readfile_script = "echo 'hello'"
kernel_args = [
    "earlyprintk=ttyS0",
    "console=ttyS0",
    "lpj=7999923",
    "root=/dev/sda2",
]
root_partition = (
    disk_image.get_root_partition()
    if None != disk_image.get_root_partition()
    else ""
)
print(f"root_partition:{root_partition}")
system.workload.command_line = (" ".join(kernel_args)).format(
    root_value=(disk_device) + (root_partition),
    disk_device=disk_device,
)
system.readfile = readfile_script
ide_disk = IdeDisk()
ide_disk.driveID = "device0"
ide_disk.image = CowDiskImage(
    child=RawDiskImage(read_only=True), read_only=False
)
ide_disk.image.child.image_file = disk_image.get_local_path()
system.pc.south_bridge.ide.disks = [ide_disk]
system.exit_on_work_items = True
print(f"type(system.pc):{type(system.pc)}")
# system.readfile=args.script
"""
# set the workload
from m5.objects import Process
process=[Process(pid=100+i) for i in range(len(binary))]
for bin_file,proc,args_var in zip(binary,process,arguments):
    binary_path=bin_file;   #binary_path=bin_file.get_local_path()
    system.workload=SEWorkload.init_compatible(binary_path)
    proc.executable=binary_path
    proc.cmd=[binary_path]+args_var

for i,cpu in enumerate(cpus):
    #cpu.set_workload(process[i])
    cpu.workload=process[i]
"""

# -----------------------
# run simulation
# -----------------------

# root = Root(full_system=False, system=system)
root = Root(full_system=FS_var, system=system)
# root.system.mem_mode = "timing"
"""
for cpu in system.cpu:
    if cpu.type[-6:]=="KvmCPU":
        root.sim_quantum=int(1e9)
"""
# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency("1ps")
print("\n----numa_high_bit----\n")
print(args.numa_high_bit)

# instantiate configuration
# m5.instantiate()

if args.restore_from:
    m5.instantiate(args.restore_from)
else:
    m5.instantiate()
exit(7)
print("\n----\n")
# print(buildEnv["PROTOCOL"])
print(f"----buildEnv----\n{buildEnv}")
print(f"----args----\n{args}")


# simulate until program terminates
# print(f"args.abs_max_tick:{args.abs_max_tick}")
# exit_event = m5.simulate(args.abs_max_tick)
# exit_event = m5.simulate(2000)
# print("Exiting @ tick", m5.curTick(), \
# "because", exit_event.getCause())

if args.fast_boot:
    exit_event = m5.simulate(20000)
    m5.checkpoint("m5out/kvm_boot_checkpoint")
    exit(0)
else:
    exit_event = m5.simulate(20000)
    exit(0)


# command used to execute
# ./build/ALL/gem5.fast configs/example/\
# my_garnet_synthetic_traffic_v5.py --protocol MESI_Two_Level
