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

# Get paths we might need.
# It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)

parser = argparse.ArgumentParser()
Options.addNoISAOptions(parser)

parser.add_argument(
    "--fast-boot",
    action="store_true",
    help="use Kvm for fast boot and checkpoint os state",
)

Ruby.define_options(parser)
args = parser.parse_args()

print(f"----Initial args----\n:{args}")

args.network = "garnet"
args.num_dirs = 4
args.num_cpus = 16
args.num_dirs = 4
args.topology = "Mesh_XY"
args.mesh_rows = 4
args.num_l2_caches = 16

args.ruby = False
system = System(
    mem_ranges=[AddrRange(args.mem_size)], mem_mode="atomic_noncaching"
)
print(f"system.mem_ranges:{system.mem_ranges[0]}")
cpus = [X86KvmCPU(cpu_id=i) for i in range(args.num_cpus)]
system.cpu = cpus

# Create a top-level voltage domain and clock domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)

for i, cpu in enumerate(system.cpu):
    cpu.clk_domain = system.clk_domain
    cpu.createThreads()
    cpu.createInterruptController()

if args.fast_boot:
    system.kvm_vm = KvmVM()
    for i, cpu in enumerate(system.cpu):
        for obj in cpu.descendants():
            obj.eventq_index = 0
        cpu.eventq_index = i + 1

from m5.objects import Pc

system.pc = Pc()
system.workload = X86FsLinux()
system.m5ops_base = 0xFFFF0000

system.iobus = IOXBar()
system.membus = SystemXBar(width=64)

# Constants similar to x86_traits.hh
IO_address_space_base = 0x8000000000000000
pci_config_address_space_base = 0xC000000000000000
interrupts_address_space_base = 0xA000000000000000
APIC_range_size = 1 << 12

system.bridge = Bridge(delay="50ns")
system.bridge.mem_side_port = system.iobus.cpu_side_ports
system.bridge.cpu_side_port = system.membus.mem_side_ports
"""
for cpu in system.cpu:
    for ic in cpu.interrupts:
        ic.pio = system.iobus.mem_side_ports
        ic.int_requestor = system.iobus.cpu_side_ports
        ic.int_responder = system.iobus.mem_side_ports

for cpu in system.cpu:
    # Connect CPU interrupt controller to the IO bus
    cpu.interrupts[0].pio = system.iobus.mem_side_ports
    cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    cpu.interrupts[0].int_responder = system.membus.mem_side_ports
"""
system.bridge.ranges = [
    AddrRange(0xC0000000, 0xFFFF0000),
    AddrRange(IO_address_space_base, interrupts_address_space_base - 1),
    AddrRange(pci_config_address_space_base, Addr.max),
]

system.apicbridge = Bridge(delay="50ns")
system.apicbridge.cpu_side_port = system.iobus.mem_side_ports
system.apicbridge.mem_side_port = system.membus.cpu_side_ports
system.apicbridge.ranges = [
    AddrRange(
        interrupts_address_space_base,
        interrupts_address_space_base + len(system.cpu) * APIC_range_size - 1,
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

madt = X86ACPIMadt(local_apic_address=0, records=madt_entries, oem_id="madt")

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

binary = [
    "/home/docker_share/gem5_mar15/materials/\
            02-using-gem5/03-running-in-gem5/02-annotate-this-x86"
    for i in range(len(cpus))
]
arguments = [[i] for i in range(len(cpus))]
# these are arguments to processes
# pass the kernel to be used
from gem5.resources.resource import (
    DiskImageResource,
    KernelResource,
)

kernel_abs_path = "/home/docker_share/gem5_mar20/materials/\
    02-using-gem5/07-full-system/fs-granular/disk-image/vmlinux-x86-ubuntu"
kernel_used = KernelResource(kernel_abs_path)
system.workload.object_file = kernel_used.get_local_path()
disk_image_abs_path = "/home/docker_share/gem5_mar20/materials/02-using-gem5/\
    07-full-system/fs-granular/disk-image-ubuntu-24-04-borrowed/\
    x86-ubuntu-24-04-gapbs"
disk_image = DiskImageResource(disk_image_abs_path)
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

FS_var = True
root = Root(full_system=FS_var, system=system)
root.sim_quantum(1e9)

m5.ticks.setGlobalFrequency("1ps")

for e in system.workload.e820_table.entries:
    print(f"E820: addr={e.addr}, size={e.size}, type={e.range_type}")

for cpu in system.cpu:
    print(cpu.interrupts[0].pio)
    print(cpu.interrupts[0].int_requestor)
    print(cpu.interrupts[0].int_responder)

m5.instantiate()
