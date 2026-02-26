import argparse
import os

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath

addToPath("../")
from common import Options
from ruby import Ruby

parser = argparse.ArgumentParser()
Options.addNoISAOptions(parser)

parser.add_argument(
    "--fast-boot", action="store_true", help="Use KVM for fast boot"
)
parser.add_argument(
    "--restore-from",
    type=str,
    default="",
    help="Checkpoint path to restore from",
)
Ruby.define_options(parser)
args = parser.parse_args()

FS_var = True  # full system
args.num_cpus = 4
args.mem_size = "2GB"
args.network = "garnet"
args.topology = "Mesh_XY"

# -----------------------------
# 1️⃣ System Setup
# -----------------------------
if args.fast_boot:
    args.ruby = False  # Ruby cannot be used with KVM
    system = System(
        mem_ranges=[AddrRange(args.mem_size)], mem_mode="atomic_noncaching"
    )
    system.kvm_vm = KvmVM()
    cpus = [X86KvmCPU(cpu_id=i) for i in range(args.num_cpus)]
elif args.restore_from:
    args.ruby = True  # restore with Ruby
    system = System(mem_ranges=[AddrRange(args.mem_size)], mem_mode="timing")
    cpus = [X86O3CPU(cpu_id=i) for i in range(args.num_cpus)]
else:
    raise Exception("Must specify --fast-boot or --restore-from")

system.cpu = cpus
system.voltage_domain = VoltageDomain(voltage="1.0V")
system.clk_domain = SrcClockDomain(
    clock="3GHz", voltage_domain=system.voltage_domain
)

for cpu in system.cpu:
    cpu.clk_domain = system.clk_domain
    cpu.createThreads()
    cpu.createInterruptController()
    if args.fast_boot:
        cpu.eventq_index = 0

# -----------------------------
# 2️⃣ Full System Setup
# -----------------------------
if FS_var:
    system.pc = Pc()
    system.workload = X86FsLinux()
    system.iobus = IOXBar()
    system.m5ops_base = 0xFFFF0000

    if args.ruby:
        # -----------------------------
        # Ruby Setup (only on restore)
        # -----------------------------
        Ruby.create_system(
            args,
            FS_var,
            system,
            system.iobus,
            [system.pc.south_bridge.ide.dma],
        )
        system.pc.attachIO(system.iobus, [system.pc.south_bridge.ide.dma])
        system.ruby.clk_domain = SrcClockDomain(
            clock=args.ruby_clock, voltage_domain=system.voltage_domain
        )

        # Connect CPUs to Ruby ports
        for i, cpu in enumerate(system.cpu):
            system.ruby._cpu_ports[i].connectCpuPorts(cpu)

    else:
        # -----------------------------
        # Normal atomic system (KVM fast boot)
        # -----------------------------
        system.membus = SystemXBar(width=64)
        # Bridge for IO devices
        system.bridge = Bridge(delay="50ns")
        system.bridge.mem_side_port = system.iobus.cpu_side_ports
        system.bridge.cpu_side_port = system.membus.mem_side_ports
        system.bridge.ranges = [
            AddrRange(0xC0000000, 0xFFFF0000),
            AddrRange(0x8000000000000000, 0xA000000000000000 - 1),
            AddrRange(0xC000000000000000, Addr.max),
        ]

        # APIC bridge
        system.apicbridge = Bridge(delay="50ns")
        system.apicbridge.cpu_side_port = system.iobus.mem_side_ports
        system.apicbridge.mem_side_port = system.membus.cpu_side_ports
        APIC_range_size = 1 << 12
        system.apicbridge.ranges = [
            AddrRange(
                0xA000000000000000,
                0xA000000000000000 + len(system.cpu) * APIC_range_size - 1,
            )
        ]

        system.pc.attachIO(system.iobus)

# -----------------------------
# 3️⃣ Kernel and Disk Setup
# -----------------------------
from gem5.resources.resource import (
    DiskImageResource,
    KernelResource,
)

kernel_used = KernelResource(
    "/home/docker_share/gem5_mar20/materials/02-using-gem5/\
    07-full-system/fs-granular/disk-image/vmlinux-x86-ubuntu"
)
system.workload.object_file = kernel_used.get_local_path()

disk_image = DiskImageResource(
    "/home/docker_share/gem5_mar20/materials/02-using-gem5/07-full-system/\
    fs-granular/disk-image-ubuntu-24-04-borrowed/x86-ubuntu-24-04-gapbs"
)
ide_disk = IdeDisk()
ide_disk.driveID = "device0"
ide_disk.image = CowDiskImage(child=RawDiskImage(read_only=True))
ide_disk.image.child.image_file = disk_image.get_local_path()
system.pc.south_bridge.ide.disks = [ide_disk]

system.workload.command_line = (
    "earlyprintk=ttyS0 console=ttyS0 lpj=7999923 root=/dev/hda2"
)
system.readfile = "echo 'hello'"
system.exit_on_work_items = True

# -----------------------------
# 4️⃣ Root & Simulation
# -----------------------------
root = Root(full_system=FS_var, system=system)
m5.ticks.setGlobalFrequency("1ps")

if args.restore_from:
    m5.instantiate(args.restore_from)
else:
    m5.instantiate()

if args.fast_boot:
    # Run a few cycles, checkpoint, then exit
    exit_event = m5.simulate(20000)
    m5.checkpoint("m5out/kvm_boot_checkpoint")
else:
    exit_event = m5.simulate(20000)

print("Simulation finished")
