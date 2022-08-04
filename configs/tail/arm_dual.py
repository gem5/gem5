# Copyright (c) 2016-2017, 2020 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

"""This script is the full system example script from the ARM
Research Starter Kit on System Modeling. More information can be found
at: http://www.arm.com/ResearchEnablement/SystemModeling
"""

from os.path import join as joinpath

import os
import m5
from m5.util import addToPath
from m5.objects import *
from m5.options import *
import argparse

m5.util.addToPath('../')
m5.util.addToPath('../example/arm')

from common import SysPaths
from common import ObjectList
from common import MemConfig
from common.cores.arm import HPI

import devices

default_kernel = 'vmlinux.arm64'
default_disk = 'linaro-minimal-aarch64.img'
default_root_device = '/dev/vda1'


# Pre-defined CPU configurations. Each tuple must be ordered as : (cpu_class,
# l1_icache_class, l1_dcache_class, walk_cache_class, l2_Cache_class). Any of
# the cache class may be 'None' if the particular cache is not present.
cpu_types = {

    "atomic" : ( AtomicSimpleCPU, devices.L1I, devices.L1D, None),
    "minor" : (MinorCPU,
               devices.L1I, devices.L1D,
               devices.L2),
    "hpi" : ( HPI.HPI,
              HPI.HPI_ICache, HPI.HPI_DCache,
              HPI.HPI_L2)
}

def create_cow_image(name):
    """Helper function to create a Copy-on-Write disk image"""
    image = CowDiskImage()
    image.child.image_file = SysPaths.disk(name)

    return image;


def create(args):
    ''' Create and configure the system object. '''

    if args.script and not os.path.isfile(args.script):
        print("Error: Bootscript %s does not exist" % args.script)
        sys.exit(1)

    cpu_class = cpu_types[args.cpu][0]
    mem_mode = cpu_class.memory_mode()
    # Only simulate caches when using a timing CPU (e.g., the HPI model)
    want_caches = True if mem_mode == "timing" else False
    system = devices.SimpleSystem(want_caches,
                                  args.mem_size,
                                  mem_mode=mem_mode,
                                  workload=ArmFsLinux(
                                      object_file=
                                      SysPaths.binary(args.kernel)),
                                  readfile=args.script)
    MemConfig.config_mem(args, system)
    print("done MemConfig in config")
    # Add the PCI devices we need for this system. The base system
    # doesn't have any PCI devices by default since they are assumed
    # to be added by the configuration scripts needing them.
    # ethernet = IGbE_e1000(InterruptLine=1, InterruptPin=1)
    system.pci_devices = [
        # Create a VirtIO block device for the system's boot
        # disk. Attach the disk image using gem5's Copy-on-Write
        # functionality to avoid writing changes to the stored copy of
        # the disk image.
        PciVirtIO(vio=VirtIOBlock(image=create_cow_image(args.disk_image))),
    ]

    # Attach the PCI devices to the system. The helper method in the
    # system assigns a unique PCI bus ID to each of the devices and
    # connects them to the IO bus.
    for dev in system.pci_devices:
        system.attach_pci(dev)

    # Wire up the system's memory system
    system.connect()

    # Add CPU clusters to the system
    system.cpu_cluster = [
        devices.CpuCluster(system,
                           args.num_cores,
                           args.cpu_freq, "1.0V",
                           *cpu_types[args.cpu]),
    ]

    # Create a cache hierarchy for the cluster. We are assuming that
    # clusters have core-private L1 caches and an L2 that's shared
    # within the cluster.
    system.addCaches(want_caches, last_cache_level=2)

    # Setup gem5's minimal Linux boot loader.
    system.realview.setupBootLoader(system, SysPaths.binary,args.bootloader)

    if args.dtb:
        system.workload.dtb_filename = args.dtb
    else:
        # No DTB specified: autogenerate DTB
        system.workload.dtb_filename = \
            os.path.join(m5.options.outdir, 'system.dtb')
        system.generateDtb(system.workload.dtb_filename)

    # Linux boot command flags
    kernel_cmd = [
        # Tell Linux to use the simulated serial port as a console
        "console=ttyAMA0",
        # Hard-code timi
        "lpj=19988480",
        # Disable address space randomisation to get a consistent
        # memory layout.
        "norandmaps",
        # Tell Linux where to find the root disk image.
        "root=%s" % args.root_device,
        # Mount the root disk read-write by default.
        "rw",
        # Tell Linux about the amount of physical memory present.
        "mem=%s" % args.mem_size,
    ]
    system.workload.command_line = " ".join(kernel_cmd)

    return system


def scriptCheckpoints(options, maxtick, cptdir):
    when, period = options.take_checkpoints.split(",", 1)
    when = int(when)
    period = int(period)
    num_checkpoints = 0

    exit_event = m5.simulate(when - m5.curTick())
    exit_cause = exit_event.getCause()
    while exit_cause == "checkpoint":
        exit_event = m5.simulate(when - m5.curTick())
        exit_cause = exit_event.getCause()

    if exit_cause == "simulate() limit reached":
        m5.checkpoint(joinpath(cptdir, "cpt.%d"))
        num_checkpoints += 1

    sim_ticks = when
    max_checkpoints = options.max_checkpoints

    while num_checkpoints < max_checkpoints and \
            exit_cause == "simulate() limit reached":
        if (sim_ticks + period) > maxtick:
            exit_event = m5.simulate(maxtick - sim_ticks)
            exit_cause = exit_event.getCause()
            break
        else:
            exit_event = m5.simulate(period)
            exit_cause = exit_event.getCause()
            sim_ticks += period
            while exit_event.getCause() == "checkpoint":
                exit_event = m5.simulate(sim_ticks - m5.curTick())
            if exit_event.getCause() == "simulate() limit reached":
                m5.checkpoint(joinpath(cptdir, "cpt.%d"))
                num_checkpoints += 1

    return exit_event

def run(args):
    cptdir = m5.options.outdir
    if args.checkpoint:
        print("Checkpoint directory: %s" % cptdir)

    while True:
        if args.take_checkpoints != None :
            # Checkpoints being taken via the command line at <when> and at
            # subsequent periods of <period>.  Checkpoint instructions
            # received from the benchmark running are ignored and skipped in
            # favor of command line checkpoint instructions.
            event = scriptCheckpoints(args, args.maxtick, cptdir)
        else:
            event = m5.simulate()

        exit_msg = event.getCause()
        if exit_msg == "checkpoint":
            print("Dropping checkpoint at tick %d" % m5.curTick())
            cpt_dir = os.path.join(m5.options.outdir, "cpt.%d" % m5.curTick())
            m5.checkpoint(os.path.join(cpt_dir))
            print("Checkpoint done.")
        else:
            print(exit_msg, " @ ", m5.curTick())
            break

    sys.exit(event.getCode())

def main():
    parser = argparse.ArgumentParser(epilog=__doc__)

    parser.add_argument("--dtb", type=str, default=None,
                        help="DTB file to load")
    parser.add_argument("--kernel", type=str, default=default_kernel,
                        help="Linux kernel")
    parser.add_argument("--disk-image", type=str,
                        default=default_disk,
                        help="Disk to instantiate")
    parser.add_argument("--root-device", type=str,
                        default=default_root_device,
                        help="OS device name for root partition (default: {})"
                             .format(default_root_device))
    parser.add_argument("--script", type=str, default="",
                        help = "Linux bootscript")
    parser.add_argument("--cpu", type=str, choices=list(cpu_types.keys()),
                        default="atomic",
                        help="CPU model to use")
    parser.add_argument("--cpu-freq", type=str, default="4GHz")
    parser.add_argument("--num-cores", type=int, default=1,
                        help="Number of CPU cores")
    parser.add_argument("--mem-type", default="DDR3_1600_8x8",
                        choices=ObjectList.mem_list.get_names(),
                        help = "type of memory to use")
    parser.add_argument("--mem-channels", type=int, default=1,
                        help = "number of memory channels")
    parser.add_argument("--mem-ranks", type=int, default=None,
                        help = "number of memory ranks per channel")
    parser.add_argument("--mem-size", action="store", type=str,
                        default="2GB",
                        help="Specify the physical memory size")
    parser.add_argument("--checkpoint", action="store_true")
    parser.add_argument("--restore", type=str, default=None)
    parser.add_argument("--bootloader", action="append",
                        help="executable file that runs before the --kernel")
    parser.add_argument("--maxinsts", action="store", type=int,
                        default=None, help="""Total number of instructions to
                                            simulate (default: run forever)""")

    parser.add_argument("--server-script", type=str, default="",
                        help = "Linux server bootscript")
    parser.add_argument("--client-script", type=str, default="",
                        help = "Linux client bootscript")

    #  Checkpointing options
    # Note that performing checkpointing via python script files will override
    # checkpoint instructions built into binaries.
    parser.add_argument(
        "--take-checkpoints", action="store", type=str,
        help="<M,N> take checkpoints at tick M and every N ticks thereafter")
    parser.add_argument(
        "--maxtick", action="store", type=int,  default=m5.MaxTick,
        help="maxtick usedfor guide")
    parser.add_argument(
        "--max-checkpoints", action="store", type=int,  default=1,
        help="max number of checkpoint taking")

    args = parser.parse_args()
    print("before Root in config")
    root = Root(full_system=True)

    args.script = args.server_script
    root.testsys = create(args)
    print(args.script)

    # create driver node, as client, and setup link between driver and test
    args.script = args.client_script
    root.drivesys = create(args)
    root.etherlink = EtherLink()
    root.etherlink.int0 = root.testsys.realview.ethernet.interface
    root.etherlink.int1 = root.drivesys.realview.ethernet.interface

    if args.maxinsts:
        for cluster in root.testsys._clusters:
            for cpu in cluster.cpus:
                cpu.max_insts_any_thread = args.maxinsts

    if args.restore is not None:
        m5.instantiate(args.restore)
    else:
        m5.instantiate()

    run(args)


if __name__ == "__m5_main__":
    print("before main in config")
    main()
