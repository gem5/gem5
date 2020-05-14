# Copyright (c) 2016-2017,2019-2020 ARM Limited
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
#

"""This script is the full system example script from the ARM
Research Starter Kit on System Modeling. More information can be found
at: http://www.arm.com/ResearchEnablement/SystemModeling
"""

from __future__ import print_function
from __future__ import absolute_import

import os
import m5
from m5.util import addToPath
from m5.objects import *
from m5.options import *
import argparse

m5.util.addToPath('../..')

from common import SysPaths
from common import MemConfig
from common import ObjectList
from common.cores.arm import HPI

import devices
import workloads

# Pre-defined CPU configurations. Each tuple must be ordered as : (cpu_class,
# l1_icache_class, l1_dcache_class, walk_cache_class, l2_Cache_class). Any of
# the cache class may be 'None' if the particular cache is not present.
cpu_types = {

    "atomic" : ( AtomicSimpleCPU, None, None, None, None),
    "minor" : (MinorCPU,
               devices.L1I, devices.L1D,
               devices.WalkCache,
               devices.L2),
    "hpi" : ( HPI.HPI,
              HPI.HPI_ICache, HPI.HPI_DCache,
              HPI.HPI_WalkCache,
              HPI.HPI_L2)
}

def create_cow_image(name):
    """Helper function to create a Copy-on-Write disk image"""
    image = CowDiskImage()
    image.child.image_file = name
    return image;


def create(args):
    ''' Create and configure the system object. '''

    if args.readfile and not os.path.isfile(args.readfile):
        print("Error: Bootscript %s does not exist" % args.readfile)
        sys.exit(1)

    object_file = args.kernel if args.kernel else ""

    cpu_class = cpu_types[args.cpu][0]
    mem_mode = cpu_class.memory_mode()
    # Only simulate caches when using a timing CPU (e.g., the HPI model)
    want_caches = True if mem_mode == "timing" else False

    platform = ObjectList.platform_list.get(args.machine_type)

    system = devices.simpleSystem(ArmSystem,
                                  want_caches,
                                  args.mem_size,
                                  platform=platform(),
                                  mem_mode=mem_mode,
                                  readfile=args.readfile)

    MemConfig.config_mem(args, system)

    if args.semi_enable:
        system.semihosting = ArmSemihosting(
            stdin=args.semi_stdin,
            stdout=args.semi_stdout,
            stderr=args.semi_stderr,
            files_root_dir=args.semi_path,
            cmd_line = " ".join([ object_file ] + args.args)
        )

    # Add the PCI devices we need for this system. The base system
    # doesn't have any PCI devices by default since they are assumed
    # to be added by the configurastion scripts needin them.
    pci_devices = []
    if args.disk_image:
        # Create a VirtIO block device for the system's boot
        # disk. Attach the disk image using gem5's Copy-on-Write
        # functionality to avoid writing changes to the stored copy of
        # the disk image.
        system.disk = PciVirtIO(vio=VirtIOBlock(
            image=create_cow_image(args.disk_image)))
        pci_devices.append(system.disk)

    # Attach the PCI devices to the system. The helper method in the
    # system assigns a unique PCI bus ID to each of the devices and
    # connects them to the IO bus.
    for dev in pci_devices:
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
    for cluster in system.cpu_cluster:
        system.addCaches(want_caches, last_cache_level=2)

    # Setup gem5's minimal Linux boot loader.
    system.auto_reset_addr = True

    # Using GICv3
    system.realview.gic.gicv4 = False

    system.highest_el_is_64 = True
    system.have_virtualization = True
    system.have_security = True

    workload_class = workloads.workload_list.get(args.workload)
    system.workload = workload_class(
        object_file, system)

    return system

def run(args):
    cptdir = m5.options.outdir
    if args.checkpoint:
        print("Checkpoint directory: %s" % cptdir)

    while True:
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

    parser.add_argument("--kernel", type=str,
                        default=None,
                        help="Binary to run")
    parser.add_argument("--workload", type=str,
                        default="ArmBaremetal",
                        choices=workloads.workload_list.get_names(),
                        help="Workload type")
    parser.add_argument("--disk-image", type=str,
                        default=None,
                        help="Disk to instantiate")
    parser.add_argument("--readfile", type=str, default="",
                        help = "File to return with the m5 readfile command")
    parser.add_argument("--cpu", type=str, choices=list(cpu_types.keys()),
                        default="atomic",
                        help="CPU model to use")
    parser.add_argument("--cpu-freq", type=str, default="4GHz")
    parser.add_argument("--num-cores", type=int, default=1,
                        help="Number of CPU cores")
    parser.add_argument("--machine-type", type=str,
                        choices=ObjectList.platform_list.get_names(),
                        default="VExpress_GEM5_V2",
                        help="Hardware platform class")
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
    parser.add_argument("--dtb-gen", action="store_true",
                        help="Doesn't run simulation, it generates a DTB only")
    parser.add_argument("--semi-enable", action="store_true",
                        help="Enable semihosting support")
    parser.add_argument("--semi-stdin", type=str, default="stdin",
                        help="Standard input for semihosting " \
                        "(default: gem5's stdin)")
    parser.add_argument("--semi-stdout", type=str, default="stdout",
                        help="Standard output for semihosting " \
                        "(default: gem5's stdout)")
    parser.add_argument("--semi-stderr", type=str, default="stderr",
                        help="Standard error for semihosting " \
                        "(default: gem5's stderr)")
    parser.add_argument('--semi-path', type=str,
                        default="",
                        help=('Search path for files to be loaded through '
                              'Arm Semihosting'))
    parser.add_argument("args", default=[], nargs="*",
                        help="Semihosting arguments to pass to benchmark")
    parser.add_argument("-P", "--param", action="append", default=[],
        help="Set a SimObject parameter relative to the root node. "
             "An extended Python multi range slicing syntax can be used "
             "for arrays. For example: "
             "'system.cpu[0,1,3:8:2].max_insts_all_threads = 42' "
             "sets max_insts_all_threads for cpus 0, 1, 3, 5 and 7 "
             "Direct parameters of the root object are not accessible, "
             "only parameters of its children.")

    args = parser.parse_args()

    root = Root(full_system=True)
    root.system = create(args)

    root.apply_config(args.param)

    if args.restore is not None:
        m5.instantiate(args.restore)
    else:
        m5.instantiate()

    if args.dtb_gen:
        # No run, autogenerate DTB and exit
        root.system.generateDtb(os.path.join(m5.options.outdir, 'system.dtb'))
    else:
        run(args)

if __name__ == "__m5_main__":
    main()
