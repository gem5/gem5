# Copyright (c) 2016-2017 ARM Limited
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
# Authors: Gabor Dozsa
#          Andreas Sandberg

# This is an example configuration script for full system simulation of
# a generic ARM bigLITTLE system.


from __future__ import print_function

import argparse
import os
import sys
import m5
import m5.util
from m5.objects import *

m5.util.addToPath("../../")

from common import SysPaths
from common import CpuConfig
from common.cores.arm import ex5_big, ex5_LITTLE

import devices
from devices import AtomicCluster, KvmCluster


default_kernel = 'vmlinux4.3.aarch64'
default_disk = 'aarch64-ubuntu-trusty-headless.img'
default_rcs = 'bootscript.rcS'

default_mem_size= "2GB"

def _to_ticks(value):
    """Helper function to convert a latency from string format to Ticks"""

    return m5.ticks.fromSeconds(m5.util.convert.anyToLatency(value))

def _using_pdes(root):
    """Determine if the simulator is using multiple parallel event queues"""

    for obj in root.descendants():
        if not m5.proxy.isproxy(obj.eventq_index) and \
               obj.eventq_index != root.eventq_index:
            return True

    return False


class BigCluster(devices.CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock,
                 cpu_voltage="1.0V"):
        cpu_config = [ CpuConfig.get("O3_ARM_v7a_3"), devices.L1I, devices.L1D,
                    devices.WalkCache, devices.L2 ]
        super(BigCluster, self).__init__(system, num_cpus, cpu_clock,
                                         cpu_voltage, *cpu_config)

class LittleCluster(devices.CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock,
                 cpu_voltage="1.0V"):
        cpu_config = [ CpuConfig.get("MinorCPU"), devices.L1I, devices.L1D,
                       devices.WalkCache, devices.L2 ]
        super(LittleCluster, self).__init__(system, num_cpus, cpu_clock,
                                         cpu_voltage, *cpu_config)

class Ex5BigCluster(devices.CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock,
                 cpu_voltage="1.0V"):
        cpu_config = [ CpuConfig.get("ex5_big"), ex5_big.L1I, ex5_big.L1D,
                    ex5_big.WalkCache, ex5_big.L2 ]
        super(Ex5BigCluster, self).__init__(system, num_cpus, cpu_clock,
                                         cpu_voltage, *cpu_config)

class Ex5LittleCluster(devices.CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock,
                 cpu_voltage="1.0V"):
        cpu_config = [ CpuConfig.get("ex5_LITTLE"), ex5_LITTLE.L1I,
                    ex5_LITTLE.L1D, ex5_LITTLE.WalkCache, ex5_LITTLE.L2 ]
        super(Ex5LittleCluster, self).__init__(system, num_cpus, cpu_clock,
                                         cpu_voltage, *cpu_config)

def createSystem(caches, kernel, bootscript, disks=[]):
    sys = devices.SimpleSystem(caches, default_mem_size,
                               kernel=SysPaths.binary(kernel),
                               readfile=bootscript)

    sys.mem_ctrls = [ SimpleMemory(range=r, port=sys.membus.master)
                      for r in sys.mem_ranges ]

    sys.connect()

    # Attach disk images
    if disks:
        def cow_disk(image_file):
            image = CowDiskImage()
            image.child.image_file = SysPaths.disk(image_file)
            return image

        sys.disk_images = [ cow_disk(f) for f in disks ]
        sys.pci_vio_block = [ PciVirtIO(vio=VirtIOBlock(image=img))
                              for img in sys.disk_images ]
        for dev in sys.pci_vio_block:
            sys.attach_pci(dev)

    sys.realview.setupBootLoader(sys.membus, sys, SysPaths.binary)

    return sys

cpu_types = {
    "atomic" : (AtomicCluster, AtomicCluster),
    "timing" : (BigCluster, LittleCluster),
    "exynos" : (Ex5BigCluster, Ex5LittleCluster),
}

# Only add the KVM CPU if it has been compiled into gem5
if devices.have_kvm:
    cpu_types["kvm"] = (KvmCluster, KvmCluster)


def addOptions(parser):
    parser.add_argument("--restore-from", type=str, default=None,
                        help="Restore from checkpoint")
    parser.add_argument("--dtb", type=str, default=None,
                        help="DTB file to load")
    parser.add_argument("--kernel", type=str, default=default_kernel,
                        help="Linux kernel")
    parser.add_argument("--disk", action="append", type=str, default=[],
                        help="Disks to instantiate")
    parser.add_argument("--bootscript", type=str, default=default_rcs,
                        help="Linux bootscript")
    parser.add_argument("--cpu-type", type=str, choices=cpu_types.keys(),
                        default="timing",
                        help="CPU simulation mode. Default: %(default)s")
    parser.add_argument("--kernel-init", type=str, default="/sbin/init",
                        help="Override init")
    parser.add_argument("--big-cpus", type=int, default=1,
                        help="Number of big CPUs to instantiate")
    parser.add_argument("--little-cpus", type=int, default=1,
                        help="Number of little CPUs to instantiate")
    parser.add_argument("--caches", action="store_true", default=False,
                        help="Instantiate caches")
    parser.add_argument("--last-cache-level", type=int, default=2,
                        help="Last level of caches (e.g. 3 for L3)")
    parser.add_argument("--big-cpu-clock", type=str, default="2GHz",
                        help="Big CPU clock frequency")
    parser.add_argument("--little-cpu-clock", type=str, default="1GHz",
                        help="Little CPU clock frequency")
    parser.add_argument("--sim-quantum", type=str, default="1ms",
                        help="Simulation quantum for parallel simulation. " \
                        "Default: %(default)s")
    return parser

def build(options):
    m5.ticks.fixGlobalFrequency()

    kernel_cmd = [
        "earlyprintk=pl011,0x1c090000",
        "console=ttyAMA0",
        "lpj=19988480",
        "norandmaps",
        "loglevel=8",
        "mem=%s" % default_mem_size,
        "root=/dev/vda1",
        "rw",
        "init=%s" % options.kernel_init,
        "vmalloc=768MB",
    ]

    root = Root(full_system=True)

    disks = [default_disk] if len(options.disk) == 0 else options.disk
    system = createSystem(options.caches,
                          options.kernel,
                          options.bootscript,
                          disks=disks)

    root.system = system
    system.boot_osflags = " ".join(kernel_cmd)

    if options.big_cpus + options.little_cpus == 0:
        m5.util.panic("Empty CPU clusters")

    big_model, little_model = cpu_types[options.cpu_type]

    all_cpus = []
    # big cluster
    if options.big_cpus > 0:
        system.bigCluster = big_model(system, options.big_cpus,
                                      options.big_cpu_clock)
        system.mem_mode = system.bigCluster.memoryMode()
        all_cpus += system.bigCluster.cpus

    # little cluster
    if options.little_cpus > 0:
        system.littleCluster = little_model(system, options.little_cpus,
                                            options.little_cpu_clock)
        system.mem_mode = system.littleCluster.memoryMode()
        all_cpus += system.littleCluster.cpus

    # Figure out the memory mode
    if options.big_cpus > 0 and options.little_cpus > 0 and \
       system.littleCluster.memoryMode() != system.littleCluster.memoryMode():
        m5.util.panic("Memory mode missmatch among CPU clusters")


    # create caches
    system.addCaches(options.caches, options.last_cache_level)
    if not options.caches:
        if options.big_cpus > 0 and system.bigCluster.requireCaches():
            m5.util.panic("Big CPU model requires caches")
        if options.little_cpus > 0 and system.littleCluster.requireCaches():
            m5.util.panic("Little CPU model requires caches")

    # Create a KVM VM and do KVM-specific configuration
    if issubclass(big_model, KvmCluster):
        _build_kvm(system, all_cpus)

    # Linux device tree
    if options.dtb is not None:
        system.dtb_filename = SysPaths.binary(options.dtb)
    else:
        def create_dtb_for_system(system, filename):
            state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
            rootNode = system.generateDeviceTree(state)

            fdt = Fdt()
            fdt.add_rootnode(rootNode)
            dtb_filename = os.path.join(m5.options.outdir, filename)
            return fdt.writeDtbFile(dtb_filename)

        system.dtb_filename = create_dtb_for_system(system, 'system.dtb')

    return root

def _build_kvm(system, cpus):
    system.kvm_vm = KvmVM()

    # Assign KVM CPUs to their own event queues / threads. This
    # has to be done after creating caches and other child objects
    # since these mustn't inherit the CPU event queue.
    if len(cpus) > 1:
        device_eq = 0
        first_cpu_eq = 1
        for idx, cpu in enumerate(cpus):
            # Child objects usually inherit the parent's event
            # queue. Override that and use the same event queue for
            # all devices.
            for obj in cpu.descendants():
                obj.eventq_index = device_eq
            cpu.eventq_index = first_cpu_eq + idx



def instantiate(options, checkpoint_dir=None):
    # Setup the simulation quantum if we are running in PDES-mode
    # (e.g., when using KVM)
    root = Root.getInstance()
    if root and _using_pdes(root):
        m5.util.inform("Running in PDES mode with a %s simulation quantum.",
                       options.sim_quantum)
        root.sim_quantum = _to_ticks(options.sim_quantum)

    # Get and load from the chkpt or simpoint checkpoint
    if options.restore_from:
        if checkpoint_dir and not os.path.isabs(options.restore_from):
            cpt = os.path.join(checkpoint_dir, options.restore_from)
        else:
            cpt = options.restore_from

        m5.util.inform("Restoring from checkpoint %s", cpt)
        m5.instantiate(cpt)
    else:
        m5.instantiate()


def run(checkpoint_dir=m5.options.outdir):
    # start simulation (and drop checkpoints when requested)
    while True:
        event = m5.simulate()
        exit_msg = event.getCause()
        if exit_msg == "checkpoint":
            print("Dropping checkpoint at tick %d" % m5.curTick())
            cpt_dir = os.path.join(checkpoint_dir, "cpt.%d" % m5.curTick())
            m5.checkpoint(cpt_dir)
            print("Checkpoint done.")
        else:
            print(exit_msg, " @ ", m5.curTick())
            break

    sys.exit(event.getCode())


def main():
    parser = argparse.ArgumentParser(
        description="Generic ARM big.LITTLE configuration")
    addOptions(parser)
    options = parser.parse_args()
    root = build(options)
    instantiate(options)
    run()


if __name__ == "__m5_main__":
    main()
