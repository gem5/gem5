# Copyright (c) 2016 ARM Limited
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


import argparse
import os
import sys
import m5
from m5.objects import *

m5.util.addToPath("../../")

from common import SysPaths
from common import CpuConfig

import devices


default_dtb = 'armv8_gem5_v1_big_little_2_2.dtb'
default_kernel = 'vmlinux4.3.aarch64'
default_disk = 'aarch64-ubuntu-trusty-headless.img'
default_rcs = 'bootscript.rcS'

default_mem_size= "2GB"


class BigCluster(devices.CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock,
                 cpu_voltage="1.0V"):
        cpu_config = [ CpuConfig.get("arm_detailed"), devices.L1I, devices.L1D,
                    devices.WalkCache, devices.L2 ]
        super(BigCluster, self).__init__(system, num_cpus, cpu_clock,
                                         cpu_voltage, *cpu_config)

class LittleCluster(devices.CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock,
                 cpu_voltage="1.0V"):
        cpu_config = [ CpuConfig.get("minor"), devices.L1I, devices.L1D,
                       devices.WalkCache, devices.L2 ]
        super(LittleCluster, self).__init__(system, num_cpus, cpu_clock,
                                         cpu_voltage, *cpu_config)


def createSystem(kernel, bootscript, disks=[]):
    sys = devices.SimpleSystem(kernel=SysPaths.binary(kernel),
                               readfile=bootscript,
                               machine_type="DTOnly")

    mem_region = sys.realview._mem_regions[0]
    sys.mem_ctrls = SimpleMemory(
        range=AddrRange(start=mem_region[0], size=default_mem_size))
    sys.mem_ctrls.port = sys.membus.master

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


def main():
    parser = argparse.ArgumentParser(
        description="Generic ARM big.LITTLE configuration")

    parser.add_argument("--restore-from", type=str, default=None,
                        help="Restore from checkpoint")
    parser.add_argument("--dtb", type=str, default=default_dtb,
                        help="DTB file to load")
    parser.add_argument("--kernel", type=str, default=default_kernel,
                        help="Linux kernel")
    parser.add_argument("--disk", action="append", type=str, default=[],
                        help="Disks to instantiate")
    parser.add_argument("--bootscript", type=str, default=default_rcs,
                        help="Linux bootscript")
    parser.add_argument("--atomic", action="store_true", default=False,
                        help="Use atomic CPUs")
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

    m5.ticks.fixGlobalFrequency()

    options = parser.parse_args()

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

    disks = default_disk if len(options.disk) == 0 else options.disk
    system = createSystem(options.kernel, options.bootscript, disks=disks)

    root.system = system
    system.boot_osflags = " ".join(kernel_cmd)

    AtomicCluster = devices.AtomicCluster

    if options.big_cpus + options.little_cpus == 0:
        m5.util.panic("Empty CPU clusters")

    # big cluster
    if options.big_cpus > 0:
        if options.atomic:
            system.bigCluster = AtomicCluster(system, options.big_cpus,
                                              options.big_cpu_clock)
        else:
            system.bigCluster = BigCluster(system, options.big_cpus,
                                           options.big_cpu_clock)
        mem_mode = system.bigCluster.memoryMode()
    # little cluster
    if options.little_cpus > 0:
        if options.atomic:
            system.littleCluster = AtomicCluster(system, options.little_cpus,
                                                 options.little_cpu_clock)

        else:
            system.littleCluster = LittleCluster(system, options.little_cpus,
                                                 options.little_cpu_clock)
        mem_mode = system.littleCluster.memoryMode()

    if options.big_cpus > 0 and options.little_cpus > 0:
        if system.bigCluster.memoryMode() != system.littleCluster.memoryMode():
            m5.util.panic("Memory mode missmatch among CPU clusters")
    system.mem_mode = mem_mode

    # create caches
    system.addCaches(options.caches, options.last_cache_level)
    if not options.caches:
        if options.big_cpus > 0 and system.bigCluster.requireCaches():
            m5.util.panic("Big CPU model requires caches")
        if options.little_cpus > 0 and system.littleCluster.requireCaches():
            m5.util.panic("Little CPU model requires caches")

    # Linux device tree
    system.dtb_filename = SysPaths.binary(options.dtb)

    # Get and load from the chkpt or simpoint checkpoint
    if options.restore_from is not None:
        m5.instantiate(options.restore_from)
    else:
        m5.instantiate()

    # start simulation (and drop checkpoints when requested)
    while True:
        event = m5.simulate()
        exit_msg = event.getCause()
        if exit_msg == "checkpoint":
            print "Dropping checkpoint at tick %d" % m5.curTick()
            cpt_dir = os.path.join(m5.options.outdir, "cpt.%d" % m5.curTick())
            m5.checkpoint(os.path.join(cpt_dir))
            print "Checkpoint done."
        else:
            print exit_msg, " @ ", m5.curTick()
            break

    sys.exit(event.getCode())


if __name__ == "__m5_main__":
    main()
