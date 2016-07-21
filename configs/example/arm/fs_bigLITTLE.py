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

m5.util.addToPath("../../common")
import SysPaths
import CpuConfig

import devices


default_dtb = 'armv8_gem5_v1_big_little_2_2.dtb'
default_kernel = 'vmlinux4.3.aarch64'
default_disk = 'aarch64-ubuntu-trusty-headless.img'
default_rcs = 'bootscript.rcS'

default_mem_size= "2GB"

def createSystem(kernel, mem_mode, bootscript, disks=[]):
    sys = devices.SimpleSystem(kernel=SysPaths.binary(kernel),
                               readfile=bootscript,
                               mem_mode=mem_mode,
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


class CpuCluster(SubSystem):
    def addCPUs(self, cpu_config, num_cpus, cpu_clock, cpu_voltage="1.0V"):
        try:
            self._cluster_id
            m5.util.panic("CpuCluster.addCPUs() must be called exactly once")
        except AttributeError:
            pass

        assert num_cpus > 0
        system = self._parent
        self._cluster_id = len(system._clusters)
        system._clusters.append(self)
        self._config = cpu_config

        self.voltage_domain = VoltageDomain(voltage=cpu_voltage)
        self.clk_domain = SrcClockDomain(clock=cpu_clock,
                                         voltage_domain=self.voltage_domain)

        cpu_class = cpu_config['cpu']
        self.cpus = [ cpu_class(cpu_id=len(system._cpus) + idx,
                                clk_domain=self.clk_domain)
                      for idx in range(num_cpus) ]

        for cpu in self.cpus:
            cpu.createThreads()
            cpu.createInterruptController()
            cpu.socket_id = self._cluster_id
            system._cpus.append(cpu)

    def createCache(self, key):
        try:
            return self._config[key]()
        except KeyError:
            return None

    def addL1(self):
        self._cluster_id
        for cpu in self.cpus:
            l1i = self.createCache('l1i')
            l1d = self.createCache('l1d')
            iwc = self.createCache('wcache')
            dwc = self.createCache('wcache')
            cpu.addPrivateSplitL1Caches(l1i, l1d, iwc, dwc)

    def addL2(self, clk_domain):
        self._cluster_id
        self.toL2Bus = L2XBar(width=64, clk_domain=clk_domain)
        #self.toL2Bus = L2XBar(width=64, clk_domain=clk_domain,
        #snoop_filter=NULL)
        self.l2 = self._config['l2']()
        for cpu in self.cpus:
            cpu.connectAllPorts(self.toL2Bus)
        self.toL2Bus.master = self.l2.cpu_side

    def connectMemSide(self, bus):
        self._cluster_id
        bus.slave
        try:
            self.l2.mem_side = bus.slave
        except AttributeError:
            for cpu in self.cpus:
                cpu.connectAllPorts(bus)


def addCaches(system, last_cache_level):
    cluster_mem_bus = system.membus
    assert last_cache_level >= 1 and last_cache_level <= 3
    for cluster in system._clusters:
        cluster.addL1()
    if last_cache_level > 1:
        for cluster in system._clusters:
            cluster.addL2(cluster.clk_domain)
    if last_cache_level > 2:
        max_clock_cluster = max(system._clusters,
                                key=lambda c: c.clk_domain.clock[0])
        system.l3 = devices.L3(clk_domain=max_clock_cluster.clk_domain)
        system.toL3Bus = L2XBar(width=64)
        system.toL3Bus.master = system.l3.cpu_side
        system.l3.mem_side = system.membus.slave
        cluster_mem_bus = system.toL3Bus

    return cluster_mem_bus


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

    if options.atomic:
        cpu_config = { 'cpu' : AtomicSimpleCPU }
        big_cpu_config, little_cpu_config = cpu_config, cpu_config
    else:
        big_cpu_config = { 'cpu' : CpuConfig.get("arm_detailed"),
                           'l1i' : devices.L1I,
                           'l1d' : devices.L1D,
                           'wcache' : devices.WalkCache,
                           'l2' : devices.L2 }
        little_cpu_config = { 'cpu' : MinorCPU,
                              'l1i' : devices.L1I,
                              'l1d' : devices.L1D,
                              'wcache' : devices.WalkCache,
                              'l2' : devices.L2 }

    big_cpu_class = big_cpu_config['cpu']
    little_cpu_class = little_cpu_config['cpu']

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

    assert big_cpu_class.memory_mode() == little_cpu_class.memory_mode()
    disks = default_disk if len(options.disk) == 0 else options.disk
    system = createSystem(options.kernel, big_cpu_class.memory_mode(),
                          options.bootscript, disks=disks)

    root.system = system
    system.boot_osflags = " ".join(kernel_cmd)

    # big cluster
    if options.big_cpus > 0:
        system.bigCluster = CpuCluster()
        system.bigCluster.addCPUs(big_cpu_config, options.big_cpus,
                                  options.big_cpu_clock)


    # LITTLE cluster
    if options.little_cpus > 0:
        system.littleCluster = CpuCluster()
        system.littleCluster.addCPUs(little_cpu_config, options.little_cpus,
                                     options.little_cpu_clock)

    # add caches
    if options.caches:
        cluster_mem_bus = addCaches(system, options.last_cache_level)
    else:
        if big_cpu_class.require_caches():
            m5.util.panic("CPU model %s requires caches" % str(big_cpu_class))
        if little_cpu_class.require_caches():
            m5.util.panic("CPU model %s requires caches" %
                          str(little_cpu_class))
        cluster_mem_bus = system.membus

    # connect each cluster to the memory hierarchy
    for cluster in system._clusters:
        cluster.connectMemSide(cluster_mem_bus)

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
