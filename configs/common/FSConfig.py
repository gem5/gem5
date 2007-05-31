# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
# Authors: Kevin Lim

import m5
from m5 import makeList
from m5.objects import *
from Benchmarks import *

class CowIdeDisk(IdeDisk):
    image = CowDiskImage(child=RawDiskImage(read_only=True),
                         read_only=False)

    def childImage(self, ci):
        self.image.child.image_file = ci

class BaseTsunami(Tsunami):
    ethernet = NSGigE(configdata=NSGigEPciData(),
                      pci_bus=0, pci_dev=1, pci_func=0)
    etherint = NSGigEInt(device=Parent.ethernet)
    ide = IdeController(disks=[Parent.disk0, Parent.disk2],
                        pci_func=0, pci_dev=0, pci_bus=0)

def makeLinuxAlphaSystem(mem_mode, mdesc = None):
    self = LinuxAlphaSystem()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()
    self.iobus = Bus(bus_id=0)
    self.membus = Bus(bus_id=1)
    self.bridge = Bridge(fix_partial_write_b=True, delay='50ns', nack_delay='4ns')
    self.physmem = PhysicalMemory(range = AddrRange(mdesc.mem()))
    self.bridge.side_a = self.iobus.port
    self.bridge.side_b = self.membus.port
    self.physmem.port = self.membus.port
    self.disk0 = CowIdeDisk(driveID='master')
    self.disk2 = CowIdeDisk(driveID='master')
    self.disk0.childImage(mdesc.disk())
    self.disk2.childImage(disk('linux-bigswap2.img'))
    self.tsunami = BaseTsunami()
    self.tsunami.attachIO(self.iobus)
    self.tsunami.ide.pio = self.iobus.port
    self.tsunami.ethernet.pio = self.iobus.port
    self.simple_disk = SimpleDisk(disk=RawDiskImage(image_file = mdesc.disk(),
                                               read_only = True))
    self.intrctrl = IntrControl()
    self.mem_mode = mem_mode
    self.sim_console = SimConsole()
    self.kernel = binary('vmlinux')
    self.pal = binary('ts_osfpal')
    self.console = binary('console')
    self.boot_osflags = 'root=/dev/hda1 console=ttyS0'

    return self

def makeSparcSystem(mem_mode, mdesc = None):
    class CowMmDisk(MmDisk):
        image = CowDiskImage(child=RawDiskImage(read_only=True),
                             read_only=False)

        def childImage(self, ci):
            self.image.child.image_file = ci

    self = SparcSystem()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()
    self.iobus = Bus(bus_id=0)
    self.membus = Bus(bus_id=1)
    self.bridge = Bridge(fix_partial_write_b=True, delay='50ns', nack_delay='4ns')
    self.t1000 = T1000()
    self.t1000.attachOnChipIO(self.membus)
    self.t1000.attachIO(self.iobus)
    self.physmem = PhysicalMemory(range = AddrRange(Addr('1MB'), size = '64MB'), zero = True)
    self.physmem2 = PhysicalMemory(range = AddrRange(Addr('2GB'), size ='256MB'), zero = True)
    self.bridge.side_a = self.iobus.port
    self.bridge.side_b = self.membus.port
    self.physmem.port = self.membus.port
    self.physmem2.port = self.membus.port
    self.rom.port = self.membus.port
    self.nvram.port = self.membus.port
    self.hypervisor_desc.port = self.membus.port
    self.partition_desc.port = self.membus.port
    self.intrctrl = IntrControl()
    self.disk0 = CowMmDisk()
    self.disk0.childImage(disk('disk.s10hw2'))
    self.disk0.pio = self.iobus.port
    self.reset_bin = binary('reset_new.bin')
    self.hypervisor_bin = binary('q_new.bin')
    self.openboot_bin = binary('openboot_new.bin')
    self.nvram_bin = binary('nvram1')
    self.hypervisor_desc_bin = binary('1up-hv.bin')
    self.partition_desc_bin = binary('1up-md.bin')

    return self


def makeDualRoot(testSystem, driveSystem, dumpfile):
    self = Root()
    self.testsys = testSystem
    self.drivesys = driveSystem
    self.etherlink = EtherLink(int1 = Parent.testsys.tsunami.etherint[0],
                               int2 = Parent.drivesys.tsunami.etherint[0])
    if dumpfile:
        self.etherdump = EtherDump(file=dumpfile)
        self.etherlink.dump = Parent.etherdump

    return self
