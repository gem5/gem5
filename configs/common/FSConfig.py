# Copyright (c) 2006 The Regents of The University of Michigan
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
from m5.objects import *
from FullO3Config import *
from SysPaths import *
from Util import *

script.dir =  '/z/saidi/work/m5.newmem/configs/boot'
linux_image = env.get('LINUX_IMAGE', disk('linux-latest.img'))

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

def MyLinuxAlphaSystem(cpu, mem_mode, linux_image, icache=None, dcache=None, l2cache=None):
    self = LinuxAlphaSystem()
    self.iobus = Bus(bus_id=0)
    self.membus = Bus(bus_id=1)
    self.bridge = Bridge()
    self.physmem = PhysicalMemory(range = AddrRange('128MB'))
    self.bridge.side_a = self.iobus.port
    self.bridge.side_b = self.membus.port
    self.physmem.port = self.membus.port
    self.disk0 = CowIdeDisk(driveID='master')
    self.disk2 = CowIdeDisk(driveID='master')
    self.disk0.childImage(linux_image)
    self.disk2.childImage(disk('linux-bigswap2.img'))
    self.tsunami = BaseTsunami()
    self.tsunami.attachIO(self.iobus)
    self.tsunami.ide.pio = self.iobus.port
    self.tsunami.ide.dma = self.iobus.port
    self.tsunami.ide.config = self.iobus.port
    self.tsunami.ethernet.pio = self.iobus.port
    self.tsunami.ethernet.dma = self.iobus.port
    self.tsunami.ethernet.config = self.iobus.port
    self.simple_disk = SimpleDisk(disk=RawDiskImage(image_file = linux_image,
                                               read_only = True))
    self.intrctrl = IntrControl()
    self.cpu = cpu
    self.mem_mode = mem_mode
    connectCpu(self.cpu, self.membus, icache, dcache, l2cache)
    for each_cpu in listWrapper(self.cpu):
        each_cpu.itb = AlphaITB()
        each_cpu.dtb = AlphaDTB()
    self.cpu.clock = '2GHz'
    self.sim_console = SimConsole(listener=ConsoleListener(port=3456))
    self.kernel = binary('vmlinux')
    self.pal = binary('ts_osfpal')
    self.console = binary('console')
    self.boot_osflags = 'root=/dev/hda1 console=ttyS0'

    return self

class TsunamiRoot(Root):
    pass

def DualRoot(clientSystem, serverSystem):
    self = Root()
    self.client = clientSystem
    self.server = serverSystem

    self.etherdump = EtherDump(file='ethertrace')
    self.etherlink = EtherLink(int1 = Parent.client.tsunami.etherint[0],
                               int2 = Parent.server.tsunami.etherint[0],
                               dump = Parent.etherdump)
    self.clock = '1THz'
    return self
