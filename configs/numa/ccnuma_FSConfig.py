# Copyright (c) 2010-2012, 2015 ARM Limited
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
# Copyright (c) 2010-2011 Advanced Micro Devices, Inc.
# Copyright (c) 2006-2008 The Regents of The University of Michigan
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

from m5.objects import *
from Benchmarks import *
from m5.util import *

# Populate to reflect supported os types per target ISA
os_types = { 'alpha' : [ 'linux' ] }

class CowIdeDisk(IdeDisk):
    image = CowDiskImage(child=RawDiskImage(read_only=True),
                         read_only=False)

    def childImage(self, ci):
        self.image.child.image_file = ci

class MemBus(SystemXBar):
    # pass
    badaddr_responder = BadAddr()
    default = Self.badaddr_responder.pio

class CoherentSystemBus(SystemXBar):
    badaddr_responder = BadAddr()
    default = Self.badaddr_responder.pio

class NonCoherentSystemBus(IOXBar):
    badaddr_responder = BadAddr()
    default = Self.badaddr_responder.pio

def fillInCmdline(mdesc, template, **kwargs):
    kwargs.setdefault('disk', mdesc.disk())
    kwargs.setdefault('rootdev', mdesc.rootdev())
    kwargs.setdefault('mem', mdesc.mem())
    kwargs.setdefault('script', mdesc.script())
    return template % kwargs

def makeLinuxAlphaSystem(mem_mode, options, mdesc=None, cmdline=None):
    class BaseTsunami(Tsunami):
        ethernet = NSGigE(pci_bus=0, pci_dev=1, pci_func=0)
        ide = IdeController(disks=[Parent.disk0, Parent.disk2],
                            pci_func=0, pci_dev=0, pci_bus=0)

    self = LinuxAlphaSystem()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()

    self.tsunami = BaseTsunami()

    # Create the io bus to connect all device ports
    self.iobus = IOXBar()
    self.tsunami.attachIO(self.iobus)

    self.tsunami.ide.pio = self.iobus.master
    self.tsunami.ide.config = self.iobus.master

    self.tsunami.ethernet.pio = self.iobus.master
    self.tsunami.ethernet.config = self.iobus.master

    self.systembus = MemBus()
    # self.systembus = IOXBar()

    # if options.caches or options.l2cache:
    #     self.systembus = CoherentSystemBus()
    # else:
    #     self.systembus = NonCoherentSystemBus()

    # By default the bridge responds to all addresses above the I/O
    # base address (including the PCI config space)
    IO_address_space_base = 0x80000000000
    self.bridge = Bridge(delay='50ns',
                         ranges = [AddrRange(IO_address_space_base, Addr.max)])
    self.bridge.master = self.iobus.slave
    self.bridge.slave = self.systembus.master

    self.tsunami.ide.dma = self.iobus.slave
    self.tsunami.ethernet.dma = self.iobus.slave

    self.system_port = self.systembus.slave

    self.disk0 = CowIdeDisk(driveID='master')
    self.disk2 = CowIdeDisk(driveID='master')
    self.disk0.childImage(mdesc.disk())
    self.disk2.childImage(disk('linux-bigswap2.img'))
    self.simple_disk = SimpleDisk(disk=RawDiskImage(image_file = mdesc.disk(),
                                               read_only = True))
    self.intrctrl = IntrControl()
    self.mem_mode = mem_mode
    self.terminal = Terminal()
    # self.kernel = binary('vmlinux')
    self.kernel = binary('vmlinux_2.6.27-gcc_4.3.4')
    # self.pal = binary('ts_osfpal')
    self.pal = binary('tsb_osfpal')
    self.console = binary('console')
    if not cmdline:
        cmdline = 'root=/dev/hda1 console=ttyS0'
    self.boot_osflags = fillInCmdline(mdesc, cmdline)

    return self
