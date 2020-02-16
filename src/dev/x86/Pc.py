# Copyright (c) 2008 The Regents of The University of Michigan
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

from m5.params import *
from m5.proxy import *

from m5.objects.Device import IsaFake
from m5.objects.Platform import Platform
from m5.objects.SouthBridge import SouthBridge
from m5.objects.Terminal import Terminal
from m5.objects.Uart import Uart8250
from m5.objects.PciHost import GenericPciHost

def x86IOAddress(port):
    IO_address_space_base = 0x8000000000000000
    return IO_address_space_base + port;

class PcPciHost(GenericPciHost):
    conf_base = 0xC000000000000000
    conf_size = "16MB"

    pci_pio_base = 0x8000000000000000

class Pc(Platform):
    type = 'Pc'
    cxx_header = "dev/x86/pc.hh"
    system = Param.System(Parent.any, "system")

    south_bridge = SouthBridge()
    pci_host = PcPciHost()

    # "Non-existant" ports used for timing purposes by the linux kernel
    i_dont_exist1 = IsaFake(pio_addr=x86IOAddress(0x80), pio_size=1)
    i_dont_exist2 = IsaFake(pio_addr=x86IOAddress(0xed), pio_size=1)

    # Ports behind the pci config and data regsiters. These don't do anything,
    # but the linux kernel fiddles with them anway.
    behind_pci = IsaFake(pio_addr=x86IOAddress(0xcf8), pio_size=8)

    # Serial port and terminal
    com_1 = Uart8250()
    com_1.pio_addr = x86IOAddress(0x3f8)
    com_1.device = Terminal()

    # Devices to catch access to non-existant serial ports.
    fake_com_2 = IsaFake(pio_addr=x86IOAddress(0x2f8), pio_size=8)
    fake_com_3 = IsaFake(pio_addr=x86IOAddress(0x3e8), pio_size=8)
    fake_com_4 = IsaFake(pio_addr=x86IOAddress(0x2e8), pio_size=8)

    # A device to catch accesses to the non-existant floppy controller.
    fake_floppy = IsaFake(pio_addr=x86IOAddress(0x3f2), pio_size=2)

    def attachIO(self, bus, dma_ports = []):
        self.south_bridge.attachIO(bus, dma_ports)
        self.i_dont_exist1.pio = bus.master
        self.i_dont_exist2.pio = bus.master
        self.behind_pci.pio = bus.master
        self.com_1.pio = bus.master
        self.fake_com_2.pio = bus.master
        self.fake_com_3.pio = bus.master
        self.fake_com_4.pio = bus.master
        self.fake_floppy.pio = bus.master
        self.pci_host.pio = bus.default
