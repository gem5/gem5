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
# Authors: Gabe Black

from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice, PioDevice, IsaFake, BadAddr
from Platform import Platform
from Terminal import Terminal
from Uart import Uart8250


class MmDisk(BasicPioDevice):
    type = 'MmDisk'
    cxx_header = "dev/sparc/mm_disk.hh"
    image = Param.DiskImage("Disk Image")
    pio_addr = 0x1F40000000

class DumbTOD(BasicPioDevice):
    type = 'DumbTOD'
    cxx_header = "dev/sparc/dtod.hh"
    time = Param.Time('01/01/2009', "System time to use ('Now' for real time)")
    pio_addr = 0xfff0c1fff8

class Iob(PioDevice):
    type = 'Iob'
    cxx_header = "dev/sparc/iob.hh"
    platform = Param.Platform(Parent.any, "Platform this device is part of.")
    pio_latency = Param.Latency('1ns', "Programed IO latency")


class T1000(Platform):
    type = 'T1000'
    cxx_header = "dev/sparc/t1000.hh"
    system = Param.System(Parent.any, "system")

    fake_clk = IsaFake(pio_addr=0x9600000000, pio_size=0x100000000)
            #warn_access="Accessing Clock Unit -- Unimplemented!")

    fake_membnks = IsaFake(pio_addr=0x9700000000, pio_size=16384,
            ret_data64=0x0000000000000000, update_data=False)
            #warn_access="Accessing Memory Banks -- Unimplemented!")

    fake_jbi = IsaFake(pio_addr=0x8000000000, pio_size=0x100000000)
            #warn_access="Accessing JBI -- Unimplemented!")

    fake_l2_1 = IsaFake(pio_addr=0xA900000000, pio_size=0x8,
            ret_data64=0x0000000000000001, update_data=True)
            #warn_access="Accessing L2 Cache Banks -- Unimplemented!")

    fake_l2_2 = IsaFake(pio_addr=0xA900000040, pio_size=0x8,
            ret_data64=0x0000000000000001, update_data=True)
            #warn_access="Accessing L2 Cache Banks -- Unimplemented!")

    fake_l2_3 = IsaFake(pio_addr=0xA900000080, pio_size=0x8,
            ret_data64=0x0000000000000001, update_data=True)
            #warn_access="Accessing L2 Cache Banks -- Unimplemented!")

    fake_l2_4 = IsaFake(pio_addr=0xA9000000C0, pio_size=0x8,
            ret_data64=0x0000000000000001, update_data=True)
            #warn_access="Accessing L2 Cache Banks -- Unimplemented!")

    fake_l2esr_1 = IsaFake(pio_addr=0xAB00000000, pio_size=0x8,
            ret_data64=0x0000000000000000, update_data=True)
            #warn_access="Accessing L2 ESR Cache Banks -- Unimplemented!")

    fake_l2esr_2 = IsaFake(pio_addr=0xAB00000040, pio_size=0x8,
            ret_data64=0x0000000000000000, update_data=True)
            #warn_access="Accessing L2 ESR Cache Banks -- Unimplemented!")

    fake_l2esr_3 = IsaFake(pio_addr=0xAB00000080, pio_size=0x8,
            ret_data64=0x0000000000000000, update_data=True)
            #warn_access="Accessing L2 ESR Cache Banks -- Unimplemented!")

    fake_l2esr_4 = IsaFake(pio_addr=0xAB000000C0, pio_size=0x8,
            ret_data64=0x0000000000000000, update_data=True)
            #warn_access="Accessing L2 ESR Cache Banks -- Unimplemented!")

    fake_ssi = IsaFake(pio_addr=0xff00000000, pio_size=0x10000000)
            #warn_access="Accessing SSI -- Unimplemented!")

    hterm = Terminal()
    hvuart = Uart8250(pio_addr=0xfff0c2c000)
    htod = DumbTOD()

    pterm = Terminal()
    puart0 = Uart8250(pio_addr=0x1f10000000)

    iob = Iob()
    # Attach I/O devices that are on chip
    def attachOnChipIO(self, bus):
        self.iob.pio = bus.master
        self.htod.pio = bus.master


    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
        self.hvuart.device = self.hterm
        self.puart0.device = self.pterm
        self.fake_clk.pio = bus.master
        self.fake_membnks.pio = bus.master
        self.fake_l2_1.pio = bus.master
        self.fake_l2_2.pio = bus.master
        self.fake_l2_3.pio = bus.master
        self.fake_l2_4.pio = bus.master
        self.fake_l2esr_1.pio = bus.master
        self.fake_l2esr_2.pio = bus.master
        self.fake_l2esr_3.pio = bus.master
        self.fake_l2esr_4.pio = bus.master
        self.fake_ssi.pio = bus.master
        self.fake_jbi.pio = bus.master
        self.puart0.pio = bus.master
        self.hvuart.pio = bus.master
