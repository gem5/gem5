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
#
# Authors: Gabe Black

from m5.params import *
from m5.proxy import *
from Cmos import Cmos
from I82094AA import I82094AA
from I8237 import I8237
from I8254 import I8254
from I8259 import I8259
from PcSpeaker import PcSpeaker
from m5.SimObject import SimObject

def x86IOAddress(port):
    IO_address_space_base = 0x8000000000000000
    return IO_address_space_base + port;

class SouthBridge(SimObject):
    type = 'SouthBridge'
    pio_latency = Param.Latency('1ns', "Programmed IO latency in simticks")
    platform = Param.Platform(Parent.any, "Platform this device is part of")

    _pic1 = I8259(pio_addr=x86IOAddress(0x20), mode='I8259Master')
    _pic2 = I8259(pio_addr=x86IOAddress(0xA0), mode='I8259Slave')
    _cmos = Cmos(pio_addr=x86IOAddress(0x70))
    _dma1 = I8237(pio_addr=x86IOAddress(0x0))
    _pit = I8254(pio_addr=x86IOAddress(0x40))
    _speaker = PcSpeaker(pio_addr=x86IOAddress(0x61))
    _io_apic = I82094AA(pio_addr=0xFEC00000)

    pic1 = Param.I8259(_pic1, "Master PIC")
    pic2 = Param.I8259(_pic2, "Slave PIC")
    cmos = Param.Cmos(_cmos, "CMOS memory and real time clock device")
    dma1 = Param.I8237(_dma1, "The first dma controller")
    pit = Param.I8254(_pit, "Programmable interval timer")
    speaker = Param.PcSpeaker(_speaker, "PC speaker")
    io_apic = Param.I82094AA(_io_apic, "I/O APIC")

    def attachIO(self, bus):
        # Make internal connections
        self.pic1.output = self.io_apic.pin(0)
        self.pic2.output = self.pic1.pin(2)
        self.cmos.int_pin = self.pic2.pin(0)
        self.pit.int_pin = self.pic1.pin(0)
        self.speaker.i8254 = self.pit
        # Connect to the bus
        self.cmos.pio = bus.port
        self.dma1.pio = bus.port
        self.pic1.pio = bus.port
        self.pic2.pio = bus.port
        self.pit.pio = bus.port
        self.speaker.pio = bus.port
        self.io_apic.pio = bus.port
        self.io_apic.int_port = bus.port
