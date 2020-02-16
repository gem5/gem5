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
from m5.objects.Cmos import Cmos
from m5.objects.I8042 import I8042
from m5.objects.I82094AA import I82094AA
from m5.objects.I8237 import I8237
from m5.objects.I8254 import I8254
from m5.objects.I8259 import I8259
from m5.objects.Ide import IdeController
from m5.objects.PcSpeaker import PcSpeaker
from m5.SimObject import SimObject

def x86IOAddress(port):
    IO_address_space_base = 0x8000000000000000
    return IO_address_space_base + port;

class SouthBridge(SimObject):
    type = 'SouthBridge'
    cxx_header = "dev/x86/south_bridge.hh"
    platform = Param.Platform(Parent.any, "Platform this device is part of")

    _pic1 = I8259(pio_addr=x86IOAddress(0x20), mode='I8259Master')
    _pic2 = I8259(pio_addr=x86IOAddress(0xA0), mode='I8259Slave')
    _cmos = Cmos(pio_addr=x86IOAddress(0x70))
    _dma1 = I8237(pio_addr=x86IOAddress(0x0))
    _keyboard = I8042(data_port=x86IOAddress(0x60), \
            command_port=x86IOAddress(0x64))
    _pit = I8254(pio_addr=x86IOAddress(0x40))
    _speaker = PcSpeaker(pio_addr=x86IOAddress(0x61))
    _io_apic = I82094AA(pio_addr=0xFEC00000)

    pic1 = Param.I8259(_pic1, "Master PIC")
    pic2 = Param.I8259(_pic2, "Slave PIC")
    cmos = Param.Cmos(_cmos, "CMOS memory and real time clock device")
    dma1 = Param.I8237(_dma1, "The first dma controller")
    keyboard = Param.I8042(_keyboard, "The keyboard controller")
    pit = Param.I8254(_pit, "Programmable interval timer")
    speaker = Param.PcSpeaker(_speaker, "PC speaker")
    io_apic = Param.I82094AA(_io_apic, "I/O APIC")

    # IDE controller
    ide = IdeController(disks=[], pci_func=0, pci_dev=4, pci_bus=0)
    ide.BAR0 = 0x1f0
    ide.BAR0LegacyIO = True
    ide.BAR1 = 0x3f4
    ide.BAR1Size = '3B'
    ide.BAR1LegacyIO = True
    ide.BAR2 = 0x170
    ide.BAR2LegacyIO = True
    ide.BAR3 = 0x374
    ide.BAR3Size = '3B'
    ide.BAR3LegacyIO = True
    ide.BAR4 = 1
    ide.Command = 0
    ide.ProgIF = 0x80
    ide.InterruptLine = 14
    ide.InterruptPin = 1
    ide.LegacyIOBase = x86IOAddress(0)

    def attachIO(self, bus, dma_ports):
        # Route interrupt signals
        self.pic1.output = self.io_apic.inputs[0]
        self.pic2.output = self.pic1.inputs[2]
        self.cmos.int_pin = self.pic2.inputs[0]
        self.pit.int_pin = self.pic1.inputs[0]
        self.pit.int_pin = self.io_apic.inputs[2]
        self.keyboard.keyboard_int_pin = self.io_apic.inputs[1]
        self.keyboard.mouse_int_pin = self.io_apic.inputs[12]
        # Tell the devices about each other
        self.pic1.slave = self.pic2
        self.speaker.i8254 = self.pit
        self.io_apic.external_int_pic = self.pic1
        # Connect to the bus
        self.cmos.pio = bus.master
        self.dma1.pio = bus.master
        self.ide.pio = bus.master
        if dma_ports.count(self.ide.dma) == 0:
                self.ide.dma = bus.slave
        self.keyboard.pio = bus.master
        self.pic1.pio = bus.master
        self.pic2.pio = bus.master
        self.pit.pio = bus.master
        self.speaker.pio = bus.master
        self.io_apic.pio = bus.master
        self.io_apic.int_master = bus.slave
