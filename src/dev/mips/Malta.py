# Copyright (c) 2007 The Regents of The University of Michigan
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

from m5.objects.BadDevice import BadDevice
from m5.objects.Device import BasicPioDevice
from m5.objects.Platform import Platform
from m5.objects.Uart import Uart8250
from m5.params import *
from m5.proxy import *


class MaltaCChip(BasicPioDevice):
    type = "MaltaCChip"
    cxx_header = "dev/mips/malta_cchip.hh"
    cxx_class = "gem5::MaltaCChip"
    malta = Param.Malta(Parent.any, "Malta")


class MaltaIO(BasicPioDevice):
    type = "MaltaIO"
    cxx_header = "dev/mips/malta_io.hh"
    cxx_class = "gem5::MaltaIO"
    time = Param.Time(
        "01/01/2009",
        "System time to use (0 for actual time, default is 1/1/06)",
    )
    year_is_bcd = Param.Bool(
        False, "The RTC should interpret the year as a BCD value"
    )
    malta = Param.Malta(Parent.any, "Malta")
    frequency = Param.Frequency("1024Hz", "frequency of interrupts")


class Malta(Platform):
    type = "Malta"
    cxx_header = "dev/mips/malta.hh"
    cxx_class = "gem5::Malta"
    cchip = MaltaCChip(pio_addr=0x801A0000000)
    io = MaltaIO(pio_addr=0x801FC000000)
    uart = Uart8250(pio_addr=0xBFD003F8)

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
        self.cchip.pio = bus.mem_side_ports
        self.io.pio = bus.mem_side_ports
        self.uart.pio = bus.mem_side_ports
