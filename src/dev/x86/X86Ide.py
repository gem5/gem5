# Copyright 2022 Google, Inc.
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

from m5.SimObject import SimObject
from m5.params import *
from m5.objects.Ide import IdeController
from m5.objects.IntPin import IntSourcePin
from m5.objects.PciDevice import PciLegacyIoBar


class X86IdeController(IdeController):
    type = "X86IdeController"
    cxx_header = "dev/x86/ide_ctrl.hh"
    cxx_class = "gem5::X86IdeController"

    VendorID = 0x8086
    DeviceID = 0x7111
    ProgIF = 0x80
    InterruptLine = 0xFF
    InterruptPin = 0x01

    BAR0 = PciLegacyIoBar(addr=0x1F0, size="8B")
    BAR1 = PciLegacyIoBar(addr=0x3F4, size="3B")
    BAR2 = PciLegacyIoBar(addr=0x170, size="8B")
    BAR3 = PciLegacyIoBar(addr=0x374, size="3B")

    int_primary = IntSourcePin("Interrupt for the primary channel")
    int_secondary = IntSourcePin("Interrupt for the secondary channel")
