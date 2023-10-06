# Copyright (c) 2005-2007 The Regents of The University of Michigan
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
from m5.objects.PciDevice import PciDevice
from m5.objects.PciDevice import PciIoBar
from m5.params import *
from m5.SimObject import SimObject


class IdeID(Enum):
    vals = ["device0", "device1"]


class IdeDisk(SimObject):
    type = "IdeDisk"
    cxx_header = "dev/storage/ide_disk.hh"
    cxx_class = "gem5::IdeDisk"
    delay = Param.Latency("1us", "Fixed disk delay in microseconds")
    driveID = Param.IdeID("device0", "Drive ID")
    image = Param.DiskImage("Disk image")


class IdeController(PciDevice):
    type = "IdeController"
    cxx_header = "dev/storage/ide_ctrl.hh"
    cxx_class = "gem5::IdeController"
    disks = VectorParam.IdeDisk("IDE disks attached to this controller")

    VendorID = 0x8086
    DeviceID = 0x7111
    Command = 0x0
    Status = 0x280
    Revision = 0x0
    ClassCode = 0x01
    SubClassCode = 0x01
    ProgIF = 0x85
    InterruptLine = 0x1F
    InterruptPin = 0x01

    # Primary
    BAR0 = PciIoBar(size="8B")
    BAR1 = PciIoBar(size="4B")
    # Secondary
    BAR2 = PciIoBar(size="8B")
    BAR3 = PciIoBar(size="4B")
    # DMA
    BAR4 = PciIoBar(size="16B")

    io_shift = Param.UInt32(0x0, "IO port shift")
    ctrl_offset = Param.UInt32(0x0, "IDE disk control offset")
