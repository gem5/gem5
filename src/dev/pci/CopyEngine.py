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
# Authors: Ali Saidi

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *

from m5.objects.PciDevice import PciDevice

class CopyEngine(PciDevice):
    type = 'CopyEngine'
    cxx_header = "dev/pci/copy_engine.hh"
    dma = VectorMasterPort("Copy engine DMA port")
    VendorID = 0x8086
    DeviceID = 0x1a38
    Revision = 0xA2 # CM2 stepping (newest listed)
    SubsystemID = 0
    SubsystemVendorID = 0
    Status = 0x0000
    SubClassCode = 0x08
    ClassCode = 0x80
    ProgIF = 0x00
    MaximumLatency = 0x00
    MinimumGrant = 0xff
    InterruptLine = 0x20
    InterruptPin = 0x01
    BAR0Size = '1kB'

    ChanCnt = Param.UInt8(4, "Number of DMA channels that exist on device")
    XferCap = Param.MemorySize('4kB', "Number of bits of transfer size that are supported")

    latBeforeBegin = Param.Latency('20ns', "Latency after a DMA command is seen before it's proccessed")
    latAfterCompletion = Param.Latency('20ns', "Latency after a DMA command is complete before it's reported as such")


