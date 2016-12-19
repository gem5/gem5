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
#
# Authors: Nathan Binkert

from m5.params import *
from m5.proxy import *
from MemObject import MemObject

class PioDevice(MemObject):
    type = 'PioDevice'
    cxx_header = "dev/io_device.hh"
    abstract = True
    pio = SlavePort("Programmed I/O port")
    system = Param.System(Parent.any, "System this device is part of")

class BasicPioDevice(PioDevice):
    type = 'BasicPioDevice'
    cxx_header = "dev/io_device.hh"
    abstract = True
    pio_addr = Param.Addr("Device Address")
    pio_latency = Param.Latency('100ns', "Programmed IO latency")

class DmaDevice(PioDevice):
    type = 'DmaDevice'
    cxx_header = "dev/dma_device.hh"
    abstract = True
    dma = MasterPort("DMA port")


class IsaFake(BasicPioDevice):
    type = 'IsaFake'
    cxx_header = "dev/isa_fake.hh"
    pio_size = Param.Addr(0x8, "Size of address range")
    ret_data8 = Param.UInt8(0xFF, "Default data to return")
    ret_data16 = Param.UInt16(0xFFFF, "Default data to return")
    ret_data32 = Param.UInt32(0xFFFFFFFF, "Default data to return")
    ret_data64 = Param.UInt64(0xFFFFFFFFFFFFFFFF, "Default data to return")
    ret_bad_addr = Param.Bool(False, "Return pkt status bad address on access")
    update_data = Param.Bool(False, "Update the data that is returned on writes")
    warn_access = Param.String("", "String to print when device is accessed")
    fake_mem = Param.Bool(False,
      "Is this device acting like a memory and thus may get a cache line sized req")

class BadAddr(IsaFake):
    pio_addr = 0
    ret_bad_addr = Param.Bool(True, "Return pkt status bad address on access")


