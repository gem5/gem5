# Copyright (c) 2012-2013 ARM Limited
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
# Authors: Andreas Sandberg

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

from Device import PioDevice
from Platform import Platform

class BaseGic(PioDevice):
    type = 'BaseGic'
    abstract = True
    cxx_header = "dev/arm/base_gic.hh"

    platform = Param.Platform(Parent.any, "Platform this device is part of.")

class Pl390(BaseGic):
    type = 'Pl390'
    cxx_header = "dev/arm/gic_pl390.hh"

    dist_addr = Param.Addr(0x1f001000, "Address for distributor")
    cpu_addr = Param.Addr(0x1f000100, "Address for cpu")
    dist_pio_delay = Param.Latency('10ns', "Delay for PIO r/w to distributor")
    cpu_pio_delay = Param.Latency('10ns', "Delay for PIO r/w to cpu interface")
    int_latency = Param.Latency('10ns', "Delay for interrupt to get to CPU")
    it_lines = Param.UInt32(128, "Number of interrupt lines supported (max = 1020)")
    gem5_extensions = Param.Bool(False, "Enable gem5 extensions")

class Gicv2mFrame(SimObject):
    type = 'Gicv2mFrame'
    cxx_header = "dev/arm/gic_v2m.hh"
    spi_base = Param.UInt32(0x0, "Frame SPI base number");
    spi_len = Param.UInt32(0x0, "Frame SPI total number");
    addr = Param.Addr("Address for frame PIO")

class Gicv2m(PioDevice):
    type = 'Gicv2m'
    cxx_header = "dev/arm/gic_v2m.hh"

    pio_delay = Param.Latency('10ns', "Delay for PIO r/w")
    gic = Param.BaseGic(Parent.any, "Gic on which to trigger interrupts")
    frames = VectorParam.Gicv2mFrame([], "Power of two number of frames")
