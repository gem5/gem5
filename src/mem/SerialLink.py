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
# Copyright (c) 2006-2007 The Regents of The University of Michigan
# Copyright (c) 2015 The University of Bologna
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
#          Andreas Hansson
#          Erfan Azarkhish

from m5.params import *
from m5.objects.ClockedObject import ClockedObject

# SerialLink is a simple variation of the Bridge class, with the ability to
# account for the latency of packet serialization.

class SerialLink(ClockedObject):
    type = 'SerialLink'
    cxx_header = "mem/serial_link.hh"
    slave = SlavePort('Slave port')
    master = MasterPort('Master port')
    req_size = Param.Unsigned(16, "The number of requests to buffer")
    resp_size = Param.Unsigned(16, "The number of responses to buffer")
    delay = Param.Latency('0ns', "The latency of this serial_link")
    ranges = VectorParam.AddrRange([AllMemory],
                            "Address ranges to pass through the serial_link")
    # Bandwidth of the serial link is determined by the clock domain which the
    #  link belongs to and the number of lanes:
    num_lanes = Param.Unsigned(1, "Number of parallel lanes inside the serial"
        "link. (aka. lane width)")
    link_speed = Param.UInt64(1, "Gb/s Speed of each parallel lane inside the"
        "serial link. (aka. lane speed)")
