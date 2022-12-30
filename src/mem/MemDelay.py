# Copyright (c) 2018 ARM Limited
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

from m5.params import *
from m5.objects.ClockedObject import ClockedObject


class MemDelay(ClockedObject):
    type = "MemDelay"
    cxx_header = "mem/mem_delay.hh"
    cxx_class = "gem5::MemDelay"
    abstract = True

    mem_side_port = RequestPort(
        "This port sends requests and " "receives responses"
    )
    master = DeprecatedParam(
        mem_side_port, "`master` is now called `mem_side_port`"
    )
    cpu_side_port = ResponsePort(
        "This port receives requests and " "sends responses"
    )
    slave = DeprecatedParam(
        cpu_side_port, "`slave` is now called `cpu_side_port`"
    )


class SimpleMemDelay(MemDelay):
    type = "SimpleMemDelay"
    cxx_header = "mem/mem_delay.hh"
    cxx_class = "gem5::SimpleMemDelay"

    read_req = Param.Latency("0t", "Read request delay")
    read_resp = Param.Latency("0t", "Read response delay")

    write_req = Param.Latency("0t", "Write request delay")
    write_resp = Param.Latency("0t", "Write response delay")
