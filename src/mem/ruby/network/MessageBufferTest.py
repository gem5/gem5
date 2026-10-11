# Copyright (c) 2026 Arm Limited
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

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class MessageBufferProducerTest(ClockedObject):
    type = "MessageBufferProducerTest"
    cxx_class = "gem5::ruby::MessageBufferProducerTest"
    cxx_header = "mem/ruby/network/MessageBufferTest.hh"

    rate = Param.Int(
        1,
        "Number of messages to produce per cycle,"
        "note negative values are taken as fractional e.g. -5"
        "is 5 cycles per message",
    )

    num_msgs = Param.Int(1, "Total number of messages to produce")

    latency = Param.Cycles(1, "Cycles until message is ready after production")

    buffer = Param.MessageBuffer("")


class MessageBufferConsumerTest(ClockedObject):
    type = "MessageBufferConsumerTest"
    cxx_class = "gem5::ruby::MessageBufferConsumerTest"
    cxx_header = "mem/ruby/network/MessageBufferTest.hh"

    rate = Param.Int(1, "Maximum number of messages consumable within a cycle")

    num_msgs = Param.Int(1, "Total number of messages to consume")

    stall_prob = Param.Float(
        0, "Probability consumer cannot consume messages in a cycle"
    )

    buffer = Param.MessageBuffer("")
