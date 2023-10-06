# Copyright (c) 2020-2021 ARM Limited
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
# Copyright (c) 2015 Mark D. Hill and David A. Wood.
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
from m5.SimObject import SimObject


# A MessageBuffer inserts random delays to enqueued messages when the
# randomization param is set to 'enabled' or when globally enabled for the
# RubySystem and the param is set to 'ruby_system' (default). 'disabled'
# completely prevents randomization.
class MessageRandomization(ScopedEnum):
    vals = ["disabled", "enabled", "ruby_system"]


class MessageBuffer(SimObject):
    type = "MessageBuffer"
    cxx_class = "gem5::ruby::MessageBuffer"
    cxx_header = "mem/ruby/network/MessageBuffer.hh"

    ordered = Param.Bool(False, "Whether the buffer is ordered")
    buffer_size = Param.Unsigned(
        0,
        "Maximum number of entries to buffer \
                                     (0 allows infinite entries)",
    )
    randomization = Param.MessageRandomization(
        "ruby_system",
        "Randomization parameter",
    )
    allow_zero_latency = Param.Bool(
        False,
        "Allows messages to be enqueued \
                                            with zero latency. This is useful \
                                            for internall trigger queues and \
                                            should not be used if this msg. \
                                            buffer connects different objects",
    )

    out_port = RequestPort("Request port to MessageBuffer receiver")
    master = DeprecatedParam(out_port, "`master` is now called `out_port`")
    in_port = ResponsePort("Response port from MessageBuffer sender")
    slave = DeprecatedParam(in_port, "`slave` is now called `in_port`")
    max_dequeue_rate = Param.Unsigned(
        0,
        "Maximum number of messages that can \
                                          be dequeued per cycle \
                                    (0 allows dequeueing all ready messages)",
    )
    routing_priority = Param.Int(
        0,
        "Buffer priority when messages are \
                                     consumed by the network. Smaller value \
                                     means higher priority",
    )
