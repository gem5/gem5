# Copyright (c) 2021 ARM Limited
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
# Copyright (c) 2011 Advanced Micro Devices, Inc.
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
from m5.objects.BasicLink import BasicExtLink
from m5.objects.BasicLink import BasicIntLink
from m5.objects.MessageBuffer import MessageBuffer
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from m5.util import fatal


class SimpleExtLink(BasicExtLink):
    type = "SimpleExtLink"
    cxx_header = "mem/ruby/network/simple/SimpleLink.hh"
    cxx_class = "gem5::ruby::SimpleExtLink"


class SimpleIntLink(BasicIntLink):
    type = "SimpleIntLink"
    cxx_header = "mem/ruby/network/simple/SimpleLink.hh"
    cxx_class = "gem5::ruby::SimpleIntLink"

    # Buffers for this internal link.
    # One buffer is allocated per vnet when setup_buffers is called.
    # These are created by setup_buffers and the user should not
    # set these manually.
    buffers = VectorParam.MessageBuffer([], "Buffers for int_links")

    def setup_buffers(self, network):
        if len(self.buffers) > 0:
            fatal(
                "User should not manually set links' \
                   in_buffers or out_buffers",
            )

        # The network needs number_of_virtual_networks buffers per
        # in and out port
        buffers = []
        for i in range(int(network.number_of_virtual_networks)):
            buffers.append(MessageBuffer(ordered=True))

        # If physical_vnets_channels is set we adjust the buffer sizes and
        # the max_dequeue_rate in order to achieve the expected thoughput
        # assuming a fully pipelined link, i.e., throughput of 1 msg per cycle
        # per channel (assuming the channels width matches the protocol
        # logical message size, otherwise maximum thoughput may be smaller).
        # In MessageBuffer, an entry occupied by a dequeued message at cycle
        # X will available for enqueuing another message at cycle X+1. So
        # for a 1 cy enqueue latency, 2 entries are needed. For any latency,
        # the size should be at least latency+1.
        if len(network.physical_vnets_channels) != 0:
            assert len(network.physical_vnets_channels) == int(
                network.number_of_virtual_networks,
            )
            for i in range(int(network.number_of_virtual_networks)):
                buffers[i].buffer_size = network.physical_vnets_channels[i] * (
                    self.latency + 1
                )
                buffers[i].max_dequeue_rate = network.physical_vnets_channels[
                    i
                ]

        self.buffers = buffers
