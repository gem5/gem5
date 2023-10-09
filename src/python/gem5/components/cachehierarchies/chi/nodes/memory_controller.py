# Copyright (c) 2021 The Regents of the University of California
# All Rights Reserved.
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
from typing import List

from m5.objects import AddrRange
from m5.objects import Memory_Controller
from m5.objects import MessageBuffer
from m5.objects import Port
from m5.objects import RubyNetwork

from .abstract_node import TriggerMessageBuffer


class MemCtrlMessageBuffer(MessageBuffer):
    """
    MessageBuffer exchanging messages with the memory
    These buffers should also not be affected by the Ruby tester randomization.
    """

    randomization = "disabled"
    ordered = True


class MemoryController(Memory_Controller):
    """A controller that connects to memory"""

    _version = 0

    @classmethod
    def versionCount(cls):
        cls._version += 1  # Use count for this particular type
        return cls._version - 1

    def __init__(
        self, network: RubyNetwork, ranges: List[AddrRange], port: Port
    ):
        super().__init__()

        self.version = self.versionCount()

        self.addr_ranges = ranges
        self.memory_out_port = port
        self.data_channel_size = 32

        self.connectQueues(network)

    def connectQueues(self, network):
        self.triggerQueue = TriggerMessageBuffer()
        self.responseFromMemory = MemCtrlMessageBuffer()
        self.requestToMemory = MemCtrlMessageBuffer()
        self.reqRdy = TriggerMessageBuffer()

        # The Memory_Controller implementation deallocates the TBE for
        # write requests when they are queue up to memory. The size of this
        # buffer must be limited to prevent unlimited outstanding writes.
        self.requestToMemory.buffer_size = (
            int(self.to_memory_controller_latency) + 1
        )

        self.reqOut = MessageBuffer()
        self.rspOut = MessageBuffer()
        self.snpOut = MessageBuffer()
        self.datOut = MessageBuffer()
        self.reqIn = MessageBuffer()
        self.rspIn = MessageBuffer()
        self.snpIn = MessageBuffer()
        self.datIn = MessageBuffer()
        self.reqOut.out_port = network.in_port
        self.rspOut.out_port = network.in_port
        self.snpOut.out_port = network.in_port
        self.datOut.out_port = network.in_port
        self.reqIn.in_port = network.out_port
        self.rspIn.in_port = network.out_port
        self.snpIn.in_port = network.out_port
        self.datIn.in_port = network.out_port
