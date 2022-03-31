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

from abc import abstractmethod
from gem5.isas import ISA
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.abstract_core import AbstractCore

from m5.objects import CHI_Cache_Controller, MessageBuffer, RubyNetwork

import math


class TriggerMessageBuffer(MessageBuffer):
    """
    MessageBuffer for triggering internal controller events.
    These buffers should not be affected by the Ruby tester randomization
    and allow poping messages enqueued in the same cycle.
    """

    randomization = "disabled"
    allow_zero_latency = True


class OrderedTriggerMessageBuffer(TriggerMessageBuffer):
    ordered = True


class AbstractNode(CHI_Cache_Controller):
    """A node is the abstract unit for caches in the CHI protocol.

    You can extend the AbstractNode to create caches (private or shared) and
    directories with or without data caches.
    """

    _version = 0

    @classmethod
    def versionCount(cls):
        cls._version += 1  # Use count for this particular type
        return cls._version - 1

    # TODO: I don't love that we have to pass in the cache line size.
    # However, we need some way to set the index bits
    def __init__(self, network: RubyNetwork, cache_line_size: int):
        super(AbstractNode, self).__init__()

        # Note: Need to call versionCount method on *this* class, not the
        # potentially derived class
        self.version = AbstractNode.versionCount()
        self._cache_line_size = cache_line_size

        # Set somewhat large number since we really a lot on internal
        # triggers. To limit the controller performance, tweak other
        # params such as: input port buffer size, cache banks, and output
        # port latency
        self.transitions_per_cycle = 1024
        # This should be set to true in the data cache controller to enable
        # timeouts on unique lines when a store conditional fails
        self.sc_lock_enabled = False

        # Use 32-byte channels (two flits per message)
        self.data_channel_size = 32

        self.connectQueues(network)

    def getBlockSizeBits(self):
        bits = int(math.log(self._cache_line_size, 2))
        if 2**bits != self._cache_line_size.value:
            raise Exception("Cache line size not a power of 2!")
        return bits

    def connectQueues(self, network: RubyNetwork):
        """Connect all of the queues for this controller.
        This may be extended in subclasses.
        """
        self.mandatoryQueue = MessageBuffer()
        self.prefetchQueue = MessageBuffer()

        self.triggerQueue = TriggerMessageBuffer()
        self.retryTriggerQueue = OrderedTriggerMessageBuffer()
        self.replTriggerQueue = OrderedTriggerMessageBuffer()
        self.reqRdy = TriggerMessageBuffer()
        self.snpRdy = TriggerMessageBuffer()

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
