# Copyright (c) 2022 The Regents of the University of California
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

from .....processors.abstract_core import AbstractCore
from ......isas import ISA
from ......utils.override import *

from m5.objects import (
    MessageBuffer,
    RubyPrefetcher,
    RubyCache,
    ClockDomain,
    L1Cache_Controller,
)

import math

# L1Cache_Controller is ruby backend's terminology corresponding to
# L2Cache in stdlib's terms
class L2Cache(L1Cache_Controller):

    _version = 0

    @classmethod
    def versionCount(cls):
        cls._version += 1
        return cls._version - 1

    def __init__(
        self,
        l2_size,
        l2_assoc,
        network,
        core: AbstractCore,
        num_l3Caches,
        cache_line_size,
        cluster_id,
        target_isa: ISA,
        clk_domain: ClockDomain,
    ):
        super().__init__()

        # This is the cache memory object that stores the cache data and tags
        self.cache = RubyCache(
            size=l2_size,
            assoc=l2_assoc,
            start_index_bit=self.getBlockSizeBits(cache_line_size.value),
            is_icache=False,
        )
        # l2_select_num_bits is ruby backend terminology.
        # In stdlib terms, it is number of bits for selecting L3 cache.
        self.l2_select_num_bits = int(math.log(num_l3Caches, 2))
        self.cluster_id = cluster_id
        self.clk_domain = clk_domain
        self.prefetcher = RubyPrefetcher()
        self.transitions_per_cycle = 32
        # l1_request_latency, l1_response_latency, to_l2_latency are
        # ruby backend terminology.
        # In stdlib terms, they are L2 cache request latency, L2 response
        # latency, and to L3 cache latency respectively.
        self.l1_request_latency = 2
        self.l1_response_latency = 2
        self.to_l2_latency = 1

        self.version = self.versionCount()
        self.connectQueues(network)

    def getBlockSizeBits(self, cache_line_size):
        bits = int(math.log(cache_line_size, 2))
        if 2**bits != cache_line_size:
            raise Exception("Cache line size is not a power of 2!")
        return bits

    def connectQueues(self, network):
        self.mandatoryQueue = MessageBuffer()
        self.optionalQueue = MessageBuffer()

        # In the below terms, L2 are ruby backend terminology.
        # They are L3 in stdlib.

        # Request from/to L2 buffers
        self.requestFromL2 = MessageBuffer()
        self.requestFromL2.in_port = network.out_port
        self.requestToL2 = MessageBuffer()
        self.requestToL2.out_port = network.in_port

        # Response from/to L2 buffers
        self.responseFromL2 = MessageBuffer()
        self.responseFromL2.in_port = network.out_port
        self.responseToL2 = MessageBuffer()
        self.responseToL2.out_port = network.in_port

        # Unblock to L2 buffer
        self.unblockToL2 = MessageBuffer()
        self.unblockToL2.out_port = network.in_port
