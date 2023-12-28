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

import math

from m5.objects import (
    LRURP,
    ClockDomain,
    L0Cache_Controller,
    MessageBuffer,
    RubyCache,
    RubyPrefetcher,
)

from ......isas import ISA
from ......utils.override import *
from .....processors.abstract_core import AbstractCore


# L0Cache_Controller is the ruby backend's terminology corresponding to
# L1 cache in stdlib terms.
class L1Cache(L0Cache_Controller):
    _version = 0

    @classmethod
    def versionCount(cls):
        cls._version += 1
        return cls._version - 1

    def __init__(
        self,
        l1i_size,
        l1i_assoc,
        l1d_size,
        l1d_assoc,
        network,
        core: AbstractCore,
        cache_line_size,
        target_isa: ISA,
        clk_domain: ClockDomain,
    ):
        super().__init__()

        # This is the cache memory object that stores the cache data and tags
        self.Icache = RubyCache(
            size=l1i_size,
            assoc=l1i_assoc,
            start_index_bit=self.getBlockSizeBits(cache_line_size),
            is_icache=True,
            replacement_policy=LRURP(),
        )
        self.Dcache = RubyCache(
            size=l1d_size,
            assoc=l1d_assoc,
            start_index_bit=self.getBlockSizeBits(cache_line_size),
            is_icache=False,
            replacement_policy=LRURP(),
        )
        self.clk_domain = clk_domain
        self.prefetcher = RubyPrefetcher()
        self.send_evictions = core.requires_send_evicts()
        self.transitions_per_cycle = 32
        self.enable_prefetch = False
        self.request_latency = 2
        self.response_latency = 2

        self.version = self.versionCount()
        self.connectQueues(network)

    def getBlockSizeBits(self, cache_line_size):
        bits = int(math.log(cache_line_size, 2))
        if 2**bits != int(cache_line_size):
            raise Exception("Cache line size is not a power of 2!")
        return bits

    def connectQueues(self, network):
        self.prefetchQueue = MessageBuffer()
        self.mandatoryQueue = MessageBuffer()
        self.optionalQueue = MessageBuffer()

        # bufferToL1 and bufferFromL1 are ruby backend terminology.
        # In stdlib terms, they are bufferToL2 and bufferFromL2 respectively.
        # These buffers are connections between L1 cache and L2 cache.
        # Later on, we'll need to connect those buffers to L2.
        self.bufferToL1 = MessageBuffer(ordered=True)
        self.bufferFromL1 = MessageBuffer(ordered=True)
