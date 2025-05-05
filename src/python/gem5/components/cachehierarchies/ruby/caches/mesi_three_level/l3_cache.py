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
    MESI_Three_Level_L2Cache_Controller,
    MessageBuffer,
    RubyCache,
)


# L2Cache_Controller is ruby backend's terminology corresponding to
# L3 cache in stdlib.
class L3Cache(MESI_Three_Level_L2Cache_Controller):
    _version = 0

    @classmethod
    def versionCount(cls):
        cls._version += 1
        return cls._version - 1

    def __init__(
        self,
        l3_size,
        l3_assoc,
        network,
        num_l3Caches,
        cache_line_size,
        cluster_id,
    ):
        super().__init__()

        # This is the cache memory object that stores the cache data and tags
        self.L2cache = RubyCache(
            size=l3_size,
            assoc=l3_assoc,
            start_index_bit=self.getIndexBit(num_l3Caches, cache_line_size),
        )

        self.transitions_per_cycle = 4
        self.cluster_id = cluster_id
        self.l2_request_latency = 2
        self.l2_response_latency = 2
        self.to_l1_latency = 1

        self.version = self.versionCount()
        self.connectQueues(network)

    def getIndexBit(self, num_l3Caches, cache_line_size):
        l3_bits = int(math.log(num_l3Caches, 2))
        bits = int(math.log(cache_line_size, 2)) + l3_bits
        return bits

    def connectQueues(self, network):
        # In the below terms, L1 and L2 are ruby backend terminology.
        # In stdlib, they are L2 and L3 caches respectively.
        self.DirRequestFromL2Cache = MessageBuffer()
        self.DirRequestFromL2Cache.out_port = network.in_port
        self.L1RequestFromL2Cache = MessageBuffer()
        self.L1RequestFromL2Cache.out_port = network.in_port
        self.responseFromL2Cache = MessageBuffer()
        self.responseFromL2Cache.out_port = network.in_port
        self.unblockToL2Cache = MessageBuffer()
        self.unblockToL2Cache.in_port = network.out_port
        self.L1RequestToL2Cache = MessageBuffer()
        self.L1RequestToL2Cache.in_port = network.out_port
        self.responseToL2Cache = MessageBuffer()
        self.responseToL2Cache.in_port = network.out_port
