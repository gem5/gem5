# Copyright (c) 2021-2025 Arm Limited
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

import math
from typing import List

from m5.objects import (
    ClockDomain,
    RubyCache,
    RubyNetwork,
)
from m5.params import (
    NULL,
    AddrRange,
)

from .abstract_node import AbstractNode


class BaseDirectory(AbstractNode):
    """
    BaseDirectory. Mainly providing address range generation
    capabilities (see create_addr_ranges method)
    """

    def __init__(
        self,
        network: RubyNetwork,
        cache_line_size: int,
    ):
        super().__init__(network, cache_line_size)

    @classmethod
    def create_addr_ranges(
        cls,
        num_directories: int,
        dir_idx: int,
        mem_ranges: List[AddrRange],
        cache_line_size,
    ) -> List[AddrRange]:
        block_size_bits = int(math.log(cache_line_size, 2))
        llc_bits = int(math.log(num_directories, 2))
        numa_bit = block_size_bits + llc_bits - 1

        ranges = []
        for r in mem_ranges:
            addr_range = AddrRange(
                r.start,
                size=r.size(),
                intlvHighBit=numa_bit,
                intlvBits=llc_bits,
                intlvMatch=dir_idx,
            )
            ranges.append(addr_range)
        return ranges


class SimpleDirectory(BaseDirectory):
    """A directory or home node (HNF)

    This simple directory has no cache. It forwards all requests as directly
    as possible.
    """

    def __init__(
        self,
        network: RubyNetwork,
        cache_line_size: int,
        clk_domain: ClockDomain,
        addr_ranges: List[AddrRange],
    ):
        super().__init__(network, cache_line_size)

        # Dummy cache
        self.cache = RubyCache(
            dataAccessLatency=0, tagAccessLatency=1, size="128", assoc=1
        )

        self.addr_ranges = addr_ranges
        self.clk_domain = clk_domain

        # Only used for L1 controllers
        self.send_evictions = False
        self.sequencer = NULL

        self.use_prefetcher = False
        self.prefetcher = NULL

        # Set up home node that allows three hop protocols
        self.is_HN = True
        self.enable_DMT = True
        self.enable_DCT = True

        # "Owned state"
        self.allow_SD = True

        # No cache
        self.alloc_on_seq_acc = False
        self.alloc_on_seq_line_write = False
        self.alloc_on_readshared = False
        self.alloc_on_readunique = False
        self.alloc_on_readonce = False
        self.alloc_on_writeback = False
        self.alloc_on_atomic = False
        self.dealloc_on_unique = False
        self.dealloc_on_shared = False
        self.dealloc_backinv_unique = False
        self.dealloc_backinv_shared = False

        # Some reasonable default TBE params
        self.number_of_TBEs = 32
        self.number_of_repl_TBEs = 32
        self.number_of_snoop_TBEs = 1
        self.number_of_DVM_TBEs = 1  # should not receive any dvm
        self.number_of_DVM_snoop_TBEs = 1  # should not receive any dvm
        self.unify_repl_TBEs = False
