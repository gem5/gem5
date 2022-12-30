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

from gem5.components.processors.abstract_core import AbstractCore
from gem5.isas import ISA

from .abstract_node import AbstractNode

from m5.objects import ClockDomain, RubyCache


class DMARequestor(AbstractNode):
    def __init__(self, network, cache_line_size, clk_domain: ClockDomain):
        super().__init__(network, cache_line_size)

        # Dummy cache
        self.cache = RubyCache(
            dataAccessLatency=0, tagAccessLatency=1, size="128", assoc=1
        )

        self.clk_domain = clk_domain

        # Only applies to home nodes
        self.is_HN = False
        self.enable_DMT = False
        self.enable_DCT = False

        # No cache
        self.allow_SD = False
        self.alloc_on_seq_acc = False
        self.alloc_on_seq_line_write = False
        self.alloc_on_readshared = False
        self.alloc_on_readunique = False
        self.alloc_on_readonce = False
        self.alloc_on_writeback = False
        self.dealloc_on_unique = False
        self.dealloc_on_shared = False
        self.dealloc_backinv_unique = True
        self.dealloc_backinv_shared = True

        self.send_evictions = False
        self.use_prefetcher = False
        # Some reasonable default TBE params
        self.number_of_TBEs = 16
        self.number_of_repl_TBEs = 1
        self.number_of_snoop_TBEs = 1  # Should never receive snoops
        self.number_of_DVM_TBEs = 1  # should not receive any dvm
        self.number_of_DVM_snoop_TBEs = 1  # should not receive any dvm
        self.unify_repl_TBEs = False
