# Copyright (c) 2018-2020 Inria
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

class BaseReplacementPolicy(SimObject):
    type = 'BaseReplacementPolicy'
    abstract = True
    cxx_class = 'replacement_policy::Base'
    cxx_header = "mem/cache/replacement_policies/base.hh"

class DuelingRP(BaseReplacementPolicy):
    type = 'DuelingRP'
    cxx_class = 'replacement_policy::Dueling'
    cxx_header = "mem/cache/replacement_policies/dueling_rp.hh"

    constituency_size = Param.Unsigned(
        "The size of a region containing one sample")
    team_size = Param.Unsigned(
        "Number of entries in a sampling set that belong to a team")
    replacement_policy_a = Param.BaseReplacementPolicy(
        "Sub-replacement policy A")
    replacement_policy_b = Param.BaseReplacementPolicy(
        "Sub-replacement policy B")

class FIFORP(BaseReplacementPolicy):
    type = 'FIFORP'
    cxx_class = 'replacement_policy::FIFO'
    cxx_header = "mem/cache/replacement_policies/fifo_rp.hh"

class SecondChanceRP(FIFORP):
    type = 'SecondChanceRP'
    cxx_class = 'replacement_policy::SecondChance'
    cxx_header = "mem/cache/replacement_policies/second_chance_rp.hh"

class LFURP(BaseReplacementPolicy):
    type = 'LFURP'
    cxx_class = 'replacement_policy::LFU'
    cxx_header = "mem/cache/replacement_policies/lfu_rp.hh"

class LRURP(BaseReplacementPolicy):
    type = 'LRURP'
    cxx_class = 'replacement_policy::LRU'
    cxx_header = "mem/cache/replacement_policies/lru_rp.hh"

class BIPRP(LRURP):
    type = 'BIPRP'
    cxx_class = 'replacement_policy::BIP'
    cxx_header = "mem/cache/replacement_policies/bip_rp.hh"
    btp = Param.Percent(3, "Percentage of blocks to be inserted as MRU")

class LIPRP(BIPRP):
    btp = 0

class MRURP(BaseReplacementPolicy):
    type = 'MRURP'
    cxx_class = 'replacement_policy::MRU'
    cxx_header = "mem/cache/replacement_policies/mru_rp.hh"

class RandomRP(BaseReplacementPolicy):
    type = 'RandomRP'
    cxx_class = 'replacement_policy::Random'
    cxx_header = "mem/cache/replacement_policies/random_rp.hh"

class BRRIPRP(BaseReplacementPolicy):
    type = 'BRRIPRP'
    cxx_class = 'replacement_policy::BRRIP'
    cxx_header = "mem/cache/replacement_policies/brrip_rp.hh"
    num_bits = Param.Int(2, "Number of bits per RRPV")
    hit_priority = Param.Bool(False,
        "Prioritize evicting blocks that havent had a hit recently")
    btp = Param.Percent(3,
        "Percentage of blocks to be inserted with long RRPV")

class RRIPRP(BRRIPRP):
    btp = 100

class NRURP(BRRIPRP):
    btp = 100
    num_bits = 1

class TreePLRURP(BaseReplacementPolicy):
    type = 'TreePLRURP'
    cxx_class = 'replacement_policy::TreePLRU'
    cxx_header = "mem/cache/replacement_policies/tree_plru_rp.hh"
    num_leaves = Param.Int(Parent.assoc, "Number of leaves in each tree")

class WeightedLRURP(BaseReplacementPolicy):
    type = "WeightedLRURP"
    cxx_class = "replacement_policy::WeightedLRU"
    cxx_header = "mem/cache/replacement_policies/weighted_lru_rp.hh"
