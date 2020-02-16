# Copyright (c) 2019 Inria
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

class BloomFilterBase(SimObject):
    type = 'BloomFilterBase'
    abstract = True
    cxx_header = "base/filters/base.hh"
    cxx_class = 'BloomFilter::Base'

    size = Param.Int(4096, "Number of entries in the filter")

    # By default assume that bloom filters are used for 64-byte cache lines
    offset_bits = Param.Unsigned(6, "Number of bits in a cache line offset")

    # Most of the filters are booleans, and thus saturate on 1
    num_bits = Param.Int(1, "Number of bits in a filter entry")
    threshold = Param.Int(1, "Value at which an entry is considered as set")

class BloomFilterBlock(BloomFilterBase):
    type = 'BloomFilterBlock'
    cxx_class = 'BloomFilter::Block'
    cxx_header = "base/filters/block_bloom_filter.hh"

    masks_lsbs = VectorParam.Unsigned([Self.offset_bits,
        2 * Self.offset_bits], "Position of the LSB of each mask")
    masks_sizes = VectorParam.Unsigned([Self.offset_bits, Self.offset_bits],
        "Size, in number of bits, of each mask")

class BloomFilterMultiBitSel(BloomFilterBase):
    type = 'BloomFilterMultiBitSel'
    cxx_class = 'BloomFilter::MultiBitSel'
    cxx_header = "base/filters/multi_bit_sel_bloom_filter.hh"

    num_hashes = Param.Int(4, "Number of hashes")
    threshold = Self.num_hashes
    skip_bits = Param.Int(2, "Offset from block number")
    is_parallel = Param.Bool(False, "Whether hashing is done in parallel")

class BloomFilterBulk(BloomFilterMultiBitSel):
    type = 'BloomFilterBulk'
    cxx_class = 'BloomFilter::Bulk'
    cxx_header = "base/filters/bulk_bloom_filter.hh"

class BloomFilterH3(BloomFilterMultiBitSel):
    type = 'BloomFilterH3'
    cxx_class = 'BloomFilter::H3'
    cxx_header = "base/filters/h3_bloom_filter.hh"

class BloomFilterMulti(BloomFilterBase):
    type = 'BloomFilterMulti'
    cxx_class = 'BloomFilter::Multi'
    cxx_header = "base/filters/multi_bloom_filter.hh"

    # The base filter should not be used, since this filter is the combination
    # of multiple sub-filters, so we use a dummy value
    size = 1

    # By default there are two sub-filters that hash sequential bitfields
    filters = VectorParam.BloomFilterBase([
        BloomFilterBlock(size = 4096, masks_lsbs = [6, 12]),
        BloomFilterBlock(size = 1024, masks_lsbs = [18, 24])],
        "Sub-filters to be combined")

    # By default match this with the number of sub-filters
    threshold = 2

class BloomFilterPerfect(BloomFilterBase):
    type = 'BloomFilterPerfect'
    cxx_class = 'BloomFilter::Perfect'
    cxx_header = "base/filters/perfect_bloom_filter.hh"

    # The base filter is not needed. Use a dummy value.
    size = 1
