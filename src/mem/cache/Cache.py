# Copyright (c) 2012-2013, 2015, 2018, 2023-2024 ARM Limited
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
# Copyright (c) 2005-2007 The Regents of The University of Michigan
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

from m5.objects.ClockedObject import ClockedObject
from m5.objects.Compressors import BaseCacheCompressor
from m5.objects.Prefetcher import BasePrefetcher
from m5.objects.ReplacementPolicies import *
from m5.objects.Tags import *
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


# Enum for cache clusivity, currently mostly inclusive or mostly
# exclusive.
class Clusivity(Enum):
    vals = ["mostly_incl", "mostly_excl"]


class WriteAllocator(SimObject):
    type = "WriteAllocator"
    cxx_header = "mem/cache/cache.hh"
    cxx_class = "gem5::WriteAllocator"

    # Control the limits for when the cache introduces extra delays to
    # allow whole-line write coalescing, and eventually switches to a
    # write-no-allocate policy.
    coalesce_limit = Param.Unsigned(
        2, "Consecutive lines written before delaying for coalescing"
    )
    no_allocate_limit = Param.Unsigned(
        12, "Consecutive lines written before skipping allocation"
    )

    delay_threshold = Param.Unsigned(
        8,
        "Number of delay quanta imposed on an "
        "MSHR with write requests to allow for "
        "write coalescing",
    )

    block_size = Param.Int(Parent.cache_line_size, "block size in bytes")


class BaseCache(ClockedObject):
    type = "BaseCache"
    abstract = True
    cxx_header = "mem/cache/base.hh"
    cxx_class = "gem5::BaseCache"

    size = Param.MemorySize("Capacity")
    assoc = Param.Unsigned("Associativity")

    tag_latency = Param.Cycles("Tag lookup latency")
    data_latency = Param.Cycles("Data access latency")
    response_latency = Param.Cycles("Latency for the return path on a miss")

    warmup_percentage = Param.Percent(
        0, "Percentage of tags to be touched to warm up the cache"
    )

    max_miss_count = Param.Counter(
        0, "Number of misses to handle before calling exit"
    )

    mshrs = Param.Unsigned("Number of MSHRs (max outstanding requests)")
    demand_mshr_reserve = Param.Unsigned(1, "MSHRs reserved for demand access")
    tgts_per_mshr = Param.Unsigned("Max number of accesses per MSHR")
    write_buffers = Param.Unsigned(8, "Number of write buffers")

    is_read_only = Param.Bool(False, "Is this cache read only (e.g. inst)")

    prefetcher = Param.BasePrefetcher(NULL, "Prefetcher attached to cache")

    tags = Param.BaseTags(BaseSetAssoc(), "Tag store")
    replacement_policy = Param.BaseReplacementPolicy(
        LRURP(), "Replacement policy"
    )
    partitioning_policies = VectorParam.BasePartitioningPolicy(
        [],
        "Partitioning policies "
        "Setting multiple policies will enforce all of them individually "
        "in order",
    )
    partitioning_manager = Param.PartitionManager(
        NULL, "Cache partitioning manager"
    )

    compressor = Param.BaseCacheCompressor(NULL, "Cache compressor.")
    replace_expansions = Param.Bool(
        True,
        "Apply replacement policy to "
        "decide which blocks should be evicted on a data expansion",
    )
    # When a block passes from uncompressed to compressed, it may become
    # co-allocatable with another existing entry of the same superblock,
    # so try move the block to co-allocate it
    move_contractions = Param.Bool(
        True, "Try to co-allocate blocks that contract"
    )

    sequential_access = Param.Bool(
        False, "Whether to access tags and data sequentially"
    )

    cpu_side = ResponsePort("Upstream port closer to the CPU and/or device")
    mem_side = RequestPort("Downstream port closer to memory")

    addr_ranges = VectorParam.AddrRange(
        [AllMemory], "Address range for the CPU-side port (to allow striping)"
    )

    system = Param.System(Parent.any, "System we belong to")

    # Determine if this cache sends out writebacks for clean lines, or
    # simply clean evicts. If this cache does not have a downstream cache,
    # the cache should not writeback clean lines not to waste memory
    # bandwidth. If this cache has a downstream cache whose clusivity is
    # mostly exclusive (i.e., victim cache), this shoule be set to True.
    # If not, there will never be any spills from read-only caches (e.g.,
    # L1I cache, MMU cache of ARM) to the downstream cache.
    # In case of the downstream cache is mostly inclusive, this should be
    # set to False.
    writeback_clean = Param.Bool(False, "Writeback clean lines")

    # Control whether this cache should be mostly inclusive or mostly
    # exclusive with respect to upstream caches. The behaviour on a
    # fill is determined accordingly. For a mostly inclusive cache,
    # blocks are allocated on all fill operations. Thus, L1 caches
    # should be set as mostly inclusive even if they have no upstream
    # caches. In the case of a mostly exclusive cache, fills are not
    # allocating unless they came directly from a non-caching source,
    # e.g. a table walker. Additionally, on a hit from an upstream
    # cache a line is dropped for a mostly exclusive cache.
    clusivity = Param.Clusivity("mostly_incl", "Clusivity with upstream cache")

    # The write allocator enables optimizations for streaming write
    # accesses by first coalescing writes and then avoiding allocation
    # in the current cache. Typically, this would be enabled in the
    # data cache.
    write_allocator = Param.WriteAllocator(NULL, "Write allocator")


class Cache(BaseCache):
    type = "Cache"
    cxx_header = "mem/cache/cache.hh"
    cxx_class = "gem5::Cache"


class NoncoherentCache(BaseCache):
    type = "NoncoherentCache"
    cxx_header = "mem/cache/noncoherent_cache.hh"
    cxx_class = "gem5::NoncoherentCache"

    # This is typically a last level cache and any clean
    # writebacks would be unnecessary traffic to the main memory.
    writeback_clean = False
