# Copyright (c) 2012-2013, 2023-2024 ARM Limited
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
from m5.objects.IndexingPolicies import *
from m5.params import *
from m5.proxy import *


class TaggedIndexingPolicy(SimObject):
    type = "TaggedIndexingPolicy"
    abstract = True
    cxx_class = "gem5::IndexingPolicyTemplate<gem5::TaggedTypes>"
    cxx_header = "mem/cache/tags/tagged_entry.hh"
    cxx_template_params = ["class Types"]

    # Get the associativity
    assoc = Param.Int(Parent.assoc, "associativity")


class TaggedSetAssociative(TaggedIndexingPolicy):
    type = "TaggedSetAssociative"
    cxx_class = "gem5::TaggedSetAssociative"
    cxx_header = "mem/cache/tags/tagged_entry.hh"

    # Get the size from the parent (cache)
    size = Param.MemorySize(Parent.size, "capacity in bytes")

    # Get the entry size from the parent (tags)
    entry_size = Param.Int(Parent.entry_size, "entry size in bytes")


class BaseTags(ClockedObject):
    type = "BaseTags"
    abstract = True
    cxx_header = "mem/cache/tags/base.hh"
    cxx_class = "gem5::BaseTags"

    # Get system to which it belongs
    system = Param.System(Parent.any, "System we belong to")

    # Get the size from the parent (cache)
    size = Param.MemorySize(Parent.size, "capacity in bytes")

    # Get the block size from the parent (system)
    block_size = Param.Int(Parent.cache_line_size, "block size in bytes")

    # Get the tag lookup latency from the parent (cache)
    tag_latency = Param.Cycles(
        Parent.tag_latency, "The tag lookup latency for this cache"
    )

    # Get the warmup percentage from the parent (cache)
    warmup_percentage = Param.Percent(
        Parent.warmup_percentage,
        "Percentage of tags to be touched to warm up the cache",
    )

    sequential_access = Param.Bool(
        Parent.sequential_access,
        "Whether to access tags and data sequentially",
    )

    # Get indexing policy
    indexing_policy = Param.TaggedIndexingPolicy(
        TaggedSetAssociative(), "Indexing policy"
    )

    partitioning_manager = Param.PartitionManager(
        Parent.partitioning_manager, "Cache partitioning manager"
    )

    # Set the indexing entry size as the block size
    entry_size = Param.Int(
        Parent.cache_line_size, "Indexing entry size in bytes"
    )


class BaseSetAssoc(BaseTags):
    type = "BaseSetAssoc"
    cxx_header = "mem/cache/tags/base_set_assoc.hh"
    cxx_class = "gem5::BaseSetAssoc"

    # Get the cache associativity
    assoc = Param.Int(Parent.assoc, "associativity")

    # Get replacement policy from the parent (cache)
    replacement_policy = Param.BaseReplacementPolicy(
        Parent.replacement_policy, "Replacement policy"
    )


class SectorTags(BaseTags):
    type = "SectorTags"
    cxx_header = "mem/cache/tags/sector_tags.hh"
    cxx_class = "gem5::SectorTags"

    # Get the cache associativity
    assoc = Param.Int(Parent.assoc, "associativity")

    # Number of sub-sectors (data blocks) per sector
    num_blocks_per_sector = Param.Int(1, "Number of sub-sectors per sector")

    # The indexing entry now is a sector block
    entry_size = Parent.cache_line_size * Self.num_blocks_per_sector

    # Get replacement policy from the parent (cache)
    replacement_policy = Param.BaseReplacementPolicy(
        Parent.replacement_policy, "Replacement policy"
    )


class CompressedTags(SectorTags):
    type = "CompressedTags"
    cxx_header = "mem/cache/tags/compressed_tags.hh"
    cxx_class = "gem5::CompressedTags"

    # Maximum number of compressed blocks per tag
    max_compression_ratio = Param.Int(
        2, "Maximum number of compressed blocks per tag."
    )

    # We simulate superblock as sector blocks
    num_blocks_per_sector = Self.max_compression_ratio

    # We virtually increase the number of data blocks per tag by multiplying
    # the cache size by the compression ratio
    size = Parent.size * Self.max_compression_ratio


class FALRU(BaseTags):
    type = "FALRU"
    cxx_header = "mem/cache/tags/fa_lru.hh"
    cxx_class = "gem5::FALRU"

    min_tracked_cache_size = Param.MemorySize(
        "128KiB", "Minimum cache size for which we track statistics"
    )

    # This tag uses its own embedded indexing
    indexing_policy = NULL
