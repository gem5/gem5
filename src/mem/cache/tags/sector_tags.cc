/*
 * Copyright (c) 2024 ARM Limited
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2018, 2020 Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Definitions of a sector tag store.
 */

#include "mem/cache/tags/sector_tags.hh"

#include <cassert>
#include <memory>
#include <string>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"

namespace gem5
{

SectorTags::SectorTags(const SectorTagsParams &p)
    : BaseTags(p),
      allocAssoc(p.assoc),
      sequentialAccess(p.sequential_access),
      replacementPolicy(p.replacement_policy),
      numBlocksPerSector(p.num_blocks_per_sector),
      numSectors(numBlocks / numBlocksPerSector),
      sectorShift(floorLog2(blkSize)),
      sectorMask(numBlocksPerSector - 1),
      sectorStats(stats, *this)
{
    // There must be a indexing policy
    fatal_if(!p.indexing_policy, "An indexing policy is required");

    // Check parameters
    fatal_if(blkSize < 4 || !isPowerOf2(blkSize),
             "Block size must be at least 4 and a power of 2");
    fatal_if(!isPowerOf2(numBlocksPerSector),
             "# of blocks per sector must be non-zero and a power of 2");
    warn_if((p.partitioning_policies.size() > 0),
            "Using cache partitioning policies with sector and/or compressed "
            "tags is not fully tested.");
}

void
SectorTags::tagsInit()
{
    // Create blocks and sector blocks
    blks = std::vector<SectorSubBlk>(numBlocks);
    secBlks = std::vector<SectorBlk>(numSectors);

    // Initialize all blocks
    unsigned blk_index = 0; // index into blks array
    for (unsigned sec_blk_index = 0; sec_blk_index < numSectors;
         sec_blk_index++) {
        // Locate next cache sector
        SectorBlk *sec_blk = &secBlks[sec_blk_index];

        // Associate a replacement data entry to the sector
        sec_blk->replacementData = replacementPolicy->instantiateEntry();

        // Initialize all blocks in this sector
        sec_blk->blks.resize(numBlocksPerSector);
        for (unsigned k = 0; k < numBlocksPerSector; ++k) {
            // Select block within the set to be linked
            SectorSubBlk *&blk = sec_blk->blks[k];

            // Locate next cache block
            blk = &blks[blk_index];

            // Associate a data chunk to the block
            blk->data = &dataBlks[blkSize * blk_index];

            // Associate sector block to this block
            blk->setSectorBlock(sec_blk);

            // Associate the sector replacement data to this block
            blk->replacementData = sec_blk->replacementData;

            // Set its index and sector offset
            blk->setSectorOffset(k);

            // Update block index
            ++blk_index;
        }

        // Link block to indexing policy
        indexingPolicy->setEntry(sec_blk, sec_blk_index);
    }
}

void
SectorTags::invalidate(CacheBlk *blk)
{
    BaseTags::invalidate(blk);

    // Get block's sector
    SectorSubBlk *sub_blk = static_cast<SectorSubBlk *>(blk);
    const SectorBlk *sector_blk = sub_blk->getSectorBlock();

    // When a block in a sector is invalidated, it does not make the tag
    // invalid automatically, as there might be other blocks in the sector
    // using it. The tag is invalidated only when there is a single block
    // in the sector.
    if (!sector_blk->isValid()) {
        // Decrease the number of tags in use
        stats.tagsInUse--;
        assert(stats.tagsInUse.value() >= 0);

        // Invalidate replacement data, as we're invalidating the sector
        replacementPolicy->invalidate(sector_blk->replacementData);
    }
}

CacheBlk *
SectorTags::accessBlock(const PacketPtr pkt, Cycles &lat)
{
    CacheBlk *blk = findBlock(pkt->getAddr(), pkt->isSecure());

    // Access all tags in parallel, hence one in each way.  The data side
    // either accesses all blocks in parallel, or one block sequentially on
    // a hit.  Sequential access with a miss doesn't access data.
    stats.tagAccesses += allocAssoc;
    if (sequentialAccess) {
        if (blk != nullptr) {
            stats.dataAccesses += 1;
        }
    } else {
        stats.dataAccesses += allocAssoc * numBlocksPerSector;
    }

    // If a cache hit
    if (blk != nullptr) {
        // Update number of references to accessed block
        blk->increaseRefCount();

        // Get block's sector
        SectorSubBlk *sub_blk = static_cast<SectorSubBlk *>(blk);
        const SectorBlk *sector_blk = sub_blk->getSectorBlock();

        // Update replacement data of accessed block, which is shared with
        // the whole sector it belongs to
        replacementPolicy->touch(sector_blk->replacementData, pkt);
    }

    // The tag lookup latency is the same for a hit or a miss
    lat = lookupLatency;

    return blk;
}

void
SectorTags::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    // Get block's sector
    SectorSubBlk *sub_blk = static_cast<SectorSubBlk *>(blk);
    const SectorBlk *sector_blk = sub_blk->getSectorBlock();

    // When a block is inserted, the tag is only a newly used tag if the
    // sector was not previously present in the cache.
    if (sector_blk->isValid()) {
        // An existing entry's replacement data is just updated
        replacementPolicy->touch(sector_blk->replacementData, pkt);
    } else {
        // Increment tag counter
        stats.tagsInUse++;
        assert(stats.tagsInUse.value() <= numSectors);

        // A new entry resets the replacement data
        replacementPolicy->reset(sector_blk->replacementData, pkt);
    }

    // Do common block insertion functionality
    BaseTags::insertBlock(pkt, blk);
}

void
SectorTags::moveBlock(CacheBlk *src_blk, CacheBlk *dest_blk)
{
    const bool dest_was_valid =
        static_cast<SectorSubBlk *>(dest_blk)->getSectorBlock()->isValid();

    BaseTags::moveBlock(src_blk, dest_blk);

    // Get blocks' sectors. The blocks have effectively been swapped by now,
    // so src points to an invalid block, and dest to the moved valid one.
    SectorSubBlk *src_sub_blk = static_cast<SectorSubBlk *>(src_blk);
    const SectorBlk *src_sector_blk = src_sub_blk->getSectorBlock();
    SectorSubBlk *dest_sub_blk = static_cast<SectorSubBlk *>(dest_blk);
    const SectorBlk *dest_sector_blk = dest_sub_blk->getSectorBlock();

    // Since the blocks were using different replacement data pointers,
    // we must touch the replacement data of the new entry, and invalidate
    // the one that is being moved.
    // When a block in a sector is invalidated, it does not make the tag
    // invalid automatically, as there might be other blocks in the sector
    // using it. The tag is invalidated only when there is a single block
    // in the sector.
    if (!src_sector_blk->isValid()) {
        // Invalidate replacement data, as we're invalidating the sector
        replacementPolicy->invalidate(src_sector_blk->replacementData);

        if (dest_was_valid) {
            // If destination sector was valid, and the source sector became
            // invalid, there is one less tag being used
            stats.tagsInUse--;
            assert(stats.tagsInUse.value() >= 0);
        }
    } else if (!dest_was_valid) {
        // If destination sector was invalid and became valid, and the source
        // sector is still valid, there is one extra tag being used
        stats.tagsInUse++;
        assert(stats.tagsInUse.value() <= numSectors);
    }

    if (dest_was_valid) {
        replacementPolicy->touch(dest_sector_blk->replacementData);
    } else {
        replacementPolicy->reset(dest_sector_blk->replacementData);
    }
}

CacheBlk *
SectorTags::findBlock(Addr addr, bool is_secure) const
{
    // Extract sector tag
    const Addr tag = extractTag(addr);

    // The address can only be mapped to a specific location of a sector
    // due to sectors being composed of contiguous-address entries
    const Addr offset = extractSectorOffset(addr);

    // Find all possible sector entries that may contain the given address
    const std::vector<ReplaceableEntry *> entries =
        indexingPolicy->getPossibleEntries(addr);

    // Search for block
    for (const auto &sector : entries) {
        auto blk = static_cast<SectorBlk *>(sector)->blks[offset];
        if (blk->matchTag(tag, is_secure)) {
            return blk;
        }
    }

    // Did not find block
    return nullptr;
}

CacheBlk *
SectorTags::findVictim(Addr addr, const bool is_secure, const std::size_t size,
                       std::vector<CacheBlk *> &evict_blks,
                       const uint64_t partition_id)
{
    // Get possible entries to be victimized
    std::vector<ReplaceableEntry *> sector_entries =
        indexingPolicy->getPossibleEntries(addr);

    // Filter entries based on PartitionID
    for (auto partitioning_policy : partitioningPolicies)
        partitioning_policy->filterByPartition(sector_entries, partition_id);

    // Check if the sector this address belongs to has been allocated
    Addr tag = extractTag(addr);
    SectorBlk *victim_sector = nullptr;
    for (const auto &sector : sector_entries) {
        SectorBlk *sector_blk = static_cast<SectorBlk *>(sector);
        if (sector_blk->matchTag(tag, is_secure)) {
            victim_sector = sector_blk;
            break;
        }
    }

    // If the sector is not present
    if (victim_sector == nullptr) {
        // check if partitioning policy limited allocation and if true - return
        // this assumes that sector_entries would not be empty if partitioning
        // policy is not in place
        if (sector_entries.size() == 0) {
            return nullptr;
        }
        // Choose replacement victim from replacement candidates
        victim_sector = static_cast<SectorBlk *>(
            replacementPolicy->getVictim(sector_entries));
    }

    // Get the entry of the victim block within the sector
    SectorSubBlk *victim = victim_sector->blks[extractSectorOffset(addr)];

    // Get evicted blocks. Blocks are only evicted if the sectors mismatch and
    // the currently existing sector is valid.
    if (victim_sector->matchTag(tag, is_secure)) {
        // It would be a hit if victim was valid, and upgrades do not call
        // findVictim, so it cannot happen
        assert(!victim->isValid());
    } else {
        // The whole sector must be evicted to make room for the new sector
        for (const auto &blk : victim_sector->blks) {
            if (blk->isValid()) {
                evict_blks.push_back(blk);
            }
        }
    }

    // Update number of sub-blocks evicted due to a replacement
    sectorStats.evictionsReplacement[evict_blks.size()]++;

    return victim;
}

int
SectorTags::extractSectorOffset(Addr addr) const
{
    return (addr >> sectorShift) & sectorMask;
}

Addr
SectorTags::regenerateBlkAddr(const CacheBlk *blk) const
{
    const SectorSubBlk *blk_cast = static_cast<const SectorSubBlk *>(blk);
    const SectorBlk *sec_blk = blk_cast->getSectorBlock();
    const Addr sec_addr =
        indexingPolicy->regenerateAddr(blk->getTag(), sec_blk);
    return sec_addr | ((Addr)blk_cast->getSectorOffset() << sectorShift);
}

SectorTags::SectorTagsStats::SectorTagsStats(BaseTagStats &base_group,
                                             SectorTags &_tags)
    : statistics::Group(&base_group),
      tags(_tags),
      ADD_STAT(evictionsReplacement, statistics::units::Count::get(),
               "Number of blocks evicted due to a replacement")
{}

void
SectorTags::SectorTagsStats::regStats()
{
    statistics::Group::regStats();

    evictionsReplacement.init(tags.numBlocksPerSector + 1);
    for (unsigned i = 0; i <= tags.numBlocksPerSector; ++i) {
        evictionsReplacement.subname(i, std::to_string(i));
        evictionsReplacement.subdesc(i, "Number of replacements that caused "
                                        "the eviction of " +
                                            std::to_string(i) + " blocks");
    }
}

bool
SectorTags::anyBlk(std::function<bool(CacheBlk &)> visitor)
{
    for (SectorSubBlk &blk : blks) {
        if (visitor(blk)) {
            return true;
        }
    }
    return false;
}

} // namespace gem5
