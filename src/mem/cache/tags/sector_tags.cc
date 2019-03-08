/*
 * Copyright (c) 2018 Inria
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
 *
 * Authors: Daniel Carvalho
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

SectorTags::SectorTags(const SectorTagsParams *p)
    : BaseTags(p), allocAssoc(p->assoc),
      sequentialAccess(p->sequential_access),
      replacementPolicy(p->replacement_policy),
      numBlocksPerSector(p->num_blocks_per_sector),
      numSectors(numBlocks / p->num_blocks_per_sector), blks(numBlocks),
      secBlks(numSectors), sectorShift(floorLog2(blkSize)),
      sectorMask(numBlocksPerSector - 1)
{
    // Check parameters
    fatal_if(blkSize < 4 || !isPowerOf2(blkSize),
             "Block size must be at least 4 and a power of 2");
    fatal_if(!isPowerOf2(numBlocksPerSector),
             "# of blocks per sector must be non-zero and a power of 2");
}

void
SectorTags::tagsInit()
{
    // Initialize all blocks
    unsigned blk_index = 0;       // index into blks array
    for (unsigned sec_blk_index = 0; sec_blk_index < numSectors;
         sec_blk_index++)
    {
        // Locate next cache sector
        SectorBlk* sec_blk = &secBlks[sec_blk_index];

        // Link block to indexing policy
        indexingPolicy->setEntry(sec_blk, sec_blk_index);

        // Associate a replacement data entry to the sector
        sec_blk->replacementData = replacementPolicy->instantiateEntry();

        // Initialize all blocks in this sector
        sec_blk->blks.resize(numBlocksPerSector);
        for (unsigned k = 0; k < numBlocksPerSector; ++k){
            // Select block within the set to be linked
            SectorSubBlk*& blk = sec_blk->blks[k];

            // Locate next cache block
            blk = &blks[blk_index];

            // Associate a data chunk to the block
            blk->data = &dataBlks[blkSize*blk_index];

            // Associate sector block to this block
            blk->setSectorBlock(sec_blk);

            // Associate the sector replacement data to this block
            blk->replacementData = sec_blk->replacementData;

            // Set its index and sector offset
            blk->setSectorOffset(k);

            // Update block index
            ++blk_index;
        }
    }
}

void
SectorTags::invalidate(CacheBlk *blk)
{
    BaseTags::invalidate(blk);

    // Get block's sector
    SectorSubBlk* sub_blk = static_cast<SectorSubBlk*>(blk);
    const SectorBlk* sector_blk = sub_blk->getSectorBlock();

    // When a block in a sector is invalidated, it does not make the tag
    // invalid automatically, as there might be other blocks in the sector
    // using it. The tag is invalidated only when there is a single block
    // in the sector.
    if (!sector_blk->isValid()) {
        // Decrease the number of tags in use
        tagsInUse--;

        // Invalidate replacement data, as we're invalidating the sector
        replacementPolicy->invalidate(sector_blk->replacementData);
    }
}

CacheBlk*
SectorTags::accessBlock(Addr addr, bool is_secure, Cycles &lat)
{
    CacheBlk *blk = findBlock(addr, is_secure);

    // Access all tags in parallel, hence one in each way.  The data side
    // either accesses all blocks in parallel, or one block sequentially on
    // a hit.  Sequential access with a miss doesn't access data.
    tagAccesses += allocAssoc;
    if (sequentialAccess) {
        if (blk != nullptr) {
            dataAccesses += 1;
        }
    } else {
        dataAccesses += allocAssoc*numBlocksPerSector;
    }

    // If a cache hit
    if (blk != nullptr) {
        // Update number of references to accessed block
        blk->refCount++;

        // Get block's sector
        SectorSubBlk* sub_blk = static_cast<SectorSubBlk*>(blk);
        const SectorBlk* sector_blk = sub_blk->getSectorBlock();

        // Update replacement data of accessed block, which is shared with
        // the whole sector it belongs to
        replacementPolicy->touch(sector_blk->replacementData);
    }

    // The tag lookup latency is the same for a hit or a miss
    lat = lookupLatency;

    return blk;
}

void
SectorTags::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    // Get block's sector
    SectorSubBlk* sub_blk = static_cast<SectorSubBlk*>(blk);
    const SectorBlk* sector_blk = sub_blk->getSectorBlock();

    // When a block is inserted, the tag is only a newly used tag if the
    // sector was not previously present in the cache.
    if (sector_blk->isValid()) {
        // An existing entry's replacement data is just updated
        replacementPolicy->touch(sector_blk->replacementData);
    } else {
        // Increment tag counter
        tagsInUse++;

        // A new entry resets the replacement data
        replacementPolicy->reset(sector_blk->replacementData);
    }

    // Do common block insertion functionality
    BaseTags::insertBlock(pkt, blk);
}

CacheBlk*
SectorTags::findBlock(Addr addr, bool is_secure) const
{
    // Extract sector tag
    const Addr tag = extractTag(addr);

    // The address can only be mapped to a specific location of a sector
    // due to sectors being composed of contiguous-address entries
    const Addr offset = extractSectorOffset(addr);

    // Find all possible sector entries that may contain the given address
    const std::vector<ReplaceableEntry*> entries =
        indexingPolicy->getPossibleEntries(addr);

    // Search for block
    for (const auto& sector : entries) {
        auto blk = static_cast<SectorBlk*>(sector)->blks[offset];
        if (blk->getTag() == tag && blk->isValid() &&
            blk->isSecure() == is_secure) {
            return blk;
        }
    }

    // Did not find block
    return nullptr;
}

CacheBlk*
SectorTags::findVictim(Addr addr, const bool is_secure,
                       std::vector<CacheBlk*>& evict_blks) const
{
    // Get possible entries to be victimized
    const std::vector<ReplaceableEntry*> sector_entries =
        indexingPolicy->getPossibleEntries(addr);

    // Check if the sector this address belongs to has been allocated
    Addr tag = extractTag(addr);
    SectorBlk* victim_sector = nullptr;
    for (const auto& sector : sector_entries) {
        SectorBlk* sector_blk = static_cast<SectorBlk*>(sector);
        if ((tag == sector_blk->getTag()) && sector_blk->isValid() &&
            (is_secure == sector_blk->isSecure())){
            victim_sector = sector_blk;
            break;
        }
    }

    // If the sector is not present
    if (victim_sector == nullptr){
        // Choose replacement victim from replacement candidates
        victim_sector = static_cast<SectorBlk*>(replacementPolicy->getVictim(
                                                sector_entries));
    }

    // Get the entry of the victim block within the sector
    SectorSubBlk* victim = victim_sector->blks[extractSectorOffset(addr)];

    // Get evicted blocks. Blocks are only evicted if the sectors mismatch and
    // the currently existing sector is valid.
    if ((tag == victim_sector->getTag()) &&
        (is_secure == victim_sector->isSecure())){
        // It would be a hit if victim was valid, and upgrades do not call
        // findVictim, so it cannot happen
        assert(!victim->isValid());
    } else {
        // The whole sector must be evicted to make room for the new sector
        for (const auto& blk : victim_sector->blks){
            evict_blks.push_back(blk);
        }
    }

    return victim;
}

int
SectorTags::extractSectorOffset(Addr addr) const
{
    return (addr >> sectorShift) & sectorMask;
}

Addr
SectorTags::regenerateBlkAddr(const CacheBlk* blk) const
{
    const SectorSubBlk* blk_cast = static_cast<const SectorSubBlk*>(blk);
    const SectorBlk* sec_blk = blk_cast->getSectorBlock();
    const Addr sec_addr = indexingPolicy->regenerateAddr(blk->tag, sec_blk);
    return sec_addr | ((Addr)blk_cast->getSectorOffset() << sectorShift);
}

void
SectorTags::forEachBlk(std::function<void(CacheBlk &)> visitor)
{
    for (SectorSubBlk& blk : blks) {
        visitor(blk);
    }
}

bool
SectorTags::anyBlk(std::function<bool(CacheBlk &)> visitor)
{
    for (SectorSubBlk& blk : blks) {
        if (visitor(blk)) {
            return true;
        }
    }
    return false;
}

SectorTags *
SectorTagsParams::create()
{
    // There must be a indexing policy
    fatal_if(!indexing_policy, "An indexing policy is required");

    return new SectorTags(this);
}
