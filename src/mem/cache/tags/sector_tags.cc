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
 * Definitions of a base set associative sector tag store.
 */

#include "mem/cache/tags/sector_tags.hh"

#include <cassert>
#include <memory>
#include <string>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "debug/CacheRepl.hh"
#include "mem/cache/base.hh"
#include "mem/cache/replacement_policies/base.hh"

SectorTags::SectorTags(const SectorTagsParams *p)
    : BaseTags(p), assoc(p->assoc), allocAssoc(p->assoc),
      sequentialAccess(p->sequential_access),
      replacementPolicy(p->replacement_policy),
      numBlocksPerSector(p->num_blocks_per_sector),
      numSectors(numBlocks / p->num_blocks_per_sector),
      numSets(numSectors / p->assoc),
      blks(numBlocks), secBlks(numSectors), sets(numSets),
      sectorShift(floorLog2(blkSize)),
      setShift(sectorShift + floorLog2(numBlocksPerSector)),
      tagShift(setShift + floorLog2(numSets)),
      sectorMask(numBlocksPerSector - 1), setMask(numSets - 1)
{
    // Check parameters
    fatal_if(blkSize < 4 || !isPowerOf2(blkSize),
             "Block size must be at least 4 and a power of 2");
    fatal_if(!isPowerOf2(numSets),
             "# of sets must be non-zero and a power of 2");
    fatal_if(!isPowerOf2(numBlocksPerSector),
             "# of blocks per sector must be non-zero and a power of 2");
    fatal_if(assoc <= 0, "associativity must be greater than zero");

    // Initialize all sets
    unsigned sec_blk_index = 0;   // index into sector blks array
    unsigned blk_index = 0;       // index into blks array
    for (unsigned i = 0; i < numSets; ++i) {
        sets[i].resize(assoc);

        // Initialize all sectors in this set
        for (unsigned j = 0; j < assoc; ++j) {
            // Select block within the set to be linked
            SectorBlk*& sec_blk = sets[i][j];

            // Locate next cache sector
            sec_blk = &secBlks[sec_blk_index];

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

                // Set its set, way and sector offset
                blk->set = i;
                blk->way = j;
                blk->setSectorOffset(k);

                // Update block index
                ++blk_index;
            }

            // Update sector block index
            ++sec_blk_index;
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

    if (blk != nullptr) {
        // If a cache hit
        lat = accessLatency;
        // Check if the block to be accessed is available. If not,
        // apply the accessLatency on top of block->whenReady.
        if (blk->whenReady > curTick() &&
            cache->ticksToCycles(blk->whenReady - curTick()) >
            accessLatency) {
            lat = cache->ticksToCycles(blk->whenReady - curTick()) +
            accessLatency;
        }

        // Update number of references to accessed block
        blk->refCount++;

        // Get block's sector
        SectorSubBlk* sub_blk = static_cast<SectorSubBlk*>(blk);
        const SectorBlk* sector_blk = sub_blk->getSectorBlock();

        // Update replacement data of accessed block, which is shared with
        // the whole sector it belongs to
        replacementPolicy->touch(sector_blk->replacementData);
    } else {
        // If a cache miss
        lat = lookupLatency;
    }

    return blk;
}

const std::vector<SectorBlk*>
SectorTags::getPossibleLocations(Addr addr) const
{
    return sets[extractSet(addr)];
}

void
SectorTags::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    // Insert block
    BaseTags::insertBlock(pkt, blk);

    // Get block's sector
    SectorSubBlk* sub_blk = static_cast<SectorSubBlk*>(blk);
    const SectorBlk* sector_blk = sub_blk->getSectorBlock();

    // When a block is inserted, the tag is only a newly used tag if the
    // sector was not previously present in the cache.
    // This assumes BaseTags::insertBlock does not set the valid bit.
    if (sector_blk->isValid()) {
        // An existing entry's replacement data is just updated
        replacementPolicy->touch(sector_blk->replacementData);
    } else {
        // Increment tag counter
        tagsInUse++;

        // A new entry resets the replacement data
        replacementPolicy->reset(sector_blk->replacementData);
    }
}

CacheBlk*
SectorTags::findBlock(Addr addr, bool is_secure) const
{
    // Extract sector tag
    const Addr tag = extractTag(addr);

    // The address can only be mapped to a specific location of a sector
    // due to sectors being composed of contiguous-address entries
    const Addr offset = extractSectorOffset(addr);

    // Find all possible sector locations for the given address
    const std::vector<SectorBlk*> locations = getPossibleLocations(addr);

    // Search for block
    for (const auto& sector : locations) {
        auto blk = sector->blks[offset];
        if (blk->getTag() == tag && blk->isValid() &&
            blk->isSecure() == is_secure) {
            return blk;
        }
    }

    // Did not find block
    return nullptr;
}

ReplaceableEntry*
SectorTags::findBlockBySetAndWay(int set, int way) const
{
    return sets[set][way];
}

CacheBlk*
SectorTags::findVictim(Addr addr, const bool is_secure,
                       std::vector<CacheBlk*>& evict_blks) const
{
    // Get all possible locations of this sector
    const std::vector<SectorBlk*> sector_locations =
        getPossibleLocations(addr);

    // Check if the sector this address belongs to has been allocated
    Addr tag = extractTag(addr);
    SectorBlk* victim_sector = nullptr;
    for (const auto& sector : sector_locations){
        if ((tag == sector->getTag()) && sector->isValid() &&
            (is_secure == sector->isSecure())){
            victim_sector = sector;
            break;
        }
    }

    // If the sector is not present
    if (victim_sector == nullptr){
        // Choose replacement victim from replacement candidates
        victim_sector = static_cast<SectorBlk*>(replacementPolicy->getVictim(
                          std::vector<ReplaceableEntry*>(
                          sector_locations.begin(), sector_locations.end())));
    }

    // Get the location of the victim block within the sector
    CacheBlk* victim = victim_sector->blks[extractSectorOffset(addr)];

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

    SectorSubBlk* victim_cast = static_cast<SectorSubBlk*>(victim);
    DPRINTF(CacheRepl, "set %x, way %x, sector offset %x: %s\n",
            "selecting blk for replacement\n", victim->set, victim->way,
            victim_cast->getSectorOffset());

    return victim;
}

Addr
SectorTags::extractTag(Addr addr) const
{
    return addr >> tagShift;
}

int
SectorTags::extractSet(Addr addr) const
{
    return (addr >> setShift) & setMask;
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
    return ((blk_cast->getTag() << tagShift) | ((Addr)blk->set << setShift) |
            ((Addr)blk_cast->getSectorOffset() << sectorShift));
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
    return new SectorTags(this);
}
