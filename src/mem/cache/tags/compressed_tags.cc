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
 */

/**
 * @file
 * Definitions of a base set associative compressed superblocks tag store.
 */

#include "mem/cache/tags/compressed_tags.hh"

#include "base/trace.hh"
#include "debug/CacheComp.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/packet.hh"
#include "params/CompressedTags.hh"

namespace gem5
{

CompressedTags::CompressedTags(const Params &p)
    : SectorTags(p)
{
}

void
CompressedTags::tagsInit()
{
    // Create blocks and superblocks
    blks = std::vector<CompressionBlk>(numBlocks);
    superBlks = std::vector<SuperBlk>(numSectors);

    // Initialize all blocks
    unsigned blk_index = 0;          // index into blks array
    for (unsigned superblock_index = 0; superblock_index < numSectors;
         superblock_index++)
    {
        // Locate next cache superblock
        SuperBlk* superblock = &superBlks[superblock_index];

        // Superblocks must be aware of the block size due to their co-
        // allocation conditions
        superblock->setBlkSize(blkSize);

        // Associate a replacement data entry to the block
        superblock->replacementData = replacementPolicy->instantiateEntry();

        // Initialize all blocks in this superblock
        superblock->blks.resize(numBlocksPerSector, nullptr);
        for (unsigned k = 0; k < numBlocksPerSector; ++k){
            // Select block within the set to be linked
            SectorSubBlk*& blk = superblock->blks[k];

            // Locate next cache block
            blk = &blks[blk_index];

            // Associate a data chunk to the block
            blk->data = &dataBlks[blkSize*blk_index];

            // Associate superblock to this block
            blk->setSectorBlock(superblock);

            // Associate the superblock replacement data to this block
            blk->replacementData = superblock->replacementData;

            // Set its index and sector offset
            blk->setSectorOffset(k);

            // Update block index
            ++blk_index;
        }

        // Link block to indexing policy
        indexingPolicy->setEntry(superblock, superblock_index);
    }
}

CacheBlk*
CompressedTags::findVictim(Addr addr, const bool is_secure,
                           const std::size_t compressed_size,
                           std::vector<CacheBlk*>& evict_blks)
{
    // Get all possible locations of this superblock
    const std::vector<ReplaceableEntry*> superblock_entries =
        indexingPolicy->getPossibleEntries(addr);

    // Check if the superblock this address belongs to has been allocated. If
    // so, try co-allocating
    Addr tag = extractTag(addr);
    SuperBlk* victim_superblock = nullptr;
    bool is_co_allocation = false;
    const uint64_t offset = extractSectorOffset(addr);
    for (const auto& entry : superblock_entries){
        SuperBlk* superblock = static_cast<SuperBlk*>(entry);
        if (superblock->matchTag(tag, is_secure) &&
            !superblock->blks[offset]->isValid() &&
            superblock->isCompressed() &&
            superblock->canCoAllocate(compressed_size))
        {
            victim_superblock = superblock;
            is_co_allocation = true;
            break;
        }
    }

    // If the superblock is not present or cannot be co-allocated a
    // superblock must be replaced
    if (victim_superblock == nullptr){
        // Choose replacement victim from replacement candidates
        victim_superblock = static_cast<SuperBlk*>(
            replacementPolicy->getVictim(superblock_entries));

        // The whole superblock must be evicted to make room for the new one
        for (const auto& blk : victim_superblock->blks){
            if (blk->isValid()) {
                evict_blks.push_back(blk);
            }
        }
    }

    // Get the location of the victim block within the superblock
    SectorSubBlk* victim = victim_superblock->blks[offset];

    // It would be a hit if victim was valid in a co-allocation, and upgrades
    // do not call findVictim, so it cannot happen
    if (is_co_allocation){
        assert(!victim->isValid());

        // Print all co-allocated blocks
        DPRINTF(CacheComp, "Co-Allocation: offset %d of %s\n", offset,
                victim_superblock->print());
    }

    // Update number of sub-blocks evicted due to a replacement
    sectorStats.evictionsReplacement[evict_blks.size()]++;

    return victim;
}

bool
CompressedTags::anyBlk(std::function<bool(CacheBlk &)> visitor)
{
    for (CompressionBlk& blk : blks) {
        if (visitor(blk)) {
            return true;
        }
    }
    return false;
}

} // namespace gem5
