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
 * Declaration of a sector tag store.
 */

#ifndef __MEM_CACHE_TAGS_SECTOR_TAGS_HH__
#define __MEM_CACHE_TAGS_SECTOR_TAGS_HH__

#include <cstdint>
#include <string>
#include <vector>

#include "base/statistics.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/sector_blk.hh"
#include "mem/packet.hh"
#include "params/SectorTags.hh"

namespace gem5
{

class BaseReplacementPolicy;
class ReplaceableEntry;

/**
 * A SectorTags cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The SectorTags placement policy divides the cache into s sectors of w
 * consecutive sectors (ways). Each sector then consists of a number of
 * sequential cache lines that may or may not be present.
 */
class SectorTags : public BaseTags
{
  private:
    /** The cache blocks. */
    std::vector<SectorSubBlk> blks;
    /** The cache sector blocks. */
    std::vector<SectorBlk> secBlks;

  protected:
    /** The allocatable associativity of the cache (alloc mask). */
    unsigned allocAssoc;

    /** Whether tags and data are accessed sequentially. */
    const bool sequentialAccess;

    /** Replacement policy */
    BaseReplacementPolicy *replacementPolicy;

    /** Number of data blocks per sector. */
    const unsigned numBlocksPerSector;

    /** The number of sectors in the cache. */
    const unsigned numSectors;

    // Organization of an address:
    // Tag | Placement Location | Sector Offset # | Offset #
    /** The amount to shift the address to get the sector tag. */
    const int sectorShift;

    /** Mask out all bits that aren't part of the sector tag. */
    const unsigned sectorMask;

    struct SectorTagsStats : public statistics::Group
    {
        const SectorTags& tags;

        SectorTagsStats(BaseTagStats &base_group, SectorTags& _tags);

        void regStats() override;

        /** Number of sub-blocks evicted due to a replacement. */
        statistics::Vector evictionsReplacement;
    } sectorStats;

  public:
    /** Convenience typedef. */
     typedef SectorTagsParams Params;

    /**
     * Construct and initialize this tag store.
     */
    SectorTags(const Params &p);

    /**
     * Destructor.
     */
    virtual ~SectorTags() {};

    /**
     * Initialize blocks as SectorBlk and SectorSubBlk instances.
     */
    void tagsInit() override;

    /**
     * This function updates the tags when a block is invalidated but does
     * not invalidate the block itself. It also updates the replacement data.
     *
     * @param blk The block to invalidate.
     */
    void invalidate(CacheBlk *blk) override;

    /**
     * Access block and update replacement data. May not succeed, in which
     * case nullptr is returned. This has all the implications of a cache
     * access and should only be used as such. Returns the tag lookup latency
     * as a side effect.
     *
     * @param pkt The packet holding the address to find.
     * @param lat The latency of the tag lookup.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* accessBlock(const PacketPtr pkt, Cycles &lat) override;

    /**
     * Insert the new block into the cache and update replacement data.
     *
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
    void insertBlock(const PacketPtr pkt, CacheBlk *blk) override;

    void moveBlock(CacheBlk *src_blk, CacheBlk *dest_blk) override;

    /**
     * Finds the given address in the cache, do not update replacement data.
     * i.e. This is a no-side-effect find of a block.
     *
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* findBlock(Addr addr, bool is_secure) const override;

    /**
     * Find replacement victim based on address.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param size Size, in bits, of new block to allocate.
     * @param evict_blks Cache blocks to be evicted.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         const std::size_t size,
                         std::vector<CacheBlk*>& evict_blks) override;

    /**
     * Calculate a block's offset in a sector from the address.
     *
     * @param addr The address to get the offset from.
     * @return Offset of the corresponding block within its sector.
     */
    int extractSectorOffset(Addr addr) const;

    /**
     * Regenerate the block address from the tag and location.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override;

    /**
     * Find if any of the sub-blocks satisfies a condition.
     *
     * The visitor should be a std::function that takes a cache block
     * reference as its parameter. The visitor will terminate the
     * traversal early if the condition is satisfied.
     *
     * @param visitor Visitor to call on each block.
     */
    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override;
};

} // namespace gem5

#endif //__MEM_CACHE_TAGS_SECTOR_TAGS_HH__
