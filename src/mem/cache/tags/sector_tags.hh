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
 * Declaration of a sector set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_SECTOR_TAGS_HH__
#define __MEM_CACHE_TAGS_SECTOR_TAGS_HH__

#include <string>
#include <vector>

#include "mem/cache/sector_blk.hh"
#include "mem/cache/tags/base.hh"
#include "mem/packet.hh"
#include "params/SectorTags.hh"

class BaseReplacementPolicy;

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
  protected:
    /** Typedef the set type used in this tag store. */
    typedef std::vector<SectorBlk*> SetType;

    /** The associativity of the cache. */
    const unsigned assoc;
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
    /** The number of sets in the cache. */
    const unsigned numSets;

    /** The cache blocks. */
    std::vector<SectorSubBlk> blks;
    /** The cache sector blocks. */
    std::vector<SectorBlk> secBlks;
    /** The cache sets. */
    std::vector<SetType> sets;

    // Organization of an address: Tag | Set # | Sector Offset # | Offset #
    /** The amount to shift the address to get the sector tag. */
    const int sectorShift;
    /** The amount to shift the address to get the set. */
    const int setShift;
    /** The amount to shift the address to get the tag. */
    const int tagShift;

    /** Mask out all bits that aren't part of the sector tag. */
    const unsigned sectorMask;
    /** Mask out all bits that aren't part of the set index. */
    const unsigned setMask;

  public:
    /** Convenience typedef. */
     typedef SectorTagsParams Params;

    /**
     * Construct and initialize this tag store.
     */
    SectorTags(const Params *p);

    /**
     * Destructor.
     */
    virtual ~SectorTags() {};

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
     * access and should only be used as such. Returns the access latency
     * as a side effect.
     *
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param lat The access latency.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat) override;

    /**
     * Find all possible block locations for insertion and replacement of
     * an address. Should be called immediately before ReplacementPolicy's
     * findVictim() not to break cache resizing.
     * Returns sector blocks in all ways belonging to the set of the address.
     *
     * @param addr The addr to a find possible locations for.
     * @return The possible locations.
     */
    virtual const std::vector<SectorBlk*> getPossibleLocations(Addr addr)
                                                                   const;

    /**
     * Insert the new block into the cache and update replacement data.
     *
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
    void insertBlock(const PacketPtr pkt, CacheBlk *blk) override;

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
     * Find a sector block given set and way.
     *
     * @param set The set of the block.
     * @param way The way of the block.
     * @return The block.
     */
    ReplaceableEntry* findBlockBySetAndWay(int set, int way) const override;

    /**
     * Find replacement victim based on address.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param evict_blks Cache blocks to be evicted.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const override;

    /**
     * Generate the sector tag from the given address.
     *
     * @param addr The address to get the sector tag from.
     * @return The sector tag of the address.
     */
    Addr extractTag(Addr addr) const override;

    /**
     * Calculate the set index from the address.
     *
     * @param addr The address to get the set from.
     * @return The set index of the address.
     */
    int extractSet(Addr addr) const;

    /**
     * Calculate a block's offset in a sector from the address.
     *
     * @param addr The address to get the offset from.
     * @return Offset of the corresponding block within its sector.
     */
    int extractSectorOffset(Addr addr) const;

    /**
     * Regenerate the block address from the tag and set.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override;

    /**
     * Visit each sub-block in the tags and apply a visitor.
     *
     * The visitor should be a std::function that takes a cache block.
     * reference as its parameter.
     *
     * @param visitor Visitor to call on each block.
     */
    void forEachBlk(std::function<void(CacheBlk &)> visitor) override;

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

#endif //__MEM_CACHE_TAGS_SECTOR_TAGS_HH__
