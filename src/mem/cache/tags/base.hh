/*
 * Copyright (c) 2012-2014, 2016-2019, 2023-2024 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Declaration of a common base class for cache tagstore objects.
 */

#ifndef __MEM_CACHE_TAGS_BASE_HH__
#define __MEM_CACHE_TAGS_BASE_HH__

#include <cassert>
#include <cstdint>
#include <functional>
#include <string>

#include "base/callback.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/packet.hh"
#include "params/BaseTags.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class System;
class IndexingPolicy;
class ReplaceableEntry;

/**
 * A common base class of Cache tagstore objects.
 */
class BaseTags : public ClockedObject
{
  protected:
    /** The block size of the cache. */
    const unsigned blkSize;
    /** Mask out all bits that aren't part of the block offset. */
    const Addr blkMask;
    /** The size of the cache. */
    const unsigned size;
    /** The tag lookup latency of the cache. */
    const Cycles lookupLatency;

    /** System we are currently operating in. */
    System *system;

    /** Indexing policy */
    BaseIndexingPolicy *indexingPolicy;

    /** Partitioning manager */
    partitioning_policy::PartitionManager *partitionManager;

    /**
     * The number of tags that need to be touched to meet the warmup
     * percentage.
     */
    const unsigned warmupBound;
    /** Marked true when the cache is warmed up. */
    bool warmedUp;

    /** the number of blocks in the cache */
    const unsigned numBlocks;

    /** The data blocks, 1 per cache block. */
    std::unique_ptr<uint8_t[]> dataBlks;

    /**
     * TODO: It would be good if these stats were acquired after warmup.
     */
    struct BaseTagStats : public statistics::Group
    {
        BaseTagStats(BaseTags &tags);

        void regStats() override;
        void preDumpStats() override;

        BaseTags &tags;

        /** Per tick average of the number of tags that hold valid data. */
        statistics::Average tagsInUse;

        /** The total number of references to a block before it is replaced. */
        statistics::Scalar totalRefs;

        /**
         * The number of reference counts sampled. This is different
         * from replacements because we sample all the valid blocks
         * when the simulator exits.
         */
        statistics::Scalar sampledRefs;

        /**
         * Average number of references to a block before is was replaced.
         * @todo This should change to an average stat once we have them.
         */
        statistics::Formula avgRefs;

        /** The tick that the warmup percentage was hit. 0 on failure. */
        statistics::Scalar warmupTick;

        /** Average occupancy of each requestor using the cache */
        statistics::AverageVector occupancies;

        /** Average occ % of each requestor using the cache */
        statistics::Formula avgOccs;

        /** Occupancy of each context/cpu using the cache */
        statistics::Vector occupanciesTaskId;

        /** Occupancy of each context/cpu using the cache */
        statistics::Vector2d ageTaskId;

        /** Occ ratio of each context/cpu using the cache */
        statistics::Formula ratioOccsTaskId;

        /** Number of tags consulted over all accesses. */
        statistics::Scalar tagAccesses;
        /** Number of data blocks consulted over all accesses. */
        statistics::Scalar dataAccesses;
    } stats;

  public:
    typedef BaseTagsParams Params;
    BaseTags(const Params &p);

    /**
     * Destructor.
     */
    virtual ~BaseTags() {}

    /**
     * Initialize blocks. Must be overriden by every subclass that uses
     * a block type different from its parent's, as the current Python
     * code generation does not allow templates.
     */
    virtual void tagsInit() = 0;

    /**
     * Average in the reference count for valid blocks when the simulation
     * exits.
     */
    void cleanupRefs();

    /**
     * Computes stats just prior to dump event
     */
    void computeStats();

    /**
     * Print all tags used
     */
    std::string print();

    /**
     * Finds the block in the cache without touching it.
     *
     * @param addr The address to look for.
     * @param is_secure True if the target memory space is secure.
     * @return Pointer to the cache block.
     */
    virtual CacheBlk *findBlock(Addr addr, bool is_secure) const;

    /**
     * Find a block given set and way.
     *
     * @param set The set of the block.
     * @param way The way of the block.
     * @return The block.
     */
    virtual ReplaceableEntry *findBlockBySetAndWay(int set, int way) const;

    /**
     * Align an address to the block size.
     * @param addr the address to align.
     * @return The block address.
     */
    Addr
    blkAlign(Addr addr) const
    {
        return addr & ~blkMask;
    }

    /**
     * Calculate the block offset of an address.
     * @param addr the address to get the offset of.
     * @return the block offset.
     */
    int
    extractBlkOffset(Addr addr) const
    {
        return (addr & blkMask);
    }

    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void
    setWayAllocationMax(int ways)
    {
        panic("This tag class does not implement way allocation limit!\n");
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int
    getWayAllocationMax() const
    {
        panic("This tag class does not implement way allocation limit!\n");
        return -1;
    }

    /**
     * This function updates the tags when a block is invalidated
     *
     * @param blk A valid block to invalidate.
     */
    virtual void
    invalidate(CacheBlk *blk)
    {
        assert(blk);
        assert(blk->isValid());

        stats.occupancies[blk->getSrcRequestorId()]--;
        stats.totalRefs += blk->getRefCount();
        stats.sampledRefs++;

        blk->invalidate();
    }

    /**
     * Find replacement victim based on address. If the address requires
     * blocks to be evicted, their locations are listed for eviction. If a
     * conventional cache is being used, the list only contains the victim.
     * However, if using sector or compressed caches, the victim is one of
     * the blocks to be evicted, but its location is the only one that will
     * be assigned to the newly allocated block associated to this address.
     * @sa insertBlock
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param size Size, in bits, of new block to allocate.
     * @param evict_blks Cache blocks to be evicted.
     * @param partition_id Partition ID for resource management.
     * @return Cache block to be replaced.
     */
    virtual CacheBlk *findVictim(Addr addr, const bool is_secure,
                                 const std::size_t size,
                                 std::vector<CacheBlk *> &evict_blks,
                                 const uint64_t partition_id = 0) = 0;

    /**
     * Access block and update replacement data. May not succeed, in which case
     * nullptr is returned. This has all the implications of a cache access and
     * should only be used as such. Returns the tag lookup latency as a side
     * effect.
     *
     * @param pkt The packet holding the address to find.
     * @param lat The latency of the tag lookup.
     * @return Pointer to the cache block if found.
     */
    virtual CacheBlk *accessBlock(const PacketPtr pkt, Cycles &lat) = 0;

    /**
     * Generate the tag from the given address.
     *
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    virtual Addr extractTag(const Addr addr) const;

    /**
     * Insert the new block into the cache and update stats.
     *
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
    virtual void insertBlock(const PacketPtr pkt, CacheBlk *blk);

    /**
     * Move a block's metadata to another location decided by the replacement
     * policy. It behaves as a swap, however, since the destination block
     * should be invalid, the result is a move.
     *
     * @param src_blk The source block.
     * @param dest_blk The destination block. Must be invalid.
     */
    virtual void moveBlock(CacheBlk *src_blk, CacheBlk *dest_blk);

    /**
     * Regenerate the block address.
     *
     * @param block The block.
     * @return the block address.
     */
    virtual Addr regenerateBlkAddr(const CacheBlk *blk) const = 0;

    /**
     * Visit each block in the tags and apply a visitor
     *
     * The visitor should be a std::function that takes a cache block
     * reference as its parameter.
     *
     * @param visitor Visitor to call on each block.
     */
    void forEachBlk(std::function<void(CacheBlk &)> visitor);

    /**
     * Find if any of the blocks satisfies a condition
     *
     * The visitor should be a std::function that takes a cache block
     * reference as its parameter. The visitor will terminate the
     * traversal early if the condition is satisfied.
     *
     * @param visitor Visitor to call on each block.
     */
    virtual bool anyBlk(std::function<bool(CacheBlk &)> visitor) = 0;

  private:
    /**
     * Update the reference stats using data from the input block
     *
     * @param blk The input block
     */
    void cleanupRefsVisitor(CacheBlk &blk);

    /**
     * Update the occupancy and age stats using data from the input block
     *
     * @param blk The input block
     */
    void computeStatsVisitor(CacheBlk &blk);
};

} // namespace gem5

#endif //__MEM_CACHE_TAGS_BASE_HH__
