/*
 * Copyright (c) 2012-2014,2016 ARM Limited
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
 *
 * Authors: Erik Hallnor
 *          Ron Dreslinski
 */

/**
 * @file
 * Declaration of a common base class for cache tagstore objects.
 */

#ifndef __BASE_TAGS_HH__
#define __BASE_TAGS_HH__

#include <string>

#include "base/callback.hh"
#include "base/statistics.hh"
#include "mem/cache/blk.hh"
#include "params/BaseTags.hh"
#include "sim/clocked_object.hh"

class BaseCache;

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
    /**
     * The total access latency of the cache. This latency
     * is different depending on the cache access mode
     * (parallel or sequential)
     */
    const Cycles accessLatency;
    /** Pointer to the parent cache. */
    BaseCache *cache;

    /**
     * The number of tags that need to be touched to meet the warmup
     * percentage.
     */
    int warmupBound;
    /** Marked true when the cache is warmed up. */
    bool warmedUp;

    /** the number of blocks in the cache */
    unsigned numBlocks;

    // Statistics
    /**
     * @addtogroup CacheStatistics
     * @{
     */

    /** Number of replacements of valid blocks per thread. */
    Stats::Vector replacements;
    /** Per cycle average of the number of tags that hold valid data. */
    Stats::Average tagsInUse;

    /** The total number of references to a block before it is replaced. */
    Stats::Scalar totalRefs;

    /**
     * The number of reference counts sampled. This is different from
     * replacements because we sample all the valid blocks when the simulator
     * exits.
     */
    Stats::Scalar sampledRefs;

    /**
     * Average number of references to a block before is was replaced.
     * @todo This should change to an average stat once we have them.
     */
    Stats::Formula avgRefs;

    /** The cycle that the warmup percentage was hit. */
    Stats::Scalar warmupCycle;

    /** Average occupancy of each requestor using the cache */
    Stats::AverageVector occupancies;

    /** Average occ % of each requestor using the cache */
    Stats::Formula avgOccs;

    /** Occupancy of each context/cpu using the cache */
    Stats::Vector occupanciesTaskId;

    /** Occupancy of each context/cpu using the cache */
    Stats::Vector2d ageTaskId;

    /** Occ % of each context/cpu using the cache */
    Stats::Formula percentOccsTaskId;

    /** Number of tags consulted over all accesses. */
    Stats::Scalar tagAccesses;
    /** Number of data blocks consulted over all accesses. */
    Stats::Scalar dataAccesses;

    /**
     * @}
     */

  public:
    typedef BaseTagsParams Params;
    BaseTags(const Params *p);

    /**
     * Destructor.
     */
    virtual ~BaseTags() {}

    /**
     * Set the parent cache back pointer.
     * @param _cache Pointer to parent cache.
     */
    void setCache(BaseCache *_cache);

    /**
     * Register local statistics.
     */
    void regStats();

    /**
     * Average in the reference count for valid blocks when the simulation
     * exits.
     */
    virtual void cleanupRefs() {}

    /**
     * Computes stats just prior to dump event
     */
    virtual void computeStats() {}

    /**
     * Print all tags used
     */
    virtual std::string print() const = 0;

    /**
     * Find a block using the memory address
     */
    virtual CacheBlk * findBlock(Addr addr, bool is_secure) const = 0;

    /**
     * Align an address to the block size.
     * @param addr the address to align.
     * @return The block address.
     */
    Addr blkAlign(Addr addr) const
    {
        return addr & ~blkMask;
    }

    /**
     * Calculate the block offset of an address.
     * @param addr the address to get the offset of.
     * @return the block offset.
     */
    int extractBlkOffset(Addr addr) const
    {
        return (addr & blkMask);
    }

    /**
     * Find the cache block given set and way
     * @param set The set of the block.
     * @param way The way of the block.
     * @return The cache block.
     */
    virtual CacheBlk *findBlockBySetAndWay(int set, int way) const = 0;

    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void setWayAllocationMax(int ways)
    {
        panic("This tag class does not implement way allocation limit!\n");
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int getWayAllocationMax() const
    {
        panic("This tag class does not implement way allocation limit!\n");
        return -1;
    }

    virtual void invalidate(CacheBlk *blk) = 0;

    virtual CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat) = 0;

    virtual Addr extractTag(Addr addr) const = 0;

    virtual void insertBlock(PacketPtr pkt, CacheBlk *blk) = 0;

    virtual Addr regenerateBlkAddr(Addr tag, unsigned set) const = 0;

    virtual CacheBlk* findVictim(Addr addr) = 0;

    virtual int extractSet(Addr addr) const = 0;

    virtual void forEachBlk(CacheBlkVisitor &visitor) = 0;
};

class BaseTagsCallback : public Callback
{
    BaseTags *tags;
  public:
    BaseTagsCallback(BaseTags *t) : tags(t) {}
    virtual void process() { tags->cleanupRefs(); };
};

class BaseTagsDumpCallback : public Callback
{
    BaseTags *tags;
  public:
    BaseTagsDumpCallback(BaseTags *t) : tags(t) {}
    virtual void process() { tags->computeStats(); };
};

#endif //__BASE_TAGS_HH__
