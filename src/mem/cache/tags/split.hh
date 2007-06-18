/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Lisa Hsu
 */

/**
 * @file
 * Declaration of a split/partitioned tag store.
 */

#ifndef __SPLIT_HH__
#define __SPLIT_HH__

#include <cstring>
#include <list>

#include "mem/cache/cache_blk.hh" // base class
#include "mem/cache/tags/split_blk.hh"
#include "mem/packet.hh" // for inlined functions
#include <assert.h>
#include "mem/cache/tags/base_tags.hh"
#include "base/hashmap.hh"

class BaseCache;
class SplitLRU;
class SplitLIFO;

/**
 * A  cache tag store.
 */
class Split : public BaseTags
{
  public:
    /** Typedef the block type used in this tag store. */
    typedef SplitBlk BlkType;
    /** Typedef for a list of pointers to the local block class. */
    typedef std::list<SplitBlk*> BlkList;
  protected:
    /** The number of sets in the cache. */
    const int numSets;
    /** The number of bytes in a block. */
    const int blkSize;
    /** Whether the 2nd partition (for the nic) is LIFO or not */
    const bool lifo;
    /** The hit latency. */
    const int hitLatency;

    Addr blkMask;

    /** Number of NIC requests that hit in the NIC partition */
    Stats::Scalar<> NR_NP_hits;
    /** Number of NIC requests that hit in the CPU partition */
    Stats::Scalar<> NR_CP_hits;
    /** Number of CPU requests that hit in the NIC partition */
    Stats::Scalar<> CR_NP_hits;
    /** Number of CPU requests that hit in the CPU partition */
    Stats::Scalar<> CR_CP_hits;
    /** The number of nic replacements (i.e. misses) */
    Stats::Scalar<> nic_repl;
    /** The number of cpu replacements (i.e. misses) */
    Stats::Scalar<> cpu_repl;

    //For latency studies
    /** the number of NIC blks that were used before evicted */
    Stats::Scalar<> nicUsedWhenEvicted;
    /** the total latency of used NIC blocks in the cache */
    Stats::Scalar<> nicUsedTotLatency;
    /** the total number of used NIC blocks evicted */
    Stats::Scalar<> nicUsedTotEvicted;
    /** the average number of cycles a used NIC blk is in the cache */
    Stats::Formula nicUsedAvgLatency;
    /** the Distribution of used NIC blk eviction times */
    Stats::Distribution<> usedEvictDist;

    /** the number of NIC blks that were unused before evicted */
    Stats::Scalar<> nicUnusedWhenEvicted;
    /** the total latency of unused NIC blks in the cache */
    Stats::Scalar<> nicUnusedTotLatency;
    /** the total number of unused NIC blocks evicted */
    Stats::Scalar<> nicUnusedTotEvicted;
    /** the average number of cycles an unused NIC blk is in the cache */
    Stats::Formula nicUnusedAvgLatency;
    /** the Distribution of unused NIC blk eviction times */
    Stats::Distribution<> unusedEvictDist;

    /** The total latency of NIC blocks to 1st usage time by CPU */
    Stats::Scalar<> nicUseByCPUCycleTotal;
    /** The total number of NIC blocks used */
    Stats::Scalar<> nicBlksUsedByCPU;
    /** the average number of cycles before a NIC blk that is used gets used by CPU */
    Stats::Formula nicAvgUsageByCPULatency;
    /** the Distribution of cycles time before a NIC blk is used by CPU*/
    Stats::Distribution<> useByCPUCycleDist;

    /** the number of CPU blks that were used before evicted */
    Stats::Scalar<> cpuUsedBlks;
    /** the number of CPU blks that were unused before evicted */
    Stats::Scalar<> cpuUnusedBlks;

    /** the avg number of cycles before a NIC blk is evicted */
    Stats::Formula nicAvgLatency;

    typedef m5::hash_map<Addr, int, m5::hash<Addr> > hash_t;
    typedef hash_t::const_iterator memIter;
    hash_t memHash;


  private:
    SplitLRU *lru;
    SplitLRU *lru_net;
    SplitLIFO *lifo_net;

  public:
    /**
     * Construct and initialize this tag store.
     * @param _numSets The number of sets in the cache.
     * @param _blkSize The number of bytes in a block.
     * @param _assoc The associativity of the cache.
     * @param _hit_latency The latency in cycles for a hit.
     */
    Split(int _numSets, int _blkSize, int total_ways, int LRU1_assoc,
          bool _lifo, bool _two_queue, int _hit_latency);

    /**
     * Destructor
     */
    virtual ~Split();

    /**
     * Register the stats for this object
     * @param name The name to prepend to the stats name.
     */
    void regStats(const std::string &name);

    /**
     * Return the block size.
     * @return the block size.
     */
    int getBlockSize()
    {
        return blkSize;
    }

    /**
     * Return the subblock size. In the case of Split it is always the block
     * size.
     * @return The block size.
     */
    int getSubBlockSize()
    {
        return blkSize;
    }

    /**
     * Search for the address in the cache.
     * @param asid The address space ID.
     * @param addr The address to find.
     * @return True if the address is in the cache.
     */
    bool probe(Addr addr) const;

    /**
     * Invalidate the given block.
     * @param blk The block to invalidate.
     */
    void invalidateBlk(BlkType *blk);

    /**
     * Finds the given address in the cache and update replacement data.
     * Returns the access latency as a side effect.
     * @param addr The address to find.
     * @param asid The address space ID.
     * @param lat The access latency.
     * @return Pointer to the cache block if found.
     */
    SplitBlk* findBlock(Addr addr, int &lat);

    /**
     * Finds the given address in the cache, do not update replacement data.
     * @param addr The address to find.
     * @param asid The address space ID.
     * @return Pointer to the cache block if found.
     */
    SplitBlk* findBlock(Addr addr) const;

    /**
     * Find a replacement block for the address provided.
     * @param pkt The request to a find a replacement candidate for.
     * @param writebacks List for any writebacks to be performed.
     * @return The block to place the replacement in.
     */
    SplitBlk* findReplacement(Addr addr, PacketList &writebacks);


    /**
     * Generate the tag from the given address.
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    Addr extractTag(Addr addr) const;

    /**
     * Calculate the set index from the address.
     * @param addr The address to get the set from.
     * @return The set index of the address.
     */
    int extractSet(Addr addr) const
    {
        panic("should never call this!\n");
        M5_DUMMY_RETURN
    }

    /**
     * Get the block offset from an address.
     * @param addr The address to get the offset of.
     * @return The block offset.
     */
    int extractBlkOffset(Addr addr) const
    {
        return (addr & blkMask);
    }

    /**
     * Align an address to the block size.
     * @param addr the address to align.
     * @return The block address.
     */
    Addr blkAlign(Addr addr) const
    {
        return (addr & ~(Addr) (blkMask));
    }

    /**
     * Regenerate the block address from the tag.
     * @param tag The tag of the block.
     * @param set The set of the block.
     * @return The block address.
     */
    Addr regenerateBlkAddr(Addr tag, int set) const;

    /**
     * Return the hit latency.
     * @return the hit latency.
     */
    int getHitLatency() const
    {
        return hitLatency;
    }

    /**
     * Read the data out of the internal storage of the given cache block.
     * @param blk The cache block to read.
     * @param data The buffer to read the data into.
     * @return The cache block's data.
     */
    void readData(SplitBlk *blk, uint8_t *data)
    {
        std::memcpy(data, blk->data, blk->size);
    }

    /**
     * Write data into the internal storage of the given cache block. Since in
     * Split does not store data differently this just needs to update the size.
     * @param blk The cache block to write.
     * @param data The data to write.
     * @param size The number of bytes to write.
     * @param writebacks A list for any writebacks to be performed. May be
     * needed when writing to a compressed block.
     */
    void writeData(SplitBlk *blk, uint8_t *data, int size,
                   PacketList & writebacks)
    {
        assert(size <= blkSize);
        blk->size = size;
    }

    /**
     * Called at end of simulation to complete average block reference stats.
     */
    virtual void cleanupRefs();
};

#endif
