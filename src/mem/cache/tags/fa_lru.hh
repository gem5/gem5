/*
 * Copyright (c) 2012-2013 ARM Limited
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
 */

/**
 * @file
 * Declaration of a fully associative LRU tag store.
 */

#ifndef __MEM_CACHE_TAGS_FA_LRU_HH__
#define __MEM_CACHE_TAGS_FA_LRU_HH__

#include <list>
#include <unordered_map>

#include "mem/cache/blk.hh"
#include "mem/cache/tags/base.hh"
#include "mem/packet.hh"
#include "params/FALRU.hh"

/**
 * A fully associative cache block.
 */
class FALRUBlk : public CacheBlk
{
public:
    /** The previous block in LRU order. */
    FALRUBlk *prev;
    /** The next block in LRU order. */
    FALRUBlk *next;
    /** Has this block been touched? */
    bool isTouched;

    /**
     * A bit mask of the sizes of cache that this block is resident in.
     * Each bit represents a power of 2 in MB size cache.
     * If bit 0 is set, this block is in a 1MB cache
     * If bit 2 is set, this block is in a 4MB cache, etc.
     * There is one bit for each cache smaller than the full size (default
     * 16MB).
     */
    int inCache;
};

/**
 * A fully associative LRU cache. Keeps statistics for accesses to a number of
 * cache sizes at once.
 */
class FALRU : public BaseTags
{
  public:
    /** Typedef the block type used in this class. */
    typedef FALRUBlk BlkType;
    /** Typedef a list of pointers to the local block type. */
    typedef std::list<FALRUBlk*> BlkList;

  protected:
    /** Array of pointers to blocks at the cache size  boundaries. */
    FALRUBlk **cacheBoundaries;
    /** A mask for the FALRUBlk::inCache bits. */
    int cacheMask;
    /** The number of different size caches being tracked. */
    unsigned numCaches;

    /** The cache blocks. */
    FALRUBlk *blks;

    /** The MRU block. */
    FALRUBlk *head;
    /** The LRU block. */
    FALRUBlk *tail;

    /** Hash table type mapping addresses to cache block pointers. */
    typedef std::unordered_map<Addr, FALRUBlk *, std::hash<Addr> > hash_t;
    /** Iterator into the address hash table. */
    typedef hash_t::const_iterator tagIterator;

    /** The address hash table. */
    hash_t tagHash;

    /**
     * Find the cache block for the given address.
     * @param addr The address to find.
     * @return The cache block of the address, if any.
     */
    FALRUBlk * hashLookup(Addr addr) const;

    /**
     * Move a cache block to the MRU position.
     * @param blk The block to promote.
     */
    void moveToHead(FALRUBlk *blk);

    /**
     * Check to make sure all the cache boundaries are still where they should
     * be. Used for debugging.
     * @return True if everything is correct.
     */
    bool check();

    /**
     * @defgroup FALRUStats Fully Associative LRU specific statistics
     * The FA lru stack lets us track multiple cache sizes at once. These
     * statistics track the hits and misses for different cache sizes.
     * @{
     */

    /** Hits in each cache size >= 128K. */
    Stats::Vector hits;
    /** Misses in each cache size >= 128K. */
    Stats::Vector misses;
    /** Total number of accesses. */
    Stats::Scalar accesses;

    /**
     * @}
     */

public:

    typedef FALRUParams Params;

    /**
     * Construct and initialize this cache tagstore.
     */
    FALRU(const Params *p);
    ~FALRU();

    /**
     * Register the stats for this object.
     * @param name The name to prepend to the stats name.
     */
    void regStats() override;

    /**
     * Invalidate a cache block.
     * @param blk The block to invalidate.
     */
    void invalidate(CacheBlk *blk) override;

    /**
     * Access block and update replacement data.  May not succeed, in which
     * case nullptr pointer is returned.  This has all the implications of a
     * cache access and should only be used as such.
     * Returns the access latency and inCache flags as a side effect.
     * @param addr The address to look for.
     * @param is_secure True if the target memory space is secure.
     * @param asid The address space ID.
     * @param lat The latency of the access.
     * @param inCache The FALRUBlk::inCache flags.
     * @return Pointer to the cache block.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat,
                          int context_src, int *inCache);

    /**
     * Just a wrapper of above function to conform with the base interface.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat,
                          int context_src) override;

    /**
     * Find the block in the cache, do not update the replacement data.
     * @param addr The address to look for.
     * @param is_secure True if the target memory space is secure.
     * @param asid The address space ID.
     * @return Pointer to the cache block.
     */
    CacheBlk* findBlock(Addr addr, bool is_secure) const override;

    /**
     * Find a replacement block for the address provided.
     * @param pkt The request to a find a replacement candidate for.
     * @return The block to place the replacement in.
     */
    CacheBlk* findVictim(Addr addr) override;

    void insertBlock(PacketPtr pkt, CacheBlk *blk) override;

    /**
     * Return the block size of this cache.
     * @return The block size.
     */
    unsigned
    getBlockSize() const
    {
        return blkSize;
    }

    /**
     * Return the subblock size of this cache, always the block size.
     * @return The block size.
     */
    unsigned
    getSubBlockSize() const
    {
        return blkSize;
    }

    /**
     * Return the number of sets this cache has
     * @return The number of sets.
     */
    unsigned
    getNumSets() const override
    {
        return 1;
    }

    /**
     * Return the number of ways this cache has
     * @return The number of ways.
     */
    unsigned
    getNumWays() const override
    {
        return numBlocks;
    }

    /**
     * Find the cache block given set and way
     * @param set The set of the block.
     * @param way The way of the block.
     * @return The cache block.
     */
    CacheBlk* findBlockBySetAndWay(int set, int way) const override;

    /**
     * Align an address to the block size.
     * @param addr the address to align.
     * @return The aligned address.
     */
    Addr blkAlign(Addr addr) const
    {
        return (addr & ~(Addr)(blkSize-1));
    }

    /**
     * Generate the tag from the addres. For fully associative this is just the
     * block address.
     * @param addr The address to get the tag from.
     * @return The tag.
     */
    Addr extractTag(Addr addr) const override
    {
        return blkAlign(addr);
    }

    /**
     * Return the set of an address. Only one set in a fully associative cache.
     * @param addr The address to get the set from.
     * @return 0.
     */
    int extractSet(Addr addr) const override
    {
        return 0;
    }

    /**
     * Regenerate the block address from the tag and the set.
     * @param tag The tag of the block.
     * @param set The set the block belongs to.
     * @return the block address.
     */
    Addr regenerateBlkAddr(Addr tag, unsigned set) const override
    {
        return (tag);
    }

    /**
     * @todo Implement as in lru. Currently not used
     */
    virtual std::string print() const override { return ""; }

    /**
     * Visit each block in the tag store and apply a visitor to the
     * block.
     *
     * The visitor should be a function (or object that behaves like a
     * function) that takes a cache block reference as its parameter
     * and returns a bool. A visitor can request the traversal to be
     * stopped by returning false, returning true causes it to be
     * called for the next block in the tag store.
     *
     * \param visitor Visitor to call on each block.
     */
    void forEachBlk(CacheBlkVisitor &visitor) override {
        for (int i = 0; i < numBlocks; i++) {
            if (!visitor(blks[i]))
                return;
        }
    }

};

#endif // __MEM_CACHE_TAGS_FA_LRU_HH__
