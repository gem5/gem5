/*
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
 * Declaration of a LRU tag store.
 */

#ifndef __LRU_HH__
#define __LRU_HH__

#include <cstring>
#include <list>

#include "mem/cache/cache_blk.hh" // base class
#include "mem/packet.hh" // for inlined functions
#include <assert.h>
#include "mem/cache/tags/base_tags.hh"

class BaseCache;

/**
 * LRU cache block.
 */
class LRUBlk : public CacheBlk {
  public:
    /** Has this block been touched? Used to aid calculation of warmup time. */
    bool isTouched;
};

/**
 * An associative set of cache blocks.
 */
class CacheSet
{
  public:
    /** The associativity of this set. */
    int assoc;

    /** Cache blocks in this set, maintained in LRU order 0 = MRU. */
    LRUBlk **blks;

    /**
     * Find a block matching the tag in this set.
     * @param asid The address space ID.
     * @param tag The Tag to find.
     * @return Pointer to the block if found.
     */
    LRUBlk* findBlk(Addr tag) const;

    /**
     * Move the given block to the head of the list.
     * @param blk The block to move.
     */
    void moveToHead(LRUBlk *blk);
};

/**
 * A LRU cache tag store.
 */
class LRU : public BaseTags
{
  public:
    /** Typedef the block type used in this tag store. */
    typedef LRUBlk BlkType;
    /** Typedef for a list of pointers to the local block class. */
    typedef std::list<LRUBlk*> BlkList;
  protected:
    /** The number of sets in the cache. */
    const int numSets;
    /** The number of bytes in a block. */
    const int blkSize;
    /** The associativity of the cache. */
    const int assoc;
    /** The hit latency. */
    const int hitLatency;

    /** The cache sets. */
    CacheSet *sets;

    /** The cache blocks. */
    LRUBlk *blks;
    /** The data blocks, 1 per cache block. */
    uint8_t *dataBlks;

    /** The amount to shift the address to get the set. */
    int setShift;
    /** The amount to shift the address to get the tag. */
    int tagShift;
    /** Mask out all bits that aren't part of the set index. */
    unsigned setMask;
    /** Mask out all bits that aren't part of the block offset. */
    unsigned blkMask;

public:
    /**
     * Construct and initialize this tag store.
     * @param _numSets The number of sets in the cache.
     * @param _blkSize The number of bytes in a block.
     * @param _assoc The associativity of the cache.
     * @param _hit_latency The latency in cycles for a hit.
     */
    LRU(int _numSets, int _blkSize,	int _assoc, int _hit_latency);

    /**
     * Destructor
     */
    virtual ~LRU();

    /**
     * Return the block size.
     * @return the block size.
     */
    int getBlockSize()
    {
        return blkSize;
    }

    /**
     * Return the subblock size. In the case of LRU it is always the block
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
    LRUBlk* findBlock(Addr addr, int &lat);

    /**
     * Finds the given address in the cache, do not update replacement data.
     * @param addr The address to find.
     * @param asid The address space ID.
     * @return Pointer to the cache block if found.
     */
    LRUBlk* findBlock(Addr addr) const;

    /**
     * Find a replacement block for the address provided.
     * @param pkt The request to a find a replacement candidate for.
     * @param writebacks List for any writebacks to be performed.
     * @return The block to place the replacement in.
     */
    LRUBlk* findReplacement(Addr addr, PacketList &writebacks);

    /**
     * Generate the tag from the given address.
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    Addr extractTag(Addr addr) const
    {
        return (addr >> tagShift);
    }

    /**
     * Calculate the set index from the address.
     * @param addr The address to get the set from.
     * @return The set index of the address.
     */
    int extractSet(Addr addr) const
    {
        return ((addr >> setShift) & setMask);
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
        return (addr & ~(Addr)blkMask);
    }

    /**
     * Regenerate the block address from the tag.
     * @param tag The tag of the block.
     * @param set The set of the block.
     * @return The block address.
     */
    Addr regenerateBlkAddr(Addr tag, unsigned set) const
    {
        return ((tag << tagShift) | ((Addr)set << setShift));
    }

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
    void readData(LRUBlk *blk, uint8_t *data)
    {
        std::memcpy(data, blk->data, blk->size);
    }

    /**
     * Write data into the internal storage of the given cache block. Since in
     * LRU does not store data differently this just needs to update the size.
     * @param blk The cache block to write.
     * @param data The data to write.
     * @param size The number of bytes to write.
     * @param writebacks A list for any writebacks to be performed. May be
     * needed when writing to a compressed block.
     */
    void writeData(LRUBlk *blk, uint8_t *data, int size,
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
