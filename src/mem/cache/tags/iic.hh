/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Declaration of the Indirect Index Cache (IIC) tags store.
 */

#ifndef __IIC_HH__
#define __IIC_HH__

#include <list>
#include <vector>

#include "base/statistics.hh"
#include "mem/cache/tags/iic_repl/repl.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/blk.hh"
#include "mem/packet.hh"

class BaseCache; // Forward declaration

/**
 * IIC cache blk.
 */
class IICTag : public CacheBlk
{
  public:
    /**
     * Copy the contents of the given IICTag into this one.
     * @param rhs The tag to copy.
     * @return const reference to this tag.
     */
    const IICTag& operator=(const IICTag& rhs)
    {
        CacheBlk::operator=(rhs);
        chain_ptr = rhs.chain_ptr;
        re = rhs.re;
        set = rhs.set;
        trivialData = rhs.trivialData;
        numData = rhs.numData;
        data_ptr.clear();
        for (int i = 0; i < rhs.numData; ++i) {
            data_ptr.push_back(rhs.data_ptr[i]);
        }
        return *this;
    }

    /** Hash chain pointer into secondary store. */
    unsigned long chain_ptr;
    /** Data array pointers for each subblock. */
    std::vector<unsigned long> data_ptr;
    /** Replacement Entry pointer. */
    void *re;
    /**
     * An array to store small compressed data. Conceputally the same size
     * as the unsused data array pointers.
     */
    uint8_t *trivialData;
    /**
     * The number of allocated subblocks.
     */
    int numData;
};

/**
 * A hash set for the IIC primary lookup table.
 */
class IICSet{
  public:
    /** The associativity of the primary table. */
    int assoc;

    /** The number of hash chains followed when finding the last block. */
    int depth;
    /** The current number of blocks on the chain. */
    int size;

    /** Tag pointer into the secondary tag storage. */
    unsigned long chain_ptr;

    /** The LRU list of the primary table. MRU is at 0 index. */
    IICTag ** tags;

    /**
     * Find the addr in this set, return the chain pointer to the secondary if
     * it isn't found.
     * @param asid The address space ID.
     * @param tag The address to find.
     * @param chain_ptr The chain pointer to start the search of the secondary
     * @return Pointer to the tag, NULL if not found.
     */
    IICTag* findTag( Addr tag, unsigned long &chain_ptr)
    {
        depth = 1;
        for (int i = 0; i < assoc; ++i) {
            if (tags[i]->tag == tag && tags[i]->isValid()) {
                return tags[i];
            }
        }
        chain_ptr = this->chain_ptr;
        return 0;
    }

    /**
     * Find an usused tag in this set.
     * @return Pointer to the unused tag, NULL if none are free.
     */
    IICTag* findFree()
    {
        for (int i = 0; i < assoc; ++i) {
            if (!tags[i]->isValid()) {
                return tags[i];
            }
        }
        return 0;
    }

    /**
     * Move a tag to the head of the LRU list
     * @param tag The tag to move.
     */
    void moveToHead(IICTag *tag);

    /**
     * Move a tag to the tail (LRU) of the LRU list
     * @param tag The tag to move.
     */
    void moveToTail(IICTag *tag);
};

/**
 * The IIC tag store. This is a hardware-realizable, fully-associative tag
 * store that uses software replacement, e.g. Gen.
 */
class IIC : public BaseTags
{
  public:
    /** Typedef of the block type used in this class. */
    typedef IICTag BlkType;
    /** Typedef for list of pointers to the local block type. */
    typedef std::list<IICTag*> BlkList;

  protected:
    /** The number of set in the primary table. */
    const unsigned hashSets;
    /** The block size in bytes. */
    const unsigned blkSize;
    /** The associativity of the primary table. */
    const unsigned assoc;
    /** The base hit latency. */
    const unsigned hitLatency;
    /** The subblock size, used for compression. */
    const unsigned subSize;

    /** The number of subblocks */
    const unsigned numSub;
    /** The number of bytes used by data pointers */
    const unsigned trivialSize;

    /** The amount to shift address to get the tag. */
    const unsigned tagShift;
    /** The mask to get block offset bits. */
    const unsigned blkMask;

    /** The amount to shift to get the subblock number. */
    const unsigned subShift;
    /** The mask to get the correct subblock number. */
    const unsigned subMask;

    /** The latency of a hash lookup. */
    const unsigned hashDelay;
    /** The total number of tags in primary and secondary. */
    const unsigned numTags;
    /** The number of tags in the secondary tag store. */
    const unsigned numSecondary;

    /** The Null tag pointer. */
    const unsigned tagNull;
    /** The last tag in the primary table. */
    const unsigned primaryBound;

    /** All of the tags */
    IICTag *tagStore;
    /**
     * Pointer to the head of the secondary freelist (maintained with chain
     * pointers.
     */
    unsigned long freelist;
    /**
     * The data block freelist.
     */
    std::list<unsigned long> blkFreelist;

    /** The primary table. */
    IICSet *sets;

    /** The replacement policy. */
    Repl *repl;

    /** An array of data reference counters. */
    int *dataReferenceCount;

    /** The data blocks. */
    uint8_t *dataStore;

    /** Storage for the fast access data of each cache block. */
    uint8_t **dataBlks;

    /**
     * Count of the current number of free secondary tags.
     * Used for debugging.
     */
    int freeSecond;

    // IIC Statistics
    /**
     * @addtogroup IICStatistics IIC Statistics
     * @{
     */

    /** Hash hit depth of cache hits. */
    Stats::Distribution hitHashDepth;
    /** Hash depth for cache misses. */
    Stats::Distribution missHashDepth;
    /** Count of accesses to each hash set. */
    Stats::Distribution setAccess;

    /** The total hash depth for every miss. */
    Stats::Scalar missDepthTotal;
    /** The total hash depth for all hits. */
    Stats::Scalar hitDepthTotal;
    /** The number of hash misses. */
    Stats::Scalar hashMiss;
    /** The number of hash hits. */
    Stats::Scalar hashHit;
    /** @} */

  public:
    /**
     * Collection of parameters for the IIC.
     */
    class Params {
      public:
        /** The size in bytes of the cache. */
        unsigned size;
        /** The number of sets in the primary table. */
        unsigned numSets;
        /** The block size in bytes. */
        unsigned blkSize;
        /** The associativity of the primary table. */
        unsigned assoc;
        /** The number of cycles for each hash lookup. */
        unsigned hashDelay;
        /** The number of cycles to read the data. */
        unsigned hitLatency;
        /** The replacement policy. */
        Repl *rp;
        /** The subblock size in bytes. */
        unsigned subblockSize;
    };

    /**
     * Construct and initialize this tag store.
     * @param params The IIC parameters.
     * @todo
     * Should make a way to have less tags in the primary than blks in the
     * cache. Also should be able to specify number of secondary blks.
     */
    IIC(Params &params);

    /**
     * Destructor.
     */
    virtual ~IIC();

    /**
     * Register the statistics.
     * @param name The name to prepend to the statistic descriptions.
     */
    void regStats(const std::string &name);

    /**
     * Regenerate the block address from the tag.
     * @param tag The tag of the block.
     * @param set Not needed for the iic.
     * @return The block address.
     */
    Addr regenerateBlkAddr(Addr tag, int set) {
        return (((Addr)tag << tagShift));
    }

    /**
     * Return the block size.
     * @return The block size.
     */
    unsigned
    getBlockSize() const
    {
        return blkSize;
    }

    /**
     * Return the subblock size.
     * @return The subblock size.
     */
    unsigned
    getSubBlockSize() const
    {
        return subSize;
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
     * Generate the tag from the address.
     * @param addr The address to a get a tag for.
     * @return the tag.
     */
    Addr extractTag(Addr addr) const
    {
        return (addr >> tagShift);
    }

   /**
     * Return the set, always 0 for IIC.
     * @return 0.
     */
    int extractSet(Addr addr) const
    {
        return 0;
    }

    /**
     * Get the block offset of an address.
     * @param addr The address to get the offset of.
     * @return the block offset of the address.
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
     * Swap the position of two tags.
     * @param index1 The first tag location.
     * @param index2 The second tag location.
     */
    void tagSwap(unsigned long index1, unsigned long index2);

    /**
     * Clear the reference bit of the tag and return its old value.
     * @param index The pointer of the tag to manipulate.
     * @return The previous state of the reference bit.
     */
    bool clearRef(unsigned long index)
    {
        bool tmp = tagStore[index].isReferenced();
        tagStore[index].status &= ~BlkReferenced;
        return tmp;
    }

    /**
     * Invalidate a block.
     * @param blk The block to invalidate.
     */
    void invalidateBlk(BlkType *blk);

    /**
     * Access block and update replacement data.  May not succeed, in which case
     * NULL pointer is returned.  This has all the implications of a cache
     * access and should only be used as such.
     * Returns the access latency and inCache flags as a side effect.
     * @param addr The address to find.
     * @param asid The address space ID.
     * @param lat The access latency.
     * @return A pointer to the block found, if any.
     */
    IICTag* accessBlock(Addr addr, int &lat, int context_src);

    /**
     * Find the block, do not update the replacement data.
     * @param addr The address to find.
     * @param asid The address space ID.
     * @return A pointer to the block found, if any.
     */
    IICTag* findBlock(Addr addr) const;

    /**
     * Find a replacement block for the address provided.
     * @param pkt The request to a find a replacement candidate for.
     * @param writebacks List for any writebacks to be performed.
     * @return The block to place the replacement in.
     */
    IICTag* findVictim(Addr addr, PacketList &writebacks);

    void insertBlock(Addr addr, BlkType *blk, int context_src);
    /**
     *iterated through all blocks and clear all locks
     *Needed to clear all lock tracking at once
     */
    virtual void clearLocks();

    /**
     * Called at end of simulation to complete average block reference stats.
     */
    virtual void cleanupRefs();

private:
    /**
     * Return the hash of the address.
     * @param addr The address to hash.
     * @return the hash of the address.
     */
    unsigned hash(Addr addr) const;

    /**
     * Search for a block in the secondary tag store. Returns the number of
     * hash lookups as a side effect.
     * @param asid The address space ID.
     * @param tag The tag to match.
     * @param chain_ptr The first entry to search.
     * @param depth The number of hash lookups made while searching.
     * @return A pointer to the block if found.
     */
    IICTag *secondaryChain(Addr tag, unsigned long chain_ptr,
                            int *depth) const;

    /**
     * Free the resources associated with the next replacement block.
     * @param writebacks A list of any writebacks to perform.
     */
    void freeReplacementBlock(PacketList & writebacks);

    /**
     * Return the pointer to a free data block.
     * @param writebacks A list of any writebacks to perform.
     * @return A pointer to a free data block.
     */
    unsigned long getFreeDataBlock(PacketList & writebacks);

    /**
     * Get a free tag in the given hash set.
     * @param set The hash set to search.
     * @param writebacks A list of any writebacks to perform.
     * @return a pointer to a free tag.
     */
    IICTag* getFreeTag(int set, PacketList & writebacks);

    /**
     * Free the resources associated with the given tag.
     * @param tag_ptr The tag to free.
     */
    void freeTag(IICTag *tag_ptr);

    /**
     * Mark the given data block as being available.
     * @param data_ptr The data block to free.
     */
    void freeDataBlock(unsigned long data_ptr);

};
#endif // __IIC_HH__

