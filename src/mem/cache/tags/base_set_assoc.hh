/*
 * Copyright (c) 2012-2014,2017 ARM Limited
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
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
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
 * Declaration of a base set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
#define __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__

#include <cassert>
#include <cstring>
#include <memory>
#include <vector>

#include "mem/cache/base.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/cacheset.hh"
#include "mem/packet.hh"
#include "params/BaseSetAssoc.hh"

/**
 * A BaseSetAssoc cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The BaseSetAssoc placement policy divides the cache into s sets of w
 * cache lines (ways). A cache line is mapped onto a set, and can be placed
 * into any of the ways of this set.
 */
class BaseSetAssoc : public BaseTags
{
  public:
    /** Typedef the block type used in this tag store. */
    typedef CacheBlk BlkType;
    /** Typedef the set type used in this tag store. */
    typedef CacheSet<CacheBlk> SetType;


  protected:
    /** The associativity of the cache. */
    const unsigned assoc;
    /** The allocatable associativity of the cache (alloc mask). */
    unsigned allocAssoc;

    /** The cache blocks. */
    std::vector<BlkType> blks;
    /** The data blocks, 1 per cache block. */
    std::unique_ptr<uint8_t[]> dataBlks;

    /** The number of sets in the cache. */
    const unsigned numSets;

    /** Whether tags and data are accessed sequentially. */
    const bool sequentialAccess;

    /** The cache sets. */
    std::vector<SetType> sets;

    /** The amount to shift the address to get the set. */
    int setShift;
    /** The amount to shift the address to get the tag. */
    int tagShift;
    /** Mask out all bits that aren't part of the set index. */
    unsigned setMask;

    /** Replacement policy */
    BaseReplacementPolicy *replacementPolicy;

  public:

    /** Convenience typedef. */
     typedef BaseSetAssocParams Params;

    /**
     * Construct and initialize this tag store.
     */
    BaseSetAssoc(const Params *p);

    /**
     * Destructor
     */
    virtual ~BaseSetAssoc() {};

    /**
     * Find the cache block given set and way
     * @param set The set of the block.
     * @param way The way of the block.
     * @return The cache block.
     */
    CacheBlk *findBlockBySetAndWay(int set, int way) const override;

    /**
     * Access block and update replacement data. May not succeed, in which case
     * nullptr is returned. This has all the implications of a cache
     * access and should only be used as such. Returns the access latency as a
     * side effect.
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param lat The access latency.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat) override
    {
        BlkType *blk = findBlock(addr, is_secure);

        // Access all tags in parallel, hence one in each way.  The data side
        // either accesses all blocks in parallel, or one block sequentially on
        // a hit.  Sequential access with a miss doesn't access data.
        tagAccesses += allocAssoc;
        if (sequentialAccess) {
            if (blk != nullptr) {
                dataAccesses += 1;
            }
        } else {
            dataAccesses += allocAssoc;
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

            // Update replacement data of accessed block
            replacementPolicy->touch(blk);
        } else {
            // If a cache miss
            lat = lookupLatency;
        }

        return blk;
    }

    /**
     * Finds the given address in the cache, do not update replacement data.
     * i.e. This is a no-side-effect find of a block.
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param asid The address space ID.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* findBlock(Addr addr, bool is_secure) const override;

    /**
     * Find replacement victim based on address.
     *
     * @param addr Address to find a victim for.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr) override
    {
        // Choose replacement victim from replacement candidates
        return replacementPolicy->getVictim(getPossibleLocations(addr));
    }

    /**
     * Find all possible block locations for insertion and replacement of
     * an address. Should be called immediately before ReplacementPolicy's
     * findVictim() not to break cache resizing.
     * Returns blocks in all ways belonging to the set of the address.
     *
     * @param addr The addr to a find possible locations for.
     * @return The possible locations.
     */
    const std::vector<CacheBlk*> getPossibleLocations(Addr addr)
    {
        return sets[extractSet(addr)].blks;
    }

    /**
     * Insert the new block into the cache.
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
     void insertBlock(PacketPtr pkt, CacheBlk *blk) override
     {
         Addr addr = pkt->getAddr();
         MasterID master_id = pkt->req->masterId();
         uint32_t task_id = pkt->req->taskId();

         if (!blk->isTouched) {
             if (!warmedUp && tagsInUse.value() >= warmupBound) {
                 warmedUp = true;
                 warmupCycle = curTick();
             }
         }

         // If we're replacing a block that was previously valid update
         // stats for it. This can't be done in findBlock() because a
         // found block might not actually be replaced there if the
         // coherence protocol says it can't be.
         if (blk->isValid()) {
             replacements[0]++;
             totalRefs += blk->refCount;
             ++sampledRefs;

             invalidate(blk);
             blk->invalidate();
         }

         // Previous block, if existed, has been removed, and now we have
         // to insert the new one
         tagsInUse++;

         // Set tag for new block.  Caller is responsible for setting status.
         blk->tag = extractTag(addr);

         // deal with what we are bringing in
         assert(master_id < cache->system->maxMasters());
         occupancies[master_id]++;
         blk->srcMasterId = master_id;
         blk->task_id = task_id;

         // We only need to write into one tag and one data block.
         tagAccesses += 1;
         dataAccesses += 1;

         replacementPolicy->reset(blk);
     }

    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void setWayAllocationMax(int ways) override
    {
        fatal_if(ways < 1, "Allocation limit must be greater than zero");
        allocAssoc = ways;
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int getWayAllocationMax() const override
    {
        return allocAssoc;
    }

    /**
     * Generate the tag from the given address.
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    Addr extractTag(Addr addr) const override
    {
        return (addr >> tagShift);
    }

    /**
     * Calculate the set index from the address.
     * @param addr The address to get the set from.
     * @return The set index of the address.
     */
    int extractSet(Addr addr) const override
    {
        return ((addr >> setShift) & setMask);
    }

    /**
     * Regenerate the block address from the tag and set.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override
    {
        return ((blk->tag << tagShift) | ((Addr)blk->set << setShift));
    }

    /**
     * Called at end of simulation to complete average block reference stats.
     */
    void cleanupRefs() override;

    /**
     * Print all tags used
     */
    std::string print() const override;

    /**
     * Called prior to dumping stats to compute task occupancy
     */
    void computeStats() override;

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
        for (unsigned i = 0; i < numSets * assoc; ++i) {
            if (!visitor(blks[i]))
                return;
        }
    }
};

#endif //__MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
