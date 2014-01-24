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
 *          Dave Greene
 *          Steve Reinhardt
 *          Ron Dreslinski
 *          Andreas Hansson
 */

/**
 * @file
 * Describes a cache based on template policies.
 */

#ifndef __CACHE_HH__
#define __CACHE_HH__

#include "base/misc.hh" // fatal, panic, and warn
#include "mem/cache/base.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/mshr.hh"
#include "sim/eventq.hh"

//Forward decleration
class BasePrefetcher;

/**
 * A template-policy based cache. The behavior of the cache can be altered by
 * supplying different template policies. TagStore handles all tag and data
 * storage @sa TagStore, \ref gem5MemorySystem "gem5 Memory System"
 */
template <class TagStore>
class Cache : public BaseCache
{
  public:
    /** Define the type of cache block to use. */
    typedef typename TagStore::BlkType BlkType;
    /** A typedef for a list of BlkType pointers. */
    typedef typename TagStore::BlkList BlkList;

  protected:
    typedef CacheBlkVisitorWrapper<Cache<TagStore>, BlkType> WrappedBlkVisitor;

    /**
     * The CPU-side port extends the base cache slave port with access
     * functions for functional, atomic and timing requests.
     */
    class CpuSidePort : public CacheSlavePort
    {
      private:

        // a pointer to our specific cache implementation
        Cache<TagStore> *cache;

      protected:

        virtual bool recvTimingSnoopResp(PacketPtr pkt);

        virtual bool recvTimingReq(PacketPtr pkt);

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);

        virtual AddrRangeList getAddrRanges() const;

      public:

        CpuSidePort(const std::string &_name, Cache<TagStore> *_cache,
                    const std::string &_label);

    };

    /**
     * Override the default behaviour of sendDeferredPacket to enable
     * the memory-side cache port to also send requests based on the
     * current MSHR status. This queue has a pointer to our specific
     * cache implementation and is used by the MemSidePort.
     */
    class MemSidePacketQueue : public MasterPacketQueue
    {

      protected:

        Cache<TagStore> &cache;

      public:

        MemSidePacketQueue(Cache<TagStore> &cache, MasterPort &port,
                           const std::string &label) :
            MasterPacketQueue(cache, port, label), cache(cache) { }

        /**
         * Override the normal sendDeferredPacket and do not only
         * consider the transmit list (used for responses), but also
         * requests.
         */
        virtual void sendDeferredPacket();

    };

    /**
     * The memory-side port extends the base cache master port with
     * access functions for functional, atomic and timing snoops.
     */
    class MemSidePort : public CacheMasterPort
    {
      private:

        /** The cache-specific queue. */
        MemSidePacketQueue _queue;

        // a pointer to our specific cache implementation
        Cache<TagStore> *cache;

      protected:

        virtual void recvTimingSnoopReq(PacketPtr pkt);

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual Tick recvAtomicSnoop(PacketPtr pkt);

        virtual void recvFunctionalSnoop(PacketPtr pkt);

      public:

        MemSidePort(const std::string &_name, Cache<TagStore> *_cache,
                    const std::string &_label);
    };

    /** Tag and data Storage */
    TagStore *tags;

    /** Prefetcher */
    BasePrefetcher *prefetcher;

    /** Temporary cache block for occasional transitory use */
    BlkType *tempBlock;

    /**
     * This cache should allocate a block on a line-sized write miss.
     */
    const bool doFastWrites;

    /**
     * Notify the prefetcher on every access, not just misses.
     */
    const bool prefetchOnAccess;

    /**
     * @todo this is a temporary workaround until the 4-phase code is committed.
     * upstream caches need this packet until true is returned, so hold it for
     * deletion until a subsequent call
     */
    std::vector<PacketPtr> pendingDelete;

    /**
     * Does all the processing necessary to perform the provided request.
     * @param pkt The memory request to perform.
     * @param lat The latency of the access.
     * @param writebacks List for any writebacks that need to be performed.
     * @param update True if the replacement data should be updated.
     * @return Boolean indicating whether the request was satisfied.
     */
    bool access(PacketPtr pkt, BlkType *&blk,
                Cycles &lat, PacketList &writebacks);

    /**
     *Handle doing the Compare and Swap function for SPARC.
     */
    void cmpAndSwap(BlkType *blk, PacketPtr pkt);

    /**
     * Find a block frame for new block at address addr targeting the
     * given security space, assuming that the block is not currently
     * in the cache.  Append writebacks if any to provided packet
     * list.  Return free block frame.  May return NULL if there are
     * no replaceable blocks at the moment.
     */
    BlkType *allocateBlock(Addr addr, bool is_secure, PacketList &writebacks);

    /**
     * Populates a cache block and handles all outstanding requests for the
     * satisfied fill request. This version takes two memory requests. One
     * contains the fill data, the other is an optional target to satisfy.
     * @param pkt The memory request with the fill data.
     * @param blk The cache block if it already exists.
     * @param writebacks List for any writebacks that need to be performed.
     * @return Pointer to the new cache block.
     */
    BlkType *handleFill(PacketPtr pkt, BlkType *blk,
                        PacketList &writebacks);


    /**
     * Performs the access specified by the request.
     * @param pkt The request to perform.
     * @return The result of the access.
     */
    bool recvTimingReq(PacketPtr pkt);

    /**
     * Handles a response (cache line fill/write ack) from the bus.
     * @param pkt The response packet
     */
    void recvTimingResp(PacketPtr pkt);

    /**
     * Snoops bus transactions to maintain coherence.
     * @param pkt The current bus transaction.
     */
    void recvTimingSnoopReq(PacketPtr pkt);

    /**
     * Handle a snoop response.
     * @param pkt Snoop response packet
     */
    void recvTimingSnoopResp(PacketPtr pkt);

    /**
     * Performs the access specified by the request.
     * @param pkt The request to perform.
     * @return The number of ticks required for the access.
     */
    Tick recvAtomic(PacketPtr pkt);

    /**
     * Snoop for the provided request in the cache and return the estimated
     * time taken.
     * @param pkt The memory request to snoop
     * @return The number of ticks required for the snoop.
     */
    Tick recvAtomicSnoop(PacketPtr pkt);

    /**
     * Performs the access specified by the request.
     * @param pkt The request to perform.
     * @param fromCpuSide from the CPU side port or the memory side port
     */
    void functionalAccess(PacketPtr pkt, bool fromCpuSide);

    void satisfyCpuSideRequest(PacketPtr pkt, BlkType *blk,
                               bool deferred_response = false,
                               bool pending_downgrade = false);
    bool satisfyMSHR(MSHR *mshr, PacketPtr pkt, BlkType *blk);

    void doTimingSupplyResponse(PacketPtr req_pkt, uint8_t *blk_data,
                                bool already_copied, bool pending_inval);

    /**
     * Sets the blk to the new state.
     * @param blk The cache block being snooped.
     * @param new_state The new coherence state for the block.
     */
    void handleSnoop(PacketPtr ptk, BlkType *blk,
                     bool is_timing, bool is_deferred, bool pending_inval);

    /**
     * Create a writeback request for the given block.
     * @param blk The block to writeback.
     * @return The writeback request for the block.
     */
    PacketPtr writebackBlk(BlkType *blk);


    void memWriteback();
    void memInvalidate();
    bool isDirty() const;

    /**
     * Cache block visitor that writes back dirty cache blocks using
     * functional writes.
     *
     * \return Always returns true.
     */
    bool writebackVisitor(BlkType &blk);
    /**
     * Cache block visitor that invalidates all blocks in the cache.
     *
     * @warn Dirty cache lines will not be written back to memory.
     *
     * \return Always returns true.
     */
    bool invalidateVisitor(BlkType &blk);

    /**
     * Flush a cache line due to an uncacheable memory access to the
     * line.
     *
     * @note This shouldn't normally happen, but we need to handle it
     * since some architecture models don't implement cache
     * maintenance operations. We won't even try to get a decent
     * timing here since the line should have been flushed earlier by
     * a cache maintenance operation.
     */
    void uncacheableFlush(PacketPtr pkt);

    /**
     * Squash all requests associated with specified thread.
     * intended for use by I-cache.
     * @param threadNum The thread to squash.
     */
    void squash(int threadNum);

    /**
     * Generate an appropriate downstream bus request packet for the
     * given parameters.
     * @param cpu_pkt  The upstream request that needs to be satisfied.
     * @param blk The block currently in the cache corresponding to
     * cpu_pkt (NULL if none).
     * @param needsExclusive  Indicates that an exclusive copy is required
     * even if the request in cpu_pkt doesn't indicate that.
     * @return A new Packet containing the request, or NULL if the
     * current request in cpu_pkt should just be forwarded on.
     */
    PacketPtr getBusPacket(PacketPtr cpu_pkt, BlkType *blk,
                           bool needsExclusive) const;

    /**
     * Return the next MSHR to service, either a pending miss from the
     * mshrQueue, a buffered write from the write buffer, or something
     * from the prefetcher.  This function is responsible for
     * prioritizing among those sources on the fly.
     */
    MSHR *getNextMSHR();

    /**
     * Selects an outstanding request to service.  Called when the
     * cache gets granted the downstream bus in timing mode.
     * @return The request to service, NULL if none found.
     */
    PacketPtr getTimingPacket();

    /**
     * Marks a request as in service (sent on the bus). This can have side
     * effect since storage for no response commands is deallocated once they
     * are successfully sent.
     * @param pkt The request that was sent on the bus.
     */
    void markInService(MSHR *mshr, PacketPtr pkt = 0);

    /**
     * Return whether there are any outstanding misses.
     */
    bool outstandingMisses() const
    {
        return mshrQueue.allocated != 0;
    }

    CacheBlk *findBlock(Addr addr, bool is_secure) const {
        return tags->findBlock(addr, is_secure);
    }

    bool inCache(Addr addr, bool is_secure) const {
        return (tags->findBlock(addr, is_secure) != 0);
    }

    bool inMissQueue(Addr addr, bool is_secure) const {
        return (mshrQueue.findMatch(addr, is_secure) != 0);
    }

    /**
     * Find next request ready time from among possible sources.
     */
    Tick nextMSHRReadyTime() const;

  public:
    /** Instantiates a basic cache object. */
    Cache(const Params *p);

    /** Non-default destructor is needed to deallocate memory. */
    virtual ~Cache();

    void regStats();

    /** serialize the state of the caches
     * We currently don't support checkpointing cache state, so this panics.
     */
    virtual void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __CACHE_HH__
