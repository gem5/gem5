/*
 * Copyright (c) 2012-2013, 2015-2016, 2018-2019, 2023 ARM Limited
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
 * Declares a basic cache interface BaseCache.
 */

#ifndef __MEM_CACHE_BASE_HH__
#define __MEM_CACHE_BASE_HH__

#include <cassert>
#include <cstdint>
#include <string>

#include "base/addr_range.hh"
#include "base/compiler.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Cache.hh"
#include "debug/CachePort.hh"
#include "enums/Clusivity.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/cache_probe_arg.hh"
#include "mem/cache/compressors/base.hh"
#include "mem/cache/mshr_queue.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/write_queue.hh"
#include "mem/cache/write_queue_entry.hh"
#include "mem/packet.hh"
#include "mem/packet_queue.hh"
#include "mem/qport.hh"
#include "mem/request.hh"
#include "params/WriteAllocator.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/probe/probe.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

namespace prefetch
{
class Base;
}
class MSHR;
class RequestPort;
class QueueEntry;
struct BaseCacheParams;

/**
 * A basic cache interface. Implements some common functions for speed.
 */
class BaseCache : public ClockedObject
{
  protected:
    /**
     * Indexes to enumerate the MSHR queues.
     */
    enum MSHRQueueIndex
    {
        MSHRQueue_MSHRs,
        MSHRQueue_WriteBuffer
    };

  public:
    /**
     * Reasons for caches to be blocked.
     */
    enum BlockedCause
    {
        Blocked_NoMSHRs = MSHRQueue_MSHRs,
        Blocked_NoWBBuffers = MSHRQueue_WriteBuffer,
        Blocked_NoTargets,
        NUM_BLOCKED_CAUSES
    };

  protected:
    /**
     * A cache request port is used for the memory-side port of the
     * cache, and in addition to the basic timing port that only sends
     * response packets through a transmit list, it also offers the
     * ability to schedule and send request packets (requests &
     * writebacks). The send event is scheduled through schedSendEvent,
     * and the sendDeferredPacket of the timing port is modified to
     * consider both the transmit list and the requests from the MSHR.
     */
    class CacheRequestPort : public QueuedRequestPort
    {
      public:
        /**
         * Schedule a send of a request packet (from the MSHR). Note
         * that we could already have a retry outstanding.
         */
        void
        schedSendEvent(Tick time)
        {
            DPRINTF(CachePort, "Scheduling send event at %llu\n", time);
            reqQueue.schedSendEvent(time);
        }

      protected:
        CacheRequestPort(const std::string &_name, ReqPacketQueue &_reqQueue,
                         SnoopRespPacketQueue &_snoopRespQueue)
            : QueuedRequestPort(_name, _reqQueue, _snoopRespQueue)
        {}

        /**
         * Memory-side port always snoops.
         *
         * @return always true
         */
        virtual bool
        isSnooping() const
        {
            return true;
        }
    };

    /**
     * Override the default behaviour of sendDeferredPacket to enable
     * the memory-side cache port to also send requests based on the
     * current MSHR status. This queue has a pointer to our specific
     * cache implementation and is used by the MemSidePort.
     */
    class CacheReqPacketQueue : public ReqPacketQueue
    {
      protected:
        BaseCache &cache;
        SnoopRespPacketQueue &snoopRespQueue;

      public:
        CacheReqPacketQueue(BaseCache &cache, RequestPort &port,
                            SnoopRespPacketQueue &snoop_resp_queue,
                            const std::string &label)
            : ReqPacketQueue(cache, port, label),
              cache(cache),
              snoopRespQueue(snoop_resp_queue)
        {}

        /**
         * Override the normal sendDeferredPacket and do not only
         * consider the transmit list (used for responses), but also
         * requests.
         */
        virtual void sendDeferredPacket();

        /**
         * Check if there is a conflicting snoop response about to be
         * send out, and if so simply stall any requests, and schedule
         * a send event at the same time as the next snoop response is
         * being sent out.
         *
         * @param pkt The packet to check for conflicts against.
         */
        bool
        checkConflictingSnoop(const PacketPtr pkt)
        {
            if (snoopRespQueue.checkConflict(pkt, cache.blkSize)) {
                DPRINTF(CachePort, "Waiting for snoop response to be "
                                   "sent\n");
                Tick when = snoopRespQueue.deferredPacketReadyTime();
                schedSendEvent(when);
                return true;
            }
            return false;
        }
    };

    /**
     * The memory-side port extends the base cache request port with
     * access functions for functional, atomic and timing snoops.
     */
    class MemSidePort : public CacheRequestPort
    {
      private:
        /** The cache-specific queue. */
        CacheReqPacketQueue _reqQueue;

        SnoopRespPacketQueue _snoopRespQueue;

        // a pointer to our specific cache implementation
        BaseCache *cache;

      protected:
        virtual void recvTimingSnoopReq(PacketPtr pkt);

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual Tick recvAtomicSnoop(PacketPtr pkt);

        virtual void recvFunctionalSnoop(PacketPtr pkt);

      public:
        MemSidePort(const std::string &_name, BaseCache *_cache,
                    const std::string &_label);
    };

    /**
     * A cache response port is used for the CPU-side port of the cache,
     * and it is basically a simple timing port that uses a transmit
     * list for responses to the CPU (or connected requestor). In
     * addition, it has the functionality to block the port for
     * incoming requests. If blocked, the port will issue a retry once
     * unblocked.
     */
    class CacheResponsePort : public QueuedResponsePort
    {
      public:
        /** Do not accept any new requests. */
        void setBlocked();

        /** Return to normal operation and accept new requests. */
        void clearBlocked();

        bool
        isBlocked() const
        {
            return blocked;
        }

      protected:
        CacheResponsePort(const std::string &_name, BaseCache &_cache,
                          const std::string &_label);

        BaseCache &cache;

        /** A normal packet queue used to store responses. */
        RespPacketQueue queue;

        bool blocked;

        bool mustSendRetry;

      private:
        void processSendRetry();

        EventFunctionWrapper sendRetryEvent;
    };

    /**
     * The CPU-side port extends the base cache response port with access
     * functions for functional, atomic and timing requests.
     */
    class CpuSidePort : public CacheResponsePort
    {
      protected:
        virtual bool recvTimingSnoopResp(PacketPtr pkt) override;

        virtual bool tryTiming(PacketPtr pkt) override;

        virtual bool recvTimingReq(PacketPtr pkt) override;

        virtual Tick recvAtomic(PacketPtr pkt) override;

        virtual void recvFunctional(PacketPtr pkt) override;

        virtual AddrRangeList getAddrRanges() const override;

      public:
        CpuSidePort(const std::string &_name, BaseCache &_cache,
                    const std::string &_label);
    };

    CpuSidePort cpuSidePort;
    MemSidePort memSidePort;

  protected:
    struct CacheAccessorImpl : CacheAccessor
    {
        BaseCache &cache;

        CacheAccessorImpl(BaseCache &_cache) : cache(_cache) {}

        bool
        inCache(Addr addr, bool is_secure) const override
        {
            return cache.inCache(addr, is_secure);
        }

        bool
        hasBeenPrefetched(Addr addr, bool is_secure) const override
        {
            return cache.hasBeenPrefetched(addr, is_secure);
        }

        bool
        hasBeenPrefetched(Addr addr, bool is_secure,
                          RequestorID requestor) const override
        {
            return cache.hasBeenPrefetched(addr, is_secure, requestor);
        }

        bool
        inMissQueue(Addr addr, bool is_secure) const override
        {
            return cache.inMissQueue(addr, is_secure);
        }

        bool
        coalesce() const override
        {
            return cache.coalesce();
        }

    } accessor;

    /** Miss status registers */
    MSHRQueue mshrQueue;

    /** Write/writeback buffer */
    WriteQueue writeBuffer;

    /** Tag and data Storage */
    BaseTags *tags;

    /** Compression method being used. */
    compression::Base *compressor;

    /** Prefetcher */
    prefetch::Base *prefetcher;

    /** To probe when a cache hit occurs */
    ProbePointArg<CacheAccessProbeArg> *ppHit;

    /** To probe when a cache miss occurs */
    ProbePointArg<CacheAccessProbeArg> *ppMiss;

    /** To probe when a cache fill occurs */
    ProbePointArg<CacheAccessProbeArg> *ppFill;

    /**
     * To probe when the contents of a block are updated. Content updates
     * include data fills, overwrites, and invalidations, which means that
     * this probe partially overlaps with other probes.
     */
    ProbePointArg<CacheDataUpdateProbeArg> *ppDataUpdate;

    /**
     * The writeAllocator drive optimizations for streaming writes.
     * It first determines whether a WriteReq MSHR should be delayed,
     * thus ensuring that we wait longer in cases when we are write
     * coalescing and allowing all the bytes of the line to be written
     * before the MSHR packet is sent downstream. This works in unison
     * with the tracking in the MSHR to check if the entire line is
     * written. The write mode also affects the behaviour on filling
     * any whole-line writes. Normally the cache allocates the line
     * when receiving the InvalidateResp, but after seeing enough
     * consecutive lines we switch to using the tempBlock, and thus
     * end up not allocating the line, and instead turning the
     * whole-line write into a writeback straight away.
     */
    WriteAllocator *const writeAllocator;

    /**
     * Temporary cache block for occasional transitory use.  We use
     * the tempBlock to fill when allocation fails (e.g., when there
     * is an outstanding request that accesses the victim block) or
     * when we want to avoid allocation (e.g., exclusive caches)
     */
    TempCacheBlk *tempBlock;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

    /**
     * Mark a request as in service (sent downstream in the memory
     * system), effectively making this MSHR the ordering point.
     */
    void
    markInService(MSHR *mshr, bool pending_modified_resp)
    {
        bool wasFull = mshrQueue.isFull();
        mshrQueue.markInService(mshr, pending_modified_resp);

        if (wasFull && !mshrQueue.isFull()) {
            clearBlocked(Blocked_NoMSHRs);
        }
    }

    void
    markInService(WriteQueueEntry *entry)
    {
        bool wasFull = writeBuffer.isFull();
        writeBuffer.markInService(entry);

        if (wasFull && !writeBuffer.isFull()) {
            clearBlocked(Blocked_NoWBBuffers);
        }
    }

    /**
     * Determine whether we should allocate on a fill or not. If this
     * cache is mostly inclusive with regards to the upstream cache(s)
     * we always allocate (for any non-forwarded and cacheable
     * requests). In the case of a mostly exclusive cache, we allocate
     * on fill if the packet did not come from a cache, thus if we:
     * are dealing with a whole-line write (the latter behaves much
     * like a writeback), the original target packet came from a
     * non-caching source, or if we are performing a prefetch or LLSC.
     *
     * @param cmd Command of the incoming requesting packet
     * @return Whether we should allocate on the fill
     */
    inline bool
    allocOnFill(MemCmd cmd) const
    {
        return clusivity == enums::mostly_incl ||
               cmd == MemCmd::WriteLineReq || cmd == MemCmd::ReadReq ||
               cmd == MemCmd::WriteReq || cmd.isPrefetch() || cmd.isLLSC();
    }

    /**
     * Regenerate block address using tags.
     * Block address regeneration depends on whether we're using a temporary
     * block or not.
     *
     * @param blk The block to regenerate address.
     * @return The block's address.
     */
    Addr regenerateBlkAddr(CacheBlk *blk);

    /**
     * Calculate latency of accesses that only touch the tag array.
     * @sa calculateAccessLatency
     *
     * @param delay The delay until the packet's metadata is present.
     * @param lookup_lat Latency of the respective tag lookup.
     * @return The number of ticks that pass due to a tag-only access.
     */
    Cycles calculateTagOnlyLatency(const uint32_t delay,
                                   const Cycles lookup_lat) const;
    /**
     * Calculate access latency in ticks given a tag lookup latency, and
     * whether access was a hit or miss.
     *
     * @param blk The cache block that was accessed.
     * @param delay The delay until the packet's metadata is present.
     * @param lookup_lat Latency of the respective tag lookup.
     * @return The number of ticks that pass due to a block access.
     */
    Cycles calculateAccessLatency(const CacheBlk *blk, const uint32_t delay,
                                  const Cycles lookup_lat) const;

    /**
     * Does all the processing necessary to perform the provided request.
     * @param pkt The memory request to perform.
     * @param blk The cache block to be updated.
     * @param lat The latency of the access.
     * @param writebacks List for any writebacks that need to be performed.
     * @return Boolean indicating whether the request was satisfied.
     */
    virtual bool access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                        PacketList &writebacks);

    /*
     * Handle a timing request that hit in the cache
     *
     * @param ptk The request packet
     * @param blk The referenced block
     * @param request_time The tick at which the block lookup is compete
     */
    virtual void handleTimingReqHit(PacketPtr pkt, CacheBlk *blk,
                                    Tick request_time);

    /*
     * Handle a timing request that missed in the cache
     *
     * Implementation specific handling for different cache
     * implementations
     *
     * @param ptk The request packet
     * @param blk The referenced block
     * @param forward_time The tick at which we can process dependent requests
     * @param request_time The tick at which the block lookup is compete
     */
    virtual void handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk,
                                     Tick forward_time, Tick request_time) = 0;

    /*
     * Handle a timing request that missed in the cache
     *
     * Common functionality across different cache implementations
     *
     * @param ptk The request packet
     * @param blk The referenced block
     * @param mshr Any existing mshr for the referenced cache block
     * @param forward_time The tick at which we can process dependent requests
     * @param request_time The tick at which the block lookup is compete
     */
    void handleTimingReqMiss(PacketPtr pkt, MSHR *mshr, CacheBlk *blk,
                             Tick forward_time, Tick request_time);

    /**
     * Performs the access specified by the request.
     * @param pkt The request to perform.
     */
    virtual void recvTimingReq(PacketPtr pkt);

    /**
     * Handling the special case of uncacheable write responses to
     * make recvTimingResp less cluttered.
     */
    void handleUncacheableWriteResp(PacketPtr pkt);

    /**
     * Service non-deferred MSHR targets using the received response
     *
     * Iterates through the list of targets that can be serviced with
     * the current response.
     *
     * @param mshr The MSHR that corresponds to the reponse
     * @param pkt The response packet
     * @param blk The reference block
     */
    virtual void serviceMSHRTargets(MSHR *mshr, const PacketPtr pkt,
                                    CacheBlk *blk) = 0;

    /**
     * Handles a response (cache line fill/write ack) from the bus.
     * @param pkt The response packet
     */
    virtual void recvTimingResp(PacketPtr pkt);

    /**
     * Snoops bus transactions to maintain coherence.
     * @param pkt The current bus transaction.
     */
    virtual void recvTimingSnoopReq(PacketPtr pkt) = 0;

    /**
     * Handle a snoop response.
     * @param pkt Snoop response packet
     */
    virtual void recvTimingSnoopResp(PacketPtr pkt) = 0;

    /**
     * Handle a request in atomic mode that missed in this cache
     *
     * Creates a downstream request, sends it to the memory below and
     * handles the response. As we are in atomic mode all operations
     * are performed immediately.
     *
     * @param pkt The packet with the requests
     * @param blk The referenced block
     * @param writebacks A list with packets for any performed writebacks
     * @return Cycles for handling the request
     */
    virtual Cycles handleAtomicReqMiss(PacketPtr pkt, CacheBlk *&blk,
                                       PacketList &writebacks) = 0;

    /**
     * Performs the access specified by the request.
     * @param pkt The request to perform.
     * @return The number of ticks required for the access.
     */
    virtual Tick recvAtomic(PacketPtr pkt);

    /**
     * Snoop for the provided request in the cache and return the estimated
     * time taken.
     * @param pkt The memory request to snoop
     * @return The number of ticks required for the snoop.
     */
    virtual Tick recvAtomicSnoop(PacketPtr pkt) = 0;

    /**
     * Performs the access specified by the request.
     *
     * @param pkt The request to perform.
     * @param fromCpuSide from the CPU side port or the memory side port
     */
    virtual void functionalAccess(PacketPtr pkt, bool from_cpu_side);

    /**
     * Update the data contents of a block. When no packet is provided no
     * data will be written to the block, which means that this was likely
     * triggered by an invalidation.
     *
     * @param blk The block being updated.
     * @param cpkt The packet containing the new data.
     * @param has_old_data Whether this block had data previously.
     */
    void updateBlockData(CacheBlk *blk, const PacketPtr cpkt,
                         bool has_old_data);

    /**
     * Handle doing the Compare and Swap function for SPARC.
     */
    void cmpAndSwap(CacheBlk *blk, PacketPtr pkt);

    /**
     * Return the next queue entry to service, either a pending miss
     * from the MSHR queue, a buffered write from the write buffer, or
     * something from the prefetcher. This function is responsible
     * for prioritizing among those sources on the fly.
     */
    QueueEntry *getNextQueueEntry();

    /**
     * Insert writebacks into the write buffer
     */
    virtual void doWritebacks(PacketList &writebacks, Tick forward_time) = 0;

    /**
     * Send writebacks down the memory hierarchy in atomic mode
     */
    virtual void doWritebacksAtomic(PacketList &writebacks) = 0;

    /**
     * Create an appropriate downstream bus request packet.
     *
     * Creates a new packet with the request to be send to the memory
     * below, or nullptr if the current request in cpu_pkt should just
     * be forwarded on.
     *
     * @param cpu_pkt The miss packet that needs to be satisfied.
     * @param blk The referenced block, can be nullptr.
     * @param needs_writable Indicates that the block must be writable
     * even if the request in cpu_pkt doesn't indicate that.
     * @param is_whole_line_write True if there are writes for the
     * whole line
     * @return A packet send to the memory below
     */
    virtual PacketPtr createMissPacket(PacketPtr cpu_pkt, CacheBlk *blk,
                                       bool needs_writable,
                                       bool is_whole_line_write) const = 0;

    /**
     * Determine if clean lines should be written back or not. In
     * cases where a downstream cache is mostly inclusive we likely
     * want it to act as a victim cache also for lines that have not
     * been modified. Hence, we cannot simply drop the line (or send a
     * clean evict), but rather need to send the actual data.
     */
    const bool writebackClean;

    /**
     * Writebacks from the tempBlock, resulting on the response path
     * in atomic mode, must happen after the call to recvAtomic has
     * finished (for the right ordering of the packets). We therefore
     * need to hold on to the packets, and have a method and an event
     * to send them.
     */
    PacketPtr tempBlockWriteback;

    /**
     * Send the outstanding tempBlock writeback. To be called after
     * recvAtomic finishes in cases where the block we filled is in
     * fact the tempBlock, and now needs to be written back.
     */
    void
    writebackTempBlockAtomic()
    {
        assert(tempBlockWriteback != nullptr);
        PacketList writebacks{ tempBlockWriteback };
        doWritebacksAtomic(writebacks);
        tempBlockWriteback = nullptr;
    }

    /**
     * An event to writeback the tempBlock after recvAtomic
     * finishes. To avoid other calls to recvAtomic getting in
     * between, we create this event with a higher priority.
     */
    EventFunctionWrapper writebackTempBlockAtomicEvent;

    /**
     * When a block is overwriten, its compression information must be updated,
     * and it may need to be recompressed. If the compression size changes, the
     * block may either become smaller, in which case there is no side effect,
     * or bigger (data expansion; fat write), in which case the block might not
     * fit in its current location anymore. If that happens, there are usually
     * two options to be taken:
     *
     * - The co-allocated blocks must be evicted to make room for this block.
     *   Simpler, but ignores replacement data.
     * - The block itself is moved elsewhere (used in policies where the CF
     *   determines the location of the block).
     *
     * This implementation uses the first approach.
     *
     * Notice that this is only called for writebacks, which means that L1
     * caches (which see regular Writes), do not support compression.
     * @sa CompressedTags
     *
     * @param blk The block to be overwriten.
     * @param data A pointer to the data to be compressed (blk's new data).
     * @param writebacks List for any writebacks that need to be performed.
     * @return Whether operation is successful or not.
     */
    bool updateCompressionData(CacheBlk *&blk, const uint64_t *data,
                               PacketList &writebacks);

    /**
     * Perform any necessary updates to the block and perform any data
     * exchange between the packet and the block. The flags of the
     * packet are also set accordingly.
     *
     * @param pkt Request packet from upstream that hit a block
     * @param blk Cache block that the packet hit
     * @param deferred_response Whether this request originally missed
     * @param pending_downgrade Whether the writable flag is to be removed
     */
    virtual void satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                                bool deferred_response = false,
                                bool pending_downgrade = false);

    /**
     * Maintain the clusivity of this cache by potentially
     * invalidating a block. This method works in conjunction with
     * satisfyRequest, but is separate to allow us to handle all MSHR
     * targets before potentially dropping a block.
     *
     * @param from_cache Whether we have dealt with a packet from a cache
     * @param blk The block that should potentially be dropped
     */
    void maintainClusivity(bool from_cache, CacheBlk *blk);

    /**
     * Try to evict the given blocks. If any of them is a transient eviction,
     * that is, the block is present in the MSHR queue all evictions are
     * cancelled since handling such cases has not been implemented.
     *
     * @param evict_blks Blocks marked for eviction.
     * @param writebacks List for any writebacks that need to be performed.
     * @return False if any of the evicted blocks is in transient state.
     */
    bool handleEvictions(std::vector<CacheBlk *> &evict_blks,
                         PacketList &writebacks);

    /**
     * Handle a fill operation caused by a received packet.
     *
     * Populates a cache block and handles all outstanding requests for the
     * satisfied fill request. This version takes two memory requests. One
     * contains the fill data, the other is an optional target to satisfy.
     * Note that the reason we return a list of writebacks rather than
     * inserting them directly in the write buffer is that this function
     * is called by both atomic and timing-mode accesses, and in atomic
     * mode we don't mess with the write buffer (we just perform the
     * writebacks atomically once the original request is complete).
     *
     * @param pkt The memory request with the fill data.
     * @param blk The cache block if it already exists.
     * @param writebacks List for any writebacks that need to be performed.
     * @param allocate Whether to allocate a block or use the temp block
     * @return Pointer to the new cache block.
     */
    CacheBlk *handleFill(PacketPtr pkt, CacheBlk *blk, PacketList &writebacks,
                         bool allocate);

    /**
     * Allocate a new block and perform any necessary writebacks
     *
     * Find a victim block and if necessary prepare writebacks for any
     * existing data. May return nullptr if there are no replaceable
     * blocks. If a replaceable block is found, it inserts the new block in
     * its place. The new block, however, is not set as valid yet.
     *
     * @param pkt Packet holding the address to update
     * @param writebacks A list of writeback packets for the evicted blocks
     * @return the allocated block
     */
    CacheBlk *allocateBlock(const PacketPtr pkt, PacketList &writebacks);
    /**
     * Evict a cache block.
     *
     * Performs a writeback if necesssary and invalidates the block
     *
     * @param blk Block to invalidate
     * @return A packet with the writeback, can be nullptr
     */
    [[nodiscard]] virtual PacketPtr evictBlock(CacheBlk *blk) = 0;

    /**
     * Evict a cache block.
     *
     * Performs a writeback if necesssary and invalidates the block
     *
     * @param blk Block to invalidate
     * @param writebacks Return a list of packets with writebacks
     */
    void evictBlock(CacheBlk *blk, PacketList &writebacks);

    /**
     * Invalidate a cache block.
     *
     * @param blk Block to invalidate
     */
    void invalidateBlock(CacheBlk *blk);

    /**
     * Create a writeback request for the given block.
     *
     * @param blk The block to writeback.
     * @return The writeback request for the block.
     */
    PacketPtr writebackBlk(CacheBlk *blk);

    /**
     * Create a writeclean request for the given block.
     *
     * Creates a request that writes the block to the cache below
     * without evicting the block from the current cache.
     *
     * @param blk The block to write clean.
     * @param dest The destination of the write clean operation.
     * @param id Use the given packet id for the write clean operation.
     * @return The generated write clean packet.
     */
    PacketPtr writecleanBlk(CacheBlk *blk, Request::Flags dest, PacketId id);

    /**
     * Write back dirty blocks in the cache using functional accesses.
     */
    virtual void memWriteback() override;

    /**
     * Invalidates all blocks in the cache.
     *
     * @warn Dirty cache lines will not be written back to
     * memory. Make sure to call functionalWriteback() first if you
     * want the to write them to memory.
     */
    virtual void memInvalidate() override;

    /**
     * Determine if there are any dirty blocks in the cache.
     *
     * @return true if at least one block is dirty, false otherwise.
     */
    bool isDirty() const;

    /**
     * Determine if an address is in the ranges covered by this
     * cache. This is useful to filter snoops.
     *
     * @param addr Address to check against
     *
     * @return If the address in question is in range
     */
    bool inRange(Addr addr) const;

    /**
     * Find next request ready time from among possible sources.
     */
    Tick nextQueueReadyTime() const;

    /** Block size of this cache */
    const unsigned blkSize;

    /**
     * The latency of tag lookup of a cache. It occurs when there is
     * an access to the cache.
     */
    const Cycles lookupLatency;

    /**
     * The latency of data access of a cache. It occurs when there is
     * an access to the cache.
     */
    const Cycles dataLatency;

    /**
     * This is the forward latency of the cache. It occurs when there
     * is a cache miss and a request is forwarded downstream, in
     * particular an outbound miss.
     */
    const Cycles forwardLatency;

    /** The latency to fill a cache block */
    const Cycles fillLatency;

    /**
     * The latency of sending reponse to its upper level cache/core on
     * a linefill. The responseLatency parameter captures this
     * latency.
     */
    const Cycles responseLatency;

    /**
     * Whether tags and data are accessed sequentially.
     */
    const bool sequentialAccess;

    /** The number of targets for each MSHR. */
    const int numTarget;

    /** Do we forward snoops from mem side port through to cpu side port? */
    bool forwardSnoops;

    /**
     * Clusivity with respect to the upstream cache, determining if we
     * fill into both this cache and the cache above on a miss. Note
     * that we currently do not support strict clusivity policies.
     */
    const enums::Clusivity clusivity;

    /**
     * Is this cache read only, for example the instruction cache, or
     * table-walker cache. A cache that is read only should never see
     * any writes, and should never get any dirty data (and hence
     * never have to do any writebacks).
     */
    const bool isReadOnly;

    /**
     * when a data expansion of a compressed block happens it will not be
     * able to co-allocate where it is at anymore. If true, the replacement
     * policy is called to chose a new location for the block. Otherwise,
     * all co-allocated blocks are evicted.
     */
    const bool replaceExpansions;

    /**
     * Similar to data expansions, after a block improves its compression,
     * it may need to be moved elsewhere compatible with the new compression
     * factor, or, if not required by the compaction method, it may be moved
     * to co-allocate with an existing block and thus free an entry.
     */
    const bool moveContractions;

    /**
     * Bit vector of the blocking reasons for the access path.
     * @sa #BlockedCause
     */
    uint8_t blocked;

    /** Increasing order number assigned to each incoming request. */
    uint64_t order;

    /** Stores time the cache blocked for statistics. */
    Cycles blockedCycle;

    /** Pointer to the MSHR that has no targets. */
    MSHR *noTargetMSHR;

    /** The number of misses to trigger an exit event. */
    Counter missCount;

    /**
     * The address range to which the cache responds on the CPU side.
     * Normally this is all possible memory addresses. */
    const AddrRangeList addrRanges;

  public:
    /** System we are currently operating in. */
    System *system;

    struct CacheCmdStats : public statistics::Group
    {
        CacheCmdStats(BaseCache &c, const std::string &name);

        /**
         * Callback to register stats from parent
         * CacheStats::regStats(). We can't use the normal flow since
         * there is is no guaranteed order and CacheStats::regStats()
         * needs to rely on these stats being initialised.
         */
        void regStatsFromParent();

        const BaseCache &cache;

        /** Number of hits per thread for each type of command.
            @sa Packet::Command */
        statistics::Vector hits;
        /** Number of misses per thread for each type of command.
            @sa Packet::Command */
        statistics::Vector misses;
        /**
         * Total number of ticks per thread/command spent waiting for a hit.
         * Used to calculate the average hit latency.
         */
        statistics::Vector hitLatency;
        /**
         * Total number of ticks per thread/command spent waiting for a miss.
         * Used to calculate the average miss latency.
         */
        statistics::Vector missLatency;
        /** The number of accesses per command and thread. */
        statistics::Formula accesses;
        /** The miss rate per command and thread. */
        statistics::Formula missRate;
        /** The average miss latency per command and thread. */
        statistics::Formula avgMissLatency;
        /** Number of misses that hit in the MSHRs per command and thread. */
        statistics::Vector mshrHits;
        /** Number of misses that miss in the MSHRs, per command and thread. */
        statistics::Vector mshrMisses;
        /** Number of misses that miss in the MSHRs, per command and thread. */
        statistics::Vector mshrUncacheable;
        /** Total tick latency of each MSHR miss, per command and thread. */
        statistics::Vector mshrMissLatency;
        /** Total tick latency of each MSHR miss, per command and thread. */
        statistics::Vector mshrUncacheableLatency;
        /** The miss rate in the MSHRs pre command and thread. */
        statistics::Formula mshrMissRate;
        /** The average latency of an MSHR miss, per command and thread. */
        statistics::Formula avgMshrMissLatency;
        /** The average latency of an MSHR miss, per command and thread. */
        statistics::Formula avgMshrUncacheableLatency;
    };

    struct CacheStats : public statistics::Group
    {
        CacheStats(BaseCache &c);

        void regStats() override;

        CacheCmdStats &
        cmdStats(const PacketPtr p)
        {
            return *cmd[p->cmdToIndex()];
        }

        const BaseCache &cache;

        /** Number of hits for demand accesses. */
        statistics::Formula demandHits;
        /** Number of hit for all accesses. */
        statistics::Formula overallHits;
        /** Total number of ticks spent waiting for demand hits. */
        statistics::Formula demandHitLatency;
        /** Total number of ticks spent waiting for all hits. */
        statistics::Formula overallHitLatency;

        /** Number of misses for demand accesses. */
        statistics::Formula demandMisses;
        /** Number of misses for all accesses. */
        statistics::Formula overallMisses;

        /** Total number of ticks spent waiting for demand misses. */
        statistics::Formula demandMissLatency;
        /** Total number of ticks spent waiting for all misses. */
        statistics::Formula overallMissLatency;

        /** The number of demand accesses. */
        statistics::Formula demandAccesses;
        /** The number of overall accesses. */
        statistics::Formula overallAccesses;

        /** The miss rate of all demand accesses. */
        statistics::Formula demandMissRate;
        /** The miss rate for all accesses. */
        statistics::Formula overallMissRate;

        /** The average miss latency for demand misses. */
        statistics::Formula demandAvgMissLatency;
        /** The average miss latency for all misses. */
        statistics::Formula overallAvgMissLatency;

        /** The total number of cycles blocked for each blocked cause. */
        statistics::Vector blockedCycles;
        /** The number of times this cache blocked for each blocked cause. */
        statistics::Vector blockedCauses;

        /** The average number of cycles blocked for each blocked cause. */
        statistics::Formula avgBlocked;

        /** Number of blocks written back per thread. */
        statistics::Vector writebacks;

        /** Demand misses that hit in the MSHRs. */
        statistics::Formula demandMshrHits;
        /** Total number of misses that hit in the MSHRs. */
        statistics::Formula overallMshrHits;

        /** Demand misses that miss in the MSHRs. */
        statistics::Formula demandMshrMisses;
        /** Total number of misses that miss in the MSHRs. */
        statistics::Formula overallMshrMisses;

        /** Total number of misses that miss in the MSHRs. */
        statistics::Formula overallMshrUncacheable;

        /** Total tick latency of demand MSHR misses. */
        statistics::Formula demandMshrMissLatency;
        /** Total tick latency of overall MSHR misses. */
        statistics::Formula overallMshrMissLatency;

        /** Total tick latency of overall MSHR misses. */
        statistics::Formula overallMshrUncacheableLatency;

        /** The demand miss rate in the MSHRs. */
        statistics::Formula demandMshrMissRate;
        /** The overall miss rate in the MSHRs. */
        statistics::Formula overallMshrMissRate;

        /** The average latency of a demand MSHR miss. */
        statistics::Formula demandAvgMshrMissLatency;
        /** The average overall latency of an MSHR miss. */
        statistics::Formula overallAvgMshrMissLatency;

        /** The average overall latency of an MSHR miss. */
        statistics::Formula overallAvgMshrUncacheableLatency;

        /** Number of replacements of valid blocks. */
        statistics::Scalar replacements;

        /** Number of data expansions. */
        statistics::Scalar dataExpansions;

        /**
         * Number of data contractions (blocks that had their compression
         * factor improved).
         */
        statistics::Scalar dataContractions;

        /** Per-command statistics */
        std::vector<std::unique_ptr<CacheCmdStats>> cmd;
    } stats;

    /** Registers probes. */
    void regProbePoints() override;

  public:
    BaseCache(const BaseCacheParams &p, unsigned blk_size);
    ~BaseCache();

    void init() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    /**
     * Query block size of a cache.
     * @return  The block size
     */
    unsigned
    getBlockSize() const
    {
        return blkSize;
    }

    const AddrRangeList &
    getAddrRanges() const
    {
        return addrRanges;
    }

    MSHR *
    allocateMissBuffer(PacketPtr pkt, Tick time, bool sched_send = true)
    {
        MSHR *mshr =
            mshrQueue.allocate(pkt->getBlockAddr(blkSize), blkSize, pkt, time,
                               order++, allocOnFill(pkt->cmd));

        if (mshrQueue.isFull()) {
            setBlocked((BlockedCause)MSHRQueue_MSHRs);
        }

        if (sched_send) {
            // schedule the send
            schedMemSideSendEvent(time);
        }

        return mshr;
    }

    void
    allocateWriteBuffer(PacketPtr pkt, Tick time)
    {
        // should only see writes or clean evicts here
        assert(pkt->isWrite() || pkt->cmd == MemCmd::CleanEvict);

        Addr blk_addr = pkt->getBlockAddr(blkSize);

        // If using compression, on evictions the block is decompressed and
        // the operation's latency is added to the payload delay. Consume
        // that payload delay here, meaning that the data is always stored
        // uncompressed in the writebuffer
        if (compressor) {
            time += pkt->payloadDelay;
            pkt->payloadDelay = 0;
        }

        WriteQueueEntry *wq_entry =
            writeBuffer.findMatch(blk_addr, pkt->isSecure());
        if (wq_entry && !wq_entry->inService) {
            DPRINTF(Cache, "Potential to merge writeback %s", pkt->print());
        }

        writeBuffer.allocate(blk_addr, blkSize, pkt, time, order++);

        if (writeBuffer.isFull()) {
            setBlocked((BlockedCause)MSHRQueue_WriteBuffer);
        }

        // schedule the send
        schedMemSideSendEvent(time);
    }

    /**
     * Returns true if the cache is blocked for accesses.
     */
    bool
    isBlocked() const
    {
        return blocked != 0;
    }

    /**
     * Marks the access path of the cache as blocked for the given cause. This
     * also sets the blocked flag in the response interface.
     * @param cause The reason for the cache blocking.
     */
    void
    setBlocked(BlockedCause cause)
    {
        uint8_t flag = 1 << cause;
        if (blocked == 0) {
            stats.blockedCauses[cause]++;
            blockedCycle = curCycle();
            cpuSidePort.setBlocked();
        }
        blocked |= flag;
        DPRINTF(Cache, "Blocking for cause %d, mask=%d\n", cause, blocked);
    }

    /**
     * Marks the cache as unblocked for the given cause. This also clears the
     * blocked flags in the appropriate interfaces.
     * @param cause The newly unblocked cause.
     * @warning Calling this function can cause a blocked request on the bus to
     * access the cache. The cache must be in a state to handle that request.
     */
    void
    clearBlocked(BlockedCause cause)
    {
        uint8_t flag = 1 << cause;
        blocked &= ~flag;
        DPRINTF(Cache, "Unblocking for cause %d, mask=%d\n", cause, blocked);
        if (blocked == 0) {
            stats.blockedCycles[cause] += curCycle() - blockedCycle;
            cpuSidePort.clearBlocked();
        }
    }

    /**
     * Schedule a send event for the memory-side port. If already
     * scheduled, this may reschedule the event at an earlier
     * time. When the specified time is reached, the port is free to
     * send either a response, a request, or a prefetch request.
     *
     * @param time The time when to attempt sending a packet.
     */
    void
    schedMemSideSendEvent(Tick time)
    {
        memSidePort.schedSendEvent(time);
    }

    bool
    inCache(Addr addr, bool is_secure) const
    {
        return tags->findBlock(addr, is_secure);
    }

    bool
    hasBeenPrefetched(Addr addr, bool is_secure) const
    {
        CacheBlk *block = tags->findBlock(addr, is_secure);
        return block && block->wasPrefetched();
    }

    bool
    hasBeenPrefetched(Addr addr, bool is_secure, RequestorID requestor) const
    {
        CacheBlk *block = tags->findBlock(addr, is_secure);
        return block && block->wasPrefetched() &&
               (block->getSrcRequestorId() == requestor);
    }

    bool
    inMissQueue(Addr addr, bool is_secure) const
    {
        return mshrQueue.findMatch(addr, is_secure);
    }

    void
    incMissCount(PacketPtr pkt)
    {
        assert(pkt->req->requestorId() < system->maxRequestors());
        stats.cmdStats(pkt).misses[pkt->req->requestorId()]++;
        pkt->req->incAccessDepth();
        if (missCount) {
            --missCount;
            if (missCount == 0)
                exitSimLoop("A cache reached the maximum miss count");
        }
    }

    void
    incHitCount(PacketPtr pkt)
    {
        assert(pkt->req->requestorId() < system->maxRequestors());
        stats.cmdStats(pkt).hits[pkt->req->requestorId()]++;
    }

    /**
     * Checks if the cache is coalescing writes
     *
     * @return True if the cache is coalescing writes
     */
    bool coalesce() const;

    /**
     * Cache block visitor that writes back dirty cache blocks using
     * functional writes.
     */
    void writebackVisitor(CacheBlk &blk);

    /**
     * Cache block visitor that invalidates all blocks in the cache.
     *
     * @warn Dirty cache lines will not be written back to memory.
     */
    void invalidateVisitor(CacheBlk &blk);

    /**
     * Take an MSHR, turn it into a suitable downstream packet, and
     * send it out. This construct allows a queue entry to choose a suitable
     * approach based on its type.
     *
     * @param mshr The MSHR to turn into a packet and send
     * @return True if the port is waiting for a retry
     */
    virtual bool sendMSHRQueuePacket(MSHR *mshr);

    /**
     * Similar to sendMSHR, but for a write-queue entry
     * instead. Create the packet, and send it, and if successful also
     * mark the entry in service.
     *
     * @param wq_entry The write-queue entry to turn into a packet and send
     * @return True if the port is waiting for a retry
     */
    bool sendWriteQueuePacket(WriteQueueEntry *wq_entry);

    /**
     * Serialize the state of the caches
     *
     * We currently don't support checkpointing cache state, so this panics.
     */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

/**
 * The write allocator inspects write packets and detects streaming
 * patterns. The write allocator supports a single stream where writes
 * are expected to access consecutive locations and keeps track of
 * size of the area covered by the concecutive writes in byteCount.
 *
 * 1) When byteCount has surpassed the coallesceLimit the mode
 * switches from ALLOCATE to COALESCE where writes should be delayed
 * until the whole block is written at which point a single packet
 * (whole line write) can service them.
 *
 * 2) When byteCount has also exceeded the noAllocateLimit (whole
 * line) we switch to NO_ALLOCATE when writes should not allocate in
 * the cache but rather send a whole line write to the memory below.
 */
class WriteAllocator : public SimObject
{
  public:
    WriteAllocator(const WriteAllocatorParams &p)
        : SimObject(p),
          coalesceLimit(p.coalesce_limit * p.block_size),
          noAllocateLimit(p.no_allocate_limit * p.block_size),
          delayThreshold(p.delay_threshold)
    {
        reset();
    }

    /**
     * Should writes be coalesced? This is true if the mode is set to
     * NO_ALLOCATE.
     *
     * @return return true if the cache should coalesce writes.
     */
    bool
    coalesce() const
    {
        return mode != WriteMode::ALLOCATE;
    }

    /**
     * Should writes allocate?
     *
     * @return return true if the cache should not allocate for writes.
     */
    bool
    allocate() const
    {
        return mode != WriteMode::NO_ALLOCATE;
    }

    /**
     * Reset the write allocator state, meaning that it allocates for
     * writes and has not recorded any information about qualifying
     * writes that might trigger a switch to coalescing and later no
     * allocation.
     */
    void
    reset()
    {
        mode = WriteMode::ALLOCATE;
        byteCount = 0;
        nextAddr = 0;
    }

    /**
     * Access whether we need to delay the current write.
     *
     * @param blk_addr The block address the packet writes to
     * @return true if the current packet should be delayed
     */
    bool
    delay(Addr blk_addr)
    {
        if (delayCtr[blk_addr] > 0) {
            --delayCtr[blk_addr];
            return true;
        } else {
            return false;
        }
    }

    /**
     * Clear delay counter for the input block
     *
     * @param blk_addr The accessed cache block
     */
    void
    resetDelay(Addr blk_addr)
    {
        delayCtr.erase(blk_addr);
    }

    /**
     * Update the write mode based on the current write
     * packet. This method compares the packet's address with any
     * current stream, and updates the tracking and the mode
     * accordingly.
     *
     * @param write_addr Start address of the write request
     * @param write_size Size of the write request
     * @param blk_addr The block address that this packet writes to
     */
    void updateMode(Addr write_addr, unsigned write_size, Addr blk_addr);

  private:
    /**
     * The current mode for write coalescing and allocation, either
     * normal operation (ALLOCATE), write coalescing (COALESCE), or
     * write coalescing without allocation (NO_ALLOCATE).
     */
    enum class WriteMode : char
    {
        ALLOCATE,
        COALESCE,
        NO_ALLOCATE,
    };
    WriteMode mode;

    /** Address to match writes against to detect streams. */
    Addr nextAddr;

    /**
     * Bytes written contiguously. Saturating once we no longer
     * allocate.
     */
    uint32_t byteCount;

    /**
     * Limits for when to switch between the different write modes.
     */
    const uint32_t coalesceLimit;
    const uint32_t noAllocateLimit;
    /**
     * The number of times the allocator will delay an WriteReq MSHR.
     */
    const uint32_t delayThreshold;

    /**
     * Keep track of the number of times the allocator has delayed an
     * WriteReq MSHR.
     */
    std::unordered_map<Addr, Counter> delayCtr;
};

} // namespace gem5

#endif //__MEM_CACHE_BASE_HH__
