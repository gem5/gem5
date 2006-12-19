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
 *          Dave Greene
 *          Steve Reinhardt
 */

/**
 * @file
 * Describes a cache based on template policies.
 */

#ifndef __CACHE_HH__
#define __CACHE_HH__

#include "base/compression/base.hh"
#include "base/misc.hh" // fatal, panic, and warn
#include "cpu/smt.hh" // SMT_MAX_THREADS

#include "mem/cache/base_cache.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/miss/miss_buffer.hh"

//Forward decleration
class MSHR;
class BasePrefetcher;

/**
 * A template-policy based cache. The behavior of the cache can be altered by
 * supplying different template policies. TagStore handles all tag and data
 * storage @sa TagStore. Buffering handles all misses and writes/writebacks
 * @sa MissQueue. Coherence handles all coherence policy details @sa
 * UniCoherence, SimpleMultiCoherence.
 */
template <class TagStore, class Coherence>
class Cache : public BaseCache
{
  public:
    /** Define the type of cache block to use. */
    typedef typename TagStore::BlkType BlkType;
    /** A typedef for a list of BlkType pointers. */
    typedef typename TagStore::BlkList BlkList;

    bool prefetchAccess;

  protected:

    class CpuSidePort : public CachePort
    {
      public:
        CpuSidePort(const std::string &_name,
                    Cache<TagStore,Coherence> *_cache);

        // BaseCache::CachePort just has a BaseCache *; this function
        // lets us get back the type info we lost when we stored the
        // cache pointer there.
        Cache<TagStore,Coherence> *myCache() {
            return static_cast<Cache<TagStore,Coherence> *>(cache);
        }

        virtual bool recvTiming(PacketPtr pkt);

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);
    };

    class MemSidePort : public CachePort
    {
      public:
        MemSidePort(const std::string &_name,
                    Cache<TagStore,Coherence> *_cache);

        // BaseCache::CachePort just has a BaseCache *; this function
        // lets us get back the type info we lost when we stored the
        // cache pointer there.
        Cache<TagStore,Coherence> *myCache() {
            return static_cast<Cache<TagStore,Coherence> *>(cache);
        }

        virtual bool recvTiming(PacketPtr pkt);

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);
    };

    /** Tag and data Storage */
    TagStore *tags;
    /** Miss and Writeback handler */
    MissBuffer *missQueue;
    /** Coherence protocol. */
    Coherence *coherence;

    /** Prefetcher */
    BasePrefetcher *prefetcher;

    /**
     * The clock ratio of the outgoing bus.
     * Used for calculating critical word first.
     */
    int busRatio;

     /**
      * The bus width in bytes of the outgoing bus.
      * Used for calculating critical word first.
      */
    int busWidth;

    /**
     * The latency of a hit in this device.
     */
    int hitLatency;

     /**
      * A permanent mem req to always be used to cause invalidations.
      * Used to append to target list, to cause an invalidation.
      */
    PacketPtr invalidatePkt;
    Request *invalidateReq;

    /**
     * Policy class for performing compression.
     */
    CompressionAlgorithm *compressionAlg;

    /**
     * The block size of this cache. Set to value in the Tags object.
     */
    const int16_t blkSize;

    /**
     * Can this cache should allocate a block on a line-sized write miss.
     */
    const bool doFastWrites;

    const bool prefetchMiss;

    /**
     * Can the data can be stored in a compressed form.
     */
    const bool storeCompressed;

    /**
     * Do we need to compress blocks on writebacks (i.e. because
     * writeback bus is compressed but storage is not)?
     */
    const bool compressOnWriteback;

    /**
     * The latency of a compression operation.
     */
    const int16_t compLatency;

    /**
     * Should we use an adaptive compression scheme.
     */
    const bool adaptiveCompression;

    /**
     * Do writebacks need to be compressed (i.e. because writeback bus
     * is compressed), whether or not they're already compressed for
     * storage.
     */
    const bool writebackCompressed;

    /**
     * Compare the internal block data to the fast access block data.
     * @param blk The cache block to check.
     * @return True if the data is the same.
     */
    bool verifyData(BlkType *blk);

    /**
     * Update the internal data of the block. The data to write is assumed to
     * be in the fast access data.
     * @param blk The block with the data to update.
     * @param writebacks A list to store any generated writebacks.
     * @param compress_block True if we should compress this block
     */
    void updateData(BlkType *blk, PacketList &writebacks, bool compress_block);

    /**
     * Handle a replacement for the given request.
     * @param blk A pointer to the block, usually NULL
     * @param pkt The memory request to satisfy.
     * @param new_state The new state of the block.
     * @param writebacks A list to store any generated writebacks.
     */
    BlkType* doReplacement(BlkType *blk, PacketPtr &pkt,
                           CacheBlk::State new_state, PacketList &writebacks);

    /**
     * Does all the processing necessary to perform the provided request.
     * @param pkt The memory request to perform.
     * @param lat The latency of the access.
     * @param writebacks List for any writebacks that need to be performed.
     * @param update True if the replacement data should be updated.
     * @return Pointer to the cache block touched by the request. NULL if it
     * was a miss.
     */
    BlkType* handleAccess(PacketPtr &pkt, int & lat,
                          PacketList & writebacks, bool update = true);

    /**
     * Populates a cache block and handles all outstanding requests for the
     * satisfied fill request. This version takes an MSHR pointer and uses its
     * request to fill the cache block, while repsonding to its targets.
     * @param blk The cache block if it already exists.
     * @param mshr The MSHR that contains the fill data and targets to satisfy.
     * @param new_state The state of the new cache block.
     * @param writebacks List for any writebacks that need to be performed.
     * @return Pointer to the new cache block.
     */
    BlkType* handleFill(BlkType *blk, MSHR * mshr, CacheBlk::State new_state,
                        PacketList & writebacks, PacketPtr pkt);

    /**
     * Populates a cache block and handles all outstanding requests for the
     * satisfied fill request. This version takes two memory requests. One
     * contains the fill data, the other is an optional target to satisfy.
     * Used for Cache::probe.
     * @param blk The cache block if it already exists.
     * @param pkt The memory request with the fill data.
     * @param new_state The state of the new cache block.
     * @param writebacks List for any writebacks that need to be performed.
     * @param target The memory request to perform after the fill.
     * @return Pointer to the new cache block.
     */
    BlkType* handleFill(BlkType *blk, PacketPtr &pkt,
                        CacheBlk::State new_state,
                        PacketList & writebacks, PacketPtr target = NULL);

    /**
     * Sets the blk to the new state and handles the given request.
     * @param blk The cache block being snooped.
     * @param new_state The new coherence state for the block.
     * @param pkt The request to satisfy
     */
    void handleSnoop(BlkType *blk, CacheBlk::State new_state,
                     PacketPtr &pkt);

    /**
     * Sets the blk to the new state.
     * @param blk The cache block being snooped.
     * @param new_state The new coherence state for the block.
     */
    void handleSnoop(BlkType *blk, CacheBlk::State new_state);

    /**
     * Create a writeback request for the given block.
     * @param blk The block to writeback.
     * @return The writeback request for the block.
     */
    PacketPtr writebackBlk(BlkType *blk);

  public:

    class Params
    {
      public:
        TagStore *tags;
        MissBuffer *missQueue;
        Coherence *coherence;
        BaseCache::Params baseParams;
        BasePrefetcher*prefetcher;
        bool prefetchAccess;
        int hitLatency;
        CompressionAlgorithm *compressionAlg;
        const int16_t blkSize;
        const bool doFastWrites;
        const bool prefetchMiss;
        const bool storeCompressed;
        const bool compressOnWriteback;
        const int16_t compLatency;
        const bool adaptiveCompression;
        const bool writebackCompressed;

        Params(TagStore *_tags, MissBuffer *mq, Coherence *coh,
               BaseCache::Params params,
               BasePrefetcher *_prefetcher,
               bool prefetch_access, int hit_latency,
               bool do_fast_writes,
               bool store_compressed, bool adaptive_compression,
               bool writeback_compressed,
               CompressionAlgorithm *_compressionAlg, int comp_latency,
               bool prefetch_miss)
            : tags(_tags), missQueue(mq), coherence(coh),
              baseParams(params),
              prefetcher(_prefetcher), prefetchAccess(prefetch_access),
              hitLatency(hit_latency),
              compressionAlg(_compressionAlg),
              blkSize(_tags->getBlockSize()),
              doFastWrites(do_fast_writes),
              prefetchMiss(prefetch_miss),
              storeCompressed(store_compressed),
              compressOnWriteback(!store_compressed && writeback_compressed),
              compLatency(comp_latency),
              adaptiveCompression(adaptive_compression),
              writebackCompressed(writeback_compressed)
        {
        }
    };

    /** Instantiates a basic cache object. */
    Cache(const std::string &_name, Params &params);

    virtual Port *getPort(const std::string &if_name, int idx = -1);

    virtual void recvStatusChange(Port::Status status, bool isCpuSide);

    void regStats();

    /**
     * Performs the access specified by the request.
     * @param pkt The request to perform.
     * @return The result of the access.
     */
    bool access(PacketPtr &pkt);

    /**
     * Selects a request to send on the bus.
     * @return The memory request to service.
     */
    virtual PacketPtr getPacket();

    /**
     * Was the request was sent successfully?
     * @param pkt The request.
     * @param success True if the request was sent successfully.
     */
    virtual void sendResult(PacketPtr &pkt, MSHR* mshr, bool success);

    /**
     * Was the CSHR request was sent successfully?
     * @param pkt The request.
     * @param success True if the request was sent successfully.
     */
    virtual void sendCoherenceResult(PacketPtr &pkt, MSHR* cshr, bool success);

    /**
     * Handles a response (cache line fill/write ack) from the bus.
     * @param pkt The request being responded to.
     */
    void handleResponse(PacketPtr &pkt);

    /**
     * Selects a coherence message to forward to lower levels of the hierarchy.
     * @return The coherence message to forward.
     */
    virtual PacketPtr getCoherencePacket();

    /**
     * Snoops bus transactions to maintain coherence.
     * @param pkt The current bus transaction.
     */
    void snoop(PacketPtr &pkt);

    void snoopResponse(PacketPtr &pkt);

    /**
     * Squash all requests associated with specified thread.
     * intended for use by I-cache.
     * @param threadNum The thread to squash.
     */
    void squash(int threadNum)
    {
        missQueue->squash(threadNum);
    }

    /**
     * Return the number of outstanding misses in a Cache.
     * Default returns 0.
     *
     * @retval unsigned The number of missing still outstanding.
     */
    unsigned outstandingMisses() const
    {
        return missQueue->getMisses();
    }

    /**
     * Perform the access specified in the request and return the estimated
     * time of completion. This function can either update the hierarchy state
     * or just perform the access wherever the data is found depending on the
     * state of the update flag.
     * @param pkt The memory request to satisfy
     * @param update If true, update the hierarchy, otherwise just perform the
     * request.
     * @return The estimated completion time.
     */
    Tick probe(PacketPtr &pkt, bool update, CachePort * otherSidePort);

    /**
     * Snoop for the provided request in the cache and return the estimated
     * time of completion.
     * @todo Can a snoop probe not change state?
     * @param pkt The memory request to satisfy
     * @param update If true, update the hierarchy, otherwise just perform the
     * request.
     * @return The estimated completion time.
     */
    Tick snoopProbe(PacketPtr &pkt);

    bool inCache(Addr addr) {
        return (tags->findBlock(addr) != 0);
    }

    bool inMissQueue(Addr addr) {
        return (missQueue->findMSHR(addr) != 0);
    }
};

#endif // __CACHE_HH__
