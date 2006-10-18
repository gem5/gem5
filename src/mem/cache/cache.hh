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

#include "base/misc.hh" // fatal, panic, and warn
#include "cpu/smt.hh" // SMT_MAX_THREADS

#include "mem/cache/base_cache.hh"
#include "mem/cache/prefetch/prefetcher.hh"

//Forward decleration
class MSHR;


/**
 * A template-policy based cache. The behavior of the cache can be altered by
 * supplying different template policies. TagStore handles all tag and data
 * storage @sa TagStore. Buffering handles all misses and writes/writebacks
 * @sa MissQueue. Coherence handles all coherence policy details @sa
 * UniCoherence, SimpleMultiCoherence.
 */
template <class TagStore, class Buffering, class Coherence>
class Cache : public BaseCache
{
  public:
    /** Define the type of cache block to use. */
    typedef typename TagStore::BlkType BlkType;

    bool prefetchAccess;
  protected:

    /** Tag and data Storage */
    TagStore *tags;
    /** Miss and Writeback handler */
    Buffering *missQueue;
    /** Coherence protocol. */
    Coherence *coherence;

    /** Prefetcher */
    Prefetcher<TagStore, Buffering> *prefetcher;

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
    Packet * invalidatePkt;
    Request *invalidateReq;

  public:

    class Params
    {
      public:
        TagStore *tags;
        Buffering *missQueue;
        Coherence *coherence;
        BaseCache::Params baseParams;
        Prefetcher<TagStore, Buffering> *prefetcher;
        bool prefetchAccess;
        int hitLatency;

        Params(TagStore *_tags, Buffering *mq, Coherence *coh,
               BaseCache::Params params,
               Prefetcher<TagStore, Buffering> *_prefetcher,
               bool prefetch_access, int hit_latency)
            : tags(_tags), missQueue(mq), coherence(coh),
              baseParams(params),
              prefetcher(_prefetcher), prefetchAccess(prefetch_access),
              hitLatency(hit_latency)
        {
        }
    };

    /** Instantiates a basic cache object. */
    Cache(const std::string &_name, Params &params);

    virtual bool doTimingAccess(Packet *pkt, CachePort *cachePort,
                        bool isCpuSide);

    virtual Tick doAtomicAccess(Packet *pkt, bool isCpuSide);

    virtual void doFunctionalAccess(Packet *pkt, bool isCpuSide);

    virtual void recvStatusChange(Port::Status status, bool isCpuSide);

    void regStats();

    /**
     * Performs the access specified by the request.
     * @param pkt The request to perform.
     * @return The result of the access.
     */
    bool access(Packet * &pkt);

    /**
     * Selects a request to send on the bus.
     * @return The memory request to service.
     */
    virtual Packet * getPacket();

    /**
     * Was the request was sent successfully?
     * @param pkt The request.
     * @param success True if the request was sent successfully.
     */
    virtual void sendResult(Packet * &pkt, MSHR* mshr, bool success);

    /**
     * Was the CSHR request was sent successfully?
     * @param pkt The request.
     * @param success True if the request was sent successfully.
     */
    virtual void sendCoherenceResult(Packet * &pkt, MSHR* cshr, bool success);

    /**
     * Handles a response (cache line fill/write ack) from the bus.
     * @param pkt The request being responded to.
     */
    void handleResponse(Packet * &pkt);

    /**
     * Selects a coherence message to forward to lower levels of the hierarchy.
     * @return The coherence message to forward.
     */
    virtual Packet * getCoherencePacket();

    /**
     * Snoops bus transactions to maintain coherence.
     * @param pkt The current bus transaction.
     */
    void snoop(Packet * &pkt);

    void snoopResponse(Packet * &pkt);

    /**
     * Invalidates the block containing address if found.
     * @param addr The address to look for.
     * @param asid The address space ID of the address.
     * @todo Is this function necessary?
     */
    void invalidateBlk(Addr addr);

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
    Tick probe(Packet * &pkt, bool update, CachePort * otherSidePort);

    /**
     * Snoop for the provided request in the cache and return the estimated
     * time of completion.
     * @todo Can a snoop probe not change state?
     * @param pkt The memory request to satisfy
     * @param update If true, update the hierarchy, otherwise just perform the
     * request.
     * @return The estimated completion time.
     */
    Tick snoopProbe(Packet * &pkt);
};

#endif // __CACHE_HH__
