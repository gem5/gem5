/*
 * Copyright (c) 2013 ARM Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
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
 * Authors: Ron Dreslinski
 */

/**
 * @file
 * Miss and writeback queue declarations.
 */

#ifndef __MEM_CACHE_PREFETCH_BASE_PREFETCHER_HH__
#define __MEM_CACHE_PREFETCH_BASE_PREFETCHER_HH__

#include <list>

#include "base/statistics.hh"
#include "mem/packet.hh"
#include "params/BaseCache.hh"
#include "sim/clocked_object.hh"

class BaseCache;

class BasePrefetcher : public ClockedObject
{
  protected:

    /** A deferred packet, buffered to transmit later. */
    class DeferredPacket {
      public:
        Tick tick;      ///< The tick when the packet is ready to transmit
        PacketPtr pkt;  ///< Pointer to the packet to transmit
        DeferredPacket(Tick t, PacketPtr p)
            : tick(t), pkt(p)
        {}
    };

    /** The Prefetch Queue. */
    std::list<DeferredPacket> pf;

    // PARAMETERS

    /** The number of MSHRs in the Prefetch Queue. */
    const unsigned size;

    /** Pointr to the parent cache. */
    BaseCache* cache;

    /** The block size of the parent cache. */
    int blkSize;

    /** The latency before a prefetch is issued */
    const Cycles latency;

    /** The number of prefetches to issue */
    unsigned degree;

    /** If patterns should be found per context id */
    bool useMasterId;
    /** Do we prefetch across page boundaries. */
    bool pageStop;

    /** Do we remove prefetches with later times than a new miss.*/
    bool serialSquash;

    /** Do we prefetch on only data reads, or on inst reads as well. */
    bool onlyData;

    /** System we belong to */
    System* system;

    /** Request id for prefetches */
    MasterID masterId;

  public:

    Stats::Scalar pfIdentified;
    Stats::Scalar pfMSHRHit;
    Stats::Scalar pfCacheHit;
    Stats::Scalar pfBufferHit;
    Stats::Scalar pfRemovedFull;
    Stats::Scalar pfRemovedMSHR;
    Stats::Scalar pfIssued;
    Stats::Scalar pfSpanPage;
    Stats::Scalar pfSquashed;

    void regStats();

  public:
    typedef BasePrefetcherParams Params;
    BasePrefetcher(const Params *p);

    virtual ~BasePrefetcher() {}

    void setCache(BaseCache *_cache);

    /**
     * Notify prefetcher of cache access (may be any access or just
     * misses, depending on cache parameters.)
     * @retval Time of next prefetch availability, or 0 if none.
     */
    Tick notify(PacketPtr &pkt, Tick tick);

    bool inCache(Addr addr, bool is_secure);

    bool inMissQueue(Addr addr, bool is_secure);

    PacketPtr getPacket();

    bool havePending()
    {
        return !pf.empty();
    }

    Tick nextPrefetchReadyTime()
    {
        return pf.empty() ? MaxTick : pf.front().tick;
    }

    virtual void calculatePrefetch(PacketPtr &pkt,
                                   std::list<Addr> &addresses,
                                   std::list<Cycles> &delays) = 0;

    std::list<DeferredPacket>::iterator inPrefetch(Addr address, bool is_secure);

    /**
     * Utility function: are addresses a and b on the same VM page?
     */
    bool samePage(Addr a, Addr b);
 public:
    const Params*
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

};
#endif //__MEM_CACHE_PREFETCH_BASE_PREFETCHER_HH__
