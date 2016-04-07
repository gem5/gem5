/*
 * Copyright (c) 2014-2015 ARM Limited
 * All rights reserved
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
 * Authors: Mitch Hayenga
 */

#ifndef __MEM_CACHE_PREFETCH_QUEUED_HH__
#define __MEM_CACHE_PREFETCH_QUEUED_HH__

#include <list>

#include "mem/cache/prefetch/base.hh"
#include "params/QueuedPrefetcher.hh"

class QueuedPrefetcher : public BasePrefetcher
{
  protected:
    struct DeferredPacket {
        Tick tick;
        PacketPtr pkt;
        int32_t priority;
        DeferredPacket(Tick t, PacketPtr p, int32_t pr) : tick(t), pkt(p),
                                                        priority(pr)  {}
        bool operator>(const DeferredPacket& that) const
        {
            return priority > that.priority;
        }
        bool operator<(const DeferredPacket& that) const
        {
            return priority < that.priority;
        }
        bool operator<=(const DeferredPacket& that) const
        {
            return !(*this > that);
        }
    };
    using AddrPriority = std::pair<Addr, int32_t>;

    std::list<DeferredPacket> pfq;

    // PARAMETERS

    /** Maximum size of the prefetch queue */
    const unsigned queueSize;

    /** Cycles after generation when a prefetch can first be issued */
    const Cycles latency;

    /** Squash queued prefetch if demand access observed */
    const bool queueSquash;

    /** Filter prefetches if already queued */
    const bool queueFilter;

    /** Snoop the cache before generating prefetch (cheating basically) */
    const bool cacheSnoop;

    /** Tag prefetch with PC of generating access? */
    const bool tagPrefetch;

    using const_iterator = std::list<DeferredPacket>::const_iterator;
    std::list<DeferredPacket>::const_iterator inPrefetch(Addr address,
            bool is_secure) const;
    using iterator = std::list<DeferredPacket>::iterator;
    std::list<DeferredPacket>::iterator inPrefetch(Addr address,
            bool is_secure);

    // STATS
    Stats::Scalar pfIdentified;
    Stats::Scalar pfBufferHit;
    Stats::Scalar pfInCache;
    Stats::Scalar pfRemovedFull;
    Stats::Scalar pfSpanPage;

  public:
    QueuedPrefetcher(const QueuedPrefetcherParams *p);
    virtual ~QueuedPrefetcher();

    Tick notify(const PacketPtr &pkt);
    PacketPtr insert(AddrPriority& info, bool is_secure);

    // Note: This should really be pure virtual, but doesnt go well with params
    virtual void calculatePrefetch(const PacketPtr &pkt,
                                   std::vector<AddrPriority> &addresses) = 0;
    PacketPtr getPacket();

    Tick nextPrefetchReadyTime() const
    {
        return pfq.empty() ? MaxTick : pfq.front().tick;
    }

    void regStats();
};

#endif //__MEM_CACHE_PREFETCH_QUEUED_HH__

