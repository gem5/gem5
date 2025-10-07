/*
 * Copyright (c) 2022-2023 The University of Edinburgh
 * Copyright (c) 2025 Arm Limited
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
 */

/**
 * Implementation of the fetch directed instruction prefetcher.
 */

#ifndef __MEM_CACHE_PREFETCH_FDP_HH__
#define __MEM_CACHE_PREFETCH_FDP_HH__

#include <list>

#include "arch/generic/mmu.hh"
#include "cpu/base.hh"
#include "cpu/o3/ftq.hh"
#include "mem/cache/base.hh"
#include "mem/cache/prefetch/base.hh"

namespace gem5
{

struct FetchDirectedPrefetcherParams;

namespace prefetch
{

class FetchDirectedPrefetcher : public Base
{

  public:
    FetchDirectedPrefetcher(const FetchDirectedPrefetcherParams &p);
    ~FetchDirectedPrefetcher() = default;

    /** Base class overrides */
    void regProbeListeners() override;
    void
    setCache(BaseCache *_cache)
    {
        cache = _cache;
    }

    /** Gets a packet from the prefetch queue to be prefetched. */
    PacketPtr getPacket() override;

    Tick
    nextPrefetchReadyTime() const override
    {
        return pfq.empty() ? MaxTick : pfq.front().readyTime;
    }

    /** Notify functions are not used by this prefetcher. */
    void notify(const CacheAccessProbeArg &acc,
                const PrefetchInfo &pfi) override {};

  private:
    /** Array of probe listeners */
    std::vector<ProbeListenerPtr<>> listeners;

    /** Pointer to the CPU object that contains the FTQ */
    BaseCPU *cpu;

    /** Pointer to the cache it is attached to */
    BaseCache *cache;

    /** Mark memory requests as prefetches. */
    const bool markReqAsPrefetch;

    /** Squash prefetches in case its fetch target is removed from the FTQ. */
    const bool squashPrefetches;

    /** The latency of the prefetcher */
    const unsigned int latency;

    /** Prefetch queue size: Maximum number of queued prefetches */
    const unsigned int pfqSize;

    /** Translation queue size: Maximum number of outstanding translations */
    const unsigned int tqSize;

    /** Probe the cache before a prefetch gets inserted into the PFQ */
    const bool cacheSnoop;

    /** The prefetch queue entry objects */
    struct PrefetchRequest : public BaseMMU::Translation
    {
        PrefetchRequest(FetchDirectedPrefetcher &_owner, uint64_t _addr,
                        ThreadID tid, o3::FTSeqNum ftn);

        /** Owner of the packet */
        FetchDirectedPrefetcher &owner;

        /** The virtual address. Used to scan for redundant prefetches.*/
        const uint64_t addr;

        /** The fetch target number that created this request */
        const o3::FTSeqNum ftn;

        /** The request and packet that will be sent to the cache. */
        RequestPtr req;
        PacketPtr pkt;

        /** The time when the prefetch is ready to be sent to the cache. */
        Tick readyTime;

        /** Marks a Prefetch Request as canceled if notifyFTQRemove was
         * called during translation. In this case the prefetch will not
         * proceed to the prefetch queue. */
        bool canceled;

        bool
        operator==(const int &a) const
        {
            return this->addr == a;
        }

        /** Creates the packet that is send to the memory. */
        void createPkt();

        void finish(const Fault &fault, const RequestPtr &req,
                    ThreadContext *tc, BaseMMU::Mode mode) override;

        /** Issues the translation request */
        void startTranslation();

        void
        markDelayed() override
        {}

        void
        markCanceled()
        {
            canceled = true;
        }

        bool
        isCanceled() const
        {
            return canceled;
        }
    };

    /** The prefetch queue */
    std::list<PrefetchRequest> pfq;
    std::list<PrefetchRequest> translationq;

    /** Notifies the prefetcher that a new fetch target was
     * inserted into the FTQ. */
    void notifyFTQInsert(const o3::FetchTargetPtr &ft);

    /** Notifies the prefetcher that a fetch target was
     * removed from the FTQ */
    void notifyFTQRemove(const o3::FetchTargetPtr &ft);

    /** A translation has completed and can now be added to the PFQ. */
    void translationComplete(PrefetchRequest *pf_req, const bool failed);

  protected:
    struct Stats : public statistics::Group
    {
        Stats(statistics::Group *parent, int pfq_size, int tq_size);
        statistics::Scalar fdipInsertions;

        statistics::Scalar pfIdentified;
        statistics::Scalar pfSquashed;
        statistics::Scalar pfInPFQ;
        statistics::Scalar pfInTQ;
        statistics::Scalar pfInCache;
        statistics::Scalar pfInCachePrefetched;
        statistics::Scalar pfPacketsCreated;
        statistics::Scalar pfCandidatesAdded;

        statistics::Scalar translationFail;
        statistics::Scalar translationSuccess;

        statistics::Distribution pfqSizeDistAtNotify;
        statistics::Distribution tqSizeDistAtNotify;
        statistics::Scalar pfqInserts;
        statistics::Scalar pfqPops;
        statistics::Scalar pfqDrops;
        statistics::Scalar tqInserts;
        statistics::Scalar tqPops;
        statistics::Scalar tqDrops;
    } stats;
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_FDP_HH__
