/*
 * Copyright (c) 2022-2023 The University of Edinburgh
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

#include "cpu/base.hh"
#include "cpu/o3/ftq.hh"
#include "mem/cache/prefetch/base.hh"

namespace gem5
{

struct FetchDirectedPrefetcherParams;

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

class FetchDirectedPrefetcher : public Base
{

  public:
    FetchDirectedPrefetcher(const FetchDirectedPrefetcherParams &p);
    ~FetchDirectedPrefetcher() = default;


    /** Base class overrides */
    void regProbeListeners() override;

    /** Gets a packet from the prefetch queue to be prefetched. */
    PacketPtr getPacket() override;

    Tick nextPrefetchReadyTime() const override
    {
        return pfq.empty() ? MaxTick : pfq.front().readyTime;
    }

    /** Notify functions are not used by this prefetcher. */
    void notify(const PacketPtr &pkt, const PrefetchInfo &pfi) override {};
    void notifyFill(const PacketPtr &pkt) override{};

  private:

    /** Array of probe listeners */
    std::vector<ProbeListener *> listeners;

    /** Pointer to the CPU object that contains the FTQ */
    BaseCPU *cpu;

    /** For testing purposes */
    const bool transFunctional;

    /** The latency of the prefetcher */
    const unsigned int latency;

    /** Probe the cache before a prefetch gets inserted into the PFQ*/
    const bool cacheSnoop;

    /** The prefetch queue entry objects */
    struct PFQEntry
    {
        PFQEntry(uint64_t _addr, PacketPtr p, Tick t)
            : addr(_addr), pkt(p), readyTime(t) {}

        /** The virtual address. Used to scan for redundand prefetches.*/
        uint64_t addr;

        /** The packet that will be sent to the cache. */
        PacketPtr pkt;

        /** The time when the prefetch is ready to be sent to the cache. */
        Tick readyTime;
        bool operator==(const int& a) const {
            return this->addr == a;
        }
    };

    /** The prefetch queue */
    std::list<PFQEntry> pfq;


    /** Notifies the prefetcher that a new fetch target was
     * inserted into the FTQ. */
    void notifyFTQInsert(const o3::FetchTargetPtr& ft);

    /** Notifies the prefetcher that a fetch target was
     * removed from the FTQ */
    void notifyFTQRemove(const o3::FetchTargetPtr& ft);

    /** Adds a prefetch candidate to the prefetch queue.
     * Performs the translation of the virtual address to a physical address,
     * probes the cache if enabled, and creates a prefetch packet which is
     * inserted into the prefetch queue.
     * @param addr is the start address of the fetch target
     * @param va is true if the address is a virtual address
     * */
    void notifyPfAddr(Addr addr, bool va=false);

    /** Creates a prefetch request for the given virtual address. */
    RequestPtr createPrefetchRequest(Addr vaddr);

    /** Creates a prefetch packet for the given address. */
    PacketPtr createPrefetchPacket(Addr addr, bool va=false);

    /** Performs a functional translation of the incomming packet by useing
     * the CPU's TLB. */
    bool translateFunctional(RequestPtr req);


  protected:
    struct Stats : public statistics::Group
    {
        Stats(statistics::Group *parent);
        statistics::Scalar fdipInsertions;

        statistics::Scalar pfIdentified;
        statistics::Scalar pfInCache;
        statistics::Scalar pfInCachePrefetched;
        statistics::Scalar pfPacketsCreated;
        statistics::Scalar pfCandidatesAdded;

        statistics::Scalar translationFail;
        statistics::Scalar translationSuccess;
    } stats;
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_FDP_HH__
