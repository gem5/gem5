/*
 * Copyright (c) 2013-2014, 2022 Arm Limited
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
 */

/**
 * @file
 * Hardware Prefetcher Definition.
 */

#include "mem/cache/prefetch/base.hh"

#include <cassert>

#include "base/intmath.hh"
#include "mem/cache/base.hh"
#include "params/BasePrefetcher.hh"
#include "sim/system.hh"

namespace gem5
{

namespace prefetch
{

Base::PrefetchInfo::PrefetchInfo(PacketPtr pkt, Addr addr, bool miss)
  : address(addr), pc(pkt->req->hasPC() ? pkt->req->getPC() : 0),
    requestorId(pkt->req->requestorId()), validPC(pkt->req->hasPC()),
    secure(pkt->isSecure()), size(pkt->req->getSize()), write(pkt->isWrite()),
    paddress(pkt->req->getPaddr()), cacheMiss(miss)
{
    unsigned int req_size = pkt->req->getSize();
    if (!write && miss) {
        data = nullptr;
    } else {
        data = new uint8_t[req_size];
        Addr offset = pkt->req->getPaddr() - pkt->getAddr();
        std::memcpy(data, &(pkt->getConstPtr<uint8_t>()[offset]), req_size);
    }
}

Base::PrefetchInfo::PrefetchInfo(PrefetchInfo const &pfi, Addr addr)
  : address(addr), pc(pfi.pc), requestorId(pfi.requestorId),
    validPC(pfi.validPC), secure(pfi.secure), size(pfi.size),
    write(pfi.write), paddress(pfi.paddress), cacheMiss(pfi.cacheMiss),
    data(nullptr)
{
}

void
Base::PrefetchListener::notify(const PacketPtr &pkt)
{
    if (isFill) {
        parent.notifyFill(pkt);
    } else {
        parent.probeNotify(pkt, miss);
    }
}

Base::Base(const BasePrefetcherParams &p)
    : ClockedObject(p), listeners(), cache(nullptr), blkSize(p.block_size),
      lBlkSize(floorLog2(blkSize)), onMiss(p.on_miss), onRead(p.on_read),
      onWrite(p.on_write), onData(p.on_data), onInst(p.on_inst),
      requestorId(p.sys->getRequestorId(this)),
      pageBytes(p.page_bytes),
      prefetchOnAccess(p.prefetch_on_access),
      prefetchOnPfHit(p.prefetch_on_pf_hit),
      useVirtualAddresses(p.use_virtual_addresses),
      prefetchStats(this), issuedPrefetches(0),
      usefulPrefetches(0), mmu(nullptr)
{
}

void
Base::setCache(BaseCache *_cache)
{
    assert(!cache);
    cache = _cache;

    // If the cache has a different block size from the system's, save it
    blkSize = cache->getBlockSize();
    lBlkSize = floorLog2(blkSize);
}

Base::StatGroup::StatGroup(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(demandMshrMisses, statistics::units::Count::get(),
        "demands not covered by prefetchs"),
    ADD_STAT(pfIssued, statistics::units::Count::get(),
        "number of hwpf issued"),
    ADD_STAT(pfUnused, statistics::units::Count::get(),
             "number of HardPF blocks evicted w/o reference"),
    ADD_STAT(pfUseful, statistics::units::Count::get(),
        "number of useful prefetch"),
    ADD_STAT(pfUsefulButMiss, statistics::units::Count::get(),
        "number of hit on prefetch but cache block is not in an usable "
        "state"),
    ADD_STAT(accuracy, statistics::units::Count::get(),
        "accuracy of the prefetcher"),
    ADD_STAT(coverage, statistics::units::Count::get(),
    "coverage brought by this prefetcher"),
    ADD_STAT(pfHitInCache, statistics::units::Count::get(),
        "number of prefetches hitting in cache"),
    ADD_STAT(pfHitInMSHR, statistics::units::Count::get(),
        "number of prefetches hitting in a MSHR"),
    ADD_STAT(pfHitInWB, statistics::units::Count::get(),
        "number of prefetches hit in the Write Buffer"),
    ADD_STAT(pfLate, statistics::units::Count::get(),
        "number of late prefetches (hitting in cache, MSHR or WB)")
{
    using namespace statistics;

    pfUnused.flags(nozero);

    accuracy.flags(total);
    accuracy = pfUseful / pfIssued;

    coverage.flags(total);
    coverage = pfUseful / (pfUseful + demandMshrMisses);

    pfLate = pfHitInCache + pfHitInMSHR + pfHitInWB;
}

bool
Base::observeAccess(const PacketPtr &pkt, bool miss) const
{
    bool fetch = pkt->req->isInstFetch();
    bool read = pkt->isRead();
    bool inv = pkt->isInvalidate();

    if (!miss) {
        if (prefetchOnPfHit)
            return hasBeenPrefetched(pkt->getAddr(), pkt->isSecure());
        if (!prefetchOnAccess)
            return false;
    }
    if (pkt->req->isUncacheable()) return false;
    if (fetch && !onInst) return false;
    if (!fetch && !onData) return false;
    if (!fetch && read && !onRead) return false;
    if (!fetch && !read && !onWrite) return false;
    if (!fetch && !read && inv) return false;
    if (pkt->cmd == MemCmd::CleanEvict) return false;

    if (onMiss) {
        return miss;
    }

    return true;
}

bool
Base::inCache(Addr addr, bool is_secure) const
{
    return cache->inCache(addr, is_secure);
}

bool
Base::inMissQueue(Addr addr, bool is_secure) const
{
    return cache->inMissQueue(addr, is_secure);
}

bool
Base::hasBeenPrefetched(Addr addr, bool is_secure) const
{
    return cache->hasBeenPrefetched(addr, is_secure);
}

bool
Base::samePage(Addr a, Addr b) const
{
    return roundDown(a, pageBytes) == roundDown(b, pageBytes);
}

Addr
Base::blockAddress(Addr a) const
{
    return a & ~((Addr)blkSize-1);
}

Addr
Base::blockIndex(Addr a) const
{
    return a >> lBlkSize;
}

Addr
Base::pageAddress(Addr a) const
{
    return roundDown(a, pageBytes);
}

Addr
Base::pageOffset(Addr a) const
{
    return a & (pageBytes - 1);
}

Addr
Base::pageIthBlockAddress(Addr page, uint32_t blockIndex) const
{
    return page + (blockIndex << lBlkSize);
}

void
Base::probeNotify(const PacketPtr &pkt, bool miss)
{
    // Don't notify prefetcher on SWPrefetch, cache maintenance
    // operations or for writes that we are coaslescing.
    if (pkt->cmd.isSWPrefetch()) return;
    if (pkt->req->isCacheMaintenance()) return;
    if (pkt->isCleanEviction()) return;
    if (pkt->isWrite() && cache != nullptr && cache->coalesce()) return;
    if (!pkt->req->hasPaddr()) {
        panic("Request must have a physical address");
    }

    if (hasBeenPrefetched(pkt->getAddr(), pkt->isSecure())) {
        usefulPrefetches += 1;
        prefetchStats.pfUseful++;
        if (miss)
            // This case happens when a demand hits on a prefetched line
            // that's not in the requested coherency state.
            prefetchStats.pfUsefulButMiss++;
    }

    // Verify this access type is observed by prefetcher
    if (observeAccess(pkt, miss)) {
        if (useVirtualAddresses && pkt->req->hasVaddr()) {
            PrefetchInfo pfi(pkt, pkt->req->getVaddr(), miss);
            notify(pkt, pfi);
        } else if (!useVirtualAddresses) {
            PrefetchInfo pfi(pkt, pkt->req->getPaddr(), miss);
            notify(pkt, pfi);
        }
    }
}

void
Base::regProbeListeners()
{
    /**
     * If no probes were added by the configuration scripts, connect to the
     * parent cache using the probe "Miss". Also connect to "Hit", if the
     * cache is configured to prefetch on accesses.
     */
    if (listeners.empty() && cache != nullptr) {
        ProbeManager *pm(cache->getProbeManager());
        listeners.push_back(new PrefetchListener(*this, pm, "Miss", false,
                                                true));
        listeners.push_back(new PrefetchListener(*this, pm, "Fill", true,
                                                 false));
        listeners.push_back(new PrefetchListener(*this, pm, "Hit", false,
                                                 false));
    }
}

void
Base::addEventProbe(SimObject *obj, const char *name)
{
    ProbeManager *pm(obj->getProbeManager());
    listeners.push_back(new PrefetchListener(*this, pm, name));
}

void
Base::addMMU(BaseMMU *m)
{
    fatal_if(mmu != nullptr, "Only one MMU can be registered");
    mmu = m;
}

} // namespace prefetch
} // namespace gem5
