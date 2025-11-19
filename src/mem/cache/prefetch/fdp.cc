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

#include "mem/cache/prefetch/fdp.hh"

#include <utility>

#include "debug/HWPrefetch.hh"
#include "mem/cache/base.hh"
#include "params/FetchDirectedPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

FetchDirectedPrefetcher::FetchDirectedPrefetcher(
    const FetchDirectedPrefetcherParams &p)
    : Base(p),
      cpu(p.cpu),
      cache(nullptr),
      markReqAsPrefetch(p.mark_req_as_prefetch),
      squashPrefetches(p.squash_prefetches),
      latency(cyclesToTicks(p.latency)),
      pfqSize(p.pfq_size),
      tqSize(p.tq_size),
      cacheSnoop(p.cache_snoop),
      stats(this, p.pfq_size, p.tq_size)
{}

void
FetchDirectedPrefetcher::notifyFTQInsert(const o3::FetchTargetPtr &ft)
{
    const Addr start_blk_addr = blockAddress(ft->startAddress());
    const Addr end_blk_addr = blockAddress(ft->endAddress());

    for (Addr blk_addr = start_blk_addr; blk_addr <= end_blk_addr;
         blk_addr += blkSize) {

        // Check if the address is already in the prefetch queue
        auto it = std::find(pfq.begin(), pfq.end(), blk_addr);
        if (it != pfq.end()) {
            DPRINTF(HWPrefetch, "%#x already in prefetch_queue\n", blk_addr);
            stats.pfInPFQ++;
            continue;
        }

        it = std::find(translationq.begin(), translationq.end(), blk_addr);
        if (it != translationq.end()) {
            DPRINTF(HWPrefetch, "%#x already in translation queue\n",
                    blk_addr);
            stats.pfInTQ++;
            continue;
        }

        stats.pfIdentified++;

        if (translationq.size() >= tqSize) {
            DPRINTF(HWPrefetch, "Translation queue full, dropping %#x\n",
                    blk_addr);
            stats.tqDrops++;
            continue;
        }
        // TODO add also check for pfq to save unnecessary translations
        // Maybe merge the two queues
        translationq.emplace_back(*this, blk_addr, ft->getTid(), ft->ftNum());
        DPRINTF(HWPrefetch, "Start translation for %#x, reqID=%i, ctxID=%i\n",
                blk_addr, translationq.back().req->requestorId(),
                translationq.back().req->contextId());
        translationq.back().startTranslation();
        stats.tqInserts++;
        stats.tqSizeDistAtNotify.sample(translationq.size());
        stats.pfqSizeDistAtNotify.sample(pfq.size());
    }
}

void
FetchDirectedPrefetcher::notifyFTQRemove(const o3::FetchTargetPtr &ft)
{
    if (!squashPrefetches) {
        return;
    }

    // Mark any in-flight translations associated with the fetch
    // target as canceled. This will block them from progressing to
    // the `pfq` when translation is complete. We cannot simply remove
    // them from the `translationq` as the asynchronous
    // `translationComplete` callback assumes the `translationq` still
    // contains the entry.
    for (auto &pr : translationq) {
        if (pr.ftn == ft->ftNum()) {
            pr.markCanceled();
            stats.pfSquashed++;
        }
    }

    // Remove any existing prefetch requests that belong to this fetch
    // target.
    auto it = pfq.begin();
    while (it != pfq.end()) {
        if (it->ftn == ft->ftNum()) {
            it = pfq.erase(it);
            stats.pfSquashed++;
        } else {
            ++it;
        }
    }
}

void
FetchDirectedPrefetcher::translationComplete(PrefetchRequest *pfr, bool failed)
{
    auto it = translationq.begin();
    while (it != translationq.end()) {
        if (&(*it) == pfr) {
            break;
        }
        ++it;
    }
    assert(it != translationq.end());
    warn_if_once(cacheSnoop && (cache == nullptr),
                 "Cache is not set. Cache snooping will not work!\n");

    if (failed) {
        DPRINTF(HWPrefetch, "Translation of %#x failed\n", it->addr);
        stats.translationFail++;
    } else {
        DPRINTF(HWPrefetch, "Translation of %#x succeeded\n", it->addr);
        stats.translationSuccess++;

        if (it->isCanceled()) {
            DPRINTF(HWPrefetch,
                    "Drop Packet. Canceled by notifyFTQRemove during "
                    "translation.\n");
        } else if (it->req->isUncacheable()) {
            DPRINTF(HWPrefetch, "Drop uncacheable requests.\n");
        } else if (cacheSnoop && cache &&
                   (cache->inCache(it->req->getPaddr(), it->req->isSecure()) ||
                    (cache->inMissQueue(it->req->getPaddr(),
                                        it->req->isSecure())))) {
            stats.pfInCache++;
            DPRINTF(HWPrefetch, "Drop Packet. In Cache / MSHR\n");
        } else {
            if (pfq.size() < pfqSize) {
                it->createPkt();
                it->readyTime = curTick() + latency;
                stats.pfPacketsCreated++;
                DPRINTF(HWPrefetch,
                        "Addr:%#x Add packet to PFQ. pkt PA:%#x, "
                        "PFQ sz:%i\n",
                        it->addr, it->pkt->getAddr(), pfq.size());

                stats.pfCandidatesAdded++;
                pfq.push_back(*it);
                stats.pfqInserts++;
            } else {
                DPRINTF(HWPrefetch, "Prefetch queue full, dropping %#x\n",
                        it->addr);
                stats.pfqDrops++;
            }
        }
    }
    translationq.erase(it);
    stats.tqPops++;
}

PacketPtr
FetchDirectedPrefetcher::getPacket()
{
    if (pfq.size() == 0) {
        return nullptr;
    }
    PacketPtr pkt = pfq.front().pkt;

    DPRINTF(HWPrefetch, "Issue Prefetch to: pkt:%#x, PC:%#x, PFQ size:%i\n",
            pkt->getAddr(), pfq.front().addr, pfq.size());

    pfq.pop_front();
    stats.pfqPops++;

    prefetchStats.pfIssued++;
    return pkt;
}

FetchDirectedPrefetcher::PrefetchRequest::PrefetchRequest(
    FetchDirectedPrefetcher &_owner, uint64_t _addr, ThreadID tid,
    o3::FTSeqNum _ftn)
    : owner(_owner),
      addr(_addr),
      ftn(_ftn),
      req(nullptr),
      pkt(nullptr),
      readyTime(MaxTick),
      canceled(false)
{
    req = std::make_shared<Request>(addr, owner.blkSize, Request::INST_FETCH,
                                    owner.requestorId, addr,
                                    owner.cpu->getContext(tid)->contextId());
    if (owner.markReqAsPrefetch) {
        req->setFlags(Request::PREFETCH);
    }
    assert(req);
}

void
FetchDirectedPrefetcher::PrefetchRequest::createPkt()
{
    req->taskId(context_switch_task_id::Prefetcher);
    pkt = new Packet(req, MemCmd::HardPFReq);
    pkt->allocate();
}

void
FetchDirectedPrefetcher::PrefetchRequest::startTranslation()
{
    assert(owner.mmu != nullptr);
    auto tc = owner.system->threads[req->contextId()];
    owner.mmu->translateTiming(req, tc, this, BaseMMU::Execute);
}

void
FetchDirectedPrefetcher::PrefetchRequest::finish(const Fault &fault,
                                                 const RequestPtr &req,
                                                 ThreadContext *tc,
                                                 BaseMMU::Mode mode)
{
    bool failed = (fault != NoFault);
    owner.translationComplete(this, failed);
}

void
FetchDirectedPrefetcher::regProbeListeners()
{
    Base::regProbeListeners();

    if (cpu == nullptr) {
        warn("FetchDirectedPrefetcher: No CPU to listen from registered\n");
        return;
    }
    typedef ProbeListenerArgFunc<o3::FetchTargetPtr> FetchTargetListener;
    listeners.push_back(cpu->getProbeManager()->connect<FetchTargetListener>(
        "FTQInsert",
        [this](const o3::FetchTargetPtr &ft) { notifyFTQInsert(ft); }));

    listeners.push_back(cpu->getProbeManager()->connect<FetchTargetListener>(
        "FTQRemove",
        [this](const o3::FetchTargetPtr &ft) { notifyFTQRemove(ft); }));
}

FetchDirectedPrefetcher::Stats::Stats(statistics::Group *parent, int pfq_size,
                                      int tq_size)
    : statistics::Group(parent),
      ADD_STAT(fdipInsertions, statistics::units::Count::get(),
               "Number of notifications from an insertion in the FTQ"),
      ADD_STAT(pfIdentified, statistics::units::Count::get(),
               "Number of prefetches identified."),
      ADD_STAT(pfSquashed, statistics::units::Count::get(),
               "Number of prefetches squashed."),
      ADD_STAT(pfInPFQ, statistics::units::Count::get(),
               "Number of prefetches hit in the prefetch queue"),
      ADD_STAT(pfInTQ, statistics::units::Count::get(),
               "Number of prefetches hit in the translation queue"),
      ADD_STAT(pfInCache, statistics::units::Count::get(),
               "Number of prefetches hit in in cache"),
      ADD_STAT(pfInCachePrefetched, statistics::units::Count::get(),
               "Number of prefetches hit in cache but prefetched"),
      ADD_STAT(pfPacketsCreated, statistics::units::Count::get(),
               "Number of prefetch packets created"),
      ADD_STAT(pfCandidatesAdded, statistics::units::Count::get(),
               "Number of prefetch candidates added to the prefetch queue"),
      ADD_STAT(translationFail, statistics::units::Count::get(),
               "Number of prefetches that failed translation"),
      ADD_STAT(translationSuccess, statistics::units::Count::get(),
               "Number of prefetches that succeeded translation"),
      ADD_STAT(pfqSizeDistAtNotify, statistics::units::Count::get(),
               "Distribution of the prefetch queue size at the time of "
               "notification of a new fetch target"),
      ADD_STAT(tqSizeDistAtNotify, statistics::units::Count::get(),
               "Distribution of the translation queue size at the time of "
               "notification of a new fetch target"),
      ADD_STAT(pfqInserts, statistics::units::Count::get(),
               "Number of insertions into the prefetch candidate queue"),
      ADD_STAT(pfqPops, statistics::units::Count::get(),
               "Number of uses into the prefetch candidate queue"),
      ADD_STAT(pfqDrops, statistics::units::Count::get(),
               "Number of drops into the prefetch candidate queue"),
      ADD_STAT(tqInserts, statistics::units::Count::get(),
               "Number of insertions into the prefetch translation queue"),
      ADD_STAT(tqPops, statistics::units::Count::get(),
               "Number of uses into the prefetch translation queue"),
      ADD_STAT(tqDrops, statistics::units::Count::get(),
               "Number of drops into the prefetch translation queue")
{
    pfqSizeDistAtNotify.init(0, pfq_size, 4);
    tqSizeDistAtNotify.init(0, tq_size, 4);
}

} // namespace prefetch
} // namespace gem5
