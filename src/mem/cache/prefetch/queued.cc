/*
 * Copyright (c) 2014 ARM Limited
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

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/base.hh"

QueuedPrefetcher::QueuedPrefetcher(const QueuedPrefetcherParams *p)
    : BasePrefetcher(p), queueSize(p->queue_size), latency(p->latency),
      queueSquash(p->queue_squash), queueFilter(p->queue_filter),
      cacheSnoop(p->cache_snoop), tagPrefetch(p->tag_prefetch)
{

}

QueuedPrefetcher::~QueuedPrefetcher()
{
    // Delete the queued prefetch packets
    for (DeferredPacket &p : pfq) {
        delete p.pkt->req;
        delete p.pkt;
    }
}

Tick
QueuedPrefetcher::notify(const PacketPtr &pkt)
{
    // Verify this access type is observed by prefetcher
    if (observeAccess(pkt)) {
        Addr blk_addr = pkt->getAddr() & ~(Addr)(blkSize - 1);
        bool is_secure = pkt->isSecure();

        // Squash queued prefetches if demand miss to same line
        if (queueSquash) {
            auto itr = pfq.begin();
            while (itr != pfq.end()) {
                if (itr->pkt->getAddr() == blk_addr &&
                    itr->pkt->isSecure() == is_secure) {
                    delete itr->pkt->req;
                    delete itr->pkt;
                    itr = pfq.erase(itr);
                } else {
                    ++itr;
                }
            }
        }

        // Calculate prefetches given this access
        std::vector<Addr> addresses;
        calculatePrefetch(pkt, addresses);

        // Queue up generated prefetches
        for (Addr pf_addr : addresses) {

            // Block align prefetch address
            pf_addr = pf_addr & ~(Addr)(blkSize - 1);

            pfIdentified++;
            DPRINTF(HWPrefetch, "Found a pf candidate addr: %#x, "
                    "inserting into prefetch queue.\n", pf_addr);

            if (queueFilter && inPrefetch(pf_addr, is_secure)) {
                pfBufferHit++;
                DPRINTF(HWPrefetch, "Prefetch addr already in "
                        "prefetch queue\n");
                continue;
            }

            if (cacheSnoop && (inCache(pf_addr, is_secure) ||
                        inMissQueue(pf_addr, is_secure))) {
                pfInCache++;
                DPRINTF(HWPrefetch, "Dropping redundant in "
                        "cache/MSHR prefetch addr:%#x\n", pf_addr);
                continue;
            }

            // Create a prefetch memory request
            Request *pf_req =
                new Request(pf_addr, blkSize, 0, masterId);

            if (is_secure) {
                pf_req->setFlags(Request::SECURE);
            }
            pf_req->taskId(ContextSwitchTaskId::Prefetcher);
            PacketPtr pf_pkt = new Packet(pf_req, MemCmd::HardPFReq);
            pf_pkt->allocate();

            if (pkt->req->hasContextId()) {
                pf_req->setThreadContext(pkt->req->contextId(),
                                         pkt->req->threadId());
            }

            if (tagPrefetch && pkt->req->hasPC()) {
                // Tag prefetch packet with  accessing pc
                pf_pkt->req->setPC(pkt->req->getPC());
            }

            // Verify prefetch buffer space for request
            if (pfq.size() == queueSize) {
                pfRemovedFull++;
                PacketPtr old_pkt = pfq.begin()->pkt;
                DPRINTF(HWPrefetch, "Prefetch queue full, removing "
                        "oldest packet addr: %#x", old_pkt->getAddr());
                delete old_pkt->req;
                delete old_pkt;
                pfq.pop_front();
            }

            Tick pf_time = curTick() + clockPeriod() * latency;
            DPRINTF(HWPrefetch, "Prefetch queued. "
                    "addr:%#x tick:%lld.\n", pf_addr, pf_time);

            pfq.emplace_back(pf_time, pf_pkt);
        }
    }

    return pfq.empty() ? MaxTick : pfq.front().tick;
}

PacketPtr
QueuedPrefetcher::getPacket()
{
    DPRINTF(HWPrefetch, "Requesting a prefetch to issue.\n");

    if (pfq.empty()) {
        DPRINTF(HWPrefetch, "No hardware prefetches available.\n");
        return NULL;
    }

    PacketPtr pkt = pfq.begin()->pkt;
    pfq.pop_front();

    pfIssued++;
    assert(pkt != NULL);
    DPRINTF(HWPrefetch, "Generating prefetch for %#x.\n", pkt->getAddr());
    return pkt;
}

bool
QueuedPrefetcher::inPrefetch(Addr address, bool is_secure) const
{
    for (const DeferredPacket &dp : pfq) {
        if (dp.pkt->getAddr() == address &&
            dp.pkt->isSecure() == is_secure) return true;
    }

    return false;
}

void
QueuedPrefetcher::regStats()
{
    BasePrefetcher::regStats();

    pfIdentified
        .name(name() + ".pfIdentified")
        .desc("number of prefetch candidates identified");

    pfBufferHit
        .name(name() + ".pfBufferHit")
        .desc("number of redundant prefetches already in prefetch queue");

    pfInCache
        .name(name() + ".pfInCache")
        .desc("number of redundant prefetches already in cache/mshr dropped");

    pfRemovedFull
        .name(name() + ".pfRemovedFull")
        .desc("number of prefetches dropped due to prefetch queue size");

    pfSpanPage
        .name(name() + ".pfSpanPage")
        .desc("number of prefetches not generated due to page crossing");
}
