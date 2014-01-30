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
 * Hardware Prefetcher Definition.
 */

#include <list>

#include "arch/isa_traits.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/cache/base.hh"
#include "mem/request.hh"
#include "sim/system.hh"

BasePrefetcher::BasePrefetcher(const Params *p)
    : ClockedObject(p), size(p->size), latency(p->latency), degree(p->degree),
      useMasterId(p->use_master_id), pageStop(!p->cross_pages),
      serialSquash(p->serial_squash), onlyData(p->data_accesses_only),
      onMissOnly(p->on_miss_only), onReadOnly(p->on_read_only),
      onPrefetch(p->on_prefetch), system(p->sys),
      masterId(system->getMasterId(name()))
{
}

void
BasePrefetcher::setCache(BaseCache *_cache)
{
    cache = _cache;
    blkSize = cache->getBlockSize();
}

void
BasePrefetcher::regStats()
{
    pfIdentified
        .name(name() + ".prefetcher.num_hwpf_identified")
        .desc("number of hwpf identified")
        ;

    pfMSHRHit
        .name(name() + ".prefetcher.num_hwpf_already_in_mshr")
        .desc("number of hwpf that were already in mshr")
        ;

    pfCacheHit
        .name(name() + ".prefetcher.num_hwpf_already_in_cache")
        .desc("number of hwpf that were already in the cache")
        ;

    pfBufferHit
        .name(name() + ".prefetcher.num_hwpf_already_in_prefetcher")
        .desc("number of hwpf that were already in the prefetch queue")
        ;

    pfRemovedFull
        .name(name() + ".prefetcher.num_hwpf_evicted")
        .desc("number of hwpf removed due to no buffer left")
        ;

    pfRemovedMSHR
        .name(name() + ".prefetcher.num_hwpf_removed_MSHR_hit")
        .desc("number of hwpf removed because MSHR allocated")
        ;

    pfIssued
        .name(name() + ".prefetcher.num_hwpf_issued")
        .desc("number of hwpf issued")
        ;

    pfSpanPage
        .name(name() + ".prefetcher.num_hwpf_span_page")
        .desc("number of hwpf spanning a virtual page")
        ;

    pfSquashed
        .name(name() + ".prefetcher.num_hwpf_squashed_from_miss")
        .desc("number of hwpf that got squashed due to a miss "
              "aborting calculation time")
        ;
}

inline bool
BasePrefetcher::inCache(Addr addr, bool is_secure)
{
    if (cache->inCache(addr, is_secure)) {
        pfCacheHit++;
        return true;
    }
    return false;
}

inline bool
BasePrefetcher::inMissQueue(Addr addr, bool is_secure)
{
    if (cache->inMissQueue(addr, is_secure)) {
        pfMSHRHit++;
        return true;
    }
    return false;
}

PacketPtr
BasePrefetcher::getPacket()
{
    DPRINTF(HWPrefetch, "Requesting a hw_pf to issue\n");

    if (pf.empty()) {
        DPRINTF(HWPrefetch, "No HW_PF found\n");
        return NULL;
    }

    PacketPtr pkt = pf.begin()->pkt;
    while (!pf.empty()) {
        pkt = pf.begin()->pkt;
        pf.pop_front();

        Addr blk_addr = pkt->getAddr() & ~(Addr)(blkSize-1);
        bool is_secure = pkt->isSecure();

        if (!inCache(blk_addr, is_secure) && !inMissQueue(blk_addr, is_secure))
            // we found a prefetch, return it
            break;

        DPRINTF(HWPrefetch, "addr 0x%x (%s) in cache, skipping\n",
                pkt->getAddr(), is_secure ? "s" : "ns");
        delete pkt->req;
        delete pkt;

        if (pf.empty()) {
            cache->deassertMemSideBusRequest(BaseCache::Request_PF);
            return NULL; // None left, all were in cache
        }
    }

    pfIssued++;
    assert(pkt != NULL);
    DPRINTF(HWPrefetch, "returning 0x%x (%s)\n", pkt->getAddr(),
            pkt->isSecure() ? "s" : "ns");
    return pkt;
}


Tick
BasePrefetcher::notify(PacketPtr &pkt, Tick tick)
{
    // Don't consult the prefetcher if any of the following conditons are true
    // 1) The request is uncacheable
    // 2) The request is a fetch, but we are only prefeching data
    // 3) The request is a cache hit, but we are only training on misses
    // 4) THe request is a write, but we are only training on reads
    if (!pkt->req->isUncacheable() && !(pkt->req->isInstFetch() && onlyData) &&
        !(onMissOnly && inCache(pkt->getAddr(), true)) &&
        !(onReadOnly && !pkt->isRead())) {
        // Calculate the blk address
        Addr blk_addr = pkt->getAddr() & ~(Addr)(blkSize-1);
        bool is_secure = pkt->isSecure();

        // Check if miss is in pfq, if so remove it
        std::list<DeferredPacket>::iterator iter = inPrefetch(blk_addr,
                                                              is_secure);
        if (iter != pf.end()) {
            DPRINTF(HWPrefetch, "Saw a miss to a queued prefetch addr: "
                    "0x%x (%s), removing it\n", blk_addr,
                    is_secure ? "s" : "ns");
            pfRemovedMSHR++;
            delete iter->pkt->req;
            delete iter->pkt;
            iter = pf.erase(iter);
            if (pf.empty())
                cache->deassertMemSideBusRequest(BaseCache::Request_PF);
        }

        // Remove anything in queue with delay older than time
        // since everything is inserted in time order, start from end
        // and work until pf.empty() or time is earlier
        // This is done to emulate Aborting the previous work on a new miss
        // Needed for serial calculators like GHB
        if (serialSquash) {
            iter = pf.end();
            if (iter != pf.begin())
                iter--;
            while (!pf.empty() && iter->tick >= tick) {
                pfSquashed++;
                DPRINTF(HWPrefetch, "Squashing old prefetch addr: 0x%x\n",
                        iter->pkt->getAddr());
                delete iter->pkt->req;
                delete iter->pkt;
                iter = pf.erase(iter);
                if (iter != pf.begin())
                    iter--;
            }
            if (pf.empty())
                cache->deassertMemSideBusRequest(BaseCache::Request_PF);
        }


        std::list<Addr> addresses;
        std::list<Cycles> delays;
        calculatePrefetch(pkt, addresses, delays);

        std::list<Addr>::iterator addrIter = addresses.begin();
        std::list<Cycles>::iterator delayIter = delays.begin();
        for (; addrIter != addresses.end(); ++addrIter, ++delayIter) {
            Addr addr = *addrIter;

            pfIdentified++;

            DPRINTF(HWPrefetch, "Found a pf candidate addr: 0x%x, "
                    "inserting into prefetch queue with delay %d time %d\n",
                    addr, *delayIter, time);

            // Check if it is already in the pf buffer
            if (inPrefetch(addr, is_secure) != pf.end()) {
                pfBufferHit++;
                DPRINTF(HWPrefetch, "Prefetch addr already in pf buffer\n");
                continue;
            }

            // create a prefetch memreq
            Request *prefetchReq = new Request(*addrIter, blkSize, 0, masterId);
            if (is_secure)
                prefetchReq->setFlags(Request::SECURE);
            prefetchReq->taskId(ContextSwitchTaskId::Prefetcher);
            PacketPtr prefetch =
                new Packet(prefetchReq, MemCmd::HardPFReq);
            prefetch->allocate();
            prefetch->req->setThreadContext(pkt->req->contextId(),
                                            pkt->req->threadId());

            // Tag orefetch reqeuests with corresponding PC to train lower
            // cache-level prefetchers
            if (onPrefetch && pkt->req->hasPC())
                prefetch->req->setPC(pkt->req->getPC());

            // We just remove the head if we are full
            if (pf.size() == size) {
                pfRemovedFull++;
                PacketPtr old_pkt = pf.begin()->pkt;
                DPRINTF(HWPrefetch, "Prefetch queue full, "
                        "removing oldest 0x%x\n", old_pkt->getAddr());
                delete old_pkt->req;
                delete old_pkt;
                pf.pop_front();
            }

            pf.push_back(DeferredPacket(tick + clockPeriod() * *delayIter,
                                        prefetch));
        }
    }

    return pf.empty() ? 0 : pf.front().tick;
}

std::list<BasePrefetcher::DeferredPacket>::iterator
BasePrefetcher::inPrefetch(Addr address, bool is_secure)
{
    // Guaranteed to only be one match, we always check before inserting
    std::list<DeferredPacket>::iterator iter;
    for (iter = pf.begin(); iter != pf.end(); iter++) {
        if (((*iter).pkt->getAddr() & ~(Addr)(blkSize-1)) == address &&
            (*iter).pkt->isSecure() == is_secure) {
            return iter;
        }
    }
    return pf.end();
}

bool
BasePrefetcher::samePage(Addr a, Addr b)
{
    return roundDown(a, TheISA::VMPageSize) == roundDown(b, TheISA::VMPageSize);
}


