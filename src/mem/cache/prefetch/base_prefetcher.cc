/*
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

#include "base/trace.hh"
#include "mem/cache/base_cache.hh"
#include "mem/cache/prefetch/base_prefetcher.hh"
#include "mem/request.hh"
#include <list>

BasePrefetcher::BasePrefetcher(int size, bool pageStop, bool serialSquash,
                               bool cacheCheckPush, bool onlyData)
    :size(size), pageStop(pageStop), serialSquash(serialSquash),
     cacheCheckPush(cacheCheckPush), only_data(onlyData)
{
}

void
BasePrefetcher::setCache(BaseCache *_cache)
{
    cache = _cache;
    blkSize = cache->getBlockSize();
}

void
BasePrefetcher::regStats(const std::string &name)
{
    pfIdentified
        .name(name + ".prefetcher.num_hwpf_identified")
        .desc("number of hwpf identified")
        ;

    pfMSHRHit
        .name(name + ".prefetcher.num_hwpf_already_in_mshr")
        .desc("number of hwpf that were already in mshr")
        ;

    pfCacheHit
        .name(name + ".prefetcher.num_hwpf_already_in_cache")
        .desc("number of hwpf that were already in the cache")
        ;

    pfBufferHit
        .name(name + ".prefetcher.num_hwpf_already_in_prefetcher")
        .desc("number of hwpf that were already in the prefetch queue")
        ;

    pfRemovedFull
        .name(name + ".prefetcher.num_hwpf_evicted")
        .desc("number of hwpf removed due to no buffer left")
        ;

    pfRemovedMSHR
        .name(name + ".prefetcher.num_hwpf_removed_MSHR_hit")
        .desc("number of hwpf removed because MSHR allocated")
        ;

    pfIssued
        .name(name + ".prefetcher.num_hwpf_issued")
        .desc("number of hwpf issued")
        ;

    pfSpanPage
        .name(name + ".prefetcher.num_hwpf_span_page")
        .desc("number of hwpf spanning a virtual page")
        ;

    pfSquashed
        .name(name + ".prefetcher.num_hwpf_squashed_from_miss")
        .desc("number of hwpf that got squashed due to a miss aborting calculation time")
        ;
}

Packet *
BasePrefetcher::getPacket()
{
    DPRINTF(HWPrefetch, "%s:Requesting a hw_pf to issue\n", cache->name());

    if (pf.empty()) {
        DPRINTF(HWPrefetch, "%s:No HW_PF found\n", cache->name());
        return NULL;
    }

    Packet * pkt;
    bool keepTrying = false;
    do {
        pkt = *pf.begin();
        pf.pop_front();
        if (!cacheCheckPush) {
            keepTrying = inCache(pkt);
        }
        if (pf.empty()) {
            cache->clearMasterRequest(Request_PF);
            if (keepTrying) return NULL; //None left, all were in cache
        }
    } while (keepTrying);

    pfIssued++;
    return pkt;
}

void
BasePrefetcher::handleMiss(Packet * &pkt, Tick time)
{
    if (!pkt->req->isUncacheable() && !(pkt->req->isInstRead() && only_data))
    {
        //Calculate the blk address
        Addr blkAddr = pkt->getAddr() & ~(Addr)(blkSize-1);

        //Check if miss is in pfq, if so remove it
        std::list<Packet *>::iterator iter = inPrefetch(blkAddr);
        if (iter != pf.end()) {
            DPRINTF(HWPrefetch, "%s:Saw a miss to a queued prefetch, removing it\n", cache->name());
            pfRemovedMSHR++;
            pf.erase(iter);
            if (pf.empty())
                cache->clearMasterRequest(Request_PF);
        }

        //Remove anything in queue with delay older than time
        //since everything is inserted in time order, start from end
        //and work until pf.empty() or time is earlier
        //This is done to emulate Aborting the previous work on a new miss
        //Needed for serial calculators like GHB
        if (serialSquash) {
            iter = pf.end();
            iter--;
            while (!pf.empty() && ((*iter)->time >= time)) {
                pfSquashed++;
                pf.pop_back();
                iter--;
            }
            if (pf.empty())
                cache->clearMasterRequest(Request_PF);
        }


        std::list<Addr> addresses;
        std::list<Tick> delays;
        calculatePrefetch(pkt, addresses, delays);

        std::list<Addr>::iterator addr = addresses.begin();
        std::list<Tick>::iterator delay = delays.begin();
        while (addr != addresses.end())
        {
            DPRINTF(HWPrefetch, "%s:Found a pf canidate, inserting into prefetch queue\n", cache->name());
            //temp calc this here...
            pfIdentified++;
            //create a prefetch memreq
            Request * prefetchReq = new Request(*addr, blkSize, 0);
            Packet * prefetch;
            prefetch = new Packet(prefetchReq, Packet::HardPFReq, -1);
            uint8_t *new_data = new uint8_t[blkSize];
            prefetch->dataDynamicArray<uint8_t>(new_data);
            prefetch->req->setThreadContext(pkt->req->getCpuNum(),
                                            pkt->req->getThreadNum());

            prefetch->time = time + (*delay); //@todo ADD LATENCY HERE
            //... initialize

            //Check if it is already in the cache
            if (cacheCheckPush) {
                if (inCache(prefetch)) {
                    addr++;
                    delay++;
                    continue;
                }
            }

            //Check if it is already in the miss_queue
            if (inMissQueue(prefetch->getAddr(), prefetch->req->getAsid())) {
                addr++;
                delay++;
                continue;
            }

            //Check if it is already in the pf buffer
            if (inPrefetch(prefetch->getAddr()) != pf.end()) {
                pfBufferHit++;
                addr++;
                delay++;
                continue;
            }

            //We just remove the head if we are full
            if (pf.size() == size)
            {
                DPRINTF(HWPrefetch, "%s:Inserting into prefetch queue, it was full removing oldest\n", cache->name());
                pfRemovedFull++;
                pf.pop_front();
            }

            pf.push_back(prefetch);
            prefetch->flags |= CACHE_LINE_FILL;

            //Make sure to request the bus, with proper delay
            cache->setMasterRequest(Request_PF, prefetch->time);

            //Increment through the list
            addr++;
            delay++;
        }
    }
}

std::list<Packet *>::iterator
BasePrefetcher::inPrefetch(Addr address)
{
    //Guaranteed to only be one match, we always check before inserting
    std::list<Packet *>::iterator iter;
    for (iter=pf.begin(); iter != pf.end(); iter++) {
        if (((*iter)->getAddr() & ~(Addr)(blkSize-1)) == address) {
            return iter;
        }
    }
    return pf.end();
}


