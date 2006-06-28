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
 *          Nathan Binkert
 */

/**
 * @file
 * Cache definitions.
 */

#include <assert.h>
#include <math.h>

#include <cassert>
#include <iostream>
#include <string>

#include "sim/host.hh"
#include "base/misc.hh"
#include "cpu/smt.hh"

#include "mem/cache/cache.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/miss/mshr.hh"
#include "mem/cache/prefetch/prefetcher.hh"

#include "sim/sim_events.hh" // for SimExitEvent

using namespace std;

template<class TagStore, class Buffering, class Coherence>
bool
Cache<TagStore,Buffering,Coherence>::
doTimingAccess(Packet *pkt, CachePort *cachePort, bool isCpuSide)
{
    if (isCpuSide)
    {
        access(pkt);
    }
    else
    {
        if (pkt->isResponse())
            handleResponse(pkt);
        else
            snoop(pkt);
    }
    return true; //Deal with blocking....
}

template<class TagStore, class Buffering, class Coherence>
Tick
Cache<TagStore,Buffering,Coherence>::
doAtomicAccess(Packet *pkt, CachePort *cachePort, bool isCpuSide)
{
    if (isCpuSide)
    {
        probe(pkt, true);
    }
    else
    {
        if (pkt->isResponse())
            handleResponse(pkt);
        else
            snoopProbe(pkt, true);
    }
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::
doFunctionalAccess(Packet *pkt, CachePort *cachePort, bool isCpuSide)
{
    if (isCpuSide)
    {
        probe(pkt, false);
    }
    else
    {
        if (pkt->isResponse())
            handleResponse(pkt);
        else
            snoopProbe(pkt, false);
    }
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::
recvStatusChange(Port::Status status, bool isCpuSide)
{

}


template<class TagStore, class Buffering, class Coherence>
Cache<TagStore,Buffering,Coherence>::
Cache(const std::string &_name,
      Cache<TagStore,Buffering,Coherence>::Params &params)
    : BaseCache(_name, params.baseParams),
      prefetchAccess(params.prefetchAccess),
      tags(params.tags), missQueue(params.missQueue),
      coherence(params.coherence), prefetcher(params.prefetcher),
      doCopy(params.doCopy), blockOnCopy(params.blockOnCopy)
{
    if (params.in == NULL) {
        topLevelCache = true;
    }
    tags->setCache(this, params.out->width, params.out->clockRate);
    tags->setPrefetcher(prefetcher);
    missQueue->setCache(this);
    missQueue->setPrefetcher(prefetcher);
    coherence->setCache(this);
    prefetcher->setCache(this);
    prefetcher->setTags(tags);
    prefetcher->setBuffer(missQueue);
    invalidatePkt = new Packet;
    invalidatePkt->cmd = Packet::InvalidateReq;
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::regStats()
{
    BaseCache::regStats();
    tags->regStats(name());
    missQueue->regStats(name());
    coherence->regStats(name());
    prefetcher->regStats(name());
}

template<class TagStore, class Buffering, class Coherence>
bool
Cache<TagStore,Buffering,Coherence>::access(PacketPtr &pkt)
{
//@todo Add back in MemDebug Calls
//    MemDebug::cacheAccess(pkt);
    BlkType *blk = NULL;
    PacketList writebacks;
    int size = blkSize;
    int lat = hitLatency;
    if (prefetchAccess) {
        //We are determining prefetches on access stream, call prefetcher
        prefetcher->handleMiss(pkt, curTick);
    }
    if (!pkt->req->isUncacheable()) {
        if (pkt->isInvalidate() && !pkt->isRead()
            && !pkt->isWrite()) {
            //Upgrade or Invalidate
            //Look into what happens if two slave caches on bus
            DPRINTF(Cache, "%s %d %x ? blk_addr: %x\n", pkt->cmdString(),
                    pkt->req->asid, pkt->addr & (((ULL(1))<<48)-1),
                    pkt->addr & ~((Addr)blkSize - 1));

            //@todo Should this return latency have the hit latency in it?
//	    respond(pkt,curTick+lat);
            (int)pkt->coherence |= SATISFIED;
//            return MA_HIT; //@todo, return values
            return true;
        }
        blk = tags->handleAccess(pkt, lat, writebacks);
    } else {
        size = pkt->size;
    }
    // If this is a block size write/hint (WH64) allocate the block here
    // if the coherence protocol allows it.
    /** @todo make the fast write alloc (wh64) work with coherence. */
    /** @todo Do we want to do fast writes for writebacks as well? */
    if (!blk && pkt->size >= blkSize && coherence->allowFastWrites() &&
        (pkt->cmd == Packet::WriteReq || pkt->cmd == Packet::WriteInvalidateReq) ) {
        // not outstanding misses, can do this
        MSHR* outstanding_miss = missQueue->findMSHR(pkt->addr, pkt->req->asid);
        if (pkt->cmd == Packet::WriteInvalidateReq || !outstanding_miss) {
            if (outstanding_miss) {
                warn("WriteInv doing a fastallocate"
                     "with an outstanding miss to the same address\n");
            }
            blk = tags->handleFill(NULL, pkt, BlkValid | BlkWritable,
                                   writebacks);
            ++fastWrites;
        }
    }
    while (!writebacks.empty()) {
        missQueue->doWriteback(writebacks.front());
        writebacks.pop_front();
    }
    DPRINTF(Cache, "%s %d %x %s blk_addr: %x pc %x\n", pkt->cmdString(),
            pkt->req->asid, pkt->addr & (((ULL(1))<<48)-1), (blk) ? "hit" : "miss",
            pkt->addr & ~((Addr)blkSize - 1), pkt->req->pc);
    if (blk) {
        // Hit
        hits[pkt->cmdToIndex()][pkt->req->getThreadNum()]++;
        // clear dirty bit if write through
        if (pkt->needsResponse())
            respond(pkt, curTick+lat);
//	return MA_HIT;
        return true;
    }

    // Miss
    if (!pkt->req->isUncacheable()) {
        misses[pkt->cmdToIndex()][pkt->req->getThreadNum()]++;
        /** @todo Move miss count code into BaseCache */
        if (missCount) {
            --missCount;
            if (missCount == 0)
                new SimLoopExitEvent("A cache reached the maximum miss count");
        }
    }
    missQueue->handleMiss(pkt, size, curTick + hitLatency);
//    return MA_CACHE_MISS;
    return true;
}


template<class TagStore, class Buffering, class Coherence>
Packet *
Cache<TagStore,Buffering,Coherence>::getPacket()
{
    Packet * pkt = missQueue->getPacket();
    if (pkt) {
        if (!pkt->req->isUncacheable()) {
            if (pkt->cmd == Packet::HardPFReq) misses[Packet::HardPFReq][pkt->req->getThreadNum()]++;
            BlkType *blk = tags->findBlock(pkt);
            Packet::Command cmd = coherence->getBusCmd(pkt->cmd,
                                              (blk)? blk->status : 0);
            missQueue->setBusCmd(pkt, cmd);
        }
    }

    assert(!doMasterRequest() || missQueue->havePending());
    assert(!pkt || pkt->time <= curTick);
    return pkt;
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::sendResult(PacketPtr &pkt, bool success)
{
    if (success) {
        missQueue->markInService(pkt);
          //Temp Hack for UPGRADES
          if (pkt->cmd == Packet::UpgradeReq) {
              handleResponse(pkt);
          }
    } else if (pkt && !pkt->req->isUncacheable()) {
        missQueue->restoreOrigCmd(pkt);
    }
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::handleResponse(Packet * &pkt)
{
    BlkType *blk = NULL;
    if (pkt->senderState) {
//	MemDebug::cacheResponse(pkt);
        DPRINTF(Cache, "Handling reponse to %x, blk addr: %x\n",pkt->addr,
                pkt->addr & (((ULL(1))<<48)-1));

        if (pkt->isCacheFill() && !pkt->isNoAllocate()) {
            blk = tags->findBlock(pkt);
            CacheBlk::State old_state = (blk) ? blk->status : 0;
            PacketList writebacks;
            blk = tags->handleFill(blk, pkt->senderState,
                                   coherence->getNewState(pkt,old_state),
                                   writebacks);
            while (!writebacks.empty()) {
                    missQueue->doWriteback(writebacks.front());
            }
        }
        missQueue->handleResponse(pkt, curTick + hitLatency);
    }
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::pseudoFill(Addr addr, int asid)
{
    // Need to temporarily move this blk into MSHRs
    MSHR *mshr = missQueue->allocateTargetList(addr, asid);
    int lat;
    PacketList dummy;
    // Read the data into the mshr
    BlkType *blk = tags->handleAccess(mshr->pkt, lat, dummy, false);
    assert(dummy.empty());
    assert((int)mshr->pkt->coherence & SATISFIED);
    // can overload order since it isn't used on non pending blocks
    mshr->order = blk->status;
    // temporarily remove the block from the cache.
    tags->invalidateBlk(addr, asid);
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::pseudoFill(MSHR *mshr)
{
    // Need to temporarily move this blk into MSHRs
    assert(mshr->pkt->cmd == Packet::ReadReq);
    int lat;
    PacketList dummy;
    // Read the data into the mshr
    BlkType *blk = tags->handleAccess(mshr->pkt, lat, dummy, false);
    assert(dummy.empty());
    assert((int)mshr->pkt->coherence & SATISFIED);
    // can overload order since it isn't used on non pending blocks
    mshr->order = blk->status;
    // temporarily remove the block from the cache.
    tags->invalidateBlk(mshr->pkt->addr, mshr->pkt->req->asid);
}


template<class TagStore, class Buffering, class Coherence>
Packet *
Cache<TagStore,Buffering,Coherence>::getCoherenceReq()
{
    return coherence->getPacket();
}


template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::snoop(Packet * &pkt)
{
    Addr blk_addr = pkt->addr & ~(Addr(blkSize-1));
    BlkType *blk = tags->findBlock(pkt);
    MSHR *mshr = missQueue->findMSHR(blk_addr, pkt->req->asid);
    if (isTopLevel() && coherence->hasProtocol()) { //@todo Move this into handle bus req
        //If we find an mshr, and it is in service, we need to NACK or invalidate
        if (mshr) {
            if (mshr->inService) {
                if ((mshr->pkt->isInvalidate() || !mshr->pkt->isCacheFill())
                    && (pkt->cmd != Packet::InvalidateReq && pkt->cmd != Packet::WriteInvalidateReq)) {
                    //If the outstanding request was an invalidate (upgrade,readex,..)
                    //Then we need to ACK the request until we get the data
                    //Also NACK if the outstanding request is not a cachefill (writeback)
                    (int)pkt->coherence |= NACKED_LINE;
                    return;
                }
                else {
                    //The supplier will be someone else, because we are waiting for
                    //the data.  This should cause this cache to be forced to go to
                    //the shared state, not the exclusive even though the shared line
                    //won't be asserted.  But for now we will just invlidate ourselves
                    //and allow the other cache to go into the exclusive state.
                    //@todo Make it so a read to a pending read doesn't invalidate.
                    //@todo Make it so that a read to a pending read can't be exclusive now.

                    //Set the address so find match works
                    invalidatePkt->addr = pkt->addr;

                    //Append the invalidate on
                    missQueue->addTarget(mshr,invalidatePkt);
                    DPRINTF(Cache, "Appending Invalidate to blk_addr: %x\n", pkt->addr & (((ULL(1))<<48)-1));
                    return;
                }
            }
        }
        //We also need to check the writeback buffers and handle those
        std::vector<MSHR *> writebacks;
        if (missQueue->findWrites(blk_addr, pkt->req->asid, writebacks)) {
            DPRINTF(Cache, "Snoop hit in writeback to blk_addr: %x\n", pkt->addr & (((ULL(1))<<48)-1));

            //Look through writebacks for any non-uncachable writes, use that
            for (int i=0; i<writebacks.size(); i++) {
                mshr = writebacks[i];

                if (!mshr->pkt->req->isUncacheable()) {
                    if (pkt->isRead()) {
                        //Only Upgrades don't get here
                        //Supply the data
                        (int)pkt->coherence |= SATISFIED;

                        //If we are in an exclusive protocol, make it ask again
                        //to get write permissions (upgrade), signal shared
                        (int)pkt->coherence |= SHARED_LINE;

                        assert(pkt->isRead());
                        assert(pkt->offset < blkSize);
                        assert(pkt->size <= blkSize);
                        assert(pkt->offset + pkt->size <=blkSize);
                        memcpy(pkt->data, mshr->pkt->data + pkt->offset, pkt->size);

                        respondToSnoop(pkt);
                    }

                    if (pkt->isInvalidate()) {
                        //This must be an upgrade or other cache will take ownership
                        missQueue->markInService(mshr->pkt);
                    }
                    return;
                }
            }
        }
    }
    CacheBlk::State new_state;
    bool satisfy = coherence->handleBusRequest(pkt,blk,mshr, new_state);
    if (satisfy) {
        tags->handleSnoop(blk, new_state, pkt);
        respondToSnoop(pkt);
        return;
    }
    tags->handleSnoop(blk, new_state);
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::snoopResponse(Packet * &pkt)
{
    //Need to handle the response, if NACKED
    if ((int)pkt->coherence & NACKED_LINE) {
        //Need to mark it as not in service, and retry for bus
        assert(0); //Yeah, we saw a NACK come through

        //For now this should never get called, we return false when we see a NACK
        //instead, by doing this we allow the bus_blocked mechanism to handle the retry
        //For now it retrys in just 2 cycles, need to figure out how to change that
        //Eventually we will want to also have success come in as a parameter
        //Need to make sure that we handle the functionality that happens on successufl
        //return of the sendAddr function
    }
}

template<class TagStore, class Buffering, class Coherence>
void
Cache<TagStore,Buffering,Coherence>::invalidateBlk(Addr addr, int asid)
{
    tags->invalidateBlk(addr,asid);
}


/**
 * @todo Fix to not assume write allocate
 */
template<class TagStore, class Buffering, class Coherence>
Tick
Cache<TagStore,Buffering,Coherence>::probe(Packet * &pkt, bool update)
{
//    MemDebug::cacheProbe(pkt);

    if (!pkt->req->isUncacheable()) {
        if (pkt->isInvalidate() && !pkt->isRead()
            && !pkt->isWrite()) {
            //Upgrade or Invalidate, satisfy it, don't forward
            DPRINTF(Cache, "%s %d %x ? blk_addr: %x\n", pkt->cmdString(),
                    pkt->req->asid, pkt->addr & (((ULL(1))<<48)-1),
                    pkt->addr & ~((Addr)blkSize - 1));
            (int)pkt->coherence |= SATISFIED;
            return 0;
        }
    }

    if (!update && !doData()) {
        // Nothing to do here
        return mi->sendProbe(pkt,update);
    }

    PacketList writebacks;
    int lat;
    BlkType *blk = tags->handleAccess(pkt, lat, writebacks, update);

    if (!blk) {
        // Need to check for outstanding misses and writes
        Addr blk_addr = pkt->addr & ~(blkSize - 1);

        // There can only be one matching outstanding miss.
        MSHR* mshr = missQueue->findMSHR(blk_addr, pkt->req->asid);

        // There can be many matching outstanding writes.
        vector<MSHR*> writes;
        missQueue->findWrites(blk_addr, pkt->req->asid, writes);

        if (!update) {
            mi->sendProbe(pkt, update);
            // Check for data in MSHR and writebuffer.
            if (mshr) {
                warn("Found outstanding miss on an non-update probe");
                MSHR::TargetList *targets = mshr->getTargetList();
                MSHR::TargetList::iterator i = targets->begin();
                MSHR::TargetList::iterator end = targets->end();
                for (; i != end; ++i) {
                    Packet * target = *i;
                    // If the target contains data, and it overlaps the
                    // probed request, need to update data
                    if (target->isWrite() && target->overlaps(pkt)) {
                        uint8_t* pkt_data;
                        uint8_t* write_data;
                        int data_size;
                        if (target->addr < pkt->addr) {
                            int offset = pkt->addr - target->paddr;
                            pkt_data = pkt->data;
                            write_data = target->data + offset;
                            data_size = target->size - offset;
                            assert(data_size > 0);
                            if (data_size > pkt->size)
                                data_size = pkt->size;
                        } else {
                            int offset = target->addr - pkt->addr;
                            pkt_data = pkt->data + offset;
                            write_data = target->data;
                            data_size = pkt->size - offset;
                            assert(data_size > pkt->size);
                            if (data_size > target->size)
                                data_size = target->size;
                        }

                        if (pkt->isWrite()) {
                            memcpy(pkt_data, write_data, data_size);
                        } else {
                            memcpy(write_data, pkt_data, data_size);
                        }
                    }
                }
            }
            for (int i = 0; i < writes.size(); ++i) {
                Packet * write = writes[i]->pkt;
                if (write->overlaps(pkt)) {
                    warn("Found outstanding write on an non-update probe");
                    uint8_t* pkt_data;
                    uint8_t* write_data;
                    int data_size;
                    if (write->addr < pkt->addr) {
                        int offset = pkt->addr - write->addr;
                        pkt_data = pkt->data;
                        write_data = write->data + offset;
                        data_size = write->size - offset;
                        assert(data_size > 0);
                        if (data_size > pkt->size)
                            data_size = pkt->size;
                    } else {
                        int offset = write->addr - pkt->addr;
                        pkt_data = pkt->data + offset;
                        write_data = write->data;
                        data_size = pkt->size - offset;
                        assert(data_size > pkt->size);
                        if (data_size > write->size)
                            data_size = write->size;
                    }

                    if (pkt->isWrite()) {
                        memcpy(pkt_data, write_data, data_size);
                    } else {
                        memcpy(write_data, pkt_data, data_size);
                    }

                }
            }
            return 0;
        } else {
            // update the cache state and statistics
            if (mshr || !writes.empty()){
                // Can't handle it, return pktuest unsatisfied.
                return 0;
            }
            if (!pkt->req->isUncacheable()) {
                // Fetch the cache block to fill
                Packet * busPkt = new Packet();
                busPkt->addr = blk_addr;
                busPkt->size = blkSize;
                busPkt->data = new uint8_t[blkSize];

                BlkType *blk = tags->findBlock(pkt);
                busPkt->cmd = coherence->getBusCmd(pkt->cmd,
                                                   (blk)? blk->status : 0);

                busPkt->req->asid = pkt->req->asid;
                busPkt->xc = pkt->xc;
                busPkt->req->setThreadNum() = pkt->req->getThreadNum();
                busPkt->time = curTick;

                lat = mi->sendProbe(busPkt, update);

                if (!busPkt->isSatisfied()) {
                    // blocked at a higher level, just return
                    return 0;
                }

                misses[pkt->cmdToIndex()][pkt->req->getThreadNum()]++;

                CacheBlk::State old_state = (blk) ? blk->status : 0;
                tags->handleFill(blk, busPkt,
                                 coherence->getNewState(busPkt, old_state),
                                 writebacks, pkt);
                // Handle writebacks if needed
                while (!writebacks.empty()){
                    mi->sendProbe(writebacks.front(), update);
                    writebacks.pop_front();
                }
                return lat + hitLatency;
            } else {
                return mi->sendProbe(pkt,update);
            }
        }
    } else {
        // There was a cache hit.
        // Handle writebacks if needed
        while (!writebacks.empty()){
            mi->sendProbe(writebacks.front(), update);
            writebacks.pop_front();
        }

        if (update) {
            hits[pkt->cmdToIndex()][pkt->req->getThreadNum()]++;
        } else if (pkt->isWrite()) {
            // Still need to change data in all locations.
            return mi->sendProbe(pkt, update);
        }
        return curTick + lat;
    }
    fatal("Probe not handled.\n");
    return 0;
}

template<class TagStore, class Buffering, class Coherence>
Tick
Cache<TagStore,Buffering,Coherence>::snoopProbe(PacketPtr &pkt, bool update)
{
    Addr blk_addr = pkt->addr & ~(Addr(blkSize-1));
    BlkType *blk = tags->findBlock(pkt);
    MSHR *mshr = missQueue->findMSHR(blk_addr, pkt->req->asid);
    CacheBlk::State new_state = 0;
    bool satisfy = coherence->handleBusPktuest(pkt,blk,mshr, new_state);
    if (satisfy) {
        tags->handleSnoop(blk, new_state, pkt);
        return hitLatency;
    }
    tags->handleSnoop(blk, new_state);
    return 0;
}

