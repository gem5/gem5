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

#include "sim/sim_exit.hh" // for SimExitEvent

bool SIGNAL_NACK_HACK;

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::
recvStatusChange(Port::Status status, bool isCpuSide)
{

}


template<class TagStore, class Coherence>
Cache<TagStore,Coherence>::
Cache(const std::string &_name,
      Cache<TagStore,Coherence>::Params &params)
    : BaseCache(_name, params.baseParams),
      prefetchAccess(params.prefetchAccess),
      tags(params.tags), missQueue(params.missQueue),
      coherence(params.coherence), prefetcher(params.prefetcher),
      hitLatency(params.hitLatency)
{
    tags->setCache(this);
    tags->setPrefetcher(prefetcher);
    missQueue->setCache(this);
    missQueue->setPrefetcher(prefetcher);
    coherence->setCache(this);
    prefetcher->setCache(this);
    prefetcher->setTags(tags);
    prefetcher->setBuffer(missQueue);
    invalidateReq = new Request((Addr) NULL, blkSize, 0);
    invalidatePkt = new Packet(invalidateReq, Packet::InvalidateReq, 0);
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::regStats()
{
    BaseCache::regStats();
    tags->regStats(name());
    missQueue->regStats(name());
    coherence->regStats(name());
    prefetcher->regStats(name());
}

template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::access(PacketPtr &pkt)
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
        blk = tags->handleAccess(pkt, lat, writebacks);
    } else {
        size = pkt->getSize();
    }
    // If this is a block size write/hint (WH64) allocate the block here
    // if the coherence protocol allows it.
    /** @todo make the fast write alloc (wh64) work with coherence. */
    /** @todo Do we want to do fast writes for writebacks as well? */
    if (!blk && pkt->getSize() >= blkSize && coherence->allowFastWrites() &&
        (pkt->cmd == Packet::WriteReq
         || pkt->cmd == Packet::WriteInvalidateReq) ) {
        // not outstanding misses, can do this
        MSHR* outstanding_miss = missQueue->findMSHR(pkt->getAddr());
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

    DPRINTF(Cache, "%s %x %s\n", pkt->cmdString(), pkt->getAddr(),
            (blk) ? "hit" : "miss");

    if (blk) {
        // Hit
        hits[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
        // clear dirty bit if write through
        if (pkt->needsResponse())
            respond(pkt, curTick+lat);
        if (pkt->cmd == Packet::Writeback) {
            //Signal that you can kill the pkt/req
            pkt->flags |= SATISFIED;
        }
        return true;
    }

    // Miss
    if (!pkt->req->isUncacheable()) {
        misses[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
        /** @todo Move miss count code into BaseCache */
        if (missCount) {
            --missCount;
            if (missCount == 0)
                exitSimLoop("A cache reached the maximum miss count");
        }
    }

    if (pkt->flags & SATISFIED) {
        // happens when a store conditional fails because it missed
        // the cache completely
        if (pkt->needsResponse())
            respond(pkt, curTick+lat);
    } else {
        missQueue->handleMiss(pkt, size, curTick + hitLatency);
    }

    if (pkt->cmd == Packet::Writeback) {
        //Need to clean up the packet on a writeback miss, but leave the request
        delete pkt;
    }

    return true;
}


template<class TagStore, class Coherence>
PacketPtr
Cache<TagStore,Coherence>::getPacket()
{
    assert(missQueue->havePending());
    PacketPtr pkt = missQueue->getPacket();
    if (pkt) {
        if (!pkt->req->isUncacheable()) {
            if (pkt->cmd == Packet::HardPFReq)
                misses[Packet::HardPFReq][0/*pkt->req->getThreadNum()*/]++;
            BlkType *blk = tags->findBlock(pkt);
            Packet::Command cmd = coherence->getBusCmd(pkt->cmd,
                                              (blk)? blk->status : 0);
            missQueue->setBusCmd(pkt, cmd);
        }
    }

    assert(!doMasterRequest() || missQueue->havePending());
    assert(!pkt || pkt->time <= curTick);
    SIGNAL_NACK_HACK = false;
    return pkt;
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::sendResult(PacketPtr &pkt, MSHR* mshr,
                                                bool success)
{
    if (success && !(SIGNAL_NACK_HACK)) {
        //Remember if it was an upgrade because writeback MSHR's are removed
        //in Mark in Service
        bool upgrade = (mshr->pkt && mshr->pkt->cmd == Packet::UpgradeReq);

        missQueue->markInService(mshr->pkt, mshr);

        //Temp Hack for UPGRADES
        if (upgrade) {
            assert(pkt);  //Upgrades need to be fixed
            pkt->flags &= ~CACHE_LINE_FILL;
            BlkType *blk = tags->findBlock(pkt);
            CacheBlk::State old_state = (blk) ? blk->status : 0;
            CacheBlk::State new_state = coherence->getNewState(pkt,old_state);
            if (old_state != new_state)
                DPRINTF(Cache, "Block for blk addr %x moving from state "
                        "%i to %i\n", pkt->getAddr(), old_state, new_state);
            //Set the state on the upgrade
            memcpy(pkt->getPtr<uint8_t>(), blk->data, blkSize);
            PacketList writebacks;
            tags->handleFill(blk, mshr, new_state, writebacks, pkt);
            assert(writebacks.empty());
            missQueue->handleResponse(pkt, curTick + hitLatency);
        }
    } else if (pkt && !pkt->req->isUncacheable()) {
        pkt->flags &= ~NACKED_LINE;
        SIGNAL_NACK_HACK = false;
        pkt->flags &= ~SATISFIED;
        pkt->flags &= ~SNOOP_COMMIT;

//Rmove copy from mshr
        delete mshr->pkt;
        mshr->pkt = pkt;

        missQueue->restoreOrigCmd(pkt);
    }
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::handleResponse(PacketPtr &pkt)
{
    BlkType *blk = NULL;
    if (pkt->senderState) {
        //Delete temp copy in MSHR, restore it.
        delete ((MSHR*)pkt->senderState)->pkt;
        ((MSHR*)pkt->senderState)->pkt = pkt;
        if (pkt->result == Packet::Nacked) {
            //pkt->reinitFromRequest();
            warn("NACKs from devices not connected to the same bus "
                 "not implemented\n");
            return;
        }
        if (pkt->result == Packet::BadAddress) {
            //Make the response a Bad address and send it
        }
//	MemDebug::cacheResponse(pkt);
        DPRINTF(Cache, "Handling reponse to %x\n", pkt->getAddr());

        if (pkt->isCacheFill() && !pkt->isNoAllocate()) {
            DPRINTF(Cache, "Block for addr %x being updated in Cache\n",
                    pkt->getAddr());
            blk = tags->findBlock(pkt);
            CacheBlk::State old_state = (blk) ? blk->status : 0;
            PacketList writebacks;
            CacheBlk::State new_state = coherence->getNewState(pkt,old_state);
            if (old_state != new_state)
                DPRINTF(Cache, "Block for blk addr %x moving from "
                        "state %i to %i\n",
                        pkt->getAddr(),
                        old_state, new_state);
            blk = tags->handleFill(blk, (MSHR*)pkt->senderState,
                                   new_state, writebacks, pkt);
            while (!writebacks.empty()) {
                    missQueue->doWriteback(writebacks.front());
                    writebacks.pop_front();
            }
        }
        missQueue->handleResponse(pkt, curTick + hitLatency);
    }
}

template<class TagStore, class Coherence>
PacketPtr
Cache<TagStore,Coherence>::getCoherencePacket()
{
    return coherence->getPacket();
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::sendCoherenceResult(PacketPtr &pkt,
                                                         MSHR *cshr,
                                                         bool success)
{
    coherence->sendResult(pkt, cshr, success);
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::snoop(PacketPtr &pkt)
{
    if (pkt->req->isUncacheable()) {
        //Can't get a hit on an uncacheable address
        //Revisit this for multi level coherence
        return;
    }

    //Send a timing (true) invalidate up if the protocol calls for it
    if (coherence->propogateInvalidate(pkt, true)) {
        //Temp hack, we had a functional read hit in the L1, mark as success
        pkt->flags |= SATISFIED;
        pkt->result = Packet::Success;
        respondToSnoop(pkt, curTick + hitLatency);
        return;
    }

    Addr blk_addr = pkt->getAddr() & ~(Addr(blkSize-1));
    BlkType *blk = tags->findBlock(pkt);
    MSHR *mshr = missQueue->findMSHR(blk_addr);
    if (coherence->hasProtocol() || pkt->isInvalidate()) {
        //@todo Move this into handle bus req
        //If we find an mshr, and it is in service, we need to NACK or
        //invalidate
        if (mshr) {
            if (mshr->inService) {
                if ((mshr->pkt->isInvalidate() || !mshr->pkt->isCacheFill())
                    && (pkt->cmd != Packet::InvalidateReq
                        && pkt->cmd != Packet::WriteInvalidateReq)) {
                    //If the outstanding request was an invalidate
                    //(upgrade,readex,..)  Then we need to ACK the request
                    //until we get the data Also NACK if the outstanding
                    //request is not a cachefill (writeback)
                    assert(!(pkt->flags & SATISFIED));
                    pkt->flags |= SATISFIED;
                    pkt->flags |= NACKED_LINE;
                    SIGNAL_NACK_HACK = true;
                    ///@todo NACK's from other levels
                    //warn("NACKs from devices not connected to the same bus "
                    //"not implemented\n");
                    //respondToSnoop(pkt, curTick + hitLatency);
                    return;
                }
                else {
                    //The supplier will be someone else, because we are
                    //waiting for the data.  This should cause this cache to
                    //be forced to go to the shared state, not the exclusive
                    //even though the shared line won't be asserted.  But for
                    //now we will just invlidate ourselves and allow the other
                    //cache to go into the exclusive state.  @todo Make it so
                    //a read to a pending read doesn't invalidate.  @todo Make
                    //it so that a read to a pending read can't be exclusive
                    //now.

                    //Set the address so find match works
                    //panic("Don't have invalidates yet\n");
                    invalidatePkt->addrOverride(pkt->getAddr());

                    //Append the invalidate on
                    missQueue->addTarget(mshr,invalidatePkt);
                    DPRINTF(Cache, "Appending Invalidate to addr: %x\n",
                            pkt->getAddr());
                    return;
                }
            }
        }
        //We also need to check the writeback buffers and handle those
        std::vector<MSHR *> writebacks;
        if (missQueue->findWrites(blk_addr, writebacks)) {
            DPRINTF(Cache, "Snoop hit in writeback to addr: %x\n",
                    pkt->getAddr());

            //Look through writebacks for any non-uncachable writes, use that
            for (int i=0; i<writebacks.size(); i++) {
                mshr = writebacks[i];

                if (!mshr->pkt->req->isUncacheable()) {
                    if (pkt->isRead()) {
                        //Only Upgrades don't get here
                        //Supply the data
                        assert(!(pkt->flags & SATISFIED));
                        pkt->flags |= SATISFIED;

                        //If we are in an exclusive protocol, make it ask again
                        //to get write permissions (upgrade), signal shared
                        pkt->flags |= SHARED_LINE;

                        assert(pkt->isRead());
                        Addr offset = pkt->getAddr() & (blkSize - 1);
                        assert(offset < blkSize);
                        assert(pkt->getSize() <= blkSize);
                        assert(offset + pkt->getSize() <=blkSize);
                        memcpy(pkt->getPtr<uint8_t>(), mshr->pkt->getPtr<uint8_t>() + offset, pkt->getSize());

                        respondToSnoop(pkt, curTick + hitLatency);
                    }

                    if (pkt->isInvalidate()) {
                        //This must be an upgrade or other cache will take
                        //ownership
                        missQueue->markInService(mshr->pkt, mshr);
                    }
                    return;
                }
            }
        }
    }
    CacheBlk::State new_state;
    bool satisfy = coherence->handleBusRequest(pkt,blk,mshr, new_state);

    if (blk && mshr && !mshr->inService && new_state == 0) {
            //There was a outstanding write to a shared block, not need ReadEx
            //not update, so change No Allocate param in MSHR
            mshr->pkt->flags &= ~NO_ALLOCATE;
    }

    if (satisfy) {
        DPRINTF(Cache, "Cache snooped a %s request for addr %x and "
                "now supplying data, new state is %i\n",
                pkt->cmdString(), blk_addr, new_state);

        tags->handleSnoop(blk, new_state, pkt);
        respondToSnoop(pkt, curTick + hitLatency);
        return;
    }
    if (blk)
        DPRINTF(Cache, "Cache snooped a %s request for addr %x, "
                "new state is %i\n", pkt->cmdString(), blk_addr, new_state);

    tags->handleSnoop(blk, new_state);
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::snoopResponse(PacketPtr &pkt)
{
    //Need to handle the response, if NACKED
    if (pkt->flags & NACKED_LINE) {
        //Need to mark it as not in service, and retry for bus
        assert(0); //Yeah, we saw a NACK come through

        //For now this should never get called, we return false when we see a
        //NACK instead, by doing this we allow the bus_blocked mechanism to
        //handle the retry For now it retrys in just 2 cycles, need to figure
        //out how to change that Eventually we will want to also have success
        //come in as a parameter Need to make sure that we handle the
        //functionality that happens on successufl return of the sendAddr
        //function
    }
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::invalidateBlk(Addr addr)
{
    tags->invalidateBlk(addr);
}


/**
 * @todo Fix to not assume write allocate
 */
template<class TagStore, class Coherence>
Tick
Cache<TagStore,Coherence>::probe(PacketPtr &pkt, bool update,
                                           CachePort* otherSidePort)
{
//    MemDebug::cacheProbe(pkt);
    if (!pkt->req->isUncacheable()) {
        if (pkt->isInvalidate() && !pkt->isRead() && !pkt->isWrite()) {
            //Upgrade or Invalidate, satisfy it, don't forward
            DPRINTF(Cache, "%s %x ?\n", pkt->cmdString(), pkt->getAddr());
            pkt->flags |= SATISFIED;
            return 0;
        }
    }

    if (!update && (otherSidePort == cpuSidePort)) {
        // Still need to change data in all locations.
        otherSidePort->checkAndSendFunctional(pkt);
        if (pkt->isRead() && pkt->result == Packet::Success)
            return 0;
    }

    PacketList writebacks;
    int lat;

    BlkType *blk = tags->handleAccess(pkt, lat, writebacks, update);

    DPRINTF(Cache, "%s %x %s\n", pkt->cmdString(),
            pkt->getAddr(), (blk) ? "hit" : "miss");


    // Need to check for outstanding misses and writes
    Addr blk_addr = pkt->getAddr() & ~(blkSize - 1);

    // There can only be one matching outstanding miss.
    MSHR* mshr = missQueue->findMSHR(blk_addr);

    // There can be many matching outstanding writes.
    std::vector<MSHR*> writes;
    missQueue->findWrites(blk_addr, writes);

    if (!update) {
        bool notDone = !(pkt->flags & SATISFIED); //Hit in cache (was a block)
        // Check for data in MSHR and writebuffer.
        if (mshr) {
            MSHR::TargetList *targets = mshr->getTargetList();
            MSHR::TargetList::iterator i = targets->begin();
            MSHR::TargetList::iterator end = targets->end();
            for (; i != end && notDone; ++i) {
                PacketPtr target = *i;
                // If the target contains data, and it overlaps the
                // probed request, need to update data
                if (target->intersect(pkt)) {
                    DPRINTF(Cache, "Functional %s access to blk_addr %x intersects a MSHR\n",
                            pkt->cmdString(), blk_addr);
                    notDone = fixPacket(pkt, target);
                }
            }
        }
        for (int i = 0; i < writes.size() && notDone; ++i) {
            PacketPtr write = writes[i]->pkt;
            if (write->intersect(pkt)) {
                DPRINTF(Cache, "Functional %s access to blk_addr %x intersects a writeback\n",
                        pkt->cmdString(), blk_addr);
                notDone = fixPacket(pkt, write);
            }
        }
        if (notDone && otherSidePort == memSidePort) {
            otherSidePort->checkAndSendFunctional(pkt);
            assert(pkt->result == Packet::Success);
        }
        return 0;
    } else if (!blk && !(pkt->flags & SATISFIED)) {
        // update the cache state and statistics
        if (mshr || !writes.empty()){
            // Can't handle it, return request unsatisfied.
            panic("Atomic access ran into outstanding MSHR's or WB's!");
        }
        if (!pkt->req->isUncacheable() /*Uncacheables just go through*/
            && (pkt->cmd != Packet::Writeback)/*Writebacks on miss fall through*/) {
                // Fetch the cache block to fill
            BlkType *blk = tags->findBlock(pkt);
            Packet::Command temp_cmd = coherence->getBusCmd(pkt->cmd,
                                                            (blk)? blk->status : 0);

            PacketPtr busPkt = new Packet(pkt->req,temp_cmd, -1, blkSize);

            busPkt->allocate();

            busPkt->time = curTick;

            DPRINTF(Cache, "Sending a atomic %s for %x\n",
                    busPkt->cmdString(), busPkt->getAddr());

            lat = memSidePort->sendAtomic(busPkt);

            //Be sure to flip the response to a request for coherence
            if (busPkt->needsResponse()) {
                busPkt->makeAtomicResponse();
            }

/*		if (!(busPkt->flags & SATISFIED)) {
// blocked at a higher level, just return
return 0;
}

*/		misses[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;

            CacheBlk::State old_state = (blk) ? blk->status : 0;
            CacheBlk::State new_state =
                coherence->getNewState(busPkt, old_state);
            DPRINTF(Cache, "Receive response: %s for addr %x in state %i\n",
                    busPkt->cmdString(), busPkt->getAddr(), old_state);
            if (old_state != new_state)
                DPRINTF(Cache, "Block for blk addr %x moving from state "
                        "%i to %i\n", busPkt->getAddr(), old_state, new_state);

            tags->handleFill(blk, busPkt, new_state, writebacks, pkt);
            //Free the packet
            delete busPkt;

            // Handle writebacks if needed
            while (!writebacks.empty()){
                PacketPtr wbPkt = writebacks.front();
                memSidePort->sendAtomic(wbPkt);
                writebacks.pop_front();
                delete wbPkt;
            }
                return lat + hitLatency;
        } else {
            return memSidePort->sendAtomic(pkt);
        }
    } else {
        if (blk) {
            // There was a cache hit.
            // Handle writebacks if needed
            while (!writebacks.empty()){
                memSidePort->sendAtomic(writebacks.front());
                writebacks.pop_front();
            }

            hits[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
        }

        return hitLatency;
    }

    return 0;
}

template<class TagStore, class Coherence>
Tick
Cache<TagStore,Coherence>::snoopProbe(PacketPtr &pkt)
{
    //Send a atomic (false) invalidate up if the protocol calls for it
    if (coherence->propogateInvalidate(pkt, false)) {
        //Temp hack, we had a functional read hit in the L1, mark as success
        pkt->flags |= SATISFIED;
        pkt->result = Packet::Success;
        return hitLatency;
    }

    Addr blk_addr = pkt->getAddr() & ~(Addr(blkSize-1));
    BlkType *blk = tags->findBlock(pkt);
    MSHR *mshr = missQueue->findMSHR(blk_addr);
    CacheBlk::State new_state = 0;
    bool satisfy = coherence->handleBusRequest(pkt,blk,mshr, new_state);
    if (satisfy) {
        DPRINTF(Cache, "Cache snooped a %s request for addr %x and "
                "now supplying data, new state is %i\n",
                pkt->cmdString(), blk_addr, new_state);

            tags->handleSnoop(blk, new_state, pkt);
            return hitLatency;
    }
    if (blk)
        DPRINTF(Cache, "Cache snooped a %s request for addr %x, "
                "new state is %i\n",
                    pkt->cmdString(), blk_addr, new_state);
    tags->handleSnoop(blk, new_state);
    return 0;
}

template<class TagStore, class Coherence>
Port *
Cache<TagStore,Coherence>::getPort(const std::string &if_name, int idx)
{
    if (if_name == "")
    {
        if (cpuSidePort == NULL) {
            cpuSidePort = new CpuSidePort(name() + "-cpu_side_port", this);
            sendEvent = new CacheEvent(cpuSidePort, true);
        }
        return cpuSidePort;
    }
    else if (if_name == "functional")
    {
        return new CpuSidePort(name() + "-cpu_side_port", this);
    }
    else if (if_name == "cpu_side")
    {
        if (cpuSidePort == NULL) {
            cpuSidePort = new CpuSidePort(name() + "-cpu_side_port", this);
            sendEvent = new CacheEvent(cpuSidePort, true);
        }
        return cpuSidePort;
    }
    else if (if_name == "mem_side")
    {
        if (memSidePort != NULL)
            panic("Already have a mem side for this cache\n");
        memSidePort = new MemSidePort(name() + "-mem_side_port", this);
        memSendEvent = new CacheEvent(memSidePort, true);
        return memSidePort;
    }
    else panic("Port name %s unrecognized\n", if_name);
}


template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::CpuSidePort::recvTiming(PacketPtr pkt)
{
    if (!pkt->req->isUncacheable()
        && pkt->isInvalidate()
        && !pkt->isRead() && !pkt->isWrite()) {
        //Upgrade or Invalidate
        //Look into what happens if two slave caches on bus
        DPRINTF(Cache, "%s %x ?\n", pkt->cmdString(), pkt->getAddr());

        assert(!(pkt->flags & SATISFIED));
        pkt->flags |= SATISFIED;
        //Invalidates/Upgrades need no response if they get the bus
        return true;
    }

    if (pkt->isRequest() && blocked)
    {
        DPRINTF(Cache,"Scheduling a retry while blocked\n");
        mustSendRetry = true;
        return false;
    }

    if (pkt->isWrite() && (pkt->req->isLocked())) {
        pkt->req->setScResult(1);
    }
    myCache()->access(pkt);
    return true;
}

template<class TagStore, class Coherence>
Tick
Cache<TagStore,Coherence>::CpuSidePort::recvAtomic(PacketPtr pkt)
{
    myCache()->probe(pkt, true, NULL);
    //TEMP ALWAYS SUCCES FOR NOW
    pkt->result = Packet::Success;
    //Fix this timing info
    return myCache()->hitLatency;
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::CpuSidePort::recvFunctional(PacketPtr pkt)
{
    if (checkFunctional(pkt)) {
        //TEMP USE CPU?THREAD 0 0
        pkt->req->setThreadContext(0,0);

        myCache()->probe(pkt, false, cache->memSidePort);
        //TEMP ALWAYS SUCCESFUL FOR NOW
        pkt->result = Packet::Success;
    }
}


template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::MemSidePort::recvTiming(PacketPtr pkt)
{
    if (pkt->isRequest() && blocked)
    {
        DPRINTF(Cache,"Scheduling a retry while blocked\n");
        mustSendRetry = true;
        return false;
    }

    if (pkt->isResponse())
        myCache()->handleResponse(pkt);
    else {
        //Check if we should do the snoop
        if (pkt->flags & SNOOP_COMMIT)
            myCache()->snoop(pkt);
    }
    return true;
}

template<class TagStore, class Coherence>
Tick
Cache<TagStore,Coherence>::MemSidePort::recvAtomic(PacketPtr pkt)
{
    if (pkt->isResponse())
        myCache()->handleResponse(pkt);
    else
        return myCache()->snoopProbe(pkt);
    //Fix this timing info
    return myCache()->hitLatency;
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::MemSidePort::recvFunctional(PacketPtr pkt)
{
    if (checkFunctional(pkt)) {
        myCache()->probe(pkt, false, cache->cpuSidePort);
    }
}


template<class TagStore, class Coherence>
Cache<TagStore,Coherence>::
CpuSidePort::CpuSidePort(const std::string &_name,
                         Cache<TagStore,Coherence> *_cache)
    : BaseCache::CachePort(_name, _cache, true)
{
}

template<class TagStore, class Coherence>
Cache<TagStore,Coherence>::
MemSidePort::MemSidePort(const std::string &_name,
                         Cache<TagStore,Coherence> *_cache)
    : BaseCache::CachePort(_name, _cache, false)
{
}

