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
 *          Steve Reinhardt
 *          Ron Dreslinski
 */

/**
 * @file
 * Cache definitions.
 */

#include "sim/host.hh"
#include "base/misc.hh"

#include "mem/cache/cache.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/miss/mshr.hh"
#include "mem/cache/prefetch/base_prefetcher.hh"

#include "sim/sim_exit.hh" // for SimExitEvent


template<class TagStore, class Coherence>
Cache<TagStore,Coherence>::Cache(const std::string &_name,
                                 Cache<TagStore,Coherence>::Params &params)
    : BaseCache(_name, params.baseParams),
      prefetchAccess(params.prefetchAccess),
      tags(params.tags),
      coherence(params.coherence), prefetcher(params.prefetcher),
      doFastWrites(params.doFastWrites),
      prefetchMiss(params.prefetchMiss)
{
    cpuSidePort = new CpuSidePort(_name + "-cpu_side_port", this);
    memSidePort = new MemSidePort(_name + "-mem_side_port", this);
    cpuSidePort->setOtherPort(memSidePort);
    memSidePort->setOtherPort(cpuSidePort);

    tags->setCache(this);
    coherence->setCache(this);
    prefetcher->setCache(this);
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::regStats()
{
    BaseCache::regStats();
    tags->regStats(name());
    coherence->regStats(name());
    prefetcher->regStats(name());
}

template<class TagStore, class Coherence>
Port *
Cache<TagStore,Coherence>::getPort(const std::string &if_name, int idx)
{
    if (if_name == "" || if_name == "cpu_side") {
        return cpuSidePort;
    } else if (if_name == "mem_side") {
        return memSidePort;
    } else if (if_name == "functional") {
        return new CpuSidePort(name() + "-cpu_side_funcport", this);
    } else {
        panic("Port name %s unrecognized\n", if_name);
    }
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::deletePortRefs(Port *p)
{
    if (cpuSidePort == p || memSidePort == p)
        panic("Can only delete functional ports\n");

    delete p;
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::cmpAndSwap(BlkType *blk, PacketPtr pkt)
{
    uint64_t overwrite_val;
    bool overwrite_mem;
    uint64_t condition_val64;
    uint32_t condition_val32;

    int offset = tags->extractBlkOffset(pkt->getAddr());
    uint8_t *blk_data = blk->data + offset;

    assert(sizeof(uint64_t) >= pkt->getSize());

    overwrite_mem = true;
    // keep a copy of our possible write value, and copy what is at the
    // memory address into the packet
    pkt->writeData((uint8_t *)&overwrite_val);
    pkt->setData(blk_data);

    if (pkt->req->isCondSwap()) {
        if (pkt->getSize() == sizeof(uint64_t)) {
            condition_val64 = pkt->req->getExtraData();
            overwrite_mem = !std::memcmp(&condition_val64, blk_data,
                                         sizeof(uint64_t));
        } else if (pkt->getSize() == sizeof(uint32_t)) {
            condition_val32 = (uint32_t)pkt->req->getExtraData();
            overwrite_mem = !std::memcmp(&condition_val32, blk_data,
                                         sizeof(uint32_t));
        } else
            panic("Invalid size for conditional read/write\n");
    }

    if (overwrite_mem)
        std::memcpy(blk_data, &overwrite_val, pkt->getSize());
}


/////////////////////////////////////////////////////
//
// MSHR helper functions
//
/////////////////////////////////////////////////////


template<class TagStore, class Coherence>
MSHR *
Cache<TagStore,Coherence>::allocateBuffer(PacketPtr pkt, Tick time,
                                          bool isFill, bool requestBus)
{
    int  size = isFill ? blkSize : pkt->getSize();
    Addr addr = isFill ? tags->blkAlign(pkt->getAddr()) : pkt->getAddr();

    MSHR *mshr = NULL;

    if (pkt->isWrite()) {
        /**
         * @todo Add write merging here.
         */
        mshr = writeBuffer.allocate(addr, size, pkt, isFill);
        mshr->order = order++;

        if (writeBuffer.isFull()) {
            setBlocked(Blocked_NoWBBuffers);
        }

        if (requestBus) {
            requestMemSideBus(Request_WB, time);
        }
    } else {
        mshr = mshrQueue.allocate(addr, size, pkt, isFill);
        mshr->order = order++;
        if (mshrQueue.isFull()) {
            setBlocked(Blocked_NoMSHRs);
        }
        if (requestBus) {
            requestMemSideBus(Request_MSHR, time);
        }
    }

    assert(mshr != NULL);
    return mshr;
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::markInService(MSHR *mshr)
{
    bool unblock = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    /**
     * @todo Should include MSHRQueue pointer in MSHR to select the correct
     * one.
     */
    if (mshr->queue == &writeBuffer) {
        // Forwarding a write/ writeback, don't need to change
        // the command
        unblock = writeBuffer.isFull();
        writeBuffer.markInService(mshr);
        if (!writeBuffer.havePending()){
            deassertMemSideBusRequest(Request_WB);
        }
        if (unblock) {
            // Do we really unblock?
            unblock = !writeBuffer.isFull();
            cause = Blocked_NoWBBuffers;
        }
    } else {
        assert(mshr->queue == &mshrQueue);
        unblock = mshrQueue.isFull();
        mshrQueue.markInService(mshr);
        if (!mshrQueue.havePending()){
            deassertMemSideBusRequest(Request_MSHR);
        }
#if 0
        if (mshr->originalCmd == MemCmd::HardPFReq) {
            DPRINTF(HWPrefetch, "%s:Marking a HW_PF in service\n",
                    name());
            //Also clear pending if need be
            if (!prefetcher->havePending())
            {
                deassertMemSideBusRequest(Request_PF);
            }
        }
#endif
        if (unblock) {
            unblock = !mshrQueue.isFull();
            cause = Blocked_NoMSHRs;
        }
    }
    if (unblock) {
        clearBlocked(cause);
    }
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::squash(int threadNum)
{
    bool unblock = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    if (noTargetMSHR && noTargetMSHR->threadNum == threadNum) {
        noTargetMSHR = NULL;
        unblock = true;
        cause = Blocked_NoTargets;
    }
    if (mshrQueue.isFull()) {
        unblock = true;
        cause = Blocked_NoMSHRs;
    }
    mshrQueue.squash(threadNum);
    if (!mshrQueue.havePending()) {
        deassertMemSideBusRequest(Request_MSHR);
    }
    if (unblock && !mshrQueue.isFull()) {
        clearBlocked(cause);
    }
}

/////////////////////////////////////////////////////
//
// Access path: requests coming in from the CPU side
//
/////////////////////////////////////////////////////

template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::access(PacketPtr pkt, BlkType *blk, int &lat)
{
    bool satisfied = false;  // assume the worst

    if (prefetchAccess) {
        //We are determining prefetches on access stream, call prefetcher
        prefetcher->handleMiss(pkt, curTick);
    }

    DPRINTF(Cache, "%s %x %s\n", pkt->cmdString(), pkt->getAddr(),
            (blk) ? "hit" : "miss");

    if (blk != NULL) {
        // HIT
        if (blk->isPrefetch()) {
            //Signal that this was a hit under prefetch (no need for
            //use prefetch (only can get here if true)
            DPRINTF(HWPrefetch, "Hit a block that was prefetched\n");
            blk->status &= ~BlkHWPrefetched;
            if (prefetchMiss) {
                //If we are using the miss stream, signal the
                //prefetcher otherwise the access stream would have
                //already signaled this hit
                prefetcher->handleMiss(pkt, curTick);
            }
        }

        if (pkt->needsExclusive() ? blk->isWritable() : blk->isValid()) {
            // OK to satisfy access
            hits[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
            satisfied = true;

            if (pkt->cmd == MemCmd::SwapReq) {
                cmpAndSwap(blk, pkt);
            } else if (pkt->isWrite()) {
                if (blk->checkWrite(pkt)) {
                    blk->status |= BlkDirty;
                    pkt->writeDataToBlock(blk->data, blkSize);
                }
            } else {
                assert(pkt->isRead());
                if (pkt->isLocked()) {
                    blk->trackLoadLocked(pkt);
                }
                pkt->setDataFromBlock(blk->data, blkSize);
            }
        } else {
            // permission violation... nothing to do here, leave unsatisfied
            // for statistics purposes this counts like a complete miss
            incMissCount(pkt);
        }
    } else {
        // complete miss (no matching block)
        incMissCount(pkt);

        if (pkt->isLocked() && pkt->isWrite()) {
            // miss on store conditional... just give up now
            pkt->req->setExtraData(0);
            satisfied = true;
        }
    }

    return satisfied;
}


template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::timingAccess(PacketPtr pkt)
{
//@todo Add back in MemDebug Calls
//    MemDebug::cacheAccess(pkt);

    // we charge hitLatency for doing just about anything here
    Tick time =  curTick + hitLatency;

    if (pkt->req->isUncacheable()) {
        allocateBuffer(pkt, time, false, true);
        assert(pkt->needsResponse()); // else we should delete it here??
        return true;
    }

    PacketList writebacks;
    int lat = hitLatency;
    BlkType *blk = tags->findBlock(pkt->getAddr(), lat);
    bool satisfied = false;

    Addr blk_addr = pkt->getAddr() & ~(Addr(blkSize-1));

    MSHR *mshr = mshrQueue.findMatch(blk_addr);

    if (!mshr) {
        // no outstanding access to this block, look up in cache
        // (otherwise if we allow reads while there's an outstanding
        // write miss, the read could return stale data out of the
        // cache block... a more aggressive system could detect the
        // overlap (if any) and forward data out of the MSHRs, but we
        // don't do that yet)
        satisfied = access(pkt, blk, lat);
    }

#if 0
    // If this is a block size write/hint (WH64) allocate the block here
    // if the coherence protocol allows it.
    /** @todo make the fast write alloc (wh64) work with coherence. */
    /** @todo Do we want to do fast writes for writebacks as well? */
    if (!blk && pkt->getSize() >= blkSize && coherence->allowFastWrites() &&
        (pkt->cmd == MemCmd::WriteReq
         || pkt->cmd == MemCmd::WriteInvalidateReq) ) {
        // not outstanding misses, can do this
        MSHR *outstanding_miss = mshrQueue.findMatch(pkt->getAddr());
        if (pkt->cmd == MemCmd::WriteInvalidateReq || !outstanding_miss) {
            if (outstanding_miss) {
                warn("WriteInv doing a fastallocate"
                     "with an outstanding miss to the same address\n");
            }
            blk = handleFill(NULL, pkt, BlkValid | BlkWritable,
                                   writebacks);
            ++fastWrites;
        }
    }
#endif

    // copy writebacks to write buffer
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
        allocateBuffer(wbPkt, time, false, true);
        writebacks.pop_front();
    }

    bool needsResponse = pkt->needsResponse();

    if (satisfied) {
        assert(needsResponse);
        pkt->makeTimingResponse();
        cpuSidePort->respond(pkt, curTick+lat);
    } else {
        // miss
        if (prefetchMiss)
            prefetcher->handleMiss(pkt, time);

        if (mshr) {
            // MSHR hit
            //@todo remove hw_pf here
            mshr_hits[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
            if (mshr->threadNum != 0/*pkt->req->getThreadNum()*/) {
                mshr->threadNum = -1;
            }
            mshr->allocateTarget(pkt, true);
            if (mshr->getNumTargets() == numTarget) {
                noTargetMSHR = mshr;
                setBlocked(Blocked_NoTargets);
                mshrQueue.moveToFront(mshr);
            }
        } else {
            // no MSHR
            mshr_misses[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
            // always mark as cache fill for now... if we implement
            // no-write-allocate or bypass accesses this will have to
            // be changed.
            allocateBuffer(pkt, time, true, true);
        }
    }

    if (!needsResponse) {
        // Need to clean up the packet on a writeback miss, but leave
        // the request for the next level.
        delete pkt;
    }

    return true;
}


template<class TagStore, class Coherence>
Tick
Cache<TagStore,Coherence>::atomicAccess(PacketPtr pkt)
{
    // should assert here that there are no outstanding MSHRs or
    // writebacks... that would mean that someone used an atomic
    // access in timing mode

    if (pkt->req->isUncacheable()) {
        // Uncacheables just go through
        return memSidePort->sendAtomic(pkt);
    }

    PacketList writebacks;
    int lat = hitLatency;
    BlkType *blk = tags->findBlock(pkt->getAddr(), lat);
    bool satisfied = access(pkt, blk, lat);

    if (!satisfied) {
        // MISS
        CacheBlk::State old_state = (blk) ? blk->status : 0;
        MemCmd cmd = coherence->getBusCmd(pkt->cmd, old_state);
        Packet busPkt = Packet(pkt->req, cmd, Packet::Broadcast, blkSize);
        busPkt.allocate();

        DPRINTF(Cache, "Sending a atomic %s for %x\n",
                busPkt.cmdString(), busPkt.getAddr());

        lat += memSidePort->sendAtomic(&busPkt);

        DPRINTF(Cache, "Receive response: %s for addr %x in state %i\n",
                busPkt.cmdString(), busPkt.getAddr(), old_state);

        blk = handleFill(&busPkt, blk, writebacks);
        bool status = satisfyCpuSideRequest(pkt, blk);
        assert(status);
    }

    // We now have the block one way or another (hit or completed miss)

    // Handle writebacks if needed
    while (!writebacks.empty()){
        PacketPtr wbPkt = writebacks.front();
        memSidePort->sendAtomic(wbPkt);
        writebacks.pop_front();
        delete wbPkt;
    }

    if (pkt->needsResponse()) {
        pkt->makeAtomicResponse();
        pkt->result = Packet::Success;
    }

    return lat;
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::functionalAccess(PacketPtr pkt,
                                            CachePort *otherSidePort)
{
    Addr blk_addr = pkt->getAddr() & ~(blkSize - 1);
    BlkType *blk = tags->findBlock(pkt->getAddr());

    if (blk && pkt->checkFunctional(blk_addr, blkSize, blk->data)) {
        // request satisfied from block
        return;
    }

    // Need to check for outstanding misses and writes

    // There can only be one matching outstanding miss.
    MSHR *mshr = mshrQueue.findMatch(blk_addr);
    if (mshr) {
        MSHR::TargetList *targets = mshr->getTargetList();
        MSHR::TargetList::iterator i = targets->begin();
        MSHR::TargetList::iterator end = targets->end();
        for (; i != end; ++i) {
            PacketPtr targetPkt = i->pkt;
            if (pkt->checkFunctional(targetPkt))
                return;
        }
    }

    // There can be many matching outstanding writes.
    std::vector<MSHR*> writes;
    writeBuffer.findMatches(blk_addr, writes);
    for (int i = 0; i < writes.size(); ++i) {
        MSHR *mshr = writes[i];
        if (pkt->checkFunctional(mshr->addr, mshr->size, mshr->writeData))
            return;
    }

    otherSidePort->checkAndSendFunctional(pkt);
}


/////////////////////////////////////////////////////
//
// Response handling: responses from the memory side
//
/////////////////////////////////////////////////////

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::handleResponse(PacketPtr pkt, Tick time)
{
    MSHR *mshr = dynamic_cast<MSHR*>(pkt->senderState);
#ifndef NDEBUG
    int num_targets = mshr->getNumTargets();
#endif

    bool unblock = false;
    bool unblock_target = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    if (mshr->isCacheFill) {
#if 0
        mshr_miss_latency[mshr->originalCmd.toInt()][0/*pkt->req->getThreadNum()*/] +=
            curTick - pkt->time;
#endif
        // targets were handled in the cache tags
        if (mshr == noTargetMSHR) {
            // we always clear at least one target
            unblock_target = true;
            cause = Blocked_NoTargets;
            noTargetMSHR = NULL;
        }

        if (mshr->hasTargets()) {
            // Didn't satisfy all the targets, need to resend
            mshrQueue.markPending(mshr);
            mshr->order = order++;
            requestMemSideBus(Request_MSHR, time);
        }
        else {
            unblock = mshrQueue.isFull();
            mshrQueue.deallocate(mshr);
            if (unblock) {
                unblock = !mshrQueue.isFull();
                cause = Blocked_NoMSHRs;
            }
        }
    } else {
        if (pkt->req->isUncacheable()) {
            mshr_uncacheable_lat[pkt->cmd.toInt()][0/*pkt->req->getThreadNum()*/] +=
                curTick - pkt->time;
        }
        if (mshr->hasTargets() && pkt->req->isUncacheable()) {
            // Should only have 1 target if we had any
            assert(num_targets == 1);
            MSHR::Target *target = mshr->getTarget();
            assert(target->cpuSide);
            mshr->popTarget();
            if (pkt->isRead()) {
                target->pkt->setData(pkt->getPtr<uint8_t>());
            }
            cpuSidePort->respond(target->pkt, time);
            assert(!mshr->hasTargets());
        }
        else if (mshr->hasTargets()) {
            //Must be a no_allocate with possibly more than one target
            assert(!mshr->isCacheFill);
            while (mshr->hasTargets()) {
                MSHR::Target *target = mshr->getTarget();
                assert(target->isCpuSide());
                mshr->popTarget();
                if (pkt->isRead()) {
                    target->pkt->setData(pkt->getPtr<uint8_t>());
                }
                cpuSidePort->respond(target->pkt, time);
            }
        }

        if (pkt->isWrite()) {
            // If the wrtie buffer is full, we might unblock now
            unblock = writeBuffer.isFull();
            writeBuffer.deallocate(mshr);
            if (unblock) {
                // Did we really unblock?
                unblock = !writeBuffer.isFull();
                cause = Blocked_NoWBBuffers;
            }
        } else {
            unblock = mshrQueue.isFull();
            mshrQueue.deallocate(mshr);
            if (unblock) {
                unblock = !mshrQueue.isFull();
                cause = Blocked_NoMSHRs;
            }
        }
    }
    if (unblock || unblock_target) {
        clearBlocked(cause);
    }
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::handleResponse(PacketPtr pkt)
{
    Tick time = curTick + hitLatency;
    MSHR *mshr = dynamic_cast<MSHR*>(pkt->senderState);
    assert(mshr);
    if (pkt->result == Packet::Nacked) {
        //pkt->reinitFromRequest();
        warn("NACKs from devices not connected to the same bus "
             "not implemented\n");
        return;
    }
    assert(pkt->result != Packet::BadAddress);
    assert(pkt->result == Packet::Success);
    DPRINTF(Cache, "Handling reponse to %x\n", pkt->getAddr());

    if (mshr->isCacheFill) {
        DPRINTF(Cache, "Block for addr %x being updated in Cache\n",
                pkt->getAddr());
        BlkType *blk = tags->findBlock(pkt->getAddr());
        PacketList writebacks;
        blk = handleFill(pkt, blk, writebacks);
        satisfyMSHR(mshr, pkt, blk);
        // copy writebacks to write buffer
        while (!writebacks.empty()) {
            PacketPtr wbPkt = writebacks.front();
            allocateBuffer(wbPkt, time, false, true);
            writebacks.pop_front();
        }
    }
    handleResponse(pkt, time);
}




template<class TagStore, class Coherence>
PacketPtr
Cache<TagStore,Coherence>::writebackBlk(BlkType *blk)
{
    assert(blk && blk->isValid() && blk->isDirty());

    writebacks[0/*pkt->req->getThreadNum()*/]++;

    Request *writebackReq =
        new Request(tags->regenerateBlkAddr(blk->tag, blk->set), blkSize, 0);
    PacketPtr writeback = new Packet(writebackReq, MemCmd::Writeback, -1);
    writeback->allocate();
    std::memcpy(writeback->getPtr<uint8_t>(), blk->data, blkSize);

    blk->status &= ~BlkDirty;
    return writeback;
}


// Note that the reason we return a list of writebacks rather than
// inserting them directly in the write buffer is that this function
// is called by both atomic and timing-mode accesses, and in atomic
// mode we don't mess with the write buffer (we just perform the
// writebacks atomically once the original request is complete).
template<class TagStore, class Coherence>
typename Cache<TagStore,Coherence>::BlkType*
Cache<TagStore,Coherence>::handleFill(PacketPtr pkt, BlkType *blk,
                                      PacketList &writebacks)
{
    Addr addr = pkt->getAddr();

    if (blk == NULL) {

        // need to do a replacement
        blk = tags->findReplacement(addr, writebacks);
        if (blk->isValid()) {
            DPRINTF(Cache, "replacement: replacing %x with %x: %s\n",
                    tags->regenerateBlkAddr(blk->tag, blk->set), addr,
                    blk->isDirty() ? "writeback" : "clean");

            if (blk->isDirty()) {
                // Save writeback packet for handling by caller
                writebacks.push_back(writebackBlk(blk));
            }
        }

        blk->tag = tags->extractTag(addr);
        blk->status = coherence->getNewState(pkt);
        assert(pkt->isRead());
    } else {
        // existing block... probably an upgrade
        assert(blk->tag == tags->extractTag(addr));
        // either we're getting new data or the block should already be valid
        assert(pkt->isRead() || blk->isValid());
        CacheBlk::State old_state = blk->status;
        blk->status = coherence->getNewState(pkt, old_state);
        if (blk->status != old_state)
            DPRINTF(Cache, "Block addr %x moving from state %i to %i\n",
                    addr, old_state, blk->status);
        else
            warn("Changing state to same value\n");
    }

    // if we got new data, copy it in
    if (pkt->isRead()) {
        std::memcpy(blk->data, pkt->getPtr<uint8_t>(), blkSize);
    }

    blk->whenReady = pkt->finishTime;

    return blk;
}


template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::satisfyCpuSideRequest(PacketPtr pkt, BlkType *blk)
{
    if (blk && (pkt->needsExclusive() ? blk->isWritable() : blk->isValid())) {
        assert(pkt->isWrite() || pkt->isReadWrite() || pkt->isRead());
        assert(pkt->getOffset(blkSize) + pkt->getSize() <= blkSize);

        if (pkt->isWrite()) {
            if (blk->checkWrite(pkt)) {
                blk->status |= BlkDirty;
                pkt->writeDataToBlock(blk->data, blkSize);
            }
        } else if (pkt->isReadWrite()) {
            cmpAndSwap(blk, pkt);
        } else {
            if (pkt->isLocked()) {
                blk->trackLoadLocked(pkt);
            }
            pkt->setDataFromBlock(blk->data, blkSize);
        }

        return true;
    } else {
        return false;
    }
}


template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::satisfyTarget(MSHR::Target *target, BlkType *blk)
{
    assert(target != NULL);
    assert(target->isCpuSide());
    return satisfyCpuSideRequest(target->pkt, blk);
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::satisfyMSHR(MSHR *mshr, PacketPtr pkt,
                                       BlkType *blk)
{
    // respond to MSHR targets, if any

    // First offset for critical word first calculations
    int initial_offset = 0;

    if (mshr->hasTargets()) {
        initial_offset = mshr->getTarget()->pkt->getOffset(blkSize);
    }

    while (mshr->hasTargets()) {
        MSHR::Target *target = mshr->getTarget();

        if (!satisfyTarget(target, blk)) {
            // Invalid access, need to do another request
            // can occur if block is invalidated, or not correct
            // permissions
            break;
        }


        // How many bytes pass the first request is this one
        int transfer_offset = target->pkt->getOffset(blkSize) - initial_offset;
        if (transfer_offset < 0) {
            transfer_offset += blkSize;
        }

        // If critical word (no offset) return first word time
        Tick completion_time = tags->getHitLatency() +
            transfer_offset ? pkt->finishTime : pkt->firstWordTime;

        if (!target->pkt->req->isUncacheable()) {
            missLatency[target->pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/] +=
                completion_time - target->time;
        }
        target->pkt->makeTimingResponse();
        cpuSidePort->respond(target->pkt, completion_time);
        mshr->popTarget();
    }
}


/////////////////////////////////////////////////////
//
// Snoop path: requests coming in from the memory side
//
/////////////////////////////////////////////////////

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::doTimingSupplyResponse(PacketPtr req_pkt,
                                                  uint8_t *blk_data)
{
    // timing-mode snoop responses require a new packet
    PacketPtr pkt = new Packet(req_pkt);
    pkt->allocate();
    pkt->makeTimingResponse();
    pkt->setDataFromBlock(blk_data, blkSize);
    memSidePort->respond(pkt, curTick + hitLatency);
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::handleSnoop(PacketPtr pkt, BlkType *blk,
                                       bool is_timing)
{
    if (!blk || !blk->isValid()) {
        return;
    }

    // we may end up modifying both the block state and the packet (if
    // we respond in atomic mode), so just figure out what to do now
    // and then do it later
    bool supply = blk->isDirty() && pkt->isRead();
    bool invalidate = pkt->isInvalidate();

    if (pkt->isRead() && !pkt->isInvalidate()) {
        assert(!pkt->needsExclusive());
        pkt->assertShared();
        int bits_to_clear = BlkWritable;
        const bool haveOwnershipState = true; // for now
        if (!haveOwnershipState) {
            // if we don't support pure ownership (dirty && !writable),
            // have to clear dirty bit here, assume memory snarfs data
            // on cache-to-cache xfer
            bits_to_clear |= BlkDirty;
        }
        blk->status &= ~bits_to_clear;
    }

    if (supply) {
        pkt->assertMemInhibit();
        if (is_timing) {
            doTimingSupplyResponse(pkt, blk->data);
        } else {
            pkt->makeAtomicResponse();
            pkt->setDataFromBlock(blk->data, blkSize);
        }
    }

    // Do this last in case it deallocates block data or something
    // like that
    if (invalidate) {
        tags->invalidateBlk(blk);
    }

    DPRINTF(Cache, "snooped a %s request for addr %x, %snew state is %i\n",
            pkt->cmdString(), blockAlign(pkt->getAddr()),
            supply ? "supplying data, " : "", blk->status);
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::snoopTiming(PacketPtr pkt)
{
    if (pkt->req->isUncacheable()) {
        //Can't get a hit on an uncacheable address
        //Revisit this for multi level coherence
        return;
    }

    BlkType *blk = tags->findBlock(pkt->getAddr());

    Addr blk_addr = pkt->getAddr() & ~(Addr(blkSize-1));
    MSHR *mshr = mshrQueue.findMatch(blk_addr);
    // better not be snooping a request that conflicts with something
    // we have outstanding...
    assert(!mshr || !mshr->inService);

    //We also need to check the writeback buffers and handle those
    std::vector<MSHR *> writebacks;
    if (writeBuffer.findMatches(blk_addr, writebacks)) {
        DPRINTF(Cache, "Snoop hit in writeback to addr: %x\n",
                pkt->getAddr());

        //Look through writebacks for any non-uncachable writes, use that
        for (int i=0; i<writebacks.size(); i++) {
            mshr = writebacks[i];
            assert(!mshr->isUncacheable());

            if (pkt->isRead()) {
                pkt->assertMemInhibit();
                if (!pkt->needsExclusive()) {
                    pkt->assertShared();
                } else {
                    // if we're not asserting the shared line, we need to
                    // invalidate our copy.  we'll do that below as long as
                    // the packet's invalidate flag is set...
                    assert(pkt->isInvalidate());
                }
                doTimingSupplyResponse(pkt, mshr->writeData);
            }

            if (pkt->isInvalidate()) {
                // Invalidation trumps our writeback... discard here
                assert(0);
                markInService(mshr);
            }
            return;
        }
    }

    handleSnoop(pkt, blk, true);
}


template<class TagStore, class Coherence>
Tick
Cache<TagStore,Coherence>::snoopAtomic(PacketPtr pkt)
{
    if (pkt->req->isUncacheable()) {
        // Can't get a hit on an uncacheable address
        // Revisit this for multi level coherence
        return hitLatency;
    }

    BlkType *blk = tags->findBlock(pkt->getAddr());
    handleSnoop(pkt, blk, false);
    return hitLatency;
}


template<class TagStore, class Coherence>
MSHR *
Cache<TagStore,Coherence>::getNextMSHR()
{
    // Check both MSHR queue and write buffer for potential requests
    MSHR *miss_mshr  = mshrQueue.getNextMSHR();
    MSHR *write_mshr = writeBuffer.getNextMSHR();

    // Now figure out which one to send... some cases are easy
    if (miss_mshr && !write_mshr) {
        return miss_mshr;
    }
    if (write_mshr && !miss_mshr) {
        return write_mshr;
    }

    if (miss_mshr && write_mshr) {
        // We have one of each... normally we favor the miss request
        // unless the write buffer is full
        if (writeBuffer.isFull() && writeBuffer.inServiceEntries == 0) {
            // Write buffer is full, so we'd like to issue a write;
            // need to search MSHR queue for conflicting earlier miss.
            MSHR *conflict_mshr =
                mshrQueue.findPending(write_mshr->addr, write_mshr->size);

            if (conflict_mshr && conflict_mshr->order < write_mshr->order) {
                // Service misses in order until conflict is cleared.
                return conflict_mshr;
            }

            // No conflicts; issue write
            return write_mshr;
        }

        // Write buffer isn't full, but need to check it for
        // conflicting earlier writeback
        MSHR *conflict_mshr =
            writeBuffer.findPending(miss_mshr->addr, miss_mshr->size);
        if (conflict_mshr) {
            // not sure why we don't check order here... it was in the
            // original code but commented out.

            // The only way this happens is if we are
            // doing a write and we didn't have permissions
            // then subsequently saw a writeback (owned got evicted)
            // We need to make sure to perform the writeback first
            // To preserve the dirty data, then we can issue the write

            // should we return write_mshr here instead?  I.e. do we
            // have to flush writes in order?  I don't think so... not
            // for Alpha anyway.  Maybe for x86?
            return conflict_mshr;
        }

        // No conclifts; issue read
        return miss_mshr;
    }

    // fall through... no pending requests.  Try a prefetch.
    assert(!miss_mshr && !write_mshr);
    if (!mshrQueue.isFull()) {
        // If we have a miss queue slot, we can try a prefetch
        PacketPtr pkt = prefetcher->getPacket();
        if (pkt) {
            // Update statistic on number of prefetches issued
            // (hwpf_mshr_misses)
            mshr_misses[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
            // Don't request bus, since we already have it
            return allocateBuffer(pkt, curTick, true, false);
        }
    }

    return NULL;
}


template<class TagStore, class Coherence>
PacketPtr
Cache<TagStore,Coherence>::getPacket()
{
    MSHR *mshr = getNextMSHR();

    if (mshr == NULL) {
        return NULL;
    }

    BlkType *blk = tags->findBlock(mshr->addr);

    // use request from 1st target
    MSHR::Target *tgt1 = mshr->getTarget();
    PacketPtr tgt1_pkt = tgt1->pkt;
    PacketPtr pkt;

    if (mshr->isCacheFill) {
        MemCmd cmd;
        if (blk && blk->isValid()) {
            // only reason to be here is that blk is shared
            // (read-only) and we need exclusive
            assert(mshr->needsExclusive && !blk->isWritable());
            cmd = MemCmd::UpgradeReq;
        } else {
            // block is invalid
            cmd = mshr->needsExclusive ? MemCmd::ReadExReq : MemCmd::ReadReq;
        }
        pkt = new Packet(tgt1_pkt->req, cmd, Packet::Broadcast);
    } else {
        assert(blk == NULL);
        assert(mshr->getNumTargets() == 1);
        pkt = new Packet(tgt1_pkt->req, tgt1_pkt->cmd, Packet::Broadcast);
    }

    pkt->senderState = mshr;
    pkt->allocate();
    return pkt;
}


///////////////
//
// CpuSidePort
//
///////////////

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::CpuSidePort::
getDeviceAddressRanges(AddrRangeList &resp, bool &snoop)
{
    // CPU side port doesn't snoop; it's a target only.
    bool dummy;
    otherPort->getPeerAddressRanges(resp, dummy);
    snoop = false;
}


template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::CpuSidePort::recvTiming(PacketPtr pkt)
{
    if (pkt->isRequest() && blocked) {
        DPRINTF(Cache,"Scheduling a retry while blocked\n");
        mustSendRetry = true;
        return false;
    }

    myCache()->timingAccess(pkt);
    return true;
}


template<class TagStore, class Coherence>
Tick
Cache<TagStore,Coherence>::CpuSidePort::recvAtomic(PacketPtr pkt)
{
    return myCache()->atomicAccess(pkt);
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::CpuSidePort::recvFunctional(PacketPtr pkt)
{
    checkFunctional(pkt);
    if (pkt->result != Packet::Success)
        myCache()->functionalAccess(pkt, cache->memSidePort);
}


template<class TagStore, class Coherence>
Cache<TagStore,Coherence>::
CpuSidePort::CpuSidePort(const std::string &_name,
                         Cache<TagStore,Coherence> *_cache)
    : BaseCache::CachePort(_name, _cache)
{
}

///////////////
//
// MemSidePort
//
///////////////

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::MemSidePort::
getDeviceAddressRanges(AddrRangeList &resp, bool &snoop)
{
    otherPort->getPeerAddressRanges(resp, snoop);
    // Memory-side port always snoops, so unconditionally set flag for
    // caller.
    snoop = true;
}


template<class TagStore, class Coherence>
bool
Cache<TagStore,Coherence>::MemSidePort::recvTiming(PacketPtr pkt)
{
    // this needs to be fixed so that the cache updates the mshr and sends the
    // packet back out on the link, but it probably won't happen so until this
    // gets fixed, just panic when it does
    if (pkt->result == Packet::Nacked)
        panic("Need to implement cache resending nacked packets!\n");

    if (pkt->isRequest() && blocked) {
        DPRINTF(Cache,"Scheduling a retry while blocked\n");
        mustSendRetry = true;
        return false;
    }

    if (pkt->isResponse()) {
        myCache()->handleResponse(pkt);
    } else {
        myCache()->snoopTiming(pkt);
    }
    return true;
}


template<class TagStore, class Coherence>
Tick
Cache<TagStore,Coherence>::MemSidePort::recvAtomic(PacketPtr pkt)
{
    // in atomic mode, responses go back to the sender via the
    // function return from sendAtomic(), not via a separate
    // sendAtomic() from the responder.  Thus we should never see a
    // response packet in recvAtomic() (anywhere, not just here).
    assert(!pkt->isResponse());
    return myCache()->snoopAtomic(pkt);
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::MemSidePort::recvFunctional(PacketPtr pkt)
{
    checkFunctional(pkt);
    if (pkt->result != Packet::Success)
        myCache()->functionalAccess(pkt, cache->cpuSidePort);
}



template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::MemSidePort::sendPacket()
{
    // if we have responses that are ready, they take precedence
    if (deferredPacketReady()) {
        bool success = sendTiming(transmitList.front().pkt);

        if (success) {
            //send successful, remove packet
            transmitList.pop_front();
        }

        waitingOnRetry = !success;
    } else {
        // check for non-response packets (requests & writebacks)
        PacketPtr pkt = myCache()->getPacket();
        MSHR *mshr = dynamic_cast<MSHR*>(pkt->senderState);

        bool success = sendTiming(pkt);
        DPRINTF(Cache, "Address %x was %s in sending the timing request\n",
                pkt->getAddr(), success ? "successful" : "unsuccessful");

        waitingOnRetry = !success;
        if (waitingOnRetry) {
            DPRINTF(CachePort, "%s now waiting on a retry\n", name());
        } else {
            myCache()->markInService(mshr);
        }
    }


    // tried to send packet... if it was successful (no retry), see if
    // we need to rerequest bus or not
    if (!waitingOnRetry) {
        if (isBusRequested()) {
            // more requests/writebacks: rerequest ASAP
            DPRINTF(CachePort, "%s still more MSHR requests to send\n",
                    name());
            sendEvent->schedule(curTick+1);
        } else if (!transmitList.empty()) {
            // deferred packets: rerequest bus, but possibly not until later
            Tick time = transmitList.front().tick;
            sendEvent->schedule(time <= curTick ? curTick+1 : time);
        } else {
            // no more to send right now: if we're draining, we may be done
            if (drainEvent) {
                drainEvent->process();
                drainEvent = NULL;
            }
        }
    }
}

template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::MemSidePort::recvRetry()
{
    assert(waitingOnRetry);
    sendPacket();
}


template<class TagStore, class Coherence>
void
Cache<TagStore,Coherence>::MemSidePort::processSendEvent()
{
    assert(!waitingOnRetry);
    sendPacket();
}


template<class TagStore, class Coherence>
Cache<TagStore,Coherence>::
MemSidePort::MemSidePort(const std::string &_name,
                         Cache<TagStore,Coherence> *_cache)
    : BaseCache::CachePort(_name, _cache)
{
    // override default send event from SimpleTimingPort
    delete sendEvent;
    sendEvent = new SendEvent(this);
}
