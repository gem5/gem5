/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Ron Dreslinski
 */

/**
 * @file
 * Miss and writeback queue definitions.
 */

#include "cpu/smt.hh" //for maxThreadsPerCPU
#include "mem/cache/base_cache.hh"
#include "mem/cache/miss/miss_queue.hh"
#include "mem/cache/prefetch/base_prefetcher.hh"

using namespace std;

// simple constructor
/**
 * @todo Remove the +16 from the write buffer constructor once we handle
 * stalling on writebacks do to compression writes.
 */
MissQueue::MissQueue(int numMSHRs, int numTargets, int write_buffers,
                     bool write_allocate, bool prefetch_miss)
    : mq(numMSHRs, 4), wb(write_buffers,numMSHRs+1000), numMSHR(numMSHRs),
      numTarget(numTargets), writeBuffers(write_buffers),
      writeAllocate(write_allocate), order(0), prefetchMiss(prefetch_miss)
{
    noTargetMSHR = NULL;
}

void
MissQueue::regStats(const string &name)
{
    Request temp_req((Addr) NULL, 4, 0);
    Packet::Command temp_cmd = Packet::ReadReq;
    Packet temp_pkt(&temp_req, temp_cmd, 0);  //@todo FIx command strings so this isn't neccessary
    temp_pkt.allocate();

    using namespace Stats;

    writebacks
        .init(maxThreadsPerCPU)
        .name(name + ".writebacks")
        .desc("number of writebacks")
        .flags(total)
        ;

    // MSHR hit statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        mshr_hits[access_idx]
            .init(maxThreadsPerCPU)
            .name(name + "." + cstr + "_mshr_hits")
            .desc("number of " + cstr + " MSHR hits")
            .flags(total | nozero | nonan)
            ;
    }

    demandMshrHits
        .name(name + ".demand_mshr_hits")
        .desc("number of demand (read+write) MSHR hits")
        .flags(total)
        ;
    demandMshrHits = mshr_hits[Packet::ReadReq] + mshr_hits[Packet::WriteReq];

    overallMshrHits
        .name(name + ".overall_mshr_hits")
        .desc("number of overall MSHR hits")
        .flags(total)
        ;
    overallMshrHits = demandMshrHits + mshr_hits[Packet::SoftPFReq] +
        mshr_hits[Packet::HardPFReq];

    // MSHR miss statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        mshr_misses[access_idx]
            .init(maxThreadsPerCPU)
            .name(name + "." + cstr + "_mshr_misses")
            .desc("number of " + cstr + " MSHR misses")
            .flags(total | nozero | nonan)
            ;
    }

    demandMshrMisses
        .name(name + ".demand_mshr_misses")
        .desc("number of demand (read+write) MSHR misses")
        .flags(total)
        ;
    demandMshrMisses = mshr_misses[Packet::ReadReq] + mshr_misses[Packet::WriteReq];

    overallMshrMisses
        .name(name + ".overall_mshr_misses")
        .desc("number of overall MSHR misses")
        .flags(total)
        ;
    overallMshrMisses = demandMshrMisses + mshr_misses[Packet::SoftPFReq] +
        mshr_misses[Packet::HardPFReq];

    // MSHR miss latency statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        mshr_miss_latency[access_idx]
            .init(maxThreadsPerCPU)
            .name(name + "." + cstr + "_mshr_miss_latency")
            .desc("number of " + cstr + " MSHR miss cycles")
            .flags(total | nozero | nonan)
            ;
    }

    demandMshrMissLatency
        .name(name + ".demand_mshr_miss_latency")
        .desc("number of demand (read+write) MSHR miss cycles")
        .flags(total)
        ;
    demandMshrMissLatency = mshr_miss_latency[Packet::ReadReq]
        + mshr_miss_latency[Packet::WriteReq];

    overallMshrMissLatency
        .name(name + ".overall_mshr_miss_latency")
        .desc("number of overall MSHR miss cycles")
        .flags(total)
        ;
    overallMshrMissLatency = demandMshrMissLatency +
        mshr_miss_latency[Packet::SoftPFReq] + mshr_miss_latency[Packet::HardPFReq];

    // MSHR uncacheable statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        mshr_uncacheable[access_idx]
            .init(maxThreadsPerCPU)
            .name(name + "." + cstr + "_mshr_uncacheable")
            .desc("number of " + cstr + " MSHR uncacheable")
            .flags(total | nozero | nonan)
            ;
    }

    overallMshrUncacheable
        .name(name + ".overall_mshr_uncacheable_misses")
        .desc("number of overall MSHR uncacheable misses")
        .flags(total)
        ;
    overallMshrUncacheable = mshr_uncacheable[Packet::ReadReq]
        + mshr_uncacheable[Packet::WriteReq] + mshr_uncacheable[Packet::SoftPFReq]
        + mshr_uncacheable[Packet::HardPFReq];

    // MSHR miss latency statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        mshr_uncacheable_lat[access_idx]
            .init(maxThreadsPerCPU)
            .name(name + "." + cstr + "_mshr_uncacheable_latency")
            .desc("number of " + cstr + " MSHR uncacheable cycles")
            .flags(total | nozero | nonan)
            ;
    }

    overallMshrUncacheableLatency
        .name(name + ".overall_mshr_uncacheable_latency")
        .desc("number of overall MSHR uncacheable cycles")
        .flags(total)
        ;
    overallMshrUncacheableLatency = mshr_uncacheable_lat[Packet::ReadReq]
        + mshr_uncacheable_lat[Packet::WriteReq]
        + mshr_uncacheable_lat[Packet::SoftPFReq]
        + mshr_uncacheable_lat[Packet::HardPFReq];

#if 0
    // MSHR access formulas
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        mshrAccesses[access_idx]
            .name(name + "." + cstr + "_mshr_accesses")
            .desc("number of " + cstr + " mshr accesses(hits+misses)")
            .flags(total | nozero | nonan)
            ;
        mshrAccesses[access_idx] =
            mshr_hits[access_idx] + mshr_misses[access_idx]
            + mshr_uncacheable[access_idx];
    }

    demandMshrAccesses
        .name(name + ".demand_mshr_accesses")
        .desc("number of demand (read+write) mshr accesses")
        .flags(total | nozero | nonan)
        ;
    demandMshrAccesses = demandMshrHits + demandMshrMisses;

    overallMshrAccesses
        .name(name + ".overall_mshr_accesses")
        .desc("number of overall (read+write) mshr accesses")
        .flags(total | nozero | nonan)
        ;
    overallMshrAccesses = overallMshrHits + overallMshrMisses
        + overallMshrUncacheable;
#endif

    // MSHR miss rate formulas
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        mshrMissRate[access_idx]
            .name(name + "." + cstr + "_mshr_miss_rate")
            .desc("mshr miss rate for " + cstr + " accesses")
            .flags(total | nozero | nonan)
            ;

        mshrMissRate[access_idx] =
            mshr_misses[access_idx] / cache->accesses[access_idx];
    }

    demandMshrMissRate
        .name(name + ".demand_mshr_miss_rate")
        .desc("mshr miss rate for demand accesses")
        .flags(total)
        ;
    demandMshrMissRate = demandMshrMisses / cache->demandAccesses;

    overallMshrMissRate
        .name(name + ".overall_mshr_miss_rate")
        .desc("mshr miss rate for overall accesses")
        .flags(total)
        ;
    overallMshrMissRate = overallMshrMisses / cache->overallAccesses;

    // mshrMiss latency formulas
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        avgMshrMissLatency[access_idx]
            .name(name + "." + cstr + "_avg_mshr_miss_latency")
            .desc("average " + cstr + " mshr miss latency")
            .flags(total | nozero | nonan)
            ;

        avgMshrMissLatency[access_idx] =
            mshr_miss_latency[access_idx] / mshr_misses[access_idx];
    }

    demandAvgMshrMissLatency
        .name(name + ".demand_avg_mshr_miss_latency")
        .desc("average overall mshr miss latency")
        .flags(total)
        ;
    demandAvgMshrMissLatency = demandMshrMissLatency / demandMshrMisses;

    overallAvgMshrMissLatency
        .name(name + ".overall_avg_mshr_miss_latency")
        .desc("average overall mshr miss latency")
        .flags(total)
        ;
    overallAvgMshrMissLatency = overallMshrMissLatency / overallMshrMisses;

    // mshrUncacheable latency formulas
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

        avgMshrUncacheableLatency[access_idx]
            .name(name + "." + cstr + "_avg_mshr_uncacheable_latency")
            .desc("average " + cstr + " mshr uncacheable latency")
            .flags(total | nozero | nonan)
            ;

        avgMshrUncacheableLatency[access_idx] =
            mshr_uncacheable_lat[access_idx] / mshr_uncacheable[access_idx];
    }

    overallAvgMshrUncacheableLatency
        .name(name + ".overall_avg_mshr_uncacheable_latency")
        .desc("average overall mshr uncacheable latency")
        .flags(total)
        ;
    overallAvgMshrUncacheableLatency = overallMshrUncacheableLatency / overallMshrUncacheable;

    mshr_cap_events
        .init(maxThreadsPerCPU)
        .name(name + ".mshr_cap_events")
        .desc("number of times MSHR cap was activated")
        .flags(total)
        ;

    //software prefetching stats
    soft_prefetch_mshr_full
        .init(maxThreadsPerCPU)
        .name(name + ".soft_prefetch_mshr_full")
        .desc("number of mshr full events for SW prefetching instrutions")
        .flags(total)
        ;

    mshr_no_allocate_misses
        .name(name +".no_allocate_misses")
        .desc("Number of misses that were no-allocate")
        ;

}

void
MissQueue::setCache(BaseCache *_cache)
{
    cache = _cache;
    blkSize = cache->getBlockSize();
}

void
MissQueue::setPrefetcher(BasePrefetcher *_prefetcher)
{
    prefetcher = _prefetcher;
}

MSHR*
MissQueue::allocateMiss(PacketPtr &pkt, int size, Tick time)
{
    MSHR* mshr = mq.allocate(pkt, size);
    mshr->order = order++;
    if (!pkt->req->isUncacheable() ){//&& !pkt->isNoAllocate()) {
        // Mark this as a cache line fill
        mshr->pkt->flags |= CACHE_LINE_FILL;
    }
    if (mq.isFull()) {
        cache->setBlocked(Blocked_NoMSHRs);
    }
    if (pkt->cmd != Packet::HardPFReq) {
        //If we need to request the bus (not on HW prefetch), do so
        cache->setMasterRequest(Request_MSHR, time);
    }
    return mshr;
}


MSHR*
MissQueue::allocateWrite(PacketPtr &pkt, int size, Tick time)
{
    MSHR* mshr = wb.allocate(pkt,size);
    mshr->order = order++;

//REMOVING COMPRESSION FOR NOW
#if 0
    if (pkt->isCompressed()) {
        mshr->pkt->deleteData();
        mshr->pkt->actualSize = pkt->actualSize;
        mshr->pkt->data = new uint8_t[pkt->actualSize];
        memcpy(mshr->pkt->data, pkt->data, pkt->actualSize);
    } else {
#endif
        memcpy(mshr->pkt->getPtr<uint8_t>(), pkt->getPtr<uint8_t>(), pkt->getSize());
  //{

    if (wb.isFull()) {
        cache->setBlocked(Blocked_NoWBBuffers);
    }

    cache->setMasterRequest(Request_WB, time);

    return mshr;
}


/**
 * @todo Remove SW prefetches on mshr hits.
 */
void
MissQueue::handleMiss(PacketPtr &pkt, int blkSize, Tick time)
{
//    if (!cache->isTopLevel())
    if (prefetchMiss) prefetcher->handleMiss(pkt, time);

    int size = blkSize;
    Addr blkAddr = pkt->getAddr() & ~(Addr)(blkSize-1);
    MSHR* mshr = NULL;
    if (!pkt->req->isUncacheable()) {
        mshr = mq.findMatch(blkAddr);
        if (mshr) {
            //@todo remove hw_pf here
            mshr_hits[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
            if (mshr->threadNum != 0/*pkt->req->getThreadNum()*/) {
                mshr->threadNum = -1;
            }
            mq.allocateTarget(mshr, pkt);
            if (mshr->pkt->isNoAllocate() && !pkt->isNoAllocate()) {
                //We are adding an allocate after a no-allocate
                mshr->pkt->flags &= ~NO_ALLOCATE;
            }
            if (mshr->getNumTargets() == numTarget) {
                noTargetMSHR = mshr;
                cache->setBlocked(Blocked_NoTargets);
                mq.moveToFront(mshr);
            }
            return;
        }
        if (pkt->isNoAllocate()) {
            //Count no-allocate requests differently
            mshr_no_allocate_misses++;
        }
        else {
            mshr_misses[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
        }
    } else {
        //Count uncacheable accesses
        mshr_uncacheable[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
        size = pkt->getSize();
    }
    if (pkt->isWrite() && (pkt->req->isUncacheable() || !writeAllocate ||
                               !pkt->needsResponse())) {
        /**
         * @todo Add write merging here.
         */
        mshr = allocateWrite(pkt, pkt->getSize(), time);
        return;
    }

    mshr = allocateMiss(pkt, blkSize, time);
}

MSHR*
MissQueue::fetchBlock(Addr addr, int blk_size, Tick time,
                      PacketPtr &target)
{
    Addr blkAddr = addr & ~(Addr)(blk_size - 1);
    assert(mq.findMatch(addr) == NULL);
    MSHR *mshr = mq.allocateFetch(blkAddr, blk_size, target);
    mshr->order = order++;
    mshr->pkt->flags |= CACHE_LINE_FILL;
    if (mq.isFull()) {
        cache->setBlocked(Blocked_NoMSHRs);
    }
    cache->setMasterRequest(Request_MSHR, time);
    return mshr;
}

PacketPtr
MissQueue::getPacket()
{
    PacketPtr pkt = mq.getReq();
    if (((wb.isFull() && wb.inServiceMSHRs == 0) || !pkt ||
         pkt->time > curTick) && wb.havePending()) {
        pkt = wb.getReq();
        // Need to search for earlier miss.
        MSHR *mshr = mq.findPending(pkt);
        if (mshr && mshr->order < ((MSHR*)(pkt->senderState))->order) {
            // Service misses in order until conflict is cleared.
            return mq.getReq();
        }
    }
    if (pkt) {
        MSHR* mshr = wb.findPending(pkt);
        if (mshr /*&& mshr->order < pkt->senderState->order*/) {
            // The only way this happens is if we are
            // doing a write and we didn't have permissions
            // then subsequently saw a writeback(owned got evicted)
            // We need to make sure to perform the writeback first
            // To preserve the dirty data, then we can issue the write
            return wb.getReq();
        }
    }
    else if (!mq.isFull()){
        //If we have a miss queue slot, we can try a prefetch
        pkt = prefetcher->getPacket();
        if (pkt) {
            //Update statistic on number of prefetches issued (hwpf_mshr_misses)
            mshr_misses[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;
            //It will request the bus for the future, but should clear that immedieatley
            allocateMiss(pkt, pkt->getSize(), curTick);
            pkt = mq.getReq();
            assert(pkt); //We should get back a req b/c we just put one in
        }
    }
    return pkt;
}

void
MissQueue::setBusCmd(PacketPtr &pkt, Packet::Command cmd)
{
    assert(pkt->senderState != 0);
    MSHR * mshr = (MSHR*)pkt->senderState;
    mshr->originalCmd = pkt->cmd;
    if (cmd == Packet::UpgradeReq || cmd == Packet::InvalidateReq) {
        pkt->flags |= NO_ALLOCATE;
        pkt->flags &= ~CACHE_LINE_FILL;
    }
    else if (!pkt->req->isUncacheable() && !pkt->isNoAllocate() &&
             (cmd & (1 << 6)/*NeedsResponse*/)) {
        pkt->flags |= CACHE_LINE_FILL;
    }
    if (pkt->isCacheFill() || pkt->isNoAllocate())
        pkt->cmd = cmd;
}

void
MissQueue::restoreOrigCmd(PacketPtr &pkt)
{
    pkt->cmd = ((MSHR*)(pkt->senderState))->originalCmd;
}

void
MissQueue::markInService(PacketPtr &pkt, MSHR* mshr)
{
    bool unblock = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    /**
     * @todo Should include MSHRQueue pointer in MSHR to select the correct
     * one.
     */
    if ((!pkt->isCacheFill() && pkt->isWrite())) {
        // Forwarding a write/ writeback, don't need to change
        // the command
        unblock = wb.isFull();
        wb.markInService(mshr);
        if (!wb.havePending()){
            cache->clearMasterRequest(Request_WB);
        }
        if (unblock) {
            // Do we really unblock?
            unblock = !wb.isFull();
            cause = Blocked_NoWBBuffers;
        }
    } else {
        unblock = mq.isFull();
        mq.markInService(mshr);
        if (!mq.havePending()){
            cache->clearMasterRequest(Request_MSHR);
        }
        if (mshr->originalCmd == Packet::HardPFReq) {
            DPRINTF(HWPrefetch, "%s:Marking a HW_PF in service\n",
                    cache->name());
            //Also clear pending if need be
            if (!prefetcher->havePending())
            {
                cache->clearMasterRequest(Request_PF);
            }
        }
        if (unblock) {
            unblock = !mq.isFull();
            cause = Blocked_NoMSHRs;
        }
    }
    if (unblock) {
        cache->clearBlocked(cause);
    }
}


void
MissQueue::handleResponse(PacketPtr &pkt, Tick time)
{
    MSHR* mshr = (MSHR*)pkt->senderState;
    if (((MSHR*)(pkt->senderState))->originalCmd == Packet::HardPFReq) {
        DPRINTF(HWPrefetch, "%s:Handling the response to a HW_PF\n",
                cache->name());
    }
#ifndef NDEBUG
    int num_targets = mshr->getNumTargets();
#endif

    bool unblock = false;
    bool unblock_target = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    if (pkt->isCacheFill() && !pkt->isNoAllocate()) {
        mshr_miss_latency[mshr->originalCmd][0/*pkt->req->getThreadNum()*/] +=
            curTick - pkt->time;
        // targets were handled in the cache tags
        if (mshr == noTargetMSHR) {
            // we always clear at least one target
            unblock_target = true;
            cause = Blocked_NoTargets;
            noTargetMSHR = NULL;
        }

        if (mshr->hasTargets()) {
            // Didn't satisfy all the targets, need to resend
            Packet::Command cmd = mshr->getTarget()->cmd;
            mq.markPending(mshr, cmd);
            mshr->order = order++;
            cache->setMasterRequest(Request_MSHR, time);
        }
        else {
            unblock = mq.isFull();
            mq.deallocate(mshr);
            if (unblock) {
                unblock = !mq.isFull();
                cause = Blocked_NoMSHRs;
            }
        }
    } else {
        if (pkt->req->isUncacheable()) {
            mshr_uncacheable_lat[pkt->cmd][0/*pkt->req->getThreadNum()*/] +=
                curTick - pkt->time;
        }
        if (mshr->hasTargets() && pkt->req->isUncacheable()) {
            // Should only have 1 target if we had any
            assert(num_targets == 1);
            PacketPtr target = mshr->getTarget();
            mshr->popTarget();
            if (pkt->isRead()) {
                memcpy(target->getPtr<uint8_t>(), pkt->getPtr<uint8_t>(),
                       target->getSize());
            }
            cache->respond(target, time);
            assert(!mshr->hasTargets());
        }
        else if (mshr->hasTargets()) {
            //Must be a no_allocate with possibly more than one target
            assert(mshr->pkt->isNoAllocate());
            while (mshr->hasTargets()) {
                PacketPtr target = mshr->getTarget();
                mshr->popTarget();
                if (pkt->isRead()) {
                    memcpy(target->getPtr<uint8_t>(), pkt->getPtr<uint8_t>(),
                           target->getSize());
                }
                cache->respond(target, time);
            }
        }

        if (pkt->isWrite()) {
            // If the wrtie buffer is full, we might unblock now
            unblock = wb.isFull();
            wb.deallocate(mshr);
            if (unblock) {
                // Did we really unblock?
                unblock = !wb.isFull();
                cause = Blocked_NoWBBuffers;
            }
        } else {
            unblock = mq.isFull();
            mq.deallocate(mshr);
            if (unblock) {
                unblock = !mq.isFull();
                cause = Blocked_NoMSHRs;
            }
        }
    }
    if (unblock || unblock_target) {
        cache->clearBlocked(cause);
    }
}

void
MissQueue::squash(int threadNum)
{
    bool unblock = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    if (noTargetMSHR && noTargetMSHR->threadNum == threadNum) {
        noTargetMSHR = NULL;
        unblock = true;
        cause = Blocked_NoTargets;
    }
    if (mq.isFull()) {
        unblock = true;
        cause = Blocked_NoMSHRs;
    }
    mq.squash(threadNum);
    if (!mq.havePending()) {
        cache->clearMasterRequest(Request_MSHR);
    }
    if (unblock && !mq.isFull()) {
        cache->clearBlocked(cause);
    }

}

MSHR*
MissQueue::findMSHR(Addr addr) const
{
    return mq.findMatch(addr);
}

bool
MissQueue::findWrites(Addr addr, vector<MSHR*> &writes) const
{
    return wb.findMatches(addr,writes);
}

void
MissQueue::doWriteback(Addr addr,
                       int size, uint8_t *data, bool compressed)
{
    // Generate request
    Request * req = new Request(addr, size, 0);
    PacketPtr pkt = new Packet(req, Packet::Writeback, -1);
    pkt->allocate();
    if (data) {
        memcpy(pkt->getPtr<uint8_t>(), data, size);
    }

    if (compressed) {
        pkt->flags |= COMPRESSED;
    }

    ///All writebacks charged to same thread @todo figure this out
    writebacks[0/*pkt->req->getThreadNum()*/]++;

    allocateWrite(pkt, 0, curTick);
}


void
MissQueue::doWriteback(PacketPtr &pkt)
{
    writebacks[0/*pkt->req->getThreadNum()*/]++;
    allocateWrite(pkt, 0, curTick);
}


MSHR*
MissQueue::allocateTargetList(Addr addr)
{
   MSHR* mshr = mq.allocateTargetList(addr, blkSize);
   mshr->pkt->flags |= CACHE_LINE_FILL;
   if (mq.isFull()) {
       cache->setBlocked(Blocked_NoMSHRs);
   }
   return mshr;
}

bool
MissQueue::havePending()
{
    return mq.havePending() || wb.havePending() || prefetcher->havePending();
}
