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

#include "cpu/exec_context.hh"
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
        const string &cstr = cmd.toString();

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
    demandMshrHits = mshr_hits[Read] + mshr_hits[Write];

    overallMshrHits
        .name(name + ".overall_mshr_hits")
        .desc("number of overall MSHR hits")
        .flags(total)
        ;
    overallMshrHits = demandMshrHits + mshr_hits[Soft_Prefetch] +
        mshr_hits[Hard_Prefetch];

    // MSHR miss statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::CommandEnum)access_idx;
        const string &cstr = cmd.toString();

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
    demandMshrMisses = mshr_misses[Read] + mshr_misses[Write];

    overallMshrMisses
        .name(name + ".overall_mshr_misses")
        .desc("number of overall MSHR misses")
        .flags(total)
        ;
    overallMshrMisses = demandMshrMisses + mshr_misses[Soft_Prefetch] +
        mshr_misses[Hard_Prefetch];

    // MSHR miss latency statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::CommandEnum)access_idx;
        const string &cstr = cmd.toString();

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
    demandMshrMissLatency = mshr_miss_latency[Read] + mshr_miss_latency[Write];

    overallMshrMissLatency
        .name(name + ".overall_mshr_miss_latency")
        .desc("number of overall MSHR miss cycles")
        .flags(total)
        ;
    overallMshrMissLatency = demandMshrMissLatency +
        mshr_miss_latency[Soft_Prefetch] + mshr_miss_latency[Hard_Prefetch];

    // MSHR uncacheable statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::CommandEnum)access_idx;
        const string &cstr = cmd.toString();

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
    overallMshrUncacheable = mshr_uncacheable[Read] + mshr_uncacheable[Write]
        + mshr_uncacheable[Soft_Prefetch] + mshr_uncacheable[Hard_Prefetch];

    // MSHR miss latency statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::CommandEnum)access_idx;
        const string &cstr = cmd.toString();

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
    overallMshrUncacheableLatency = mshr_uncacheable_lat[Read]
        + mshr_uncacheable_lat[Write] + mshr_uncacheable_lat[Soft_Prefetch]
        + mshr_uncacheable_lat[Hard_Prefetch];

#if 0
    // MSHR access formulas
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::CommandEnum)access_idx;
        const string &cstr = cmd.toString();

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
        Packet::Command cmd = (Packet::CommandEnum)access_idx;
        const string &cstr = cmd.toString();

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
        Packet::Command cmd = (Packet::CommandEnum)access_idx;
        const string &cstr = cmd.toString();

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
        Packet::Command cmd = (Packet::CommandEnum)access_idx;
        const string &cstr = cmd.toString();

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
MissQueue::allocateMiss(Packet * &pkt, int size, Tick time)
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
    if (pkt->cmd != Hard_Prefetch) {
        //If we need to request the bus (not on HW prefetch), do so
        cache->setMasterRequest(Request_MSHR, time);
    }
    return mshr;
}


MSHR*
MissQueue::allocateWrite(Packet * &pkt, int size, Tick time)
{
    MSHR* mshr = wb.allocate(pkt,pkt->size);
    mshr->order = order++;
    if (cache->doData()){
        if (pkt->isCompressed()) {
            delete [] mshr->pkt->data;
            mshr->pkt->actualSize = pkt->actualSize;
            mshr->pkt->data = new uint8_t[pkt->actualSize];
            memcpy(mshr->pkt->data, pkt->data, pkt->actualSize);
        } else {
            memcpy(mshr->pkt->data, pkt->data, pkt->size);
        }
    }
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
MissQueue::handleMiss(Packet * &pkt, int blkSize, Tick time)
{
//    if (!cache->isTopLevel())
    if (prefetchMiss) prefetcher->handleMiss(pkt, time);

    int size = blkSize;
    Addr blkAddr = pkt->paddr & ~(Addr)(blkSize-1);
    MSHR* mshr = NULL;
    if (!pkt->req->isUncacheable()) {
        mshr = mq.findMatch(blkAddr, pkt->req->asid);
        if (mshr) {
            //@todo remove hw_pf here
            mshr_hits[pkt->cmdToIndex()][pkt->req->getThreadNum()]++;
            if (mshr->getThreadNum() != pkt->req->getThreadNum()) {
                mshr->setThreadNum() = -1;
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
            mshr_misses[pkt->cmdToIndex()][pkt->req->getThreadNum()]++;
        }
    } else {
        //Count uncacheable accesses
        mshr_uncacheable[pkt->cmdToIndex()][pkt->req->getThreadNum()]++;
        size = pkt->size;
    }
    if (pkt->cmd.isWrite() && (pkt->req->isUncacheable() || !writeAllocate ||
                               pkt->cmd.isNoResponse())) {
        /**
         * @todo Add write merging here.
         */
        mshr = allocateWrite(pkt, pkt->size, time);
        return;
    }

    mshr = allocateMiss(pkt, size, time);
}

MSHR*
MissQueue::fetchBlock(Addr addr, int asid, int blk_size, Tick time,
                      Packet * &target)
{
    Addr blkAddr = addr & ~(Addr)(blk_size - 1);
    assert(mq.findMatch(addr, asid) == NULL);
    MSHR *mshr = mq.allocateFetch(blkAddr, asid, blk_size, target);
    mshr->order = order++;
    mshr->pkt->flags |= CACHE_LINE_FILL;
    if (mq.isFull()) {
        cache->setBlocked(Blocked_NoMSHRs);
    }
    cache->setMasterRequest(Request_MSHR, time);
    return mshr;
}

Packet *
MissQueue::getPacket()
{
    Packet * pkt = mq.getReq();
    if (((wb.isFull() && wb.inServiceMSHRs == 0) || !pkt ||
         pkt->time > curTick) && wb.havePending()) {
        pkt = wb.getReq();
        // Need to search for earlier miss.
        MSHR *mshr = mq.findPending(pkt);
        if (mshr && mshr->order < pkt->senderState->order) {
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
            mshr_misses[pkt->cmdToIndex()][pkt->req->getThreadNum()]++;
            //It will request the bus for the future, but should clear that immedieatley
            allocateMiss(pkt, pkt->size, curTick);
            pkt = mq.getReq();
            assert(pkt); //We should get back a req b/c we just put one in
        }
    }
    return pkt;
}

void
MissQueue::setBusCmd(Packet * &pkt, Packet::Command cmd)
{
    assert(pkt->senderState != 0);
    MSHR * mshr = pkt->senderState;
    mshr->originalCmd = pkt->cmd;
    if (pkt->isCacheFill() || pkt->isNoAllocate())
        pkt->cmd = cmd;
}

void
MissQueue::restoreOrigCmd(Packet * &pkt)
{
    pkt->cmd = pkt->senderState->originalCmd;
}

void
MissQueue::markInService(Packet * &pkt)
{
    assert(pkt->senderState != 0);
    bool unblock = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    /**
     * @todo Should include MSHRQueue pointer in MSHR to select the correct
     * one.
     */
    if ((!pkt->isCacheFill() && pkt->cmd.isWrite()) || pkt->cmd == Copy) {
        // Forwarding a write/ writeback, don't need to change
        // the command
        unblock = wb.isFull();
        wb.markInService(pkt->senderState);
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
        mq.markInService(pkt->senderState);
        if (!mq.havePending()){
            cache->clearMasterRequest(Request_MSHR);
        }
        if (pkt->senderState->originalCmd == Hard_Prefetch) {
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
MissQueue::handleResponse(Packet * &pkt, Tick time)
{
    MSHR* mshr = pkt->senderState;
    if (pkt->senderState->originalCmd == Hard_Prefetch) {
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
        mshr_miss_latency[mshr->originalCmd][pkt->req->getThreadNum()] +=
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
            mshr_uncacheable_lat[pkt->cmd][pkt->req->getThreadNum()] +=
                curTick - pkt->time;
        }
        if (mshr->hasTargets() && pkt->req->isUncacheable()) {
            // Should only have 1 target if we had any
            assert(num_targets == 1);
            Packet * target = mshr->getTarget();
            mshr->popTarget();
            if (cache->doData() && pkt->cmd.isRead()) {
                memcpy(target->data, pkt->data, target->size);
            }
            cache->respond(target, time);
            assert(!mshr->hasTargets());
        }
        else if (mshr->hasTargets()) {
            //Must be a no_allocate with possibly more than one target
            assert(mshr->pkt->isNoAllocate());
            while (mshr->hasTargets()) {
                Packet * target = mshr->getTarget();
                mshr->popTarget();
                if (cache->doData() && pkt->cmd.isRead()) {
                    memcpy(target->data, pkt->data, target->size);
                }
                cache->respond(target, time);
            }
        }

        if (pkt->cmd.isWrite()) {
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
MissQueue::squash(int req->getThreadNum()ber)
{
    bool unblock = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    if (noTargetMSHR && noTargetMSHR->setThreadNum() == req->getThreadNum()ber) {
        noTargetMSHR = NULL;
        unblock = true;
        cause = Blocked_NoTargets;
    }
    if (mq.isFull()) {
        unblock = true;
        cause = Blocked_NoMSHRs;
    }
    mq.squash(req->getThreadNum()ber);
    if (!mq.havePending()) {
        cache->clearMasterRequest(Request_MSHR);
    }
    if (unblock && !mq.isFull()) {
        cache->clearBlocked(cause);
    }

}

MSHR*
MissQueue::findMSHR(Addr addr, int asid) const
{
    return mq.findMatch(addr,asid);
}

bool
MissQueue::findWrites(Addr addr, int asid, vector<MSHR*> &writes) const
{
    return wb.findMatches(addr,asid,writes);
}

void
MissQueue::doWriteback(Addr addr, int asid,
                       int size, uint8_t *data, bool compressed)
{
    // Generate request
    Packet * pkt = buildWritebackReq(addr, asid, size, data,
                                      compressed);

    writebacks[pkt->req->getThreadNum()]++;

    allocateWrite(pkt, 0, curTick);
}


void
MissQueue::doWriteback(Packet * &pkt)
{
    writebacks[pkt->req->getThreadNum()]++;
    allocateWrite(pkt, 0, curTick);
}


MSHR*
MissQueue::allocateTargetList(Addr addr, int asid)
{
   MSHR* mshr = mq.allocateTargetList(addr, asid, blkSize);
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
