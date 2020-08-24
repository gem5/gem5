/*
 * Copyright (c) 2010-2018 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2010,2015 Advanced Micro Devices, Inc.
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
 * Cache definitions.
 */

#include "mem/cache/noncoherent_cache.hh"

#include <cassert>

#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Cache.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/mshr.hh"
#include "params/NoncoherentCache.hh"

NoncoherentCache::NoncoherentCache(const NoncoherentCacheParams *p)
    : BaseCache(p, p->system->cacheLineSize())
{
}

void
NoncoherentCache::satisfyRequest(PacketPtr pkt, CacheBlk *blk, bool, bool)
{
    // As this a non-coherent cache located below the point of
    // coherency, we do not expect requests that are typically used to
    // keep caches coherent (e.g., InvalidateReq or UpdateReq).
    assert(pkt->isRead() || pkt->isWrite());
    BaseCache::satisfyRequest(pkt, blk);
}

bool
NoncoherentCache::access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                         PacketList &writebacks)
{
    bool success = BaseCache::access(pkt, blk, lat, writebacks);

    if (pkt->isWriteback() || pkt->cmd == MemCmd::WriteClean) {
        assert(blk && blk->isValid());
        // Writeback and WriteClean can allocate and fill even if the
        // referenced block was not present or it was invalid. If that
        // is the case, make sure that the new block is marked as
        // writable
        blk->status |= BlkWritable;
    }

    return success;
}

void
NoncoherentCache::doWritebacks(PacketList& writebacks, Tick forward_time)
{
    while (!writebacks.empty()) {
        PacketPtr wb_pkt = writebacks.front();
        allocateWriteBuffer(wb_pkt, forward_time);
        writebacks.pop_front();
    }
}

void
NoncoherentCache::doWritebacksAtomic(PacketList& writebacks)
{
    while (!writebacks.empty()) {
        PacketPtr wb_pkt = writebacks.front();
        memSidePort.sendAtomic(wb_pkt);
        writebacks.pop_front();
        delete wb_pkt;
    }
}

void
NoncoherentCache::handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk,
                                      Tick forward_time, Tick request_time)
{
    // miss
    Addr blk_addr = pkt->getBlockAddr(blkSize);
    MSHR *mshr = mshrQueue.findMatch(blk_addr, pkt->isSecure(), false);

    // We can always write to a non coherent cache if the block is
    // present and therefore if we have reached this point then the
    // block should not be in the cache.
    assert(mshr || !blk || !blk->isValid());

    BaseCache::handleTimingReqMiss(pkt, mshr, blk, forward_time, request_time);
}

void
NoncoherentCache::recvTimingReq(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at non-coherent cache\n");

    BaseCache::recvTimingReq(pkt);
}

PacketPtr
NoncoherentCache::createMissPacket(PacketPtr cpu_pkt, CacheBlk *blk,
                                   bool needs_writable,
                                   bool is_whole_line_write) const
{
    // We also fill for writebacks from the coherent caches above us,
    // and they do not need responses
    assert(cpu_pkt->needsResponse());

    // A miss can happen only due to missing block
    assert(!blk || !blk->isValid());

    PacketPtr pkt = new Packet(cpu_pkt->req, MemCmd::ReadReq, blkSize);

    // the packet should be block aligned
    assert(pkt->getAddr() == pkt->getBlockAddr(blkSize));

    pkt->allocate();
    DPRINTF(Cache, "%s created %s from %s\n", __func__, pkt->print(),
            cpu_pkt->print());
    return pkt;
}


Cycles
NoncoherentCache::handleAtomicReqMiss(PacketPtr pkt, CacheBlk *&blk,
                                      PacketList &writebacks)
{
    PacketPtr bus_pkt = createMissPacket(pkt, blk, true,
                                         pkt->isWholeLineWrite(blkSize));
    DPRINTF(Cache, "Sending an atomic %s\n", bus_pkt->print());

    Cycles latency = ticksToCycles(memSidePort.sendAtomic(bus_pkt));

    assert(bus_pkt->isResponse());
    // At the moment the only supported downstream requests we issue
    // are ReadReq and therefore here we should only see the
    // corresponding responses
    assert(bus_pkt->isRead());
    assert(pkt->cmd != MemCmd::UpgradeResp);
    assert(!bus_pkt->isInvalidate());
    assert(!bus_pkt->hasSharers());

    // We are now dealing with the response handling
    DPRINTF(Cache, "Receive response: %s\n", bus_pkt->print());

    if (!bus_pkt->isError()) {
        // Any reponse that does not have an error should be filling,
        // afterall it is a read response
        DPRINTF(Cache, "Block for addr %#llx being updated in Cache\n",
                bus_pkt->getAddr());
        blk = handleFill(bus_pkt, blk, writebacks, allocOnFill(bus_pkt->cmd));
        assert(blk);
    }
    satisfyRequest(pkt, blk);

    maintainClusivity(true, blk);

    // Use the separate bus_pkt to generate response to pkt and
    // then delete it.
    if (!pkt->isWriteback() && pkt->cmd != MemCmd::WriteClean) {
        assert(pkt->needsResponse());
        pkt->makeAtomicResponse();
        if (bus_pkt->isError()) {
            pkt->copyError(bus_pkt);
        }
    }

    delete bus_pkt;

    return latency;
}

Tick
NoncoherentCache::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at non-coherent cache\n");

    return BaseCache::recvAtomic(pkt);
}


void
NoncoherentCache::functionalAccess(PacketPtr pkt, bool from_cpu_side)
{
    panic_if(!from_cpu_side, "Non-coherent cache received functional snoop"
            " request\n");

    BaseCache::functionalAccess(pkt, from_cpu_side);
}

void
NoncoherentCache::serviceMSHRTargets(MSHR *mshr, const PacketPtr pkt,
                                     CacheBlk *blk)
{
    // First offset for critical word first calculations
    const int initial_offset = mshr->getTarget()->pkt->getOffset(blkSize);

    MSHR::TargetList targets = mshr->extractServiceableTargets(pkt);
    for (auto &target: targets) {
        Packet *tgt_pkt = target.pkt;

        switch (target.source) {
          case MSHR::Target::FromCPU:
            // handle deferred requests comming from a cache or core
            // above

            Tick completion_time;
            // Here we charge on completion_time the delay of the xbar if the
            // packet comes from it, charged on headerDelay.
            completion_time = pkt->headerDelay;

            satisfyRequest(tgt_pkt, blk);

            // How many bytes past the first request is this one
            int transfer_offset;
            transfer_offset = tgt_pkt->getOffset(blkSize) - initial_offset;
            if (transfer_offset < 0) {
                transfer_offset += blkSize;
            }
            // If not critical word (offset) return payloadDelay.
            // responseLatency is the latency of the return path
            // from lower level caches/memory to an upper level cache or
            // the core.
            completion_time += clockEdge(responseLatency) +
                (transfer_offset ? pkt->payloadDelay : 0);

            assert(tgt_pkt->req->requestorId() < system->maxRequestors());
            stats.cmdStats(tgt_pkt).missLatency[tgt_pkt->req->requestorId()] +=
                completion_time - target.recvTime;

            tgt_pkt->makeTimingResponse();
            if (pkt->isError())
                tgt_pkt->copyError(pkt);

            // Reset the bus additional time as it is now accounted for
            tgt_pkt->headerDelay = tgt_pkt->payloadDelay = 0;
            cpuSidePort.schedTimingResp(tgt_pkt, completion_time);
            break;

          case MSHR::Target::FromPrefetcher:
            // handle deferred requests comming from a prefetcher
            // attached to this cache
            assert(tgt_pkt->cmd == MemCmd::HardPFReq);

            if (blk)
                blk->status |= BlkHWPrefetched;

            // We have filled the block and the prefetcher does not
            // require responses.
            delete tgt_pkt;
            break;

          default:
            // we should never see FromSnoop Targets as this is a
            // non-coherent cache
            panic("Illegal target->source enum %d\n", target.source);
        }
    }

    // Reponses are filling and bring in writable blocks, therefore
    // there should be no deferred targets and all the non-deferred
    // targets are now serviced.
    assert(mshr->getNumTargets() == 0);
}

void
NoncoherentCache::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());
    // At the moment the only supported downstream requests we issue
    // are ReadReq and therefore here we should only see the
    // corresponding responses
    assert(pkt->isRead());
    assert(pkt->cmd != MemCmd::UpgradeResp);
    assert(!pkt->isInvalidate());
    // This cache is non-coherent and any memories below are
    // non-coherent too (non-coherent caches or the main memory),
    // therefore the fetched block can be marked as writable.
    assert(!pkt->hasSharers());

    BaseCache::recvTimingResp(pkt);
}

PacketPtr
NoncoherentCache::evictBlock(CacheBlk *blk)
{
    // A dirty block is always written back.

    // A clean block can we written back, if we turned on writebacks
    // for clean blocks. This could be useful if there is a cache
    // below and we want to make sure the block is cached but if the
    // memory below is the main memory WritebackCleans are
    // unnecessary.

    // If we clean writebacks are not enabled, we do not take any
    // further action for evictions of clean blocks (i.e., CleanEvicts
    // are unnecessary).
    PacketPtr pkt = (blk->isDirty() || writebackClean) ?
        writebackBlk(blk) : nullptr;

    invalidateBlock(blk);

    return pkt;
}

NoncoherentCache*
NoncoherentCacheParams::create()
{
    assert(tags);
    assert(replacement_policy);

    return new NoncoherentCache(this);
}
