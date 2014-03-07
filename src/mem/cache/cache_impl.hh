/*
 * Copyright (c) 2010-2013 ARM Limited
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
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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
 *          Andreas Sandberg
 */

#ifndef __MEM_CACHE_CACHE_IMPL_HH__
#define __MEM_CACHE_CACHE_IMPL_HH__

/**
 * @file
 * Cache definitions.
 */

#include "base/misc.hh"
#include "base/types.hh"
#include "debug/Cache.hh"
#include "debug/CachePort.hh"
#include "debug/CacheTags.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/mshr.hh"
#include "sim/sim_exit.hh"

template<class TagStore>
Cache<TagStore>::Cache(const Params *p)
    : BaseCache(p),
      tags(dynamic_cast<TagStore*>(p->tags)),
      prefetcher(p->prefetcher),
      doFastWrites(true),
      prefetchOnAccess(p->prefetch_on_access)
{
    tempBlock = new BlkType();
    tempBlock->data = new uint8_t[blkSize];

    cpuSidePort = new CpuSidePort(p->name + ".cpu_side", this,
                                  "CpuSidePort");
    memSidePort = new MemSidePort(p->name + ".mem_side", this,
                                  "MemSidePort");

    tags->setCache(this);
    if (prefetcher)
        prefetcher->setCache(this);
}

template<class TagStore>
Cache<TagStore>::~Cache()
{
    delete [] tempBlock->data;
    delete tempBlock;

    delete cpuSidePort;
    delete memSidePort;
}

template<class TagStore>
void
Cache<TagStore>::regStats()
{
    BaseCache::regStats();
}

template<class TagStore>
void
Cache<TagStore>::cmpAndSwap(BlkType *blk, PacketPtr pkt)
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

    if (overwrite_mem) {
        std::memcpy(blk_data, &overwrite_val, pkt->getSize());
        blk->status |= BlkDirty;
    }
}


template<class TagStore>
void
Cache<TagStore>::satisfyCpuSideRequest(PacketPtr pkt, BlkType *blk,
                                       bool deferred_response,
                                       bool pending_downgrade)
{
    assert(blk && blk->isValid());
    // Occasionally this is not true... if we are a lower-level cache
    // satisfying a string of Read and ReadEx requests from
    // upper-level caches, a Read will mark the block as shared but we
    // can satisfy a following ReadEx anyway since we can rely on the
    // Read requester(s) to have buffered the ReadEx snoop and to
    // invalidate their blocks after receiving them.
    // assert(!pkt->needsExclusive() || blk->isWritable());
    assert(pkt->getOffset(blkSize) + pkt->getSize() <= blkSize);

    // Check RMW operations first since both isRead() and
    // isWrite() will be true for them
    if (pkt->cmd == MemCmd::SwapReq) {
        cmpAndSwap(blk, pkt);
    } else if (pkt->isWrite()) {
        if (blk->checkWrite(pkt)) {
            pkt->writeDataToBlock(blk->data, blkSize);
            blk->status |= BlkDirty;
        }
    } else if (pkt->isRead()) {
        if (pkt->isLLSC()) {
            blk->trackLoadLocked(pkt);
        }
        pkt->setDataFromBlock(blk->data, blkSize);
        if (pkt->getSize() == blkSize) {
            // special handling for coherent block requests from
            // upper-level caches
            if (pkt->needsExclusive()) {
                // if we have a dirty copy, make sure the recipient
                // keeps it marked dirty
                if (blk->isDirty()) {
                    pkt->assertMemInhibit();
                }
                // on ReadExReq we give up our copy unconditionally
                if (blk != tempBlock)
                    tags->invalidate(blk);
                blk->invalidate();
            } else if (blk->isWritable() && !pending_downgrade
                      && !pkt->sharedAsserted() && !pkt->req->isInstFetch()) {
                // we can give the requester an exclusive copy (by not
                // asserting shared line) on a read request if:
                // - we have an exclusive copy at this level (& below)
                // - we don't have a pending snoop from below
                //   signaling another read request
                // - no other cache above has a copy (otherwise it
                //   would have asseretd shared line on request)
                // - we are not satisfying an instruction fetch (this
                //   prevents dirty data in the i-cache)

                if (blk->isDirty()) {
                    // special considerations if we're owner:
                    if (!deferred_response && !isTopLevel) {
                        // if we are responding immediately and can
                        // signal that we're transferring ownership
                        // along with exclusivity, do so
                        pkt->assertMemInhibit();
                        blk->status &= ~BlkDirty;
                    } else {
                        // if we're responding after our own miss,
                        // there's a window where the recipient didn't
                        // know it was getting ownership and may not
                        // have responded to snoops correctly, so we
                        // can't pass off ownership *or* exclusivity
                        pkt->assertShared();
                    }
                }
            } else {
                // otherwise only respond with a shared copy
                pkt->assertShared();
            }
        }
    } else {
        // Not a read or write... must be an upgrade.  it's OK
        // to just ack those as long as we have an exclusive
        // copy at this level.
        assert(pkt->isUpgrade());
        assert(blk != tempBlock);
        tags->invalidate(blk);
        blk->invalidate();
    }
}


/////////////////////////////////////////////////////
//
// MSHR helper functions
//
/////////////////////////////////////////////////////


template<class TagStore>
void
Cache<TagStore>::markInService(MSHR *mshr, PacketPtr pkt)
{
    markInServiceInternal(mshr, pkt);
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
}


template<class TagStore>
void
Cache<TagStore>::squash(int threadNum)
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
    if (unblock && !mshrQueue.isFull()) {
        clearBlocked(cause);
    }
}

/////////////////////////////////////////////////////
//
// Access path: requests coming in from the CPU side
//
/////////////////////////////////////////////////////

template<class TagStore>
bool
Cache<TagStore>::access(PacketPtr pkt, BlkType *&blk,
                        Cycles &lat, PacketList &writebacks)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    if (pkt->req->isUncacheable()) {
        uncacheableFlush(pkt);
        blk = NULL;
        lat = hitLatency;
        return false;
    }

    int id = pkt->req->hasContextId() ? pkt->req->contextId() : -1;
    blk = tags->accessBlock(pkt->getAddr(), pkt->isSecure(), lat, id);

    DPRINTF(Cache, "%s%s %x (%s) %s %s\n", pkt->cmdString(),
            pkt->req->isInstFetch() ? " (ifetch)" : "",
            pkt->getAddr(), pkt->isSecure() ? "s" : "ns",
            blk ? "hit" : "miss", blk ? blk->print() : "");

    if (blk != NULL) {

        if (pkt->needsExclusive() ? blk->isWritable() : blk->isReadable()) {
            // OK to satisfy access
            incHitCount(pkt);
            satisfyCpuSideRequest(pkt, blk);
            return true;
        }
    }

    // Can't satisfy access normally... either no block (blk == NULL)
    // or have block but need exclusive & only have shared.

    // Writeback handling is special case.  We can write the block
    // into the cache without having a writeable copy (or any copy at
    // all).
    if (pkt->cmd == MemCmd::Writeback) {
        assert(blkSize == pkt->getSize());
        if (blk == NULL) {
            // need to do a replacement
            blk = allocateBlock(pkt->getAddr(), pkt->isSecure(), writebacks);
            if (blk == NULL) {
                // no replaceable block available, give up.
                // writeback will be forwarded to next level.
                incMissCount(pkt);
                return false;
            }
            tags->insertBlock(pkt, blk);
            blk->status = BlkValid | BlkReadable;
        }
        std::memcpy(blk->data, pkt->getPtr<uint8_t>(), blkSize);
        blk->status |= BlkDirty;
        if (pkt->isSupplyExclusive()) {
            blk->status |= BlkWritable;
        }
        // nothing else to do; writeback doesn't expect response
        assert(!pkt->needsResponse());
        DPRINTF(Cache, "%s new state is %s\n", __func__, blk->print());
        incHitCount(pkt);
        return true;
    }

    incMissCount(pkt);

    if (blk == NULL && pkt->isLLSC() && pkt->isWrite()) {
        // complete miss on store conditional... just give up now
        pkt->req->setExtraData(0);
        return true;
    }

    return false;
}


class ForwardResponseRecord : public Packet::SenderState
{
  public:

    PortID prevSrc;

    ForwardResponseRecord(PortID prev_src) : prevSrc(prev_src)
    {}
};

template<class TagStore>
void
Cache<TagStore>::recvTimingSnoopResp(PacketPtr pkt)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    Tick time = clockEdge(hitLatency);

    assert(pkt->isResponse());

    // must be cache-to-cache response from upper to lower level
    ForwardResponseRecord *rec =
        dynamic_cast<ForwardResponseRecord *>(pkt->senderState);
    assert(!system->bypassCaches());

    if (rec == NULL) {
        assert(pkt->cmd == MemCmd::HardPFResp);
        // Check if it's a prefetch response and handle it. We shouldn't
        // get any other kinds of responses without FRRs.
        DPRINTF(Cache, "Got prefetch response from above for addr %#x (%s)\n",
                pkt->getAddr(), pkt->isSecure() ? "s" : "ns");
        recvTimingResp(pkt);
        return;
    }

    pkt->popSenderState();
    pkt->setDest(rec->prevSrc);
    delete rec;
    // @todo someone should pay for this
    pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;
    memSidePort->schedTimingSnoopResp(pkt, time);
}

template<class TagStore>
bool
Cache<TagStore>::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(CacheTags, "%s tags: %s\n", __func__, tags->print());
//@todo Add back in MemDebug Calls
//    MemDebug::cacheAccess(pkt);


    /// @todo temporary hack to deal with memory corruption issue until
    /// 4-phase transactions are complete
    for (int x = 0; x < pendingDelete.size(); x++)
        delete pendingDelete[x];
    pendingDelete.clear();

    // we charge hitLatency for doing just about anything here
    Tick time = clockEdge(hitLatency);

    assert(pkt->isRequest());

    // Just forward the packet if caches are disabled.
    if (system->bypassCaches()) {
        memSidePort->sendTimingReq(pkt);
        return true;
    }

    if (pkt->memInhibitAsserted()) {
        DPRINTF(Cache, "mem inhibited on 0x%x (%s): not responding\n",
                pkt->getAddr(), pkt->isSecure() ? "s" : "ns");
        assert(!pkt->req->isUncacheable());
        // Special tweak for multilevel coherence: snoop downward here
        // on invalidates since there may be other caches below here
        // that have shared copies.  Not necessary if we know that
        // supplier had exclusive copy to begin with.
        if (pkt->needsExclusive() && !pkt->isSupplyExclusive()) {
            Packet *snoopPkt = new Packet(pkt, true);  // clear flags
            // also reset the bus time that the original packet has
            // not yet paid for
            snoopPkt->busFirstWordDelay = snoopPkt->busLastWordDelay = 0;
            snoopPkt->setExpressSnoop();
            snoopPkt->assertMemInhibit();
            memSidePort->sendTimingReq(snoopPkt);
            // main memory will delete snoopPkt
        }
        // since we're the official target but we aren't responding,
        // delete the packet now.

        /// @todo nominally we should just delete the packet here,
        /// however, until 4-phase stuff we can't because sending
        /// cache is still relying on it
        pendingDelete.push_back(pkt);
        return true;
    }

    if (pkt->req->isUncacheable()) {
        uncacheableFlush(pkt);

        // @todo: someone should pay for this
        pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;

        // writes go in write buffer, reads use MSHR
        if (pkt->isWrite() && !pkt->isRead()) {
            allocateWriteBuffer(pkt, time, true);
        } else {
            allocateUncachedReadBuffer(pkt, time, true);
        }
        assert(pkt->needsResponse()); // else we should delete it here??
        return true;
    }

    Cycles lat = hitLatency;
    BlkType *blk = NULL;
    PacketList writebacks;

    bool satisfied = access(pkt, blk, lat, writebacks);

#if 0
    /** @todo make the fast write alloc (wh64) work with coherence. */

    // If this is a block size write/hint (WH64) allocate the block here
    // if the coherence protocol allows it.
    if (!blk && pkt->getSize() >= blkSize && coherence->allowFastWrites() &&
        (pkt->cmd == MemCmd::WriteReq
         || pkt->cmd == MemCmd::WriteInvalidateReq) ) {
        // not outstanding misses, can do this
        MSHR *outstanding_miss = mshrQueue.findMatch(pkt->getAddr(),
                                                     pkt->isSecure());
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

    // track time of availability of next prefetch, if any
    Tick next_pf_time = 0;

    bool needsResponse = pkt->needsResponse();

    if (satisfied) {
        if (prefetcher && (prefetchOnAccess || (blk && blk->wasPrefetched()))) {
            if (blk)
                blk->status &= ~BlkHWPrefetched;
            next_pf_time = prefetcher->notify(pkt, time);
        }

        if (needsResponse) {
            pkt->makeTimingResponse();
            // @todo: Make someone pay for this
            pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;
            cpuSidePort->schedTimingResp(pkt, clockEdge(lat));
        } else {
            /// @todo nominally we should just delete the packet here,
            /// however, until 4-phase stuff we can't because sending
            /// cache is still relying on it
            pendingDelete.push_back(pkt);
        }
    } else {
        // miss

        // @todo: Make someone pay for this
        pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;

        Addr blk_addr = blockAlign(pkt->getAddr());
        MSHR *mshr = mshrQueue.findMatch(blk_addr, pkt->isSecure());

        if (mshr) {
            /// MSHR hit
            /// @note writebacks will be checked in getNextMSHR()
            /// for any conflicting requests to the same block

            //@todo remove hw_pf here
            assert(pkt->req->masterId() < system->maxMasters());
            mshr_hits[pkt->cmdToIndex()][pkt->req->masterId()]++;
            if (mshr->threadNum != 0/*pkt->req->threadId()*/) {
                mshr->threadNum = -1;
            }
            mshr->allocateTarget(pkt, time, order++);
            if (mshr->getNumTargets() == numTarget) {
                noTargetMSHR = mshr;
                setBlocked(Blocked_NoTargets);
                // need to be careful with this... if this mshr isn't
                // ready yet (i.e. time > curTick()_, we don't want to
                // move it ahead of mshrs that are ready
                // mshrQueue.moveToFront(mshr);
            }

            // We should call the prefetcher reguardless if the request is
            // satisfied or not, reguardless if the request is in the MSHR or
            // not.  The request could be a ReadReq hit, but still not
            // satisfied (potentially because of a prior write to the same
            // cache line.  So, even when not satisfied, tehre is an MSHR
            // already allocated for this, we need to let the prefetcher know
            // about the request
            if (prefetcher) {
                next_pf_time = prefetcher->notify(pkt, time);
            }
        } else {
            // no MSHR
            assert(pkt->req->masterId() < system->maxMasters());
            mshr_misses[pkt->cmdToIndex()][pkt->req->masterId()]++;
            // always mark as cache fill for now... if we implement
            // no-write-allocate or bypass accesses this will have to
            // be changed.
            if (pkt->cmd == MemCmd::Writeback) {
                allocateWriteBuffer(pkt, time, true);
            } else {
                if (blk && blk->isValid()) {
                    // If we have a write miss to a valid block, we
                    // need to mark the block non-readable.  Otherwise
                    // if we allow reads while there's an outstanding
                    // write miss, the read could return stale data
                    // out of the cache block... a more aggressive
                    // system could detect the overlap (if any) and
                    // forward data out of the MSHRs, but we don't do
                    // that yet.  Note that we do need to leave the
                    // block valid so that it stays in the cache, in
                    // case we get an upgrade response (and hence no
                    // new data) when the write miss completes.
                    // As long as CPUs do proper store/load forwarding
                    // internally, and have a sufficiently weak memory
                    // model, this is probably unnecessary, but at some
                    // point it must have seemed like we needed it...
                    assert(pkt->needsExclusive() && !blk->isWritable());
                    blk->status &= ~BlkReadable;
                }

                allocateMissBuffer(pkt, time, true);
            }

            if (prefetcher) {
                next_pf_time = prefetcher->notify(pkt, time);
            }
        }
    }

    if (next_pf_time != 0)
        requestMemSideBus(Request_PF, std::max(time, next_pf_time));

    // copy writebacks to write buffer
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
        allocateWriteBuffer(wbPkt, time, true);
        writebacks.pop_front();
    }

    return true;
}


// See comment in cache.hh.
template<class TagStore>
PacketPtr
Cache<TagStore>::getBusPacket(PacketPtr cpu_pkt, BlkType *blk,
                              bool needsExclusive) const
{
    bool blkValid = blk && blk->isValid();

    if (cpu_pkt->req->isUncacheable()) {
        //assert(blk == NULL);
        return NULL;
    }

    if (!blkValid &&
        (cpu_pkt->cmd == MemCmd::Writeback || cpu_pkt->isUpgrade())) {
        // Writebacks that weren't allocated in access() and upgrades
        // from upper-level caches that missed completely just go
        // through.
        return NULL;
    }

    assert(cpu_pkt->needsResponse());

    MemCmd cmd;
    // @TODO make useUpgrades a parameter.
    // Note that ownership protocols require upgrade, otherwise a
    // write miss on a shared owned block will generate a ReadExcl,
    // which will clobber the owned copy.
    const bool useUpgrades = true;
    if (blkValid && useUpgrades) {
        // only reason to be here is that blk is shared
        // (read-only) and we need exclusive
        assert(needsExclusive && !blk->isWritable());
        cmd = cpu_pkt->isLLSC() ? MemCmd::SCUpgradeReq : MemCmd::UpgradeReq;
    } else {
        // block is invalid
        cmd = needsExclusive ? MemCmd::ReadExReq : MemCmd::ReadReq;
    }
    PacketPtr pkt = new Packet(cpu_pkt->req, cmd, blkSize);

    pkt->allocate();
    DPRINTF(Cache, "%s created %s address %x size %d\n",
            __func__, pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    return pkt;
}


template<class TagStore>
Tick
Cache<TagStore>::recvAtomic(PacketPtr pkt)
{
    Cycles lat = hitLatency;

    // @TODO: make this a parameter
    bool last_level_cache = false;

    // Forward the request if the system is in cache bypass mode.
    if (system->bypassCaches())
        return ticksToCycles(memSidePort->sendAtomic(pkt));

    if (pkt->memInhibitAsserted()) {
        assert(!pkt->req->isUncacheable());
        // have to invalidate ourselves and any lower caches even if
        // upper cache will be responding
        if (pkt->isInvalidate()) {
            BlkType *blk = tags->findBlock(pkt->getAddr(), pkt->isSecure());
            if (blk && blk->isValid()) {
                tags->invalidate(blk);
                blk->invalidate();
                DPRINTF(Cache, "rcvd mem-inhibited %s on 0x%x (%s):"
                        " invalidating\n",
                        pkt->cmdString(), pkt->getAddr(),
                        pkt->isSecure() ? "s" : "ns");
            }
            if (!last_level_cache) {
                DPRINTF(Cache, "forwarding mem-inhibited %s on 0x%x (%s)\n",
                        pkt->cmdString(), pkt->getAddr(),
                        pkt->isSecure() ? "s" : "ns");
                lat += ticksToCycles(memSidePort->sendAtomic(pkt));
            }
        } else {
            DPRINTF(Cache, "rcvd mem-inhibited %s on 0x%x: not responding\n",
                    pkt->cmdString(), pkt->getAddr());
        }

        return lat * clockPeriod();
    }

    // should assert here that there are no outstanding MSHRs or
    // writebacks... that would mean that someone used an atomic
    // access in timing mode

    BlkType *blk = NULL;
    PacketList writebacks;

    if (!access(pkt, blk, lat, writebacks)) {
        // MISS
        PacketPtr bus_pkt = getBusPacket(pkt, blk, pkt->needsExclusive());

        bool is_forward = (bus_pkt == NULL);

        if (is_forward) {
            // just forwarding the same request to the next level
            // no local cache operation involved
            bus_pkt = pkt;
        }

        DPRINTF(Cache, "Sending an atomic %s for %x (%s)\n",
                bus_pkt->cmdString(), bus_pkt->getAddr(),
                bus_pkt->isSecure() ? "s" : "ns");

#if TRACING_ON
        CacheBlk::State old_state = blk ? blk->status : 0;
#endif

        lat += ticksToCycles(memSidePort->sendAtomic(bus_pkt));

        DPRINTF(Cache, "Receive response: %s for addr %x (%s) in state %i\n",
                bus_pkt->cmdString(), bus_pkt->getAddr(),
                bus_pkt->isSecure() ? "s" : "ns",
                old_state);

        // If packet was a forward, the response (if any) is already
        // in place in the bus_pkt == pkt structure, so we don't need
        // to do anything.  Otherwise, use the separate bus_pkt to
        // generate response to pkt and then delete it.
        if (!is_forward) {
            if (pkt->needsResponse()) {
                assert(bus_pkt->isResponse());
                if (bus_pkt->isError()) {
                    pkt->makeAtomicResponse();
                    pkt->copyError(bus_pkt);
                } else if (bus_pkt->isRead() ||
                           bus_pkt->cmd == MemCmd::UpgradeResp) {
                    // we're updating cache state to allow us to
                    // satisfy the upstream request from the cache
                    blk = handleFill(bus_pkt, blk, writebacks);
                    satisfyCpuSideRequest(pkt, blk);
                } else {
                    // we're satisfying the upstream request without
                    // modifying cache state, e.g., a write-through
                    pkt->makeAtomicResponse();
                }
            }
            delete bus_pkt;
        }
    }

    // Note that we don't invoke the prefetcher at all in atomic mode.
    // It's not clear how to do it properly, particularly for
    // prefetchers that aggressively generate prefetch candidates and
    // rely on bandwidth contention to throttle them; these will tend
    // to pollute the cache in atomic mode since there is no bandwidth
    // contention.  If we ever do want to enable prefetching in atomic
    // mode, though, this is the place to do it... see timingAccess()
    // for an example (though we'd want to issue the prefetch(es)
    // immediately rather than calling requestMemSideBus() as we do
    // there).

    // Handle writebacks if needed
    while (!writebacks.empty()){
        PacketPtr wbPkt = writebacks.front();
        memSidePort->sendAtomic(wbPkt);
        writebacks.pop_front();
        delete wbPkt;
    }

    // We now have the block one way or another (hit or completed miss)

    if (pkt->needsResponse()) {
        pkt->makeAtomicResponse();
    }

    return lat * clockPeriod();
}


template<class TagStore>
void
Cache<TagStore>::functionalAccess(PacketPtr pkt, bool fromCpuSide)
{
    if (system->bypassCaches()) {
        // Packets from the memory side are snoop request and
        // shouldn't happen in bypass mode.
        assert(fromCpuSide);

        // The cache should be flushed if we are in cache bypass mode,
        // so we don't need to check if we need to update anything.
        memSidePort->sendFunctional(pkt);
        return;
    }

    Addr blk_addr = blockAlign(pkt->getAddr());
    bool is_secure = pkt->isSecure();
    BlkType *blk = tags->findBlock(pkt->getAddr(), is_secure);
    MSHR *mshr = mshrQueue.findMatch(blk_addr, is_secure);

    pkt->pushLabel(name());

    CacheBlkPrintWrapper cbpw(blk);

    // Note that just because an L2/L3 has valid data doesn't mean an
    // L1 doesn't have a more up-to-date modified copy that still
    // needs to be found.  As a result we always update the request if
    // we have it, but only declare it satisfied if we are the owner.

    // see if we have data at all (owned or otherwise)
    bool have_data = blk && blk->isValid()
        && pkt->checkFunctional(&cbpw, blk_addr, is_secure, blkSize,
                                blk->data);

    // data we have is dirty if marked as such or if valid & ownership
    // pending due to outstanding UpgradeReq
    bool have_dirty =
        have_data && (blk->isDirty() ||
                      (mshr && mshr->inService && mshr->isPendingDirty()));

    bool done = have_dirty
        || cpuSidePort->checkFunctional(pkt)
        || mshrQueue.checkFunctional(pkt, blk_addr)
        || writeBuffer.checkFunctional(pkt, blk_addr)
        || memSidePort->checkFunctional(pkt);

    DPRINTF(Cache, "functional %s %x (%s) %s%s%s\n",
            pkt->cmdString(), pkt->getAddr(), is_secure ? "s" : "ns",
            (blk && blk->isValid()) ? "valid " : "",
            have_data ? "data " : "", done ? "done " : "");

    // We're leaving the cache, so pop cache->name() label
    pkt->popLabel();

    if (done) {
        pkt->makeResponse();
    } else {
        // if it came as a request from the CPU side then make sure it
        // continues towards the memory side
        if (fromCpuSide) {
            memSidePort->sendFunctional(pkt);
        } else if (forwardSnoops && cpuSidePort->isSnooping()) {
            // if it came from the memory side, it must be a snoop request
            // and we should only forward it if we are forwarding snoops
            cpuSidePort->sendFunctionalSnoop(pkt);
        }
    }
}


/////////////////////////////////////////////////////
//
// Response handling: responses from the memory side
//
/////////////////////////////////////////////////////


template<class TagStore>
void
Cache<TagStore>::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());

    Tick time = clockEdge(hitLatency);
    MSHR *mshr = dynamic_cast<MSHR*>(pkt->senderState);
    bool is_error = pkt->isError();

    assert(mshr);

    if (is_error) {
        DPRINTF(Cache, "Cache received packet with error for address %x (%s), "
                "cmd: %s\n", pkt->getAddr(), pkt->isSecure() ? "s" : "ns",
                pkt->cmdString());
    }

    DPRINTF(Cache, "Handling response to %s for address %x (%s)\n",
            pkt->cmdString(), pkt->getAddr(), pkt->isSecure() ? "s" : "ns");

    MSHRQueue *mq = mshr->queue;
    bool wasFull = mq->isFull();

    if (mshr == noTargetMSHR) {
        // we always clear at least one target
        clearBlocked(Blocked_NoTargets);
        noTargetMSHR = NULL;
    }

    // Initial target is used just for stats
    MSHR::Target *initial_tgt = mshr->getTarget();
    BlkType *blk = tags->findBlock(pkt->getAddr(), pkt->isSecure());
    int stats_cmd_idx = initial_tgt->pkt->cmdToIndex();
    Tick miss_latency = curTick() - initial_tgt->recvTime;
    PacketList writebacks;

    if (pkt->req->isUncacheable()) {
        assert(pkt->req->masterId() < system->maxMasters());
        mshr_uncacheable_lat[stats_cmd_idx][pkt->req->masterId()] +=
            miss_latency;
    } else {
        assert(pkt->req->masterId() < system->maxMasters());
        mshr_miss_latency[stats_cmd_idx][pkt->req->masterId()] +=
            miss_latency;
    }

    bool is_fill = !mshr->isForward &&
        (pkt->isRead() || pkt->cmd == MemCmd::UpgradeResp);

    if (is_fill && !is_error) {
        DPRINTF(Cache, "Block for addr %x being updated in Cache\n",
                pkt->getAddr());

        // give mshr a chance to do some dirty work
        mshr->handleFill(pkt, blk);

        blk = handleFill(pkt, blk, writebacks);
        assert(blk != NULL);
    }

    // First offset for critical word first calculations
    int initial_offset = 0;

    if (mshr->hasTargets()) {
        initial_offset = mshr->getTarget()->pkt->getOffset(blkSize);
    }

    while (mshr->hasTargets()) {
        MSHR::Target *target = mshr->getTarget();

        switch (target->source) {
          case MSHR::Target::FromCPU:
            Tick completion_time;
            if (is_fill) {
                satisfyCpuSideRequest(target->pkt, blk,
                                      true, mshr->hasPostDowngrade());
                // How many bytes past the first request is this one
                int transfer_offset =
                    target->pkt->getOffset(blkSize) - initial_offset;
                if (transfer_offset < 0) {
                    transfer_offset += blkSize;
                }

                // If critical word (no offset) return first word time.
                // responseLatency is the latency of the return path
                // from lower level caches/memory to an upper level cache or
                // the core.
                completion_time = clockEdge(responseLatency) +
                    (transfer_offset ? pkt->busLastWordDelay :
                     pkt->busFirstWordDelay);

                assert(!target->pkt->req->isUncacheable());

                assert(target->pkt->req->masterId() < system->maxMasters());
                missLatency[target->pkt->cmdToIndex()][target->pkt->req->masterId()] +=
                    completion_time - target->recvTime;
            } else if (pkt->cmd == MemCmd::UpgradeFailResp) {
                // failed StoreCond upgrade
                assert(target->pkt->cmd == MemCmd::StoreCondReq ||
                       target->pkt->cmd == MemCmd::StoreCondFailReq ||
                       target->pkt->cmd == MemCmd::SCUpgradeFailReq);
                // responseLatency is the latency of the return path
                // from lower level caches/memory to an upper level cache or
                // the core.
                completion_time = clockEdge(responseLatency) +
                    pkt->busLastWordDelay;
                target->pkt->req->setExtraData(0);
            } else {
                // not a cache fill, just forwarding response
                // responseLatency is the latency of the return path
                // from lower level cahces/memory to the core.
                completion_time = clockEdge(responseLatency) +
                    pkt->busLastWordDelay;
                if (pkt->isRead() && !is_error) {
                    target->pkt->setData(pkt->getPtr<uint8_t>());
                }
            }
            target->pkt->makeTimingResponse();
            // if this packet is an error copy that to the new packet
            if (is_error)
                target->pkt->copyError(pkt);
            if (target->pkt->cmd == MemCmd::ReadResp &&
                (pkt->isInvalidate() || mshr->hasPostInvalidate())) {
                // If intermediate cache got ReadRespWithInvalidate,
                // propagate that.  Response should not have
                // isInvalidate() set otherwise.
                target->pkt->cmd = MemCmd::ReadRespWithInvalidate;
                DPRINTF(Cache, "%s updated cmd to %s for address %x\n",
                        __func__, target->pkt->cmdString(),
                        target->pkt->getAddr());
            }
            // reset the bus additional time as it is now accounted for
            target->pkt->busFirstWordDelay = target->pkt->busLastWordDelay = 0;
            cpuSidePort->schedTimingResp(target->pkt, completion_time);
            break;

          case MSHR::Target::FromPrefetcher:
            assert(target->pkt->cmd == MemCmd::HardPFReq);
            if (blk)
                blk->status |= BlkHWPrefetched;
            delete target->pkt->req;
            delete target->pkt;
            break;

          case MSHR::Target::FromSnoop:
            // I don't believe that a snoop can be in an error state
            assert(!is_error);
            // response to snoop request
            DPRINTF(Cache, "processing deferred snoop...\n");
            assert(!(pkt->isInvalidate() && !mshr->hasPostInvalidate()));
            handleSnoop(target->pkt, blk, true, true,
                        mshr->hasPostInvalidate());
            break;

          default:
            panic("Illegal target->source enum %d\n", target->source);
        }

        mshr->popTarget();
    }

    if (blk && blk->isValid()) {
        if (pkt->isInvalidate() || mshr->hasPostInvalidate()) {
            assert(blk != tempBlock);
            tags->invalidate(blk);
            blk->invalidate();
        } else if (mshr->hasPostDowngrade()) {
            blk->status &= ~BlkWritable;
        }
    }

    if (mshr->promoteDeferredTargets()) {
        // avoid later read getting stale data while write miss is
        // outstanding.. see comment in timingAccess()
        if (blk) {
            blk->status &= ~BlkReadable;
        }
        mq = mshr->queue;
        mq->markPending(mshr);
        requestMemSideBus((RequestCause)mq->index, clockEdge() +
                          pkt->busLastWordDelay);
    } else {
        mq->deallocate(mshr);
        if (wasFull && !mq->isFull()) {
            clearBlocked((BlockedCause)mq->index);
        }
    }

    // copy writebacks to write buffer
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
        allocateWriteBuffer(wbPkt, time, true);
        writebacks.pop_front();
    }
    // if we used temp block, clear it out
    if (blk == tempBlock) {
        if (blk->isDirty()) {
            allocateWriteBuffer(writebackBlk(blk), time, true);
        }
        blk->invalidate();
    }

    DPRINTF(Cache, "Leaving %s with %s for address %x\n", __func__,
            pkt->cmdString(), pkt->getAddr());
    delete pkt;
}




template<class TagStore>
PacketPtr
Cache<TagStore>::writebackBlk(BlkType *blk)
{
    assert(blk && blk->isValid() && blk->isDirty());

    writebacks[Request::wbMasterId]++;

    Request *writebackReq =
        new Request(tags->regenerateBlkAddr(blk->tag, blk->set), blkSize, 0,
                Request::wbMasterId);
    if (blk->isSecure())
        writebackReq->setFlags(Request::SECURE);

    writebackReq->taskId(blk->task_id);
    blk->task_id= ContextSwitchTaskId::Unknown;
    blk->tickInserted = curTick();

    PacketPtr writeback = new Packet(writebackReq, MemCmd::Writeback);
    if (blk->isWritable()) {
        writeback->setSupplyExclusive();
    }
    writeback->allocate();
    std::memcpy(writeback->getPtr<uint8_t>(), blk->data, blkSize);

    blk->status &= ~BlkDirty;
    return writeback;
}

template<class TagStore>
void
Cache<TagStore>::memWriteback()
{
    WrappedBlkVisitor visitor(*this, &Cache<TagStore>::writebackVisitor);
    tags->forEachBlk(visitor);
}

template<class TagStore>
void
Cache<TagStore>::memInvalidate()
{
    WrappedBlkVisitor visitor(*this, &Cache<TagStore>::invalidateVisitor);
    tags->forEachBlk(visitor);
}

template<class TagStore>
bool
Cache<TagStore>::isDirty() const
{
    CacheBlkIsDirtyVisitor<BlkType> visitor;
    tags->forEachBlk(visitor);

    return visitor.isDirty();
}

template<class TagStore>
bool
Cache<TagStore>::writebackVisitor(BlkType &blk)
{
    if (blk.isDirty()) {
        assert(blk.isValid());

        Request request(tags->regenerateBlkAddr(blk.tag, blk.set),
                        blkSize, 0, Request::funcMasterId);
        request.taskId(blk.task_id);

        Packet packet(&request, MemCmd::WriteReq);
        packet.dataStatic(blk.data);

        memSidePort->sendFunctional(&packet);

        blk.status &= ~BlkDirty;
    }

    return true;
}

template<class TagStore>
bool
Cache<TagStore>::invalidateVisitor(BlkType &blk)
{

    if (blk.isDirty())
        warn_once("Invalidating dirty cache lines. Expect things to break.\n");

    if (blk.isValid()) {
        assert(!blk.isDirty());
        tags->invalidate(dynamic_cast< BlkType *>(&blk));
        blk.invalidate();
    }

    return true;
}

template<class TagStore>
void
Cache<TagStore>::uncacheableFlush(PacketPtr pkt)
{
    DPRINTF(Cache, "%s%s %x uncacheable\n", pkt->cmdString(),
            pkt->req->isInstFetch() ? " (ifetch)" : "",
            pkt->getAddr());

    if (pkt->req->isClearLL())
        tags->clearLocks();

    BlkType *blk(tags->findBlock(pkt->getAddr(), pkt->isSecure()));
    if (blk) {
        writebackVisitor(*blk);
        invalidateVisitor(*blk);
    }
}


template<class TagStore>
typename Cache<TagStore>::BlkType*
Cache<TagStore>::allocateBlock(Addr addr, bool is_secure,
                               PacketList &writebacks)
{
    BlkType *blk = tags->findVictim(addr);

    if (blk->isValid()) {
        Addr repl_addr = tags->regenerateBlkAddr(blk->tag, blk->set);
        MSHR *repl_mshr = mshrQueue.findMatch(repl_addr, blk->isSecure());
        if (repl_mshr) {
            // must be an outstanding upgrade request on block
            // we're about to replace...
            assert(!blk->isWritable());
            assert(repl_mshr->needsExclusive());
            // too hard to replace block with transient state
            // allocation failed, block not inserted
            return NULL;
        } else {
            DPRINTF(Cache, "replacement: replacing %x (%s) with %x (%s): %s\n",
                    repl_addr, blk->isSecure() ? "s" : "ns",
                    addr, is_secure ? "s" : "ns",
                    blk->isDirty() ? "writeback" : "clean");

            if (blk->isDirty()) {
                // Save writeback packet for handling by caller
                writebacks.push_back(writebackBlk(blk));
            }
        }
    }

    return blk;
}


// Note that the reason we return a list of writebacks rather than
// inserting them directly in the write buffer is that this function
// is called by both atomic and timing-mode accesses, and in atomic
// mode we don't mess with the write buffer (we just perform the
// writebacks atomically once the original request is complete).
template<class TagStore>
typename Cache<TagStore>::BlkType*
Cache<TagStore>::handleFill(PacketPtr pkt, BlkType *blk,
                            PacketList &writebacks)
{
    Addr addr = pkt->getAddr();
    bool is_secure = pkt->isSecure();
#if TRACING_ON
    CacheBlk::State old_state = blk ? blk->status : 0;
#endif

    if (blk == NULL) {
        // better have read new data...
        assert(pkt->hasData());
        // need to do a replacement
        blk = allocateBlock(addr, is_secure, writebacks);
        if (blk == NULL) {
            // No replaceable block... just use temporary storage to
            // complete the current request and then get rid of it
            assert(!tempBlock->isValid());
            blk = tempBlock;
            tempBlock->set = tags->extractSet(addr);
            tempBlock->tag = tags->extractTag(addr);
            // @todo: set security state as well...
            DPRINTF(Cache, "using temp block for %x (%s)\n", addr,
                    is_secure ? "s" : "ns");
        } else {
            tags->insertBlock(pkt, blk);
        }

        // we should never be overwriting a valid block
        assert(!blk->isValid());
    } else {
        // existing block... probably an upgrade
        assert(blk->tag == tags->extractTag(addr));
        // either we're getting new data or the block should already be valid
        assert(pkt->hasData() || blk->isValid());
        // don't clear block status... if block is already dirty we
        // don't want to lose that
    }

    if (is_secure)
        blk->status |= BlkSecure;
    blk->status |= BlkValid | BlkReadable;

    if (!pkt->sharedAsserted()) {
        blk->status |= BlkWritable;
        // If we got this via cache-to-cache transfer (i.e., from a
        // cache that was an owner) and took away that owner's copy,
        // then we need to write it back.  Normally this happens
        // anyway as a side effect of getting a copy to write it, but
        // there are cases (such as failed store conditionals or
        // compare-and-swaps) where we'll demand an exclusive copy but
        // end up not writing it.
        if (pkt->memInhibitAsserted())
            blk->status |= BlkDirty;
    }

    DPRINTF(Cache, "Block addr %x (%s) moving from state %x to %s\n",
            addr, is_secure ? "s" : "ns", old_state, blk->print());

    // if we got new data, copy it in
    if (pkt->isRead()) {
        std::memcpy(blk->data, pkt->getPtr<uint8_t>(), blkSize);
    }

    blk->whenReady = clockEdge() + responseLatency * clockPeriod() +
        pkt->busLastWordDelay;

    return blk;
}


/////////////////////////////////////////////////////
//
// Snoop path: requests coming in from the memory side
//
/////////////////////////////////////////////////////

template<class TagStore>
void
Cache<TagStore>::
doTimingSupplyResponse(PacketPtr req_pkt, uint8_t *blk_data,
                       bool already_copied, bool pending_inval)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            req_pkt->cmdString(), req_pkt->getAddr(), req_pkt->getSize());
    // timing-mode snoop responses require a new packet, unless we
    // already made a copy...
    PacketPtr pkt = already_copied ? req_pkt : new Packet(req_pkt);
    assert(req_pkt->isInvalidate() || pkt->sharedAsserted());
    pkt->allocate();
    pkt->makeTimingResponse();
    // @todo Make someone pay for this
    pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;
    if (pkt->isRead()) {
        pkt->setDataFromBlock(blk_data, blkSize);
    }
    if (pkt->cmd == MemCmd::ReadResp && pending_inval) {
        // Assume we defer a response to a read from a far-away cache
        // A, then later defer a ReadExcl from a cache B on the same
        // bus as us.  We'll assert MemInhibit in both cases, but in
        // the latter case MemInhibit will keep the invalidation from
        // reaching cache A.  This special response tells cache A that
        // it gets the block to satisfy its read, but must immediately
        // invalidate it.
        pkt->cmd = MemCmd::ReadRespWithInvalidate;
    }
    DPRINTF(Cache, "%s created response: %s address %x size %d\n",
            __func__, pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    memSidePort->schedTimingSnoopResp(pkt, clockEdge(hitLatency));
}

template<class TagStore>
void
Cache<TagStore>::handleSnoop(PacketPtr pkt, BlkType *blk,
                             bool is_timing, bool is_deferred,
                             bool pending_inval)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    // deferred snoops can only happen in timing mode
    assert(!(is_deferred && !is_timing));
    // pending_inval only makes sense on deferred snoops
    assert(!(pending_inval && !is_deferred));
    assert(pkt->isRequest());

    // the packet may get modified if we or a forwarded snooper
    // responds in atomic mode, so remember a few things about the
    // original packet up front
    bool invalidate = pkt->isInvalidate();
    bool M5_VAR_USED needs_exclusive = pkt->needsExclusive();

    if (forwardSnoops) {
        // first propagate snoop upward to see if anyone above us wants to
        // handle it.  save & restore packet src since it will get
        // rewritten to be relative to cpu-side bus (if any)
        bool alreadyResponded = pkt->memInhibitAsserted();
        if (is_timing) {
            Packet snoopPkt(pkt, true);  // clear flags
            snoopPkt.setExpressSnoop();
            snoopPkt.pushSenderState(new ForwardResponseRecord(pkt->getSrc()));
            // the snoop packet does not need to wait any additional
            // time
            snoopPkt.busFirstWordDelay = snoopPkt.busLastWordDelay = 0;
            cpuSidePort->sendTimingSnoopReq(&snoopPkt);
            if (snoopPkt.memInhibitAsserted()) {
                // cache-to-cache response from some upper cache
                assert(!alreadyResponded);
                pkt->assertMemInhibit();
            } else {
                delete snoopPkt.popSenderState();
            }
            if (snoopPkt.sharedAsserted()) {
                pkt->assertShared();
            }
        } else {
            cpuSidePort->sendAtomicSnoop(pkt);
            if (!alreadyResponded && pkt->memInhibitAsserted()) {
                // cache-to-cache response from some upper cache:
                // forward response to original requester
                assert(pkt->isResponse());
            }
        }
    }

     if (!blk || !blk->isValid()) {
         DPRINTF(Cache, "%s snoop miss for %s address %x size %d\n",
                 __func__, pkt->cmdString(), pkt->getAddr(), pkt->getSize());
         return;
     } else {
        DPRINTF(Cache, "%s snoop hit for %s for address %x size %d, "
                "old state is %s\n", __func__, pkt->cmdString(),
                pkt->getAddr(), pkt->getSize(), blk->print());
     }

    // we may end up modifying both the block state and the packet (if
    // we respond in atomic mode), so just figure out what to do now
    // and then do it later
    bool respond = blk->isDirty() && pkt->needsResponse();
    bool have_exclusive = blk->isWritable();

    if (pkt->isRead() && !invalidate) {
        assert(!needs_exclusive);
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

    if (respond) {
        assert(!pkt->memInhibitAsserted());
        pkt->assertMemInhibit();
        if (have_exclusive) {
            pkt->setSupplyExclusive();
        }
        if (is_timing) {
            doTimingSupplyResponse(pkt, blk->data, is_deferred, pending_inval);
        } else {
            pkt->makeAtomicResponse();
            pkt->setDataFromBlock(blk->data, blkSize);
        }
    } else if (is_timing && is_deferred) {
        // if it's a deferred timing snoop then we've made a copy of
        // the packet, and so if we're not using that copy to respond
        // then we need to delete it here.
        delete pkt;
    }

    // Do this last in case it deallocates block data or something
    // like that
    if (invalidate) {
        if (blk != tempBlock)
            tags->invalidate(blk);
        blk->invalidate();
    }

    DPRINTF(Cache, "new state is %s\n", blk->print());
}


template<class TagStore>
void
Cache<TagStore>::recvTimingSnoopReq(PacketPtr pkt)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    // Snoops shouldn't happen when bypassing caches
    assert(!system->bypassCaches());

    // check if the packet is for an address range covered by this
    // cache, partly to not waste time looking for it, but also to
    // ensure that we only forward the snoop upwards if it is within
    // our address ranges
    bool in_range = false;
    for (AddrRangeList::const_iterator r = addrRanges.begin();
         r != addrRanges.end(); ++r) {
        if (r->contains(pkt->getAddr())) {
            in_range = true;
            break;
        }
    }

    // Note that some deferred snoops don't have requests, since the
    // original access may have already completed
    if ((pkt->req && pkt->req->isUncacheable()) ||
        pkt->cmd == MemCmd::Writeback || !in_range) {
        //Can't get a hit on an uncacheable address
        //Revisit this for multi level coherence
        return;
    }

    bool is_secure = pkt->isSecure();
    BlkType *blk = tags->findBlock(pkt->getAddr(), is_secure);

    Addr blk_addr = blockAlign(pkt->getAddr());
    MSHR *mshr = mshrQueue.findMatch(blk_addr, is_secure);

    // Let the MSHR itself track the snoop and decide whether we want
    // to go ahead and do the regular cache snoop
    if (mshr && mshr->handleSnoop(pkt, order++)) {
        DPRINTF(Cache, "Deferring snoop on in-service MSHR to blk %x (%s)."
                "mshrs: %s\n", blk_addr, is_secure ? "s" : "ns",
                mshr->print());

        if (mshr->getNumTargets() > numTarget)
            warn("allocating bonus target for snoop"); //handle later
        return;
    }

    //We also need to check the writeback buffers and handle those
    std::vector<MSHR *> writebacks;
    if (writeBuffer.findMatches(blk_addr, is_secure, writebacks)) {
        DPRINTF(Cache, "Snoop hit in writeback to addr: %x (%s)\n",
                pkt->getAddr(), is_secure ? "s" : "ns");

        //Look through writebacks for any non-uncachable writes, use that
        if (writebacks.size()) {
            // We should only ever find a single match
            assert(writebacks.size() == 1);
            mshr = writebacks[0];
            assert(!mshr->isUncacheable());
            assert(mshr->getNumTargets() == 1);
            PacketPtr wb_pkt = mshr->getTarget()->pkt;
            assert(wb_pkt->cmd == MemCmd::Writeback);

            assert(!pkt->memInhibitAsserted());
            pkt->assertMemInhibit();
            if (!pkt->needsExclusive()) {
                pkt->assertShared();
                // the writeback is no longer the exclusive copy in the system
                wb_pkt->clearSupplyExclusive();
            } else {
                // if we're not asserting the shared line, we need to
                // invalidate our copy.  we'll do that below as long as
                // the packet's invalidate flag is set...
                assert(pkt->isInvalidate());
            }
            doTimingSupplyResponse(pkt, wb_pkt->getPtr<uint8_t>(),
                                   false, false);

            if (pkt->isInvalidate()) {
                // Invalidation trumps our writeback... discard here
                markInService(mshr);
                delete wb_pkt;
            }
        } // writebacks.size()
    }

    // If this was a shared writeback, there may still be
    // other shared copies above that require invalidation.
    // We could be more selective and return here if the
    // request is non-exclusive or if the writeback is
    // exclusive.
    handleSnoop(pkt, blk, true, false, false);
}

template<class TagStore>
bool
Cache<TagStore>::CpuSidePort::recvTimingSnoopResp(PacketPtr pkt)
{
    // Express snoop responses from master to slave, e.g., from L1 to L2
    cache->recvTimingSnoopResp(pkt);
    return true;
}

template<class TagStore>
Tick
Cache<TagStore>::recvAtomicSnoop(PacketPtr pkt)
{
    // Snoops shouldn't happen when bypassing caches
    assert(!system->bypassCaches());

    if (pkt->req->isUncacheable() || pkt->cmd == MemCmd::Writeback) {
        // Can't get a hit on an uncacheable address
        // Revisit this for multi level coherence
        return 0;
    }

    BlkType *blk = tags->findBlock(pkt->getAddr(), pkt->isSecure());
    handleSnoop(pkt, blk, false, false, false);
    return hitLatency * clockPeriod();
}


template<class TagStore>
MSHR *
Cache<TagStore>::getNextMSHR()
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
                mshrQueue.findPending(write_mshr->addr, write_mshr->size,
                                      write_mshr->isSecure);

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
            writeBuffer.findPending(miss_mshr->addr, miss_mshr->size,
                                    miss_mshr->isSecure);
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

        // No conflicts; issue read
        return miss_mshr;
    }

    // fall through... no pending requests.  Try a prefetch.
    assert(!miss_mshr && !write_mshr);
    if (prefetcher && !mshrQueue.isFull()) {
        // If we have a miss queue slot, we can try a prefetch
        PacketPtr pkt = prefetcher->getPacket();
        if (pkt) {
            Addr pf_addr = blockAlign(pkt->getAddr());
            if (!tags->findBlock(pf_addr, pkt->isSecure()) &&
                !mshrQueue.findMatch(pf_addr, pkt->isSecure()) &&
                !writeBuffer.findMatch(pf_addr, pkt->isSecure())) {
                // Update statistic on number of prefetches issued
                // (hwpf_mshr_misses)
                assert(pkt->req->masterId() < system->maxMasters());
                mshr_misses[pkt->cmdToIndex()][pkt->req->masterId()]++;
                // Don't request bus, since we already have it
                return allocateMissBuffer(pkt, curTick(), false);
            } else {
                // free the request and packet
                delete pkt->req;
                delete pkt;
            }
        }
    }

    return NULL;
}


template<class TagStore>
PacketPtr
Cache<TagStore>::getTimingPacket()
{
    MSHR *mshr = getNextMSHR();

    if (mshr == NULL) {
        return NULL;
    }

    // use request from 1st target
    PacketPtr tgt_pkt = mshr->getTarget()->pkt;
    PacketPtr pkt = NULL;

    DPRINTF(CachePort, "%s %s for address %x size %d\n", __func__,
            tgt_pkt->cmdString(), tgt_pkt->getAddr(), tgt_pkt->getSize());

    if (tgt_pkt->cmd == MemCmd::SCUpgradeFailReq ||
        tgt_pkt->cmd == MemCmd::StoreCondFailReq) {
        // SCUpgradeReq or StoreCondReq saw invalidation while queued
        // in MSHR, so now that we are getting around to processing
        // it, just treat it as if we got a failure response
        pkt = new Packet(tgt_pkt);
        pkt->cmd = MemCmd::UpgradeFailResp;
        pkt->senderState = mshr;
        pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;
        recvTimingResp(pkt);
        return NULL;
    } else if (mshr->isForwardNoResponse()) {
        // no response expected, just forward packet as it is
        assert(tags->findBlock(mshr->addr, mshr->isSecure) == NULL);
        pkt = tgt_pkt;
    } else {
        BlkType *blk = tags->findBlock(mshr->addr, mshr->isSecure);

        if (tgt_pkt->cmd == MemCmd::HardPFReq) {
            // It might be possible for a writeback to arrive between
            // the time the prefetch is placed in the MSHRs and when
            // it's selected to send... if so, this assert will catch
            // that, and then we'll have to figure out what to do.
            assert(blk == NULL);

            // We need to check the caches above us to verify that
            // they don't have a copy of this block in the dirty state
            // at the moment. Without this check we could get a stale
            // copy from memory that might get used in place of the
            // dirty one.
            Packet snoop_pkt(tgt_pkt, true);
            snoop_pkt.setExpressSnoop();
            snoop_pkt.senderState = mshr;
            cpuSidePort->sendTimingSnoopReq(&snoop_pkt);

            if (snoop_pkt.memInhibitAsserted()) {
                markInService(mshr, &snoop_pkt);
                DPRINTF(Cache, "Upward snoop of prefetch for addr"
                        " %#x (%s) hit\n",
                        tgt_pkt->getAddr(), tgt_pkt->isSecure()? "s": "ns");
                return NULL;
            }
        }

        pkt = getBusPacket(tgt_pkt, blk, mshr->needsExclusive());

        mshr->isForward = (pkt == NULL);

        if (mshr->isForward) {
            // not a cache block request, but a response is expected
            // make copy of current packet to forward, keep current
            // copy for response handling
            pkt = new Packet(tgt_pkt);
            pkt->allocate();
            if (pkt->isWrite()) {
                pkt->setData(tgt_pkt->getPtr<uint8_t>());
            }
        }
    }

    assert(pkt != NULL);
    pkt->senderState = mshr;
    return pkt;
}


template<class TagStore>
Tick
Cache<TagStore>::nextMSHRReadyTime() const
{
    Tick nextReady = std::min(mshrQueue.nextMSHRReadyTime(),
                              writeBuffer.nextMSHRReadyTime());

    if (prefetcher) {
        nextReady = std::min(nextReady,
                             prefetcher->nextPrefetchReadyTime());
    }

    return nextReady;
}

template<class TagStore>
void
Cache<TagStore>::serialize(std::ostream &os)
{
    bool dirty(isDirty());

    if (dirty) {
        warn("*** The cache still contains dirty data. ***\n");
        warn("    Make sure to drain the system using the correct flags.\n");
        warn("    This checkpoint will not restore correctly and dirty data in "
             "the cache will be lost!\n");
    }

    // Since we don't checkpoint the data in the cache, any dirty data
    // will be lost when restoring from a checkpoint of a system that
    // wasn't drained properly. Flag the checkpoint as invalid if the
    // cache contains dirty data.
    bool bad_checkpoint(dirty);
    SERIALIZE_SCALAR(bad_checkpoint);
}

template<class TagStore>
void
Cache<TagStore>::unserialize(Checkpoint *cp, const std::string &section)
{
    bool bad_checkpoint;
    UNSERIALIZE_SCALAR(bad_checkpoint);
    if (bad_checkpoint) {
        fatal("Restoring from checkpoints with dirty caches is not supported "
              "in the classic memory system. Please remove any caches or "
              " drain them properly before taking checkpoints.\n");
    }
}

///////////////
//
// CpuSidePort
//
///////////////

template<class TagStore>
AddrRangeList
Cache<TagStore>::CpuSidePort::getAddrRanges() const
{
    return cache->getAddrRanges();
}

template<class TagStore>
bool
Cache<TagStore>::CpuSidePort::recvTimingReq(PacketPtr pkt)
{
    // always let inhibited requests through even if blocked
    if (!pkt->memInhibitAsserted() && blocked) {
        assert(!cache->system->bypassCaches());
        DPRINTF(Cache,"Scheduling a retry while blocked\n");
        mustSendRetry = true;
        return false;
    }

    cache->recvTimingReq(pkt);
    return true;
}

template<class TagStore>
Tick
Cache<TagStore>::CpuSidePort::recvAtomic(PacketPtr pkt)
{
    return cache->recvAtomic(pkt);
}

template<class TagStore>
void
Cache<TagStore>::CpuSidePort::recvFunctional(PacketPtr pkt)
{
    // functional request
    cache->functionalAccess(pkt, true);
}

template<class TagStore>
Cache<TagStore>::
CpuSidePort::CpuSidePort(const std::string &_name, Cache<TagStore> *_cache,
                         const std::string &_label)
    : BaseCache::CacheSlavePort(_name, _cache, _label), cache(_cache)
{
}

///////////////
//
// MemSidePort
//
///////////////

template<class TagStore>
bool
Cache<TagStore>::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    cache->recvTimingResp(pkt);
    return true;
}

// Express snooping requests to memside port
template<class TagStore>
void
Cache<TagStore>::MemSidePort::recvTimingSnoopReq(PacketPtr pkt)
{
    // handle snooping requests
    cache->recvTimingSnoopReq(pkt);
}

template<class TagStore>
Tick
Cache<TagStore>::MemSidePort::recvAtomicSnoop(PacketPtr pkt)
{
    return cache->recvAtomicSnoop(pkt);
}

template<class TagStore>
void
Cache<TagStore>::MemSidePort::recvFunctionalSnoop(PacketPtr pkt)
{
    // functional snoop (note that in contrast to atomic we don't have
    // a specific functionalSnoop method, as they have the same
    // behaviour regardless)
    cache->functionalAccess(pkt, false);
}

template<class TagStore>
void
Cache<TagStore>::MemSidePacketQueue::sendDeferredPacket()
{
    // if we have a response packet waiting we have to start with that
    if (deferredPacketReady()) {
        // use the normal approach from the timing port
        trySendTiming();
    } else {
        // check for request packets (requests & writebacks)
        PacketPtr pkt = cache.getTimingPacket();
        if (pkt == NULL) {
            // can happen if e.g. we attempt a writeback and fail, but
            // before the retry, the writeback is eliminated because
            // we snoop another cache's ReadEx.
            waitingOnRetry = false;
        } else {
            MSHR *mshr = dynamic_cast<MSHR*>(pkt->senderState);

            waitingOnRetry = !masterPort.sendTimingReq(pkt);

            if (waitingOnRetry) {
                DPRINTF(CachePort, "now waiting on a retry\n");
                if (!mshr->isForwardNoResponse()) {
                    // we are awaiting a retry, but we
                    // delete the packet and will be creating a new packet
                    // when we get the opportunity
                    delete pkt;
                }
                // note that we have now masked any requestBus and
                // schedSendEvent (we will wait for a retry before
                // doing anything), and this is so even if we do not
                // care about this packet and might override it before
                // it gets retried
            } else {
                cache.markInService(mshr, pkt);
            }
        }
    }

    // if we succeeded and are not waiting for a retry, schedule the
    // next send, not only looking at the response transmit list, but
    // also considering when the next MSHR is ready
    if (!waitingOnRetry) {
        scheduleSend(cache.nextMSHRReadyTime());
    }
}

template<class TagStore>
Cache<TagStore>::
MemSidePort::MemSidePort(const std::string &_name, Cache<TagStore> *_cache,
                         const std::string &_label)
    : BaseCache::CacheMasterPort(_name, _cache, _queue),
      _queue(*_cache, *this, _label), cache(_cache)
{
}

#endif//__MEM_CACHE_CACHE_IMPL_HH__
