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
 *
 * Authors: Erik Hallnor
 *          Dave Greene
 *          Nathan Binkert
 *          Steve Reinhardt
 *          Ron Dreslinski
 *          Andreas Sandberg
 *          Nikos Nikoleris
 */

/**
 * @file
 * Cache definitions.
 */

#include "mem/cache/cache.hh"

#include "base/logging.hh"
#include "base/types.hh"
#include "debug/Cache.hh"
#include "debug/CachePort.hh"
#include "debug/CacheTags.hh"
#include "debug/CacheVerbose.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/prefetch/base.hh"
#include "sim/sim_exit.hh"

Cache::Cache(const CacheParams *p)
    : BaseCache(p, p->system->cacheLineSize()),
      tags(p->tags),
      prefetcher(p->prefetcher),
      doFastWrites(true),
      prefetchOnAccess(p->prefetch_on_access),
      clusivity(p->clusivity),
      writebackClean(p->writeback_clean),
      tempBlockWriteback(nullptr),
      writebackTempBlockAtomicEvent([this]{ writebackTempBlockAtomic(); },
                                    name(), false,
                                    EventBase::Delayed_Writeback_Pri)
{
    tempBlock = new CacheBlk();
    tempBlock->data = new uint8_t[blkSize];

    cpuSidePort = new CpuSidePort(p->name + ".cpu_side", this,
                                  "CpuSidePort");
    memSidePort = new MemSidePort(p->name + ".mem_side", this,
                                  "MemSidePort");

    tags->setCache(this);
    if (prefetcher)
        prefetcher->setCache(this);
}

Cache::~Cache()
{
    delete [] tempBlock->data;
    delete tempBlock;

    delete cpuSidePort;
    delete memSidePort;
}

void
Cache::regStats()
{
    BaseCache::regStats();
}

void
Cache::cmpAndSwap(CacheBlk *blk, PacketPtr pkt)
{
    assert(pkt->isRequest());

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


void
Cache::satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                      bool deferred_response, bool pending_downgrade)
{
    assert(pkt->isRequest());

    assert(blk && blk->isValid());
    // Occasionally this is not true... if we are a lower-level cache
    // satisfying a string of Read and ReadEx requests from
    // upper-level caches, a Read will mark the block as shared but we
    // can satisfy a following ReadEx anyway since we can rely on the
    // Read requester(s) to have buffered the ReadEx snoop and to
    // invalidate their blocks after receiving them.
    // assert(!pkt->needsWritable() || blk->isWritable());
    assert(pkt->getOffset(blkSize) + pkt->getSize() <= blkSize);

    // Check RMW operations first since both isRead() and
    // isWrite() will be true for them
    if (pkt->cmd == MemCmd::SwapReq) {
        cmpAndSwap(blk, pkt);
    } else if (pkt->isWrite()) {
        // we have the block in a writable state and can go ahead,
        // note that the line may be also be considered writable in
        // downstream caches along the path to memory, but always
        // Exclusive, and never Modified
        assert(blk->isWritable());
        // Write or WriteLine at the first cache with block in writable state
        if (blk->checkWrite(pkt)) {
            pkt->writeDataToBlock(blk->data, blkSize);
        }
        // Always mark the line as dirty (and thus transition to the
        // Modified state) even if we are a failed StoreCond so we
        // supply data to any snoops that have appended themselves to
        // this cache before knowing the store will fail.
        blk->status |= BlkDirty;
        DPRINTF(CacheVerbose, "%s for %s (write)\n", __func__, pkt->print());
    } else if (pkt->isRead()) {
        if (pkt->isLLSC()) {
            blk->trackLoadLocked(pkt);
        }

        // all read responses have a data payload
        assert(pkt->hasRespData());
        pkt->setDataFromBlock(blk->data, blkSize);

        // determine if this read is from a (coherent) cache or not
        if (pkt->fromCache()) {
            assert(pkt->getSize() == blkSize);
            // special handling for coherent block requests from
            // upper-level caches
            if (pkt->needsWritable()) {
                // sanity check
                assert(pkt->cmd == MemCmd::ReadExReq ||
                       pkt->cmd == MemCmd::SCUpgradeFailReq);
                assert(!pkt->hasSharers());

                // if we have a dirty copy, make sure the recipient
                // keeps it marked dirty (in the modified state)
                if (blk->isDirty()) {
                    pkt->setCacheResponding();
                    blk->status &= ~BlkDirty;
                }
            } else if (blk->isWritable() && !pending_downgrade &&
                       !pkt->hasSharers() &&
                       pkt->cmd != MemCmd::ReadCleanReq) {
                // we can give the requester a writable copy on a read
                // request if:
                // - we have a writable copy at this level (& below)
                // - we don't have a pending snoop from below
                //   signaling another read request
                // - no other cache above has a copy (otherwise it
                //   would have set hasSharers flag when
                //   snooping the packet)
                // - the read has explicitly asked for a clean
                //   copy of the line
                if (blk->isDirty()) {
                    // special considerations if we're owner:
                    if (!deferred_response) {
                        // respond with the line in Modified state
                        // (cacheResponding set, hasSharers not set)
                        pkt->setCacheResponding();

                        // if this cache is mostly inclusive, we
                        // keep the block in the Exclusive state,
                        // and pass it upwards as Modified
                        // (writable and dirty), hence we have
                        // multiple caches, all on the same path
                        // towards memory, all considering the
                        // same block writable, but only one
                        // considering it Modified

                        // we get away with multiple caches (on
                        // the same path to memory) considering
                        // the block writeable as we always enter
                        // the cache hierarchy through a cache,
                        // and first snoop upwards in all other
                        // branches
                        blk->status &= ~BlkDirty;
                    } else {
                        // if we're responding after our own miss,
                        // there's a window where the recipient didn't
                        // know it was getting ownership and may not
                        // have responded to snoops correctly, so we
                        // have to respond with a shared line
                        pkt->setHasSharers();
                    }
                }
            } else {
                // otherwise only respond with a shared copy
                pkt->setHasSharers();
            }
        }
    } else if (pkt->isUpgrade()) {
        // sanity check
        assert(!pkt->hasSharers());

        if (blk->isDirty()) {
            // we were in the Owned state, and a cache above us that
            // has the line in Shared state needs to be made aware
            // that the data it already has is in fact dirty
            pkt->setCacheResponding();
            blk->status &= ~BlkDirty;
        }
    } else {
        assert(pkt->isInvalidate());
        invalidateBlock(blk);
        DPRINTF(CacheVerbose, "%s for %s (invalidation)\n", __func__,
                pkt->print());
    }
}

/////////////////////////////////////////////////////
//
// Access path: requests coming in from the CPU side
//
/////////////////////////////////////////////////////

bool
Cache::access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
              PacketList &writebacks)
{
    // sanity check
    assert(pkt->isRequest());

    chatty_assert(!(isReadOnly && pkt->isWrite()),
                  "Should never see a write in a read-only cache %s\n",
                  name());

    DPRINTF(CacheVerbose, "%s for %s\n", __func__, pkt->print());

    if (pkt->req->isUncacheable()) {
        DPRINTF(Cache, "uncacheable: %s\n", pkt->print());

        // flush and invalidate any existing block
        CacheBlk *old_blk(tags->findBlock(pkt->getAddr(), pkt->isSecure()));
        if (old_blk && old_blk->isValid()) {
            if (old_blk->isDirty() || writebackClean)
                writebacks.push_back(writebackBlk(old_blk));
            else
                writebacks.push_back(cleanEvictBlk(old_blk));
            invalidateBlock(old_blk);
        }

        blk = nullptr;
        // lookupLatency is the latency in case the request is uncacheable.
        lat = lookupLatency;
        return false;
    }

    // Here lat is the value passed as parameter to accessBlock() function
    // that can modify its value.
    blk = tags->accessBlock(pkt->getAddr(), pkt->isSecure(), lat);

    DPRINTF(Cache, "%s %s\n", pkt->print(),
            blk ? "hit " + blk->print() : "miss");

    if (pkt->req->isCacheMaintenance()) {
        // A cache maintenance operation is always forwarded to the
        // memory below even if the block is found in dirty state.

        // We defer any changes to the state of the block until we
        // create and mark as in service the mshr for the downstream
        // packet.
        return false;
    }

    if (pkt->isEviction()) {
        // We check for presence of block in above caches before issuing
        // Writeback or CleanEvict to write buffer. Therefore the only
        // possible cases can be of a CleanEvict packet coming from above
        // encountering a Writeback generated in this cache peer cache and
        // waiting in the write buffer. Cases of upper level peer caches
        // generating CleanEvict and Writeback or simply CleanEvict and
        // CleanEvict almost simultaneously will be caught by snoops sent out
        // by crossbar.
        WriteQueueEntry *wb_entry = writeBuffer.findMatch(pkt->getAddr(),
                                                          pkt->isSecure());
        if (wb_entry) {
            assert(wb_entry->getNumTargets() == 1);
            PacketPtr wbPkt = wb_entry->getTarget()->pkt;
            assert(wbPkt->isWriteback());

            if (pkt->isCleanEviction()) {
                // The CleanEvict and WritebackClean snoops into other
                // peer caches of the same level while traversing the
                // crossbar. If a copy of the block is found, the
                // packet is deleted in the crossbar. Hence, none of
                // the other upper level caches connected to this
                // cache have the block, so we can clear the
                // BLOCK_CACHED flag in the Writeback if set and
                // discard the CleanEvict by returning true.
                wbPkt->clearBlockCached();
                return true;
            } else {
                assert(pkt->cmd == MemCmd::WritebackDirty);
                // Dirty writeback from above trumps our clean
                // writeback... discard here
                // Note: markInService will remove entry from writeback buffer.
                markInService(wb_entry);
                delete wbPkt;
            }
        }
    }

    // Writeback handling is special case.  We can write the block into
    // the cache without having a writeable copy (or any copy at all).
    if (pkt->isWriteback()) {
        assert(blkSize == pkt->getSize());

        // we could get a clean writeback while we are having
        // outstanding accesses to a block, do the simple thing for
        // now and drop the clean writeback so that we do not upset
        // any ordering/decisions about ownership already taken
        if (pkt->cmd == MemCmd::WritebackClean &&
            mshrQueue.findMatch(pkt->getAddr(), pkt->isSecure())) {
            DPRINTF(Cache, "Clean writeback %#llx to block with MSHR, "
                    "dropping\n", pkt->getAddr());
            return true;
        }

        if (blk == nullptr) {
            // need to do a replacement
            blk = allocateBlock(pkt->getAddr(), pkt->isSecure(), writebacks);
            if (blk == nullptr) {
                // no replaceable block available: give up, fwd to next level.
                incMissCount(pkt);
                return false;
            }
            tags->insertBlock(pkt, blk);

            blk->status = (BlkValid | BlkReadable);
            if (pkt->isSecure()) {
                blk->status |= BlkSecure;
            }
        }
        // only mark the block dirty if we got a writeback command,
        // and leave it as is for a clean writeback
        if (pkt->cmd == MemCmd::WritebackDirty) {
            assert(!blk->isDirty());
            blk->status |= BlkDirty;
        }
        // if the packet does not have sharers, it is passing
        // writable, and we got the writeback in Modified or Exclusive
        // state, if not we are in the Owned or Shared state
        if (!pkt->hasSharers()) {
            blk->status |= BlkWritable;
        }
        // nothing else to do; writeback doesn't expect response
        assert(!pkt->needsResponse());
        std::memcpy(blk->data, pkt->getConstPtr<uint8_t>(), blkSize);
        DPRINTF(Cache, "%s new state is %s\n", __func__, blk->print());
        incHitCount(pkt);
        // populate the time when the block will be ready to access.
        blk->whenReady = clockEdge(fillLatency) + pkt->headerDelay +
            pkt->payloadDelay;
        return true;
    } else if (pkt->cmd == MemCmd::CleanEvict) {
        if (blk != nullptr) {
            // Found the block in the tags, need to stop CleanEvict from
            // propagating further down the hierarchy. Returning true will
            // treat the CleanEvict like a satisfied write request and delete
            // it.
            return true;
        }
        // We didn't find the block here, propagate the CleanEvict further
        // down the memory hierarchy. Returning false will treat the CleanEvict
        // like a Writeback which could not find a replaceable block so has to
        // go to next level.
        return false;
    } else if (pkt->cmd == MemCmd::WriteClean) {
        // WriteClean handling is a special case. We can allocate a
        // block directly if it doesn't exist and we can update the
        // block immediately. The WriteClean transfers the ownership
        // of the block as well.
        assert(blkSize == pkt->getSize());

        if (!blk) {
            if (pkt->writeThrough()) {
                // if this is a write through packet, we don't try to
                // allocate if the block is not present
                return false;
            } else {
                // a writeback that misses needs to allocate a new block
                blk = allocateBlock(pkt->getAddr(), pkt->isSecure(),
                                    writebacks);
                if (!blk) {
                    // no replaceable block available: give up, fwd to
                    // next level.
                    incMissCount(pkt);
                    return false;
                }
                tags->insertBlock(pkt, blk);

                blk->status = (BlkValid | BlkReadable);
                if (pkt->isSecure()) {
                    blk->status |= BlkSecure;
                }
            }
        }

        // at this point either this is a writeback or a write-through
        // write clean operation and the block is already in this
        // cache, we need to update the data and the block flags
        assert(blk);
        assert(!blk->isDirty());
        if (!pkt->writeThrough()) {
            blk->status |= BlkDirty;
        }
        // nothing else to do; writeback doesn't expect response
        assert(!pkt->needsResponse());
        std::memcpy(blk->data, pkt->getConstPtr<uint8_t>(), blkSize);
        DPRINTF(Cache, "%s new state is %s\n", __func__, blk->print());

        incHitCount(pkt);
        // populate the time when the block will be ready to access.
        blk->whenReady = clockEdge(fillLatency) + pkt->headerDelay +
            pkt->payloadDelay;
        // if this a write-through packet it will be sent to cache
        // below
        return !pkt->writeThrough();
    } else if (blk && (pkt->needsWritable() ? blk->isWritable() :
                       blk->isReadable())) {
        // OK to satisfy access
        incHitCount(pkt);
        satisfyRequest(pkt, blk);
        maintainClusivity(pkt->fromCache(), blk);

        return true;
    }

    // Can't satisfy access normally... either no block (blk == nullptr)
    // or have block but need writable

    incMissCount(pkt);

    if (blk == nullptr && pkt->isLLSC() && pkt->isWrite()) {
        // complete miss on store conditional... just give up now
        pkt->req->setExtraData(0);
        return true;
    }

    return false;
}

void
Cache::maintainClusivity(bool from_cache, CacheBlk *blk)
{
    if (from_cache && blk && blk->isValid() && !blk->isDirty() &&
        clusivity == Enums::mostly_excl) {
        // if we have responded to a cache, and our block is still
        // valid, but not dirty, and this cache is mostly exclusive
        // with respect to the cache above, drop the block
        invalidateBlock(blk);
    }
}

void
Cache::doWritebacks(PacketList& writebacks, Tick forward_time)
{
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
        // We use forwardLatency here because we are copying writebacks to
        // write buffer.

        // Call isCachedAbove for Writebacks, CleanEvicts and
        // WriteCleans to discover if the block is cached above.
        if (isCachedAbove(wbPkt)) {
            if (wbPkt->cmd == MemCmd::CleanEvict) {
                // Delete CleanEvict because cached copies exist above. The
                // packet destructor will delete the request object because
                // this is a non-snoop request packet which does not require a
                // response.
                delete wbPkt;
            } else if (wbPkt->cmd == MemCmd::WritebackClean) {
                // clean writeback, do not send since the block is
                // still cached above
                assert(writebackClean);
                delete wbPkt;
            } else {
                assert(wbPkt->cmd == MemCmd::WritebackDirty ||
                       wbPkt->cmd == MemCmd::WriteClean);
                // Set BLOCK_CACHED flag in Writeback and send below, so that
                // the Writeback does not reset the bit corresponding to this
                // address in the snoop filter below.
                wbPkt->setBlockCached();
                allocateWriteBuffer(wbPkt, forward_time);
            }
        } else {
            // If the block is not cached above, send packet below. Both
            // CleanEvict and Writeback with BLOCK_CACHED flag cleared will
            // reset the bit corresponding to this address in the snoop filter
            // below.
            allocateWriteBuffer(wbPkt, forward_time);
        }
        writebacks.pop_front();
    }
}

void
Cache::doWritebacksAtomic(PacketList& writebacks)
{
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
        // Call isCachedAbove for both Writebacks and CleanEvicts. If
        // isCachedAbove returns true we set BLOCK_CACHED flag in Writebacks
        // and discard CleanEvicts.
        if (isCachedAbove(wbPkt, false)) {
            if (wbPkt->cmd == MemCmd::WritebackDirty ||
                wbPkt->cmd == MemCmd::WriteClean) {
                // Set BLOCK_CACHED flag in Writeback and send below,
                // so that the Writeback does not reset the bit
                // corresponding to this address in the snoop filter
                // below. We can discard CleanEvicts because cached
                // copies exist above. Atomic mode isCachedAbove
                // modifies packet to set BLOCK_CACHED flag
                memSidePort->sendAtomic(wbPkt);
            }
        } else {
            // If the block is not cached above, send packet below. Both
            // CleanEvict and Writeback with BLOCK_CACHED flag cleared will
            // reset the bit corresponding to this address in the snoop filter
            // below.
            memSidePort->sendAtomic(wbPkt);
        }
        writebacks.pop_front();
        // In case of CleanEvicts, the packet destructor will delete the
        // request object because this is a non-snoop request packet which
        // does not require a response.
        delete wbPkt;
    }
}


void
Cache::recvTimingSnoopResp(PacketPtr pkt)
{
    DPRINTF(Cache, "%s for %s\n", __func__, pkt->print());

    assert(pkt->isResponse());
    assert(!system->bypassCaches());

    // determine if the response is from a snoop request we created
    // (in which case it should be in the outstandingSnoop), or if we
    // merely forwarded someone else's snoop request
    const bool forwardAsSnoop = outstandingSnoop.find(pkt->req) ==
        outstandingSnoop.end();

    if (!forwardAsSnoop) {
        // the packet came from this cache, so sink it here and do not
        // forward it
        assert(pkt->cmd == MemCmd::HardPFResp);

        outstandingSnoop.erase(pkt->req);

        DPRINTF(Cache, "Got prefetch response from above for addr "
                "%#llx (%s)\n", pkt->getAddr(), pkt->isSecure() ? "s" : "ns");
        recvTimingResp(pkt);
        return;
    }

    // forwardLatency is set here because there is a response from an
    // upper level cache.
    // To pay the delay that occurs if the packet comes from the bus,
    // we charge also headerDelay.
    Tick snoop_resp_time = clockEdge(forwardLatency) + pkt->headerDelay;
    // Reset the timing of the packet.
    pkt->headerDelay = pkt->payloadDelay = 0;
    memSidePort->schedTimingSnoopResp(pkt, snoop_resp_time);
}

void
Cache::promoteWholeLineWrites(PacketPtr pkt)
{
    // Cache line clearing instructions
    if (doFastWrites && (pkt->cmd == MemCmd::WriteReq) &&
        (pkt->getSize() == blkSize) && (pkt->getOffset(blkSize) == 0)) {
        pkt->cmd = MemCmd::WriteLineReq;
        DPRINTF(Cache, "packet promoted from Write to WriteLineReq\n");
    }
}

bool
Cache::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(CacheTags, "%s tags:\n%s\n", __func__, tags->print());

    assert(pkt->isRequest());

    // Just forward the packet if caches are disabled.
    if (system->bypassCaches()) {
        // @todo This should really enqueue the packet rather
        bool M5_VAR_USED success = memSidePort->sendTimingReq(pkt);
        assert(success);
        return true;
    }

    promoteWholeLineWrites(pkt);

    // Cache maintenance operations have to visit all the caches down
    // to the specified xbar (PoC, PoU, etc.). Even if a cache above
    // is responding we forward the packet to the memory below rather
    // than creating an express snoop.
    if (pkt->cacheResponding()) {
        // a cache above us (but not where the packet came from) is
        // responding to the request, in other words it has the line
        // in Modified or Owned state
        DPRINTF(Cache, "Cache above responding to %s: not responding\n",
                pkt->print());

        // if the packet needs the block to be writable, and the cache
        // that has promised to respond (setting the cache responding
        // flag) is not providing writable (it is in Owned rather than
        // the Modified state), we know that there may be other Shared
        // copies in the system; go out and invalidate them all
        assert(pkt->needsWritable() && !pkt->responderHadWritable());

        // an upstream cache that had the line in Owned state
        // (dirty, but not writable), is responding and thus
        // transferring the dirty line from one branch of the
        // cache hierarchy to another

        // send out an express snoop and invalidate all other
        // copies (snooping a packet that needs writable is the
        // same as an invalidation), thus turning the Owned line
        // into a Modified line, note that we don't invalidate the
        // block in the current cache or any other cache on the
        // path to memory

        // create a downstream express snoop with cleared packet
        // flags, there is no need to allocate any data as the
        // packet is merely used to co-ordinate state transitions
        Packet *snoop_pkt = new Packet(pkt, true, false);

        // also reset the bus time that the original packet has
        // not yet paid for
        snoop_pkt->headerDelay = snoop_pkt->payloadDelay = 0;

        // make this an instantaneous express snoop, and let the
        // other caches in the system know that the another cache
        // is responding, because we have found the authorative
        // copy (Modified or Owned) that will supply the right
        // data
        snoop_pkt->setExpressSnoop();
        snoop_pkt->setCacheResponding();

        // this express snoop travels towards the memory, and at
        // every crossbar it is snooped upwards thus reaching
        // every cache in the system
        bool M5_VAR_USED success = memSidePort->sendTimingReq(snoop_pkt);
        // express snoops always succeed
        assert(success);

        // main memory will delete the snoop packet

        // queue for deletion, as opposed to immediate deletion, as
        // the sending cache is still relying on the packet
        pendingDelete.reset(pkt);

        // no need to take any further action in this particular cache
        // as an upstram cache has already committed to responding,
        // and we have already sent out any express snoops in the
        // section above to ensure all other copies in the system are
        // invalidated
        return true;
    }

    // anything that is merely forwarded pays for the forward latency and
    // the delay provided by the crossbar
    Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;

    // We use lookupLatency here because it is used to specify the latency
    // to access.
    Cycles lat = lookupLatency;
    CacheBlk *blk = nullptr;
    bool satisfied = false;
    {
        PacketList writebacks;
        // Note that lat is passed by reference here. The function
        // access() calls accessBlock() which can modify lat value.
        satisfied = access(pkt, blk, lat, writebacks);

        // copy writebacks to write buffer here to ensure they logically
        // proceed anything happening below
        doWritebacks(writebacks, forward_time);
    }

    // Here we charge the headerDelay that takes into account the latencies
    // of the bus, if the packet comes from it.
    // The latency charged it is just lat that is the value of lookupLatency
    // modified by access() function, or if not just lookupLatency.
    // In case of a hit we are neglecting response latency.
    // In case of a miss we are neglecting forward latency.
    Tick request_time = clockEdge(lat) + pkt->headerDelay;
    // Here we reset the timing of the packet.
    pkt->headerDelay = pkt->payloadDelay = 0;

    // track time of availability of next prefetch, if any
    Tick next_pf_time = MaxTick;

    bool needsResponse = pkt->needsResponse();

    if (satisfied) {
        // should never be satisfying an uncacheable access as we
        // flush and invalidate any existing block as part of the
        // lookup
        assert(!pkt->req->isUncacheable());

        // hit (for all other request types)

        if (prefetcher && (prefetchOnAccess ||
                           (blk && blk->wasPrefetched()))) {
            if (blk)
                blk->status &= ~BlkHWPrefetched;

            // Don't notify on SWPrefetch
            if (!pkt->cmd.isSWPrefetch()) {
                assert(!pkt->req->isCacheMaintenance());
                next_pf_time = prefetcher->notify(pkt);
            }
        }

        if (needsResponse) {
            pkt->makeTimingResponse();
            // @todo: Make someone pay for this
            pkt->headerDelay = pkt->payloadDelay = 0;

            // In this case we are considering request_time that takes
            // into account the delay of the xbar, if any, and just
            // lat, neglecting responseLatency, modelling hit latency
            // just as lookupLatency or or the value of lat overriden
            // by access(), that calls accessBlock() function.
            cpuSidePort->schedTimingResp(pkt, request_time, true);
        } else {
            DPRINTF(Cache, "%s satisfied %s, no response needed\n", __func__,
                    pkt->print());

            // queue the packet for deletion, as the sending cache is
            // still relying on it; if the block is found in access(),
            // CleanEvict and Writeback messages will be deleted
            // here as well
            pendingDelete.reset(pkt);
        }
    } else {
        // miss

        Addr blk_addr = pkt->getBlockAddr(blkSize);

        // ignore any existing MSHR if we are dealing with an
        // uncacheable request
        MSHR *mshr = pkt->req->isUncacheable() ? nullptr :
            mshrQueue.findMatch(blk_addr, pkt->isSecure());

        // Software prefetch handling:
        // To keep the core from waiting on data it won't look at
        // anyway, send back a response with dummy data. Miss handling
        // will continue asynchronously. Unfortunately, the core will
        // insist upon freeing original Packet/Request, so we have to
        // create a new pair with a different lifecycle. Note that this
        // processing happens before any MSHR munging on the behalf of
        // this request because this new Request will be the one stored
        // into the MSHRs, not the original.
        if (pkt->cmd.isSWPrefetch()) {
            assert(needsResponse);
            assert(pkt->req->hasPaddr());
            assert(!pkt->req->isUncacheable());

            // There's no reason to add a prefetch as an additional target
            // to an existing MSHR. If an outstanding request is already
            // in progress, there is nothing for the prefetch to do.
            // If this is the case, we don't even create a request at all.
            PacketPtr pf = nullptr;

            if (!mshr) {
                // copy the request and create a new SoftPFReq packet
                RequestPtr req = new Request(pkt->req->getPaddr(),
                                             pkt->req->getSize(),
                                             pkt->req->getFlags(),
                                             pkt->req->masterId());
                pf = new Packet(req, pkt->cmd);
                pf->allocate();
                assert(pf->getAddr() == pkt->getAddr());
                assert(pf->getSize() == pkt->getSize());
            }

            pkt->makeTimingResponse();

            // request_time is used here, taking into account lat and the delay
            // charged if the packet comes from the xbar.
            cpuSidePort->schedTimingResp(pkt, request_time, true);

            // If an outstanding request is in progress (we found an
            // MSHR) this is set to null
            pkt = pf;
        }

        if (mshr) {
            /// MSHR hit
            /// @note writebacks will be checked in getNextMSHR()
            /// for any conflicting requests to the same block

            //@todo remove hw_pf here

            // Coalesce unless it was a software prefetch (see above).
            if (pkt) {
                assert(!pkt->isWriteback());
                // CleanEvicts corresponding to blocks which have
                // outstanding requests in MSHRs are simply sunk here
                if (pkt->cmd == MemCmd::CleanEvict) {
                    pendingDelete.reset(pkt);
                } else if (pkt->cmd == MemCmd::WriteClean) {
                    // A WriteClean should never coalesce with any
                    // outstanding cache maintenance requests.

                    // We use forward_time here because there is an
                    // uncached memory write, forwarded to WriteBuffer.
                    allocateWriteBuffer(pkt, forward_time);
                } else {
                    DPRINTF(Cache, "%s coalescing MSHR for %s\n", __func__,
                            pkt->print());

                    assert(pkt->req->masterId() < system->maxMasters());
                    mshr_hits[pkt->cmdToIndex()][pkt->req->masterId()]++;
                    // We use forward_time here because it is the same
                    // considering new targets. We have multiple
                    // requests for the same address here. It
                    // specifies the latency to allocate an internal
                    // buffer and to schedule an event to the queued
                    // port and also takes into account the additional
                    // delay of the xbar.
                    mshr->allocateTarget(pkt, forward_time, order++,
                                         allocOnFill(pkt->cmd));
                    if (mshr->getNumTargets() == numTarget) {
                        noTargetMSHR = mshr;
                        setBlocked(Blocked_NoTargets);
                        // need to be careful with this... if this mshr isn't
                        // ready yet (i.e. time > curTick()), we don't want to
                        // move it ahead of mshrs that are ready
                        // mshrQueue.moveToFront(mshr);
                    }
                }
                // We should call the prefetcher reguardless if the request is
                // satisfied or not, reguardless if the request is in the MSHR
                // or not.  The request could be a ReadReq hit, but still not
                // satisfied (potentially because of a prior write to the same
                // cache line.  So, even when not satisfied, tehre is an MSHR
                // already allocated for this, we need to let the prefetcher
                // know about the request
                if (prefetcher) {
                    // Don't notify on SWPrefetch
                    if (!pkt->cmd.isSWPrefetch() &&
                        !pkt->req->isCacheMaintenance())
                        next_pf_time = prefetcher->notify(pkt);
                }
            }
        } else {
            // no MSHR
            assert(pkt->req->masterId() < system->maxMasters());
            if (pkt->req->isUncacheable()) {
                mshr_uncacheable[pkt->cmdToIndex()][pkt->req->masterId()]++;
            } else {
                mshr_misses[pkt->cmdToIndex()][pkt->req->masterId()]++;
            }

            if (pkt->isEviction() || pkt->cmd == MemCmd::WriteClean ||
                (pkt->req->isUncacheable() && pkt->isWrite())) {
                // We use forward_time here because there is an
                // uncached memory write, forwarded to WriteBuffer.
                allocateWriteBuffer(pkt, forward_time);
            } else {
                if (blk && blk->isValid()) {
                    // should have flushed and have no valid block
                    assert(!pkt->req->isUncacheable());

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
                    assert((pkt->needsWritable() && !blk->isWritable()) ||
                           pkt->req->isCacheMaintenance());
                    blk->status &= ~BlkReadable;
                }
                // Here we are using forward_time, modelling the latency of
                // a miss (outbound) just as forwardLatency, neglecting the
                // lookupLatency component.
                allocateMissBuffer(pkt, forward_time);
            }

            if (prefetcher) {
                // Don't notify on SWPrefetch
                if (!pkt->cmd.isSWPrefetch() &&
                    !pkt->req->isCacheMaintenance())
                    next_pf_time = prefetcher->notify(pkt);
            }
        }
    }

    if (next_pf_time != MaxTick)
        schedMemSideSendEvent(next_pf_time);

    return true;
}

PacketPtr
Cache::createMissPacket(PacketPtr cpu_pkt, CacheBlk *blk,
                        bool needsWritable) const
{
    // should never see evictions here
    assert(!cpu_pkt->isEviction());

    bool blkValid = blk && blk->isValid();

    if (cpu_pkt->req->isUncacheable() ||
        (!blkValid && cpu_pkt->isUpgrade()) ||
        cpu_pkt->cmd == MemCmd::InvalidateReq || cpu_pkt->isClean()) {
        // uncacheable requests and upgrades from upper-level caches
        // that missed completely just go through as is
        return nullptr;
    }

    assert(cpu_pkt->needsResponse());

    MemCmd cmd;
    // @TODO make useUpgrades a parameter.
    // Note that ownership protocols require upgrade, otherwise a
    // write miss on a shared owned block will generate a ReadExcl,
    // which will clobber the owned copy.
    const bool useUpgrades = true;
    if (cpu_pkt->cmd == MemCmd::WriteLineReq) {
        assert(!blkValid || !blk->isWritable());
        // forward as invalidate to all other caches, this gives us
        // the line in Exclusive state, and invalidates all other
        // copies
        cmd = MemCmd::InvalidateReq;
    } else if (blkValid && useUpgrades) {
        // only reason to be here is that blk is read only and we need
        // it to be writable
        assert(needsWritable);
        assert(!blk->isWritable());
        cmd = cpu_pkt->isLLSC() ? MemCmd::SCUpgradeReq : MemCmd::UpgradeReq;
    } else if (cpu_pkt->cmd == MemCmd::SCUpgradeFailReq ||
               cpu_pkt->cmd == MemCmd::StoreCondFailReq) {
        // Even though this SC will fail, we still need to send out the
        // request and get the data to supply it to other snoopers in the case
        // where the determination the StoreCond fails is delayed due to
        // all caches not being on the same local bus.
        cmd = MemCmd::SCUpgradeFailReq;
    } else {
        // block is invalid

        // If the request does not need a writable there are two cases
        // where we need to ensure the response will not fetch the
        // block in dirty state:
        // * this cache is read only and it does not perform
        //   writebacks,
        // * this cache is mostly exclusive and will not fill (since
        //   it does not fill it will have to writeback the dirty data
        //   immediately which generates uneccesary writebacks).
        bool force_clean_rsp = isReadOnly || clusivity == Enums::mostly_excl;
        cmd = needsWritable ? MemCmd::ReadExReq :
            (force_clean_rsp ? MemCmd::ReadCleanReq : MemCmd::ReadSharedReq);
    }
    PacketPtr pkt = new Packet(cpu_pkt->req, cmd, blkSize);

    // if there are upstream caches that have already marked the
    // packet as having sharers (not passing writable), pass that info
    // downstream
    if (cpu_pkt->hasSharers() && !needsWritable) {
        // note that cpu_pkt may have spent a considerable time in the
        // MSHR queue and that the information could possibly be out
        // of date, however, there is no harm in conservatively
        // assuming the block has sharers
        pkt->setHasSharers();
        DPRINTF(Cache, "%s: passing hasSharers from %s to %s\n",
                __func__, cpu_pkt->print(), pkt->print());
    }

    // the packet should be block aligned
    assert(pkt->getAddr() == pkt->getBlockAddr(blkSize));

    pkt->allocate();
    DPRINTF(Cache, "%s: created %s from %s\n", __func__, pkt->print(),
            cpu_pkt->print());
    return pkt;
}


Tick
Cache::recvAtomic(PacketPtr pkt)
{
    // We are in atomic mode so we pay just for lookupLatency here.
    Cycles lat = lookupLatency;

    // Forward the request if the system is in cache bypass mode.
    if (system->bypassCaches())
        return ticksToCycles(memSidePort->sendAtomic(pkt));

    promoteWholeLineWrites(pkt);

    // follow the same flow as in recvTimingReq, and check if a cache
    // above us is responding
    if (pkt->cacheResponding() && !pkt->isClean()) {
        assert(!pkt->req->isCacheInvalidate());
        DPRINTF(Cache, "Cache above responding to %s: not responding\n",
                pkt->print());

        // if a cache is responding, and it had the line in Owned
        // rather than Modified state, we need to invalidate any
        // copies that are not on the same path to memory
        assert(pkt->needsWritable() && !pkt->responderHadWritable());
        lat += ticksToCycles(memSidePort->sendAtomic(pkt));

        return lat * clockPeriod();
    }

    // should assert here that there are no outstanding MSHRs or
    // writebacks... that would mean that someone used an atomic
    // access in timing mode

    CacheBlk *blk = nullptr;
    PacketList writebacks;
    bool satisfied = access(pkt, blk, lat, writebacks);

    if (pkt->isClean() && blk && blk->isDirty()) {
        // A cache clean opearation is looking for a dirty
        // block. If a dirty block is encountered a WriteClean
        // will update any copies to the path to the memory
        // until the point of reference.
        DPRINTF(CacheVerbose, "%s: packet %s found block: %s\n",
                __func__, pkt->print(), blk->print());
        PacketPtr wb_pkt = writecleanBlk(blk, pkt->req->getDest(), pkt->id);
        writebacks.push_back(wb_pkt);
        pkt->setSatisfied();
    }

    // handle writebacks resulting from the access here to ensure they
    // logically proceed anything happening below
    doWritebacksAtomic(writebacks);

    if (!satisfied) {
        // MISS

        // deal with the packets that go through the write path of
        // the cache, i.e. any evictions and writes
        if (pkt->isEviction() || pkt->cmd == MemCmd::WriteClean ||
            (pkt->req->isUncacheable() && pkt->isWrite())) {
            lat += ticksToCycles(memSidePort->sendAtomic(pkt));
            return lat * clockPeriod();
        }
        // only misses left

        PacketPtr bus_pkt = createMissPacket(pkt, blk, pkt->needsWritable());

        bool is_forward = (bus_pkt == nullptr);

        if (is_forward) {
            // just forwarding the same request to the next level
            // no local cache operation involved
            bus_pkt = pkt;
        }

        DPRINTF(Cache, "%s: Sending an atomic %s\n", __func__,
                bus_pkt->print());

#if TRACING_ON
        CacheBlk::State old_state = blk ? blk->status : 0;
#endif

        lat += ticksToCycles(memSidePort->sendAtomic(bus_pkt));

        bool is_invalidate = bus_pkt->isInvalidate();

        // We are now dealing with the response handling
        DPRINTF(Cache, "%s: Receive response: %s in state %i\n", __func__,
                bus_pkt->print(), old_state);

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
                } else if (pkt->cmd == MemCmd::WriteLineReq) {
                    // note the use of pkt, not bus_pkt here.

                    // write-line request to the cache that promoted
                    // the write to a whole line
                    blk = handleFill(pkt, blk, writebacks,
                                     allocOnFill(pkt->cmd));
                    assert(blk != NULL);
                    is_invalidate = false;
                    satisfyRequest(pkt, blk);
                } else if (bus_pkt->isRead() ||
                           bus_pkt->cmd == MemCmd::UpgradeResp) {
                    // we're updating cache state to allow us to
                    // satisfy the upstream request from the cache
                    blk = handleFill(bus_pkt, blk, writebacks,
                                     allocOnFill(pkt->cmd));
                    satisfyRequest(pkt, blk);
                    maintainClusivity(pkt->fromCache(), blk);
                } else {
                    // we're satisfying the upstream request without
                    // modifying cache state, e.g., a write-through
                    pkt->makeAtomicResponse();
                }
            }
            delete bus_pkt;
        }

        if (is_invalidate && blk && blk->isValid()) {
            invalidateBlock(blk);
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

    // do any writebacks resulting from the response handling
    doWritebacksAtomic(writebacks);

    // if we used temp block, check to see if its valid and if so
    // clear it out, but only do so after the call to recvAtomic is
    // finished so that any downstream observers (such as a snoop
    // filter), first see the fill, and only then see the eviction
    if (blk == tempBlock && tempBlock->isValid()) {
        // the atomic CPU calls recvAtomic for fetch and load/store
        // sequentuially, and we may already have a tempBlock
        // writeback from the fetch that we have not yet sent
        if (tempBlockWriteback) {
            // if that is the case, write the prevoius one back, and
            // do not schedule any new event
            writebackTempBlockAtomic();
        } else {
            // the writeback/clean eviction happens after the call to
            // recvAtomic has finished (but before any successive
            // calls), so that the response handling from the fill is
            // allowed to happen first
            schedule(writebackTempBlockAtomicEvent, curTick());
        }

        tempBlockWriteback = (blk->isDirty() || writebackClean) ?
            writebackBlk(blk) : cleanEvictBlk(blk);
        invalidateBlock(blk);
    }

    if (pkt->needsResponse()) {
        pkt->makeAtomicResponse();
    }

    return lat * clockPeriod();
}


void
Cache::functionalAccess(PacketPtr pkt, bool fromCpuSide)
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

    Addr blk_addr = pkt->getBlockAddr(blkSize);
    bool is_secure = pkt->isSecure();
    CacheBlk *blk = tags->findBlock(pkt->getAddr(), is_secure);
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

    // data we have is dirty if marked as such or if we have an
    // in-service MSHR that is pending a modified line
    bool have_dirty =
        have_data && (blk->isDirty() ||
                      (mshr && mshr->inService && mshr->isPendingModified()));

    bool done = have_dirty
        || cpuSidePort->checkFunctional(pkt)
        || mshrQueue.checkFunctional(pkt, blk_addr)
        || writeBuffer.checkFunctional(pkt, blk_addr)
        || memSidePort->checkFunctional(pkt);

    DPRINTF(CacheVerbose, "%s: %s %s%s%s\n", __func__,  pkt->print(),
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
        } else if (cpuSidePort->isSnooping()) {
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


void
Cache::handleUncacheableWriteResp(PacketPtr pkt)
{
    Tick completion_time = clockEdge(responseLatency) +
        pkt->headerDelay + pkt->payloadDelay;

    // Reset the bus additional time as it is now accounted for
    pkt->headerDelay = pkt->payloadDelay = 0;

    cpuSidePort->schedTimingResp(pkt, completion_time, true);
}

void
Cache::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());

    // all header delay should be paid for by the crossbar, unless
    // this is a prefetch response from above
    panic_if(pkt->headerDelay != 0 && pkt->cmd != MemCmd::HardPFResp,
             "%s saw a non-zero packet delay\n", name());

    bool is_error = pkt->isError();

    if (is_error) {
        DPRINTF(Cache, "%s: Cache received %s with error\n", __func__,
                pkt->print());
    }

    DPRINTF(Cache, "%s: Handling response %s\n", __func__,
            pkt->print());

    // if this is a write, we should be looking at an uncacheable
    // write
    if (pkt->isWrite()) {
        assert(pkt->req->isUncacheable());
        handleUncacheableWriteResp(pkt);
        return;
    }

    // we have dealt with any (uncacheable) writes above, from here on
    // we know we are dealing with an MSHR due to a miss or a prefetch
    MSHR *mshr = dynamic_cast<MSHR*>(pkt->popSenderState());
    assert(mshr);

    if (mshr == noTargetMSHR) {
        // we always clear at least one target
        clearBlocked(Blocked_NoTargets);
        noTargetMSHR = nullptr;
    }

    // Initial target is used just for stats
    MSHR::Target *initial_tgt = mshr->getTarget();
    int stats_cmd_idx = initial_tgt->pkt->cmdToIndex();
    Tick miss_latency = curTick() - initial_tgt->recvTime;

    if (pkt->req->isUncacheable()) {
        assert(pkt->req->masterId() < system->maxMasters());
        mshr_uncacheable_lat[stats_cmd_idx][pkt->req->masterId()] +=
            miss_latency;
    } else {
        assert(pkt->req->masterId() < system->maxMasters());
        mshr_miss_latency[stats_cmd_idx][pkt->req->masterId()] +=
            miss_latency;
    }

    bool wasFull = mshrQueue.isFull();

    PacketList writebacks;

    Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;

    bool is_fill = !mshr->isForward &&
        (pkt->isRead() || pkt->cmd == MemCmd::UpgradeResp);

    CacheBlk *blk = tags->findBlock(pkt->getAddr(), pkt->isSecure());
    const bool valid_blk = blk && blk->isValid();
    // If the response indicates that there are no sharers and we
    // either had the block already or the response is filling we can
    // promote our copy to writable
    if (!pkt->hasSharers() &&
        (is_fill || (valid_blk && !pkt->req->isCacheInvalidate()))) {
        mshr->promoteWritable();
    }

    if (is_fill && !is_error) {
        DPRINTF(Cache, "Block for addr %#llx being updated in Cache\n",
                pkt->getAddr());

        blk = handleFill(pkt, blk, writebacks, mshr->allocOnFill());
        assert(blk != nullptr);
    }

    // allow invalidation responses originating from write-line
    // requests to be discarded
    bool is_invalidate = pkt->isInvalidate();

    // The block was marked as not readable while there was a pending
    // cache maintenance operation, restore its flag.
    if (pkt->isClean() && !is_invalidate && valid_blk) {
        blk->status |= BlkReadable;
    }

    // First offset for critical word first calculations
    int initial_offset = initial_tgt->pkt->getOffset(blkSize);

    bool from_cache = false;
    MSHR::TargetList targets = mshr->extractServiceableTargets(pkt);
    for (auto &target: targets) {
        Packet *tgt_pkt = target.pkt;
        switch (target.source) {
          case MSHR::Target::FromCPU:
            Tick completion_time;
            // Here we charge on completion_time the delay of the xbar if the
            // packet comes from it, charged on headerDelay.
            completion_time = pkt->headerDelay;

            // Software prefetch handling for cache closest to core
            if (tgt_pkt->cmd.isSWPrefetch()) {
                // a software prefetch would have already been ack'd
                // immediately with dummy data so the core would be able to
                // retire it. This request completes right here, so we
                // deallocate it.
                delete tgt_pkt->req;
                delete tgt_pkt;
                break; // skip response
            }

            // keep track of whether we have responded to another
            // cache
            from_cache = from_cache || tgt_pkt->fromCache();

            // unlike the other packet flows, where data is found in other
            // caches or memory and brought back, write-line requests always
            // have the data right away, so the above check for "is fill?"
            // cannot actually be determined until examining the stored MSHR
            // state. We "catch up" with that logic here, which is duplicated
            // from above.
            if (tgt_pkt->cmd == MemCmd::WriteLineReq) {
                assert(!is_error);
                // we got the block in a writable state, so promote
                // any deferred targets if possible
                mshr->promoteWritable();
                // NB: we use the original packet here and not the response!
                blk = handleFill(tgt_pkt, blk, writebacks,
                                 targets.allocOnFill);
                assert(blk != nullptr);

                // treat as a fill, and discard the invalidation
                // response
                is_fill = true;
                is_invalidate = false;
            }

            if (is_fill) {
                satisfyRequest(tgt_pkt, blk, true, mshr->hasPostDowngrade());

                // How many bytes past the first request is this one
                int transfer_offset =
                    tgt_pkt->getOffset(blkSize) - initial_offset;
                if (transfer_offset < 0) {
                    transfer_offset += blkSize;
                }

                // If not critical word (offset) return payloadDelay.
                // responseLatency is the latency of the return path
                // from lower level caches/memory to an upper level cache or
                // the core.
                completion_time += clockEdge(responseLatency) +
                    (transfer_offset ? pkt->payloadDelay : 0);

                assert(!tgt_pkt->req->isUncacheable());

                assert(tgt_pkt->req->masterId() < system->maxMasters());
                missLatency[tgt_pkt->cmdToIndex()][tgt_pkt->req->masterId()] +=
                    completion_time - target.recvTime;
            } else if (pkt->cmd == MemCmd::UpgradeFailResp) {
                // failed StoreCond upgrade
                assert(tgt_pkt->cmd == MemCmd::StoreCondReq ||
                       tgt_pkt->cmd == MemCmd::StoreCondFailReq ||
                       tgt_pkt->cmd == MemCmd::SCUpgradeFailReq);
                // responseLatency is the latency of the return path
                // from lower level caches/memory to an upper level cache or
                // the core.
                completion_time += clockEdge(responseLatency) +
                    pkt->payloadDelay;
                tgt_pkt->req->setExtraData(0);
            } else {
                // We are about to send a response to a cache above
                // that asked for an invalidation; we need to
                // invalidate our copy immediately as the most
                // up-to-date copy of the block will now be in the
                // cache above. It will also prevent this cache from
                // responding (if the block was previously dirty) to
                // snoops as they should snoop the caches above where
                // they will get the response from.
                if (is_invalidate && blk && blk->isValid()) {
                    invalidateBlock(blk);
                }
                // not a cache fill, just forwarding response
                // responseLatency is the latency of the return path
                // from lower level cahces/memory to the core.
                completion_time += clockEdge(responseLatency) +
                    pkt->payloadDelay;
                if (pkt->isRead() && !is_error) {
                    // sanity check
                    assert(pkt->getAddr() == tgt_pkt->getAddr());
                    assert(pkt->getSize() >= tgt_pkt->getSize());

                    tgt_pkt->setData(pkt->getConstPtr<uint8_t>());
                }
            }
            tgt_pkt->makeTimingResponse();
            // if this packet is an error copy that to the new packet
            if (is_error)
                tgt_pkt->copyError(pkt);
            if (tgt_pkt->cmd == MemCmd::ReadResp &&
                (is_invalidate || mshr->hasPostInvalidate())) {
                // If intermediate cache got ReadRespWithInvalidate,
                // propagate that.  Response should not have
                // isInvalidate() set otherwise.
                tgt_pkt->cmd = MemCmd::ReadRespWithInvalidate;
                DPRINTF(Cache, "%s: updated cmd to %s\n", __func__,
                        tgt_pkt->print());
            }
            // Reset the bus additional time as it is now accounted for
            tgt_pkt->headerDelay = tgt_pkt->payloadDelay = 0;
            cpuSidePort->schedTimingResp(tgt_pkt, completion_time, true);
            break;

          case MSHR::Target::FromPrefetcher:
            assert(tgt_pkt->cmd == MemCmd::HardPFReq);
            if (blk)
                blk->status |= BlkHWPrefetched;
            delete tgt_pkt->req;
            delete tgt_pkt;
            break;

          case MSHR::Target::FromSnoop:
            // I don't believe that a snoop can be in an error state
            assert(!is_error);
            // response to snoop request
            DPRINTF(Cache, "processing deferred snoop...\n");
            // If the response is invalidating, a snooping target can
            // be satisfied if it is also invalidating. If the reponse is, not
            // only invalidating, but more specifically an InvalidateResp and
            // the MSHR was created due to an InvalidateReq then a cache above
            // is waiting to satisfy a WriteLineReq. In this case even an
            // non-invalidating snoop is added as a target here since this is
            // the ordering point. When the InvalidateResp reaches this cache,
            // the snooping target will snoop further the cache above with the
            // WriteLineReq.
            assert(!is_invalidate || pkt->cmd == MemCmd::InvalidateResp ||
                   pkt->req->isCacheMaintenance() ||
                   mshr->hasPostInvalidate());
            handleSnoop(tgt_pkt, blk, true, true, mshr->hasPostInvalidate());
            break;

          default:
            panic("Illegal target->source enum %d\n", target.source);
        }
    }

    maintainClusivity(from_cache, blk);

    if (blk && blk->isValid()) {
        // an invalidate response stemming from a write line request
        // should not invalidate the block, so check if the
        // invalidation should be discarded
        if (is_invalidate || mshr->hasPostInvalidate()) {
            invalidateBlock(blk);
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
        mshrQueue.markPending(mshr);
        schedMemSideSendEvent(clockEdge() + pkt->payloadDelay);
    } else {
        mshrQueue.deallocate(mshr);
        if (wasFull && !mshrQueue.isFull()) {
            clearBlocked(Blocked_NoMSHRs);
        }

        // Request the bus for a prefetch if this deallocation freed enough
        // MSHRs for a prefetch to take place
        if (prefetcher && mshrQueue.canPrefetch()) {
            Tick next_pf_time = std::max(prefetcher->nextPrefetchReadyTime(),
                                         clockEdge());
            if (next_pf_time != MaxTick)
                schedMemSideSendEvent(next_pf_time);
        }
    }
    // reset the xbar additional timinig  as it is now accounted for
    pkt->headerDelay = pkt->payloadDelay = 0;

    // copy writebacks to write buffer
    doWritebacks(writebacks, forward_time);

    // if we used temp block, check to see if its valid and then clear it out
    if (blk == tempBlock && tempBlock->isValid()) {
        // We use forwardLatency here because we are copying
        // Writebacks/CleanEvicts to write buffer. It specifies the latency to
        // allocate an internal buffer and to schedule an event to the
        // queued port.
        if (blk->isDirty() || writebackClean) {
            PacketPtr wbPkt = writebackBlk(blk);
            allocateWriteBuffer(wbPkt, forward_time);
            // Set BLOCK_CACHED flag if cached above.
            if (isCachedAbove(wbPkt))
                wbPkt->setBlockCached();
        } else {
            PacketPtr wcPkt = cleanEvictBlk(blk);
            // Check to see if block is cached above. If not allocate
            // write buffer
            if (isCachedAbove(wcPkt))
                delete wcPkt;
            else
                allocateWriteBuffer(wcPkt, forward_time);
        }
        invalidateBlock(blk);
    }

    DPRINTF(CacheVerbose, "%s: Leaving with %s\n", __func__, pkt->print());
    delete pkt;
}

PacketPtr
Cache::writebackBlk(CacheBlk *blk)
{
    chatty_assert(!isReadOnly || writebackClean,
                  "Writeback from read-only cache");
    assert(blk && blk->isValid() && (blk->isDirty() || writebackClean));

    writebacks[Request::wbMasterId]++;

    Request *req = new Request(tags->regenerateBlkAddr(blk), blkSize, 0,
                               Request::wbMasterId);
    if (blk->isSecure())
        req->setFlags(Request::SECURE);

    req->taskId(blk->task_id);

    PacketPtr pkt =
        new Packet(req, blk->isDirty() ?
                   MemCmd::WritebackDirty : MemCmd::WritebackClean);

    DPRINTF(Cache, "Create Writeback %s writable: %d, dirty: %d\n",
            pkt->print(), blk->isWritable(), blk->isDirty());

    if (blk->isWritable()) {
        // not asserting shared means we pass the block in modified
        // state, mark our own block non-writeable
        blk->status &= ~BlkWritable;
    } else {
        // we are in the Owned state, tell the receiver
        pkt->setHasSharers();
    }

    // make sure the block is not marked dirty
    blk->status &= ~BlkDirty;

    pkt->allocate();
    std::memcpy(pkt->getPtr<uint8_t>(), blk->data, blkSize);

    return pkt;
}

PacketPtr
Cache::writecleanBlk(CacheBlk *blk, Request::Flags dest, PacketId id)
{
    Request *req = new Request(tags->regenerateBlkAddr(blk), blkSize, 0,
                               Request::wbMasterId);
    if (blk->isSecure()) {
        req->setFlags(Request::SECURE);
    }
    req->taskId(blk->task_id);

    PacketPtr pkt = new Packet(req, MemCmd::WriteClean, blkSize, id);

    if (dest) {
        req->setFlags(dest);
        pkt->setWriteThrough();
    }

    DPRINTF(Cache, "Create %s writable: %d, dirty: %d\n", pkt->print(),
            blk->isWritable(), blk->isDirty());

    if (blk->isWritable()) {
        // not asserting shared means we pass the block in modified
        // state, mark our own block non-writeable
        blk->status &= ~BlkWritable;
    } else {
        // we are in the Owned state, tell the receiver
        pkt->setHasSharers();
    }

    // make sure the block is not marked dirty
    blk->status &= ~BlkDirty;

    pkt->allocate();
    std::memcpy(pkt->getPtr<uint8_t>(), blk->data, blkSize);

    return pkt;
}


PacketPtr
Cache::cleanEvictBlk(CacheBlk *blk)
{
    assert(!writebackClean);
    assert(blk && blk->isValid() && !blk->isDirty());
    // Creating a zero sized write, a message to the snoop filter
    Request *req =
        new Request(tags->regenerateBlkAddr(blk), blkSize, 0,
                    Request::wbMasterId);
    if (blk->isSecure())
        req->setFlags(Request::SECURE);

    req->taskId(blk->task_id);

    PacketPtr pkt = new Packet(req, MemCmd::CleanEvict);
    pkt->allocate();
    DPRINTF(Cache, "Create CleanEvict %s\n", pkt->print());

    return pkt;
}

void
Cache::memWriteback()
{
    CacheBlkVisitorWrapper visitor(*this, &Cache::writebackVisitor);
    tags->forEachBlk(visitor);
}

void
Cache::memInvalidate()
{
    CacheBlkVisitorWrapper visitor(*this, &Cache::invalidateVisitor);
    tags->forEachBlk(visitor);
}

bool
Cache::isDirty() const
{
    CacheBlkIsDirtyVisitor visitor;
    tags->forEachBlk(visitor);

    return visitor.isDirty();
}

bool
Cache::writebackVisitor(CacheBlk &blk)
{
    if (blk.isDirty()) {
        assert(blk.isValid());

        Request request(tags->regenerateBlkAddr(&blk), blkSize, 0,
                        Request::funcMasterId);
        request.taskId(blk.task_id);
        if (blk.isSecure()) {
            request.setFlags(Request::SECURE);
        }

        Packet packet(&request, MemCmd::WriteReq);
        packet.dataStatic(blk.data);

        memSidePort->sendFunctional(&packet);

        blk.status &= ~BlkDirty;
    }

    return true;
}

bool
Cache::invalidateVisitor(CacheBlk &blk)
{

    if (blk.isDirty())
        warn_once("Invalidating dirty cache lines. Expect things to break.\n");

    if (blk.isValid()) {
        assert(!blk.isDirty());
        invalidateBlock(&blk);
    }

    return true;
}

CacheBlk*
Cache::allocateBlock(Addr addr, bool is_secure, PacketList &writebacks)
{
    // Find replacement victim
    CacheBlk *blk = tags->findVictim(addr);

    // It is valid to return nullptr if there is no victim
    if (!blk)
        return nullptr;

    if (blk->isValid()) {
        Addr repl_addr = tags->regenerateBlkAddr(blk);
        MSHR *repl_mshr = mshrQueue.findMatch(repl_addr, blk->isSecure());
        if (repl_mshr) {
            // must be an outstanding upgrade or clean request
            // on a block we're about to replace...
            assert((!blk->isWritable() && repl_mshr->needsWritable()) ||
                   repl_mshr->isCleaning());
            // too hard to replace block with transient state
            // allocation failed, block not inserted
            return nullptr;
        } else {
            DPRINTF(Cache, "replacement: replacing %#llx (%s) with %#llx "
                    "(%s): %s\n", repl_addr, blk->isSecure() ? "s" : "ns",
                    addr, is_secure ? "s" : "ns",
                    blk->isDirty() ? "writeback" : "clean");

            if (blk->wasPrefetched()) {
                unusedPrefetches++;
            }
            // Will send up Writeback/CleanEvict snoops via isCachedAbove
            // when pushing this writeback list into the write buffer.
            if (blk->isDirty() || writebackClean) {
                // Save writeback packet for handling by caller
                writebacks.push_back(writebackBlk(blk));
            } else {
                writebacks.push_back(cleanEvictBlk(blk));
            }
        }
    }

    return blk;
}

void
Cache::invalidateBlock(CacheBlk *blk)
{
    if (blk != tempBlock)
        tags->invalidate(blk);
    blk->invalidate();
}

// Note that the reason we return a list of writebacks rather than
// inserting them directly in the write buffer is that this function
// is called by both atomic and timing-mode accesses, and in atomic
// mode we don't mess with the write buffer (we just perform the
// writebacks atomically once the original request is complete).
CacheBlk*
Cache::handleFill(PacketPtr pkt, CacheBlk *blk, PacketList &writebacks,
                  bool allocate)
{
    assert(pkt->isResponse() || pkt->cmd == MemCmd::WriteLineReq);
    Addr addr = pkt->getAddr();
    bool is_secure = pkt->isSecure();
#if TRACING_ON
    CacheBlk::State old_state = blk ? blk->status : 0;
#endif

    // When handling a fill, we should have no writes to this line.
    assert(addr == pkt->getBlockAddr(blkSize));
    assert(!writeBuffer.findMatch(addr, is_secure));

    if (blk == nullptr) {
        // better have read new data...
        assert(pkt->hasData());

        // only read responses and write-line requests have data;
        // note that we don't write the data here for write-line - that
        // happens in the subsequent call to satisfyRequest
        assert(pkt->isRead() || pkt->cmd == MemCmd::WriteLineReq);

        // need to do a replacement if allocating, otherwise we stick
        // with the temporary storage
        blk = allocate ? allocateBlock(addr, is_secure, writebacks) : nullptr;

        if (blk == nullptr) {
            // No replaceable block or a mostly exclusive
            // cache... just use temporary storage to complete the
            // current request and then get rid of it
            assert(!tempBlock->isValid());
            blk = tempBlock;
            tempBlock->set = tags->extractSet(addr);
            tempBlock->tag = tags->extractTag(addr);
            if (is_secure) {
                tempBlock->status |= BlkSecure;
            }
            DPRINTF(Cache, "using temp block for %#llx (%s)\n", addr,
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

    // sanity check for whole-line writes, which should always be
    // marked as writable as part of the fill, and then later marked
    // dirty as part of satisfyRequest
    if (pkt->cmd == MemCmd::WriteLineReq) {
        assert(!pkt->hasSharers());
    }

    // here we deal with setting the appropriate state of the line,
    // and we start by looking at the hasSharers flag, and ignore the
    // cacheResponding flag (normally signalling dirty data) if the
    // packet has sharers, thus the line is never allocated as Owned
    // (dirty but not writable), and always ends up being either
    // Shared, Exclusive or Modified, see Packet::setCacheResponding
    // for more details
    if (!pkt->hasSharers()) {
        // we could get a writable line from memory (rather than a
        // cache) even in a read-only cache, note that we set this bit
        // even for a read-only cache, possibly revisit this decision
        blk->status |= BlkWritable;

        // check if we got this via cache-to-cache transfer (i.e., from a
        // cache that had the block in Modified or Owned state)
        if (pkt->cacheResponding()) {
            // we got the block in Modified state, and invalidated the
            // owners copy
            blk->status |= BlkDirty;

            chatty_assert(!isReadOnly, "Should never see dirty snoop response "
                          "in read-only cache %s\n", name());
        }
    }

    DPRINTF(Cache, "Block addr %#llx (%s) moving from state %x to %s\n",
            addr, is_secure ? "s" : "ns", old_state, blk->print());

    // if we got new data, copy it in (checking for a read response
    // and a response that has data is the same in the end)
    if (pkt->isRead()) {
        // sanity checks
        assert(pkt->hasData());
        assert(pkt->getSize() == blkSize);

        std::memcpy(blk->data, pkt->getConstPtr<uint8_t>(), blkSize);
    }
    // We pay for fillLatency here.
    blk->whenReady = clockEdge() + fillLatency * clockPeriod() +
        pkt->payloadDelay;

    return blk;
}


/////////////////////////////////////////////////////
//
// Snoop path: requests coming in from the memory side
//
/////////////////////////////////////////////////////

void
Cache::doTimingSupplyResponse(PacketPtr req_pkt, const uint8_t *blk_data,
                              bool already_copied, bool pending_inval)
{
    // sanity check
    assert(req_pkt->isRequest());
    assert(req_pkt->needsResponse());

    DPRINTF(Cache, "%s: for %s\n", __func__, req_pkt->print());
    // timing-mode snoop responses require a new packet, unless we
    // already made a copy...
    PacketPtr pkt = req_pkt;
    if (!already_copied)
        // do not clear flags, and allocate space for data if the
        // packet needs it (the only packets that carry data are read
        // responses)
        pkt = new Packet(req_pkt, false, req_pkt->isRead());

    assert(req_pkt->req->isUncacheable() || req_pkt->isInvalidate() ||
           pkt->hasSharers());
    pkt->makeTimingResponse();
    if (pkt->isRead()) {
        pkt->setDataFromBlock(blk_data, blkSize);
    }
    if (pkt->cmd == MemCmd::ReadResp && pending_inval) {
        // Assume we defer a response to a read from a far-away cache
        // A, then later defer a ReadExcl from a cache B on the same
        // bus as us. We'll assert cacheResponding in both cases, but
        // in the latter case cacheResponding will keep the
        // invalidation from reaching cache A. This special response
        // tells cache A that it gets the block to satisfy its read,
        // but must immediately invalidate it.
        pkt->cmd = MemCmd::ReadRespWithInvalidate;
    }
    // Here we consider forward_time, paying for just forward latency and
    // also charging the delay provided by the xbar.
    // forward_time is used as send_time in next allocateWriteBuffer().
    Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;
    // Here we reset the timing of the packet.
    pkt->headerDelay = pkt->payloadDelay = 0;
    DPRINTF(CacheVerbose, "%s: created response: %s tick: %lu\n", __func__,
            pkt->print(), forward_time);
    memSidePort->schedTimingSnoopResp(pkt, forward_time, true);
}

uint32_t
Cache::handleSnoop(PacketPtr pkt, CacheBlk *blk, bool is_timing,
                   bool is_deferred, bool pending_inval)
{
    DPRINTF(CacheVerbose, "%s: for %s\n", __func__, pkt->print());
    // deferred snoops can only happen in timing mode
    assert(!(is_deferred && !is_timing));
    // pending_inval only makes sense on deferred snoops
    assert(!(pending_inval && !is_deferred));
    assert(pkt->isRequest());

    // the packet may get modified if we or a forwarded snooper
    // responds in atomic mode, so remember a few things about the
    // original packet up front
    bool invalidate = pkt->isInvalidate();
    bool M5_VAR_USED needs_writable = pkt->needsWritable();

    // at the moment we could get an uncacheable write which does not
    // have the invalidate flag, and we need a suitable way of dealing
    // with this case
    panic_if(invalidate && pkt->req->isUncacheable(),
             "%s got an invalidating uncacheable snoop request %s",
             name(), pkt->print());

    uint32_t snoop_delay = 0;

    if (forwardSnoops) {
        // first propagate snoop upward to see if anyone above us wants to
        // handle it.  save & restore packet src since it will get
        // rewritten to be relative to cpu-side bus (if any)
        bool alreadyResponded = pkt->cacheResponding();
        if (is_timing) {
            // copy the packet so that we can clear any flags before
            // forwarding it upwards, we also allocate data (passing
            // the pointer along in case of static data), in case
            // there is a snoop hit in upper levels
            Packet snoopPkt(pkt, true, true);
            snoopPkt.setExpressSnoop();
            // the snoop packet does not need to wait any additional
            // time
            snoopPkt.headerDelay = snoopPkt.payloadDelay = 0;
            cpuSidePort->sendTimingSnoopReq(&snoopPkt);

            // add the header delay (including crossbar and snoop
            // delays) of the upward snoop to the snoop delay for this
            // cache
            snoop_delay += snoopPkt.headerDelay;

            if (snoopPkt.cacheResponding()) {
                // cache-to-cache response from some upper cache
                assert(!alreadyResponded);
                pkt->setCacheResponding();
            }
            // upstream cache has the block, or has an outstanding
            // MSHR, pass the flag on
            if (snoopPkt.hasSharers()) {
                pkt->setHasSharers();
            }
            // If this request is a prefetch or clean evict and an upper level
            // signals block present, make sure to propagate the block
            // presence to the requester.
            if (snoopPkt.isBlockCached()) {
                pkt->setBlockCached();
            }
            // If the request was satisfied by snooping the cache
            // above, mark the original packet as satisfied too.
            if (snoopPkt.satisfied()) {
                pkt->setSatisfied();
            }
        } else {
            cpuSidePort->sendAtomicSnoop(pkt);
            if (!alreadyResponded && pkt->cacheResponding()) {
                // cache-to-cache response from some upper cache:
                // forward response to original requester
                assert(pkt->isResponse());
            }
        }
    }

    bool respond = false;
    bool blk_valid = blk && blk->isValid();
    if (pkt->isClean()) {
        if (blk_valid && blk->isDirty()) {
            DPRINTF(CacheVerbose, "%s: packet (snoop) %s found block: %s\n",
                    __func__, pkt->print(), blk->print());
            PacketPtr wb_pkt = writecleanBlk(blk, pkt->req->getDest(), pkt->id);
            PacketList writebacks;
            writebacks.push_back(wb_pkt);

            if (is_timing) {
                // anything that is merely forwarded pays for the forward
                // latency and the delay provided by the crossbar
                Tick forward_time = clockEdge(forwardLatency) +
                    pkt->headerDelay;
                doWritebacks(writebacks, forward_time);
            } else {
                doWritebacksAtomic(writebacks);
            }
            pkt->setSatisfied();
        }
    } else if (!blk_valid) {
        DPRINTF(CacheVerbose, "%s: snoop miss for %s\n", __func__,
                pkt->print());
        if (is_deferred) {
            // we no longer have the block, and will not respond, but a
            // packet was allocated in MSHR::handleSnoop and we have
            // to delete it
            assert(pkt->needsResponse());

            // we have passed the block to a cache upstream, that
            // cache should be responding
            assert(pkt->cacheResponding());

            delete pkt;
        }
        return snoop_delay;
    } else {
        DPRINTF(Cache, "%s: snoop hit for %s, old state is %s\n", __func__,
                pkt->print(), blk->print());

        // We may end up modifying both the block state and the packet (if
        // we respond in atomic mode), so just figure out what to do now
        // and then do it later. We respond to all snoops that need
        // responses provided we have the block in dirty state. The
        // invalidation itself is taken care of below. We don't respond to
        // cache maintenance operations as this is done by the destination
        // xbar.
        respond = blk->isDirty() && pkt->needsResponse();

        chatty_assert(!(isReadOnly && blk->isDirty()), "Should never have "
                      "a dirty block in a read-only cache %s\n", name());
    }

    // Invalidate any prefetch's from below that would strip write permissions
    // MemCmd::HardPFReq is only observed by upstream caches.  After missing
    // above and in it's own cache, a new MemCmd::ReadReq is created that
    // downstream caches observe.
    if (pkt->mustCheckAbove()) {
        DPRINTF(Cache, "Found addr %#llx in upper level cache for snoop %s "
                "from lower cache\n", pkt->getAddr(), pkt->print());
        pkt->setBlockCached();
        return snoop_delay;
    }

    if (pkt->isRead() && !invalidate) {
        // reading without requiring the line in a writable state
        assert(!needs_writable);
        pkt->setHasSharers();

        // if the requesting packet is uncacheable, retain the line in
        // the current state, otherwhise unset the writable flag,
        // which means we go from Modified to Owned (and will respond
        // below), remain in Owned (and will respond below), from
        // Exclusive to Shared, or remain in Shared
        if (!pkt->req->isUncacheable())
            blk->status &= ~BlkWritable;
        DPRINTF(Cache, "new state is %s\n", blk->print());
    }

    if (respond) {
        // prevent anyone else from responding, cache as well as
        // memory, and also prevent any memory from even seeing the
        // request
        pkt->setCacheResponding();
        if (!pkt->isClean() && blk->isWritable()) {
            // inform the cache hierarchy that this cache had the line
            // in the Modified state so that we avoid unnecessary
            // invalidations (see Packet::setResponderHadWritable)
            pkt->setResponderHadWritable();

            // in the case of an uncacheable request there is no point
            // in setting the responderHadWritable flag, but since the
            // recipient does not care there is no harm in doing so
        } else {
            // if the packet has needsWritable set we invalidate our
            // copy below and all other copies will be invalidates
            // through express snoops, and if needsWritable is not set
            // we already called setHasSharers above
        }

        // if we are returning a writable and dirty (Modified) line,
        // we should be invalidating the line
        panic_if(!invalidate && !pkt->hasSharers(),
                 "%s is passing a Modified line through %s, "
                 "but keeping the block", name(), pkt->print());

        if (is_timing) {
            doTimingSupplyResponse(pkt, blk->data, is_deferred, pending_inval);
        } else {
            pkt->makeAtomicResponse();
            // packets such as upgrades do not actually have any data
            // payload
            if (pkt->hasData())
                pkt->setDataFromBlock(blk->data, blkSize);
        }
    }

    if (!respond && is_deferred) {
        assert(pkt->needsResponse());

        // if we copied the deferred packet with the intention to
        // respond, but are not responding, then a cache above us must
        // be, and we can use this as the indication of whether this
        // is a packet where we created a copy of the request or not
        if (!pkt->cacheResponding()) {
            delete pkt->req;
        }

        delete pkt;
    }

    // Do this last in case it deallocates block data or something
    // like that
    if (blk_valid && invalidate) {
        invalidateBlock(blk);
        DPRINTF(Cache, "new state is %s\n", blk->print());
    }

    return snoop_delay;
}


void
Cache::recvTimingSnoopReq(PacketPtr pkt)
{
    DPRINTF(CacheVerbose, "%s: for %s\n", __func__, pkt->print());

    // Snoops shouldn't happen when bypassing caches
    assert(!system->bypassCaches());

    // no need to snoop requests that are not in range
    if (!inRange(pkt->getAddr())) {
        return;
    }

    bool is_secure = pkt->isSecure();
    CacheBlk *blk = tags->findBlock(pkt->getAddr(), is_secure);

    Addr blk_addr = pkt->getBlockAddr(blkSize);
    MSHR *mshr = mshrQueue.findMatch(blk_addr, is_secure);

    // Update the latency cost of the snoop so that the crossbar can
    // account for it. Do not overwrite what other neighbouring caches
    // have already done, rather take the maximum. The update is
    // tentative, for cases where we return before an upward snoop
    // happens below.
    pkt->snoopDelay = std::max<uint32_t>(pkt->snoopDelay,
                                         lookupLatency * clockPeriod());

    // Inform request(Prefetch, CleanEvict or Writeback) from below of
    // MSHR hit, set setBlockCached.
    if (mshr && pkt->mustCheckAbove()) {
        DPRINTF(Cache, "Setting block cached for %s from lower cache on "
                "mshr hit\n", pkt->print());
        pkt->setBlockCached();
        return;
    }

    // Bypass any existing cache maintenance requests if the request
    // has been satisfied already (i.e., the dirty block has been
    // found).
    if (mshr && pkt->req->isCacheMaintenance() && pkt->satisfied()) {
        return;
    }

    // Let the MSHR itself track the snoop and decide whether we want
    // to go ahead and do the regular cache snoop
    if (mshr && mshr->handleSnoop(pkt, order++)) {
        DPRINTF(Cache, "Deferring snoop on in-service MSHR to blk %#llx (%s)."
                "mshrs: %s\n", blk_addr, is_secure ? "s" : "ns",
                mshr->print());

        if (mshr->getNumTargets() > numTarget)
            warn("allocating bonus target for snoop"); //handle later
        return;
    }

    //We also need to check the writeback buffers and handle those
    WriteQueueEntry *wb_entry = writeBuffer.findMatch(blk_addr, is_secure);
    if (wb_entry) {
        DPRINTF(Cache, "Snoop hit in writeback to addr %#llx (%s)\n",
                pkt->getAddr(), is_secure ? "s" : "ns");
        // Expect to see only Writebacks and/or CleanEvicts here, both of
        // which should not be generated for uncacheable data.
        assert(!wb_entry->isUncacheable());
        // There should only be a single request responsible for generating
        // Writebacks/CleanEvicts.
        assert(wb_entry->getNumTargets() == 1);
        PacketPtr wb_pkt = wb_entry->getTarget()->pkt;
        assert(wb_pkt->isEviction() || wb_pkt->cmd == MemCmd::WriteClean);

        if (pkt->isEviction()) {
            // if the block is found in the write queue, set the BLOCK_CACHED
            // flag for Writeback/CleanEvict snoop. On return the snoop will
            // propagate the BLOCK_CACHED flag in Writeback packets and prevent
            // any CleanEvicts from travelling down the memory hierarchy.
            pkt->setBlockCached();
            DPRINTF(Cache, "%s: Squashing %s from lower cache on writequeue "
                    "hit\n", __func__, pkt->print());
            return;
        }

        // conceptually writebacks are no different to other blocks in
        // this cache, so the behaviour is modelled after handleSnoop,
        // the difference being that instead of querying the block
        // state to determine if it is dirty and writable, we use the
        // command and fields of the writeback packet
        bool respond = wb_pkt->cmd == MemCmd::WritebackDirty &&
            pkt->needsResponse();
        bool have_writable = !wb_pkt->hasSharers();
        bool invalidate = pkt->isInvalidate();

        if (!pkt->req->isUncacheable() && pkt->isRead() && !invalidate) {
            assert(!pkt->needsWritable());
            pkt->setHasSharers();
            wb_pkt->setHasSharers();
        }

        if (respond) {
            pkt->setCacheResponding();

            if (have_writable) {
                pkt->setResponderHadWritable();
            }

            doTimingSupplyResponse(pkt, wb_pkt->getConstPtr<uint8_t>(),
                                   false, false);
        }

        if (invalidate && wb_pkt->cmd != MemCmd::WriteClean) {
            // Invalidation trumps our writeback... discard here
            // Note: markInService will remove entry from writeback buffer.
            markInService(wb_entry);
            delete wb_pkt;
        }
    }

    // If this was a shared writeback, there may still be
    // other shared copies above that require invalidation.
    // We could be more selective and return here if the
    // request is non-exclusive or if the writeback is
    // exclusive.
    uint32_t snoop_delay = handleSnoop(pkt, blk, true, false, false);

    // Override what we did when we first saw the snoop, as we now
    // also have the cost of the upwards snoops to account for
    pkt->snoopDelay = std::max<uint32_t>(pkt->snoopDelay, snoop_delay +
                                         lookupLatency * clockPeriod());
}

bool
Cache::CpuSidePort::recvTimingSnoopResp(PacketPtr pkt)
{
    // Express snoop responses from master to slave, e.g., from L1 to L2
    cache->recvTimingSnoopResp(pkt);
    return true;
}

Tick
Cache::recvAtomicSnoop(PacketPtr pkt)
{
    // Snoops shouldn't happen when bypassing caches
    assert(!system->bypassCaches());

    // no need to snoop requests that are not in range.
    if (!inRange(pkt->getAddr())) {
        return 0;
    }

    CacheBlk *blk = tags->findBlock(pkt->getAddr(), pkt->isSecure());
    uint32_t snoop_delay = handleSnoop(pkt, blk, false, false, false);
    return snoop_delay + lookupLatency * clockPeriod();
}


QueueEntry*
Cache::getNextQueueEntry()
{
    // Check both MSHR queue and write buffer for potential requests,
    // note that null does not mean there is no request, it could
    // simply be that it is not ready
    MSHR *miss_mshr  = mshrQueue.getNext();
    WriteQueueEntry *wq_entry = writeBuffer.getNext();

    // If we got a write buffer request ready, first priority is a
    // full write buffer, otherwise we favour the miss requests
    if (wq_entry && (writeBuffer.isFull() || !miss_mshr)) {
        // need to search MSHR queue for conflicting earlier miss.
        MSHR *conflict_mshr =
            mshrQueue.findPending(wq_entry->blkAddr,
                                  wq_entry->isSecure);

        if (conflict_mshr && conflict_mshr->order < wq_entry->order) {
            // Service misses in order until conflict is cleared.
            return conflict_mshr;

            // @todo Note that we ignore the ready time of the conflict here
        }

        // No conflicts; issue write
        return wq_entry;
    } else if (miss_mshr) {
        // need to check for conflicting earlier writeback
        WriteQueueEntry *conflict_mshr =
            writeBuffer.findPending(miss_mshr->blkAddr,
                                    miss_mshr->isSecure);
        if (conflict_mshr) {
            // not sure why we don't check order here... it was in the
            // original code but commented out.

            // The only way this happens is if we are
            // doing a write and we didn't have permissions
            // then subsequently saw a writeback (owned got evicted)
            // We need to make sure to perform the writeback first
            // To preserve the dirty data, then we can issue the write

            // should we return wq_entry here instead?  I.e. do we
            // have to flush writes in order?  I don't think so... not
            // for Alpha anyway.  Maybe for x86?
            return conflict_mshr;

            // @todo Note that we ignore the ready time of the conflict here
        }

        // No conflicts; issue read
        return miss_mshr;
    }

    // fall through... no pending requests.  Try a prefetch.
    assert(!miss_mshr && !wq_entry);
    if (prefetcher && mshrQueue.canPrefetch()) {
        // If we have a miss queue slot, we can try a prefetch
        PacketPtr pkt = prefetcher->getPacket();
        if (pkt) {
            Addr pf_addr = pkt->getBlockAddr(blkSize);
            if (!tags->findBlock(pf_addr, pkt->isSecure()) &&
                !mshrQueue.findMatch(pf_addr, pkt->isSecure()) &&
                !writeBuffer.findMatch(pf_addr, pkt->isSecure())) {
                // Update statistic on number of prefetches issued
                // (hwpf_mshr_misses)
                assert(pkt->req->masterId() < system->maxMasters());
                mshr_misses[pkt->cmdToIndex()][pkt->req->masterId()]++;

                // allocate an MSHR and return it, note
                // that we send the packet straight away, so do not
                // schedule the send
                return allocateMissBuffer(pkt, curTick(), false);
            } else {
                // free the request and packet
                delete pkt->req;
                delete pkt;
            }
        }
    }

    return nullptr;
}

bool
Cache::isCachedAbove(PacketPtr pkt, bool is_timing) const
{
    if (!forwardSnoops)
        return false;
    // Mirroring the flow of HardPFReqs, the cache sends CleanEvict and
    // Writeback snoops into upper level caches to check for copies of the
    // same block. Using the BLOCK_CACHED flag with the Writeback/CleanEvict
    // packet, the cache can inform the crossbar below of presence or absence
    // of the block.
    if (is_timing) {
        Packet snoop_pkt(pkt, true, false);
        snoop_pkt.setExpressSnoop();
        // Assert that packet is either Writeback or CleanEvict and not a
        // prefetch request because prefetch requests need an MSHR and may
        // generate a snoop response.
        assert(pkt->isEviction() || pkt->cmd == MemCmd::WriteClean);
        snoop_pkt.senderState = nullptr;
        cpuSidePort->sendTimingSnoopReq(&snoop_pkt);
        // Writeback/CleanEvict snoops do not generate a snoop response.
        assert(!(snoop_pkt.cacheResponding()));
        return snoop_pkt.isBlockCached();
    } else {
        cpuSidePort->sendAtomicSnoop(pkt);
        return pkt->isBlockCached();
    }
}

Tick
Cache::nextQueueReadyTime() const
{
    Tick nextReady = std::min(mshrQueue.nextReadyTime(),
                              writeBuffer.nextReadyTime());

    // Don't signal prefetch ready time if no MSHRs available
    // Will signal once enoguh MSHRs are deallocated
    if (prefetcher && mshrQueue.canPrefetch()) {
        nextReady = std::min(nextReady,
                             prefetcher->nextPrefetchReadyTime());
    }

    return nextReady;
}

bool
Cache::sendMSHRQueuePacket(MSHR* mshr)
{
    assert(mshr);

    // use request from 1st target
    PacketPtr tgt_pkt = mshr->getTarget()->pkt;

    DPRINTF(Cache, "%s: MSHR %s\n", __func__, tgt_pkt->print());

    CacheBlk *blk = tags->findBlock(mshr->blkAddr, mshr->isSecure);

    if (tgt_pkt->cmd == MemCmd::HardPFReq && forwardSnoops) {
        // we should never have hardware prefetches to allocated
        // blocks
        assert(blk == nullptr);

        // We need to check the caches above us to verify that
        // they don't have a copy of this block in the dirty state
        // at the moment. Without this check we could get a stale
        // copy from memory that might get used in place of the
        // dirty one.
        Packet snoop_pkt(tgt_pkt, true, false);
        snoop_pkt.setExpressSnoop();
        // We are sending this packet upwards, but if it hits we will
        // get a snoop response that we end up treating just like a
        // normal response, hence it needs the MSHR as its sender
        // state
        snoop_pkt.senderState = mshr;
        cpuSidePort->sendTimingSnoopReq(&snoop_pkt);

        // Check to see if the prefetch was squashed by an upper cache (to
        // prevent us from grabbing the line) or if a Check to see if a
        // writeback arrived between the time the prefetch was placed in
        // the MSHRs and when it was selected to be sent or if the
        // prefetch was squashed by an upper cache.

        // It is important to check cacheResponding before
        // prefetchSquashed. If another cache has committed to
        // responding, it will be sending a dirty response which will
        // arrive at the MSHR allocated for this request. Checking the
        // prefetchSquash first may result in the MSHR being
        // prematurely deallocated.
        if (snoop_pkt.cacheResponding()) {
            auto M5_VAR_USED r = outstandingSnoop.insert(snoop_pkt.req);
            assert(r.second);

            // if we are getting a snoop response with no sharers it
            // will be allocated as Modified
            bool pending_modified_resp = !snoop_pkt.hasSharers();
            markInService(mshr, pending_modified_resp);

            DPRINTF(Cache, "Upward snoop of prefetch for addr"
                    " %#x (%s) hit\n",
                    tgt_pkt->getAddr(), tgt_pkt->isSecure()? "s": "ns");
            return false;
        }

        if (snoop_pkt.isBlockCached()) {
            DPRINTF(Cache, "Block present, prefetch squashed by cache.  "
                    "Deallocating mshr target %#x.\n",
                    mshr->blkAddr);

            // Deallocate the mshr target
            if (mshrQueue.forceDeallocateTarget(mshr)) {
                // Clear block if this deallocation resulted freed an
                // mshr when all had previously been utilized
                clearBlocked(Blocked_NoMSHRs);
            }

            // given that no response is expected, delete Request and Packet
            delete tgt_pkt->req;
            delete tgt_pkt;

            return false;
        }
    }

    // either a prefetch that is not present upstream, or a normal
    // MSHR request, proceed to get the packet to send downstream
    PacketPtr pkt = createMissPacket(tgt_pkt, blk, mshr->needsWritable());

    mshr->isForward = (pkt == nullptr);

    if (mshr->isForward) {
        // not a cache block request, but a response is expected
        // make copy of current packet to forward, keep current
        // copy for response handling
        pkt = new Packet(tgt_pkt, false, true);
        assert(!pkt->isWrite());
    }

    // play it safe and append (rather than set) the sender state,
    // as forwarded packets may already have existing state
    pkt->pushSenderState(mshr);

    if (pkt->isClean() && blk && blk->isDirty()) {
        // A cache clean opearation is looking for a dirty block. Mark
        // the packet so that the destination xbar can determine that
        // there will be a follow-up write packet as well.
        pkt->setSatisfied();
    }

    if (!memSidePort->sendTimingReq(pkt)) {
        // we are awaiting a retry, but we
        // delete the packet and will be creating a new packet
        // when we get the opportunity
        delete pkt;

        // note that we have now masked any requestBus and
        // schedSendEvent (we will wait for a retry before
        // doing anything), and this is so even if we do not
        // care about this packet and might override it before
        // it gets retried
        return true;
    } else {
        // As part of the call to sendTimingReq the packet is
        // forwarded to all neighbouring caches (and any caches
        // above them) as a snoop. Thus at this point we know if
        // any of the neighbouring caches are responding, and if
        // so, we know it is dirty, and we can determine if it is
        // being passed as Modified, making our MSHR the ordering
        // point
        bool pending_modified_resp = !pkt->hasSharers() &&
            pkt->cacheResponding();
        markInService(mshr, pending_modified_resp);
        if (pkt->isClean() && blk && blk->isDirty()) {
            // A cache clean opearation is looking for a dirty
            // block. If a dirty block is encountered a WriteClean
            // will update any copies to the path to the memory
            // until the point of reference.
            DPRINTF(CacheVerbose, "%s: packet %s found block: %s\n",
                    __func__, pkt->print(), blk->print());
            PacketPtr wb_pkt = writecleanBlk(blk, pkt->req->getDest(),
                                             pkt->id);
            PacketList writebacks;
            writebacks.push_back(wb_pkt);
            doWritebacks(writebacks, 0);
        }

        return false;
    }
}

bool
Cache::sendWriteQueuePacket(WriteQueueEntry* wq_entry)
{
    assert(wq_entry);

    // always a single target for write queue entries
    PacketPtr tgt_pkt = wq_entry->getTarget()->pkt;

    DPRINTF(Cache, "%s: write %s\n", __func__, tgt_pkt->print());

    // forward as is, both for evictions and uncacheable writes
    if (!memSidePort->sendTimingReq(tgt_pkt)) {
        // note that we have now masked any requestBus and
        // schedSendEvent (we will wait for a retry before
        // doing anything), and this is so even if we do not
        // care about this packet and might override it before
        // it gets retried
        return true;
    } else {
        markInService(wq_entry);
        return false;
    }
}

void
Cache::serialize(CheckpointOut &cp) const
{
    bool dirty(isDirty());

    if (dirty) {
        warn("*** The cache still contains dirty data. ***\n");
        warn("    Make sure to drain the system using the correct flags.\n");
        warn("    This checkpoint will not restore correctly and dirty data "
             "    in the cache will be lost!\n");
    }

    // Since we don't checkpoint the data in the cache, any dirty data
    // will be lost when restoring from a checkpoint of a system that
    // wasn't drained properly. Flag the checkpoint as invalid if the
    // cache contains dirty data.
    bool bad_checkpoint(dirty);
    SERIALIZE_SCALAR(bad_checkpoint);
}

void
Cache::unserialize(CheckpointIn &cp)
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

AddrRangeList
Cache::CpuSidePort::getAddrRanges() const
{
    return cache->getAddrRanges();
}

bool
Cache::CpuSidePort::tryTiming(PacketPtr pkt)
{
    assert(!cache->system->bypassCaches());

    // always let express snoop packets through if even if blocked
    if (pkt->isExpressSnoop()) {
        return true;
    } else if (isBlocked() || mustSendRetry) {
        // either already committed to send a retry, or blocked
        mustSendRetry = true;
        return false;
    }
    mustSendRetry = false;
    return true;
}

bool
Cache::CpuSidePort::recvTimingReq(PacketPtr pkt)
{
    assert(!cache->system->bypassCaches());

    // always let express snoop packets through if even if blocked
    if (pkt->isExpressSnoop()) {
        bool M5_VAR_USED bypass_success = cache->recvTimingReq(pkt);
        assert(bypass_success);
        return true;
    }

    return tryTiming(pkt) && cache->recvTimingReq(pkt);
}

Tick
Cache::CpuSidePort::recvAtomic(PacketPtr pkt)
{
    return cache->recvAtomic(pkt);
}

void
Cache::CpuSidePort::recvFunctional(PacketPtr pkt)
{
    // functional request
    cache->functionalAccess(pkt, true);
}

Cache::
CpuSidePort::CpuSidePort(const std::string &_name, Cache *_cache,
                         const std::string &_label)
    : BaseCache::CacheSlavePort(_name, _cache, _label), cache(_cache)
{
}

Cache*
CacheParams::create()
{
    assert(tags);
    assert(replacement_policy);

    return new Cache(this);
}
///////////////
//
// MemSidePort
//
///////////////

bool
Cache::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    cache->recvTimingResp(pkt);
    return true;
}

// Express snooping requests to memside port
void
Cache::MemSidePort::recvTimingSnoopReq(PacketPtr pkt)
{
    // handle snooping requests
    cache->recvTimingSnoopReq(pkt);
}

Tick
Cache::MemSidePort::recvAtomicSnoop(PacketPtr pkt)
{
    return cache->recvAtomicSnoop(pkt);
}

void
Cache::MemSidePort::recvFunctionalSnoop(PacketPtr pkt)
{
    // functional snoop (note that in contrast to atomic we don't have
    // a specific functionalSnoop method, as they have the same
    // behaviour regardless)
    cache->functionalAccess(pkt, false);
}

void
Cache::CacheReqPacketQueue::sendDeferredPacket()
{
    // sanity check
    assert(!waitingOnRetry);

    // there should never be any deferred request packets in the
    // queue, instead we resly on the cache to provide the packets
    // from the MSHR queue or write queue
    assert(deferredPacketReadyTime() == MaxTick);

    // check for request packets (requests & writebacks)
    QueueEntry* entry = cache.getNextQueueEntry();

    if (!entry) {
        // can happen if e.g. we attempt a writeback and fail, but
        // before the retry, the writeback is eliminated because
        // we snoop another cache's ReadEx.
    } else {
        // let our snoop responses go first if there are responses to
        // the same addresses
        if (checkConflictingSnoop(entry->blkAddr)) {
            return;
        }
        waitingOnRetry = entry->sendPacket(cache);
    }

    // if we succeeded and are not waiting for a retry, schedule the
    // next send considering when the next queue is ready, note that
    // snoop responses have their own packet queue and thus schedule
    // their own events
    if (!waitingOnRetry) {
        schedSendEvent(cache.nextQueueReadyTime());
    }
}

Cache::
MemSidePort::MemSidePort(const std::string &_name, Cache *_cache,
                         const std::string &_label)
    : BaseCache::CacheMasterPort(_name, _cache, _reqQueue, _snoopRespQueue),
      _reqQueue(*_cache, *this, _snoopRespQueue, _label),
      _snoopRespQueue(*_cache, *this, _label), cache(_cache)
{
}
