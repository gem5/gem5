/*
 * Copyright (c) 2012-2013, 2015-2018 ARM Limited
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
 *          Nikos Nikoleris
 */

/**
 * @file
 * Miss Status and Handling Register (MSHR) definitions.
 */

#include "mem/cache/mshr.hh"

#include <cassert>
#include <string>

#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Cache.hh"
#include "mem/cache/base.hh"
#include "mem/request.hh"
#include "sim/core.hh"

MSHR::MSHR() : downstreamPending(false),
               pendingModified(false),
               postInvalidate(false), postDowngrade(false),
               wasWholeLineWrite(false), isForward(false)
{
}

MSHR::TargetList::TargetList()
    : needsWritable(false), hasUpgrade(false), allocOnFill(false),
      hasFromCache(false)
{}


void
MSHR::TargetList::updateFlags(PacketPtr pkt, Target::Source source,
                              bool alloc_on_fill)
{
    if (source != Target::FromSnoop) {
        if (pkt->needsWritable()) {
            needsWritable = true;
        }

        // StoreCondReq is effectively an upgrade if it's in an MSHR
        // since it would have been failed already if we didn't have a
        // read-only copy
        if (pkt->isUpgrade() || pkt->cmd == MemCmd::StoreCondReq) {
            hasUpgrade = true;
        }

        // potentially re-evaluate whether we should allocate on a fill or
        // not
        allocOnFill = allocOnFill || alloc_on_fill;

        if (source != Target::FromPrefetcher) {
            hasFromCache = hasFromCache || pkt->fromCache();

            updateWriteFlags(pkt);
        }
    }
}

void
MSHR::TargetList::populateFlags()
{
    resetFlags();
    for (auto& t: *this) {
        updateFlags(t.pkt, t.source, t.allocOnFill);
    }
}

inline void
MSHR::TargetList::add(PacketPtr pkt, Tick readyTime,
                      Counter order, Target::Source source, bool markPending,
                      bool alloc_on_fill)
{
    updateFlags(pkt, source, alloc_on_fill);
    if (markPending) {
        // Iterate over the SenderState stack and see if we find
        // an MSHR entry. If we do, set the downstreamPending
        // flag. Otherwise, do nothing.
        MSHR *mshr = pkt->findNextSenderState<MSHR>();
        if (mshr != nullptr) {
            assert(!mshr->downstreamPending);
            mshr->downstreamPending = true;
        } else {
            // No need to clear downstreamPending later
            markPending = false;
        }
    }

    emplace_back(pkt, readyTime, order, source, markPending, alloc_on_fill);
}


static void
replaceUpgrade(PacketPtr pkt)
{
    // remember if the current packet has data allocated
    bool has_data = pkt->hasData() || pkt->hasRespData();

    if (pkt->cmd == MemCmd::UpgradeReq) {
        pkt->cmd = MemCmd::ReadExReq;
        DPRINTF(Cache, "Replacing UpgradeReq with ReadExReq\n");
    } else if (pkt->cmd == MemCmd::SCUpgradeReq) {
        pkt->cmd = MemCmd::SCUpgradeFailReq;
        DPRINTF(Cache, "Replacing SCUpgradeReq with SCUpgradeFailReq\n");
    } else if (pkt->cmd == MemCmd::StoreCondReq) {
        pkt->cmd = MemCmd::StoreCondFailReq;
        DPRINTF(Cache, "Replacing StoreCondReq with StoreCondFailReq\n");
    }

    if (!has_data) {
        // there is no sensible way of setting the data field if the
        // new command actually would carry data
        assert(!pkt->hasData());

        if (pkt->hasRespData()) {
            // we went from a packet that had no data (neither request,
            // nor response), to one that does, and therefore we need to
            // actually allocate space for the data payload
            pkt->allocate();
        }
    }
}


void
MSHR::TargetList::replaceUpgrades()
{
    if (!hasUpgrade)
        return;

    for (auto& t : *this) {
        replaceUpgrade(t.pkt);
    }

    hasUpgrade = false;
}


void
MSHR::TargetList::clearDownstreamPending(MSHR::TargetList::iterator begin,
                                         MSHR::TargetList::iterator end)
{
    for (auto t = begin; t != end; t++) {
        if (t->markedPending) {
            // Iterate over the SenderState stack and see if we find
            // an MSHR entry. If we find one, clear the
            // downstreamPending flag by calling
            // clearDownstreamPending(). This recursively clears the
            // downstreamPending flag in all caches this packet has
            // passed through.
            MSHR *mshr = t->pkt->findNextSenderState<MSHR>();
            if (mshr != nullptr) {
                mshr->clearDownstreamPending();
            }
            t->markedPending = false;
        }
    }
}

void
MSHR::TargetList::clearDownstreamPending()
{
    clearDownstreamPending(begin(), end());
}


bool
MSHR::TargetList::trySatisfyFunctional(PacketPtr pkt)
{
    for (auto& t : *this) {
        if (pkt->trySatisfyFunctional(t.pkt)) {
            return true;
        }
    }

    return false;
}


void
MSHR::TargetList::print(std::ostream &os, int verbosity,
                        const std::string &prefix) const
{
    for (auto& t : *this) {
        const char *s;
        switch (t.source) {
          case Target::FromCPU:
            s = "FromCPU";
            break;
          case Target::FromSnoop:
            s = "FromSnoop";
            break;
          case Target::FromPrefetcher:
            s = "FromPrefetcher";
            break;
          default:
            s = "";
            break;
        }
        ccprintf(os, "%s%s: ", prefix, s);
        t.pkt->print(os, verbosity, "");
        ccprintf(os, "\n");
    }
}


void
MSHR::allocate(Addr blk_addr, unsigned blk_size, PacketPtr target,
               Tick when_ready, Counter _order, bool alloc_on_fill)
{
    blkAddr = blk_addr;
    blkSize = blk_size;
    isSecure = target->isSecure();
    readyTime = when_ready;
    order = _order;
    assert(target);
    isForward = false;
    wasWholeLineWrite = false;
    _isUncacheable = target->req->isUncacheable();
    inService = false;
    downstreamPending = false;

    targets.init(blkAddr, blkSize);
    deferredTargets.init(blkAddr, blkSize);

    // Don't know of a case where we would allocate a new MSHR for a
    // snoop (mem-side request), so set source according to request here
    Target::Source source = (target->cmd == MemCmd::HardPFReq) ?
        Target::FromPrefetcher : Target::FromCPU;
    targets.add(target, when_ready, _order, source, true, alloc_on_fill);
}


void
MSHR::clearDownstreamPending()
{
    assert(downstreamPending);
    downstreamPending = false;
    // recursively clear flag on any MSHRs we will be forwarding
    // responses to
    targets.clearDownstreamPending();
}

void
MSHR::markInService(bool pending_modified_resp)
{
    assert(!inService);

    inService = true;
    pendingModified = targets.needsWritable || pending_modified_resp;
    postInvalidate = postDowngrade = false;

    if (!downstreamPending) {
        // let upstream caches know that the request has made it to a
        // level where it's going to get a response
        targets.clearDownstreamPending();
    }
    // if the line is not considered a whole-line write when sent
    // downstream, make sure it is also not considered a whole-line
    // write when receiving the response, and vice versa
    wasWholeLineWrite = isWholeLineWrite();
}


void
MSHR::deallocate()
{
    assert(targets.empty());
    targets.resetFlags();
    assert(deferredTargets.isReset());
    inService = false;
}

/*
 * Adds a target to an MSHR
 */
void
MSHR::allocateTarget(PacketPtr pkt, Tick whenReady, Counter _order,
                     bool alloc_on_fill)
{
    // assume we'd never issue a prefetch when we've got an
    // outstanding miss
    assert(pkt->cmd != MemCmd::HardPFReq);

    // if there's a request already in service for this MSHR, we will
    // have to defer the new target until after the response if any of
    // the following are true:
    // - there are other targets already deferred
    // - there's a pending invalidate to be applied after the response
    //   comes back (but before this target is processed)
    // - the MSHR's first (and only) non-deferred target is a cache
    //   maintenance packet
    // - the new target is a cache maintenance packet (this is probably
    //   overly conservative but certainly safe)
    // - this target requires a writable block and either we're not
    //   getting a writable block back or we have already snooped
    //   another read request that will downgrade our writable block
    //   to non-writable (Shared or Owned)
    PacketPtr tgt_pkt = targets.front().pkt;
    if (pkt->req->isCacheMaintenance() ||
        tgt_pkt->req->isCacheMaintenance() ||
        !deferredTargets.empty() ||
        (inService &&
         (hasPostInvalidate() ||
          (pkt->needsWritable() &&
           (!isPendingModified() || hasPostDowngrade() || isForward))))) {
        // need to put on deferred list
        if (inService && hasPostInvalidate())
            replaceUpgrade(pkt);
        deferredTargets.add(pkt, whenReady, _order, Target::FromCPU, true,
                            alloc_on_fill);
    } else {
        // No request outstanding, or still OK to append to
        // outstanding request: append to regular target list.  Only
        // mark pending if current request hasn't been issued yet
        // (isn't in service).
        targets.add(pkt, whenReady, _order, Target::FromCPU, !inService,
                    alloc_on_fill);
    }
}

bool
MSHR::handleSnoop(PacketPtr pkt, Counter _order)
{
    DPRINTF(Cache, "%s for %s\n", __func__, pkt->print());

    // when we snoop packets the needsWritable and isInvalidate flags
    // should always be the same, however, this assumes that we never
    // snoop writes as they are currently not marked as invalidations
    panic_if((pkt->needsWritable() != pkt->isInvalidate()) &&
             !pkt->req->isCacheMaintenance(),
             "%s got snoop %s where needsWritable, "
             "does not match isInvalidate", name(), pkt->print());

    if (!inService || (pkt->isExpressSnoop() && downstreamPending)) {
        // Request has not been issued yet, or it's been issued
        // locally but is buffered unissued at some downstream cache
        // which is forwarding us this snoop.  Either way, the packet
        // we're snooping logically precedes this MSHR's request, so
        // the snoop has no impact on the MSHR, but must be processed
        // in the standard way by the cache.  The only exception is
        // that if we're an L2+ cache buffering an UpgradeReq from a
        // higher-level cache, and the snoop is invalidating, then our
        // buffered upgrades must be converted to read exclusives,
        // since the upper-level cache no longer has a valid copy.
        // That is, even though the upper-level cache got out on its
        // local bus first, some other invalidating transaction
        // reached the global bus before the upgrade did.
        if (pkt->needsWritable() || pkt->req->isCacheInvalidate()) {
            targets.replaceUpgrades();
            deferredTargets.replaceUpgrades();
        }

        return false;
    }

    // From here on down, the request issued by this MSHR logically
    // precedes the request we're snooping.
    if (pkt->needsWritable() || pkt->req->isCacheInvalidate()) {
        // snooped request still precedes the re-request we'll have to
        // issue for deferred targets, if any...
        deferredTargets.replaceUpgrades();
    }

    PacketPtr tgt_pkt = targets.front().pkt;
    if (hasPostInvalidate() || tgt_pkt->req->isCacheInvalidate()) {
        // a prior snoop has already appended an invalidation or a
        // cache invalidation operation is in progress, so logically
        // we don't have the block anymore; no need for further
        // snooping.
        return true;
    }

    if (isPendingModified() || pkt->isInvalidate()) {
        // We need to save and replay the packet in two cases:
        // 1. We're awaiting a writable copy (Modified or Exclusive),
        //    so this MSHR is the orgering point, and we need to respond
        //    after we receive data.
        // 2. It's an invalidation (e.g., UpgradeReq), and we need
        //    to forward the snoop up the hierarchy after the current
        //    transaction completes.

        // Start by determining if we will eventually respond or not,
        // matching the conditions checked in Cache::handleSnoop
        bool will_respond = isPendingModified() && pkt->needsResponse() &&
                      !pkt->isClean();

        // The packet we are snooping may be deleted by the time we
        // actually process the target, and we consequently need to
        // save a copy here. Clear flags and also allocate new data as
        // the original packet data storage may have been deleted by
        // the time we get to process this packet. In the cases where
        // we are not responding after handling the snoop we also need
        // to create a copy of the request to be on the safe side. In
        // the latter case the cache is responsible for deleting both
        // the packet and the request as part of handling the deferred
        // snoop.
        PacketPtr cp_pkt = will_respond ? new Packet(pkt, true, true) :
            new Packet(std::make_shared<Request>(*pkt->req), pkt->cmd,
                       blkSize, pkt->id);

        if (will_respond) {
            // we are the ordering point, and will consequently
            // respond, and depending on whether the packet
            // needsWritable or not we either pass a Shared line or a
            // Modified line
            pkt->setCacheResponding();

            // inform the cache hierarchy that this cache had the line
            // in the Modified state, even if the response is passed
            // as Shared (and thus non-writable)
            pkt->setResponderHadWritable();

            // in the case of an uncacheable request there is no need
            // to set the responderHadWritable flag, but since the
            // recipient does not care there is no harm in doing so
        }
        targets.add(cp_pkt, curTick(), _order, Target::FromSnoop,
                    downstreamPending && targets.needsWritable, false);

        if (pkt->needsWritable() || pkt->isInvalidate()) {
            // This transaction will take away our pending copy
            postInvalidate = true;
        }

        if (isPendingModified() && pkt->isClean()) {
            pkt->setSatisfied();
        }
    }

    if (!pkt->needsWritable() && !pkt->req->isUncacheable()) {
        // This transaction will get a read-shared copy, downgrading
        // our copy if we had a writable one
        postDowngrade = true;
        // make sure that any downstream cache does not respond with a
        // writable (and dirty) copy even if it has one, unless it was
        // explicitly asked for one
        pkt->setHasSharers();
    }

    return true;
}

MSHR::TargetList
MSHR::extractServiceableTargets(PacketPtr pkt)
{
    TargetList ready_targets;
    ready_targets.init(blkAddr, blkSize);
    // If the downstream MSHR got an invalidation request then we only
    // service the first of the FromCPU targets and any other
    // non-FromCPU target. This way the remaining FromCPU targets
    // issue a new request and get a fresh copy of the block and we
    // avoid memory consistency violations.
    if (pkt->cmd == MemCmd::ReadRespWithInvalidate) {
        auto it = targets.begin();
        assert((it->source == Target::FromCPU) ||
               (it->source == Target::FromPrefetcher));
        ready_targets.push_back(*it);
        it = targets.erase(it);
        while (it != targets.end()) {
            if (it->source == Target::FromCPU) {
                it++;
            } else {
                assert(it->source == Target::FromSnoop);
                ready_targets.push_back(*it);
                it = targets.erase(it);
            }
        }
        ready_targets.populateFlags();
    } else {
        std::swap(ready_targets, targets);
    }
    targets.populateFlags();

    return ready_targets;
}

bool
MSHR::promoteDeferredTargets()
{
    if (targets.empty() && deferredTargets.empty()) {
        // nothing to promote
        return false;
    }

    // the deferred targets can be generally promoted unless they
    // contain a cache maintenance request

    // find the first target that is a cache maintenance request
    auto it = std::find_if(deferredTargets.begin(), deferredTargets.end(),
                           [](MSHR::Target &t) {
                               return t.pkt->req->isCacheMaintenance();
                           });
    if (it == deferredTargets.begin()) {
        // if the first deferred target is a cache maintenance packet
        // then we can promote provided the targets list is empty and
        // we can service it on its own
        if (targets.empty()) {
            targets.splice(targets.end(), deferredTargets, it);
        }
    } else {
        // if a cache maintenance operation exists, we promote all the
        // deferred targets that precede it, or all deferred targets
        // otherwise
        targets.splice(targets.end(), deferredTargets,
                       deferredTargets.begin(), it);
    }

    deferredTargets.populateFlags();
    targets.populateFlags();
    order = targets.front().order;
    readyTime = std::max(curTick(), targets.front().readyTime);

    return true;
}

void
MSHR::promoteIf(const std::function<bool (Target &)>& pred)
{
    // if any of the deferred targets were upper-level cache
    // requests marked downstreamPending, need to clear that
    assert(!downstreamPending);  // not pending here anymore

    // find the first target does not satisfy the condition
    auto last_it = std::find_if_not(deferredTargets.begin(),
                                    deferredTargets.end(),
                                    pred);

    // for the prefix of the deferredTargets [begin(), last_it) clear
    // the downstreamPending flag and move them to the target list
    deferredTargets.clearDownstreamPending(deferredTargets.begin(),
                                           last_it);
    targets.splice(targets.end(), deferredTargets,
                   deferredTargets.begin(), last_it);
    // We need to update the flags for the target lists after the
    // modifications
    deferredTargets.populateFlags();
}

void
MSHR::promoteReadable()
{
    if (!deferredTargets.empty() && !hasPostInvalidate()) {
        // We got a non invalidating response, and we have the block
        // but we have deferred targets which are waiting and they do
        // not need writable. This can happen if the original request
        // was for a cache clean operation and we had a copy of the
        // block. Since we serviced the cache clean operation and we
        // have the block, there's no need to defer the targets, so
        // move them up to the regular target list.

        auto pred = [](Target &t) {
            assert(t.source == Target::FromCPU);
            return !t.pkt->req->isCacheInvalidate() &&
                   !t.pkt->needsWritable();
        };
        promoteIf(pred);
    }
}

void
MSHR::promoteWritable()
{
    if (deferredTargets.needsWritable &&
        !(hasPostInvalidate() || hasPostDowngrade())) {
        // We got a writable response, but we have deferred targets
        // which are waiting to request a writable copy (not because
        // of a pending invalidate).  This can happen if the original
        // request was for a read-only block, but we got a writable
        // response anyway. Since we got the writable copy there's no
        // need to defer the targets, so move them up to the regular
        // target list.
        assert(!targets.needsWritable);
        targets.needsWritable = true;

        auto pred = [](Target &t) {
            assert(t.source == Target::FromCPU);
            return !t.pkt->req->isCacheInvalidate();
        };

        promoteIf(pred);
    }
}


bool
MSHR::trySatisfyFunctional(PacketPtr pkt)
{
    // For printing, we treat the MSHR as a whole as single entity.
    // For other requests, we iterate over the individual targets
    // since that's where the actual data lies.
    if (pkt->isPrint()) {
        pkt->trySatisfyFunctional(this, blkAddr, isSecure, blkSize, nullptr);
        return false;
    } else {
        return (targets.trySatisfyFunctional(pkt) ||
                deferredTargets.trySatisfyFunctional(pkt));
    }
}

bool
MSHR::sendPacket(BaseCache &cache)
{
    return cache.sendMSHRQueuePacket(this);
}

void
MSHR::print(std::ostream &os, int verbosity, const std::string &prefix) const
{
    ccprintf(os, "%s[%#llx:%#llx](%s) %s %s %s state: %s %s %s %s %s %s\n",
             prefix, blkAddr, blkAddr + blkSize - 1,
             isSecure ? "s" : "ns",
             isForward ? "Forward" : "",
             allocOnFill() ? "AllocOnFill" : "",
             needsWritable() ? "Wrtbl" : "",
             _isUncacheable ? "Unc" : "",
             inService ? "InSvc" : "",
             downstreamPending ? "DwnPend" : "",
             postInvalidate ? "PostInv" : "",
             postDowngrade ? "PostDowngr" : "",
             hasFromCache() ? "HasFromCache" : "");

    if (!targets.empty()) {
        ccprintf(os, "%s  Targets:\n", prefix);
        targets.print(os, verbosity, prefix + "    ");
    }
    if (!deferredTargets.empty()) {
        ccprintf(os, "%s  Deferred Targets:\n", prefix);
        deferredTargets.print(os, verbosity, prefix + "      ");
    }
}

std::string
MSHR::print() const
{
    std::ostringstream str;
    print(str);
    return str.str();
}
