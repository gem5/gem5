/*
 * Copyright (c) 2012-2013 ARM Limited
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
 */

/**
 * @file
 * Miss Status and Handling Register (MSHR) definitions.
 */

#include <algorithm>
#include <cassert>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/types.hh"
#include "debug/Cache.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/mshr.hh"
#include "sim/core.hh"

using namespace std;

MSHR::MSHR() : readyTime(0), _isUncacheable(false), downstreamPending(false),
               pendingDirty(false), postInvalidate(false),
               postDowngrade(false), queue(NULL), order(0), addr(0), size(0),
               isSecure(false), inService(false), isForward(false),
               threadNum(InvalidThreadID), data(NULL)
{
}


MSHR::TargetList::TargetList()
    : needsExclusive(false), hasUpgrade(false)
{}


inline void
MSHR::TargetList::add(PacketPtr pkt, Tick readyTime,
                      Counter order, Target::Source source, bool markPending)
{
    if (source != Target::FromSnoop) {
        if (pkt->needsExclusive()) {
            needsExclusive = true;
        }

        // StoreCondReq is effectively an upgrade if it's in an MSHR
        // since it would have been failed already if we didn't have a
        // read-only copy
        if (pkt->isUpgrade() || pkt->cmd == MemCmd::StoreCondReq) {
            hasUpgrade = true;
        }
    }

    if (markPending) {
        // Iterate over the SenderState stack and see if we find
        // an MSHR entry. If we do, set the downstreamPending
        // flag. Otherwise, do nothing.
        MSHR *mshr = pkt->findNextSenderState<MSHR>();
        if (mshr != NULL) {
            assert(!mshr->downstreamPending);
            mshr->downstreamPending = true;
        }
    }

    push_back(Target(pkt, readyTime, order, source, markPending));
}


static void
replaceUpgrade(PacketPtr pkt)
{
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
}


void
MSHR::TargetList::replaceUpgrades()
{
    if (!hasUpgrade)
        return;

    Iterator end_i = end();
    for (Iterator i = begin(); i != end_i; ++i) {
        replaceUpgrade(i->pkt);
    }

    hasUpgrade = false;
}


void
MSHR::TargetList::clearDownstreamPending()
{
    Iterator end_i = end();
    for (Iterator i = begin(); i != end_i; ++i) {
        if (i->markedPending) {
            // Iterate over the SenderState stack and see if we find
            // an MSHR entry. If we find one, clear the
            // downstreamPending flag by calling
            // clearDownstreamPending(). This recursively clears the
            // downstreamPending flag in all caches this packet has
            // passed through.
            MSHR *mshr = i->pkt->findNextSenderState<MSHR>();
            if (mshr != NULL) {
                mshr->clearDownstreamPending();
            }
        }
    }
}


bool
MSHR::TargetList::checkFunctional(PacketPtr pkt)
{
    Iterator end_i = end();
    for (Iterator i = begin(); i != end_i; ++i) {
        if (pkt->checkFunctional(i->pkt)) {
            return true;
        }
    }

    return false;
}


void
MSHR::TargetList::
print(std::ostream &os, int verbosity, const std::string &prefix) const
{
    ConstIterator end_i = end();
    for (ConstIterator i = begin(); i != end_i; ++i) {
        const char *s;
        switch (i->source) {
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
        i->pkt->print(os, verbosity, "");
    }
}


void
MSHR::allocate(Addr _addr, int _size, PacketPtr target, Tick whenReady,
               Counter _order)
{
    addr = _addr;
    size = _size;
    isSecure = target->isSecure();
    readyTime = whenReady;
    order = _order;
    assert(target);
    isForward = false;
    _isUncacheable = target->req->isUncacheable();
    inService = false;
    downstreamPending = false;
    threadNum = 0;
    assert(targets.isReset());
    // Don't know of a case where we would allocate a new MSHR for a
    // snoop (mem-side request), so set source according to request here
    Target::Source source = (target->cmd == MemCmd::HardPFReq) ?
        Target::FromPrefetcher : Target::FromCPU;
    targets.add(target, whenReady, _order, source, true);
    assert(deferredTargets.isReset());
    data = NULL;
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

bool
MSHR::markInService(PacketPtr pkt)
{
    assert(!inService);
    if (isForwardNoResponse()) {
        // we just forwarded the request packet & don't expect a
        // response, so get rid of it
        assert(getNumTargets() == 1);
        popTarget();
        return true;
    }
    inService = true;
    pendingDirty = (targets.needsExclusive ||
                    (!pkt->sharedAsserted() && pkt->memInhibitAsserted()));
    postInvalidate = postDowngrade = false;

    if (!downstreamPending) {
        // let upstream caches know that the request has made it to a
        // level where it's going to get a response
        targets.clearDownstreamPending();
    }
    return false;
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
MSHR::allocateTarget(PacketPtr pkt, Tick whenReady, Counter _order)
{
    // if there's a request already in service for this MSHR, we will
    // have to defer the new target until after the response if any of
    // the following are true:
    // - there are other targets already deferred
    // - there's a pending invalidate to be applied after the response
    //   comes back (but before this target is processed)
    // - this target requires an exclusive block and either we're not
    //   getting an exclusive block back or we have already snooped
    //   another read request that will downgrade our exclusive block
    //   to shared

    // assume we'd never issue a prefetch when we've got an
    // outstanding miss
    assert(pkt->cmd != MemCmd::HardPFReq);

    if (inService &&
        (!deferredTargets.empty() || hasPostInvalidate() ||
         (pkt->needsExclusive() &&
          (!isPendingDirty() || hasPostDowngrade() || isForward)))) {
        // need to put on deferred list
        if (hasPostInvalidate())
            replaceUpgrade(pkt);
        deferredTargets.add(pkt, whenReady, _order, Target::FromCPU, true);
    } else {
        // No request outstanding, or still OK to append to
        // outstanding request: append to regular target list.  Only
        // mark pending if current request hasn't been issued yet
        // (isn't in service).
        targets.add(pkt, whenReady, _order, Target::FromCPU, !inService);
    }
}

bool
MSHR::handleSnoop(PacketPtr pkt, Counter _order)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
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
        if (pkt->needsExclusive()) {
            targets.replaceUpgrades();
            deferredTargets.replaceUpgrades();
        }

        return false;
    }

    // From here on down, the request issued by this MSHR logically
    // precedes the request we're snooping.
    if (pkt->needsExclusive()) {
        // snooped request still precedes the re-request we'll have to
        // issue for deferred targets, if any...
        deferredTargets.replaceUpgrades();
    }

    if (hasPostInvalidate()) {
        // a prior snoop has already appended an invalidation, so
        // logically we don't have the block anymore; no need for
        // further snooping.
        return true;
    }

    if (isPendingDirty() || pkt->isInvalidate()) {
        // We need to save and replay the packet in two cases:
        // 1. We're awaiting an exclusive copy, so ownership is pending,
        //    and we need to respond after we receive data.
        // 2. It's an invalidation (e.g., UpgradeReq), and we need
        //    to forward the snoop up the hierarchy after the current
        //    transaction completes.
        
        // Actual target device (typ. a memory) will delete the
        // packet on reception, so we need to save a copy here.
        PacketPtr cp_pkt = new Packet(pkt, true);
        targets.add(cp_pkt, curTick(), _order, Target::FromSnoop,
                     downstreamPending && targets.needsExclusive);

        if (isPendingDirty()) {
            pkt->assertMemInhibit();
            pkt->setSupplyExclusive();
        }

        if (pkt->needsExclusive()) {
            // This transaction will take away our pending copy
            postInvalidate = true;
        }
    }

    if (!pkt->needsExclusive()) {
        // This transaction will get a read-shared copy, downgrading
        // our copy if we had an exclusive one
        postDowngrade = true;
        pkt->assertShared();
    }

    return true;
}


bool
MSHR::promoteDeferredTargets()
{
    assert(targets.empty());
    if (deferredTargets.empty()) {
        return false;
    }

    // swap targets & deferredTargets lists
    std::swap(targets, deferredTargets);

    // clear deferredTargets flags
    deferredTargets.resetFlags();

    order = targets.front().order;
    readyTime = std::max(curTick(), targets.front().readyTime);

    return true;
}


void
MSHR::handleFill(Packet *pkt, CacheBlk *blk)
{
    if (!pkt->sharedAsserted()
        && !(hasPostInvalidate() || hasPostDowngrade())
        && deferredTargets.needsExclusive) {
        // We got an exclusive response, but we have deferred targets
        // which are waiting to request an exclusive copy (not because
        // of a pending invalidate).  This can happen if the original
        // request was for a read-only (non-exclusive) block, but we
        // got an exclusive copy anyway because of the E part of the
        // MOESI/MESI protocol.  Since we got the exclusive copy
        // there's no need to defer the targets, so move them up to
        // the regular target list.
        assert(!targets.needsExclusive);
        targets.needsExclusive = true;
        // if any of the deferred targets were upper-level cache
        // requests marked downstreamPending, need to clear that
        assert(!downstreamPending);  // not pending here anymore
        deferredTargets.clearDownstreamPending();
        // this clears out deferredTargets too
        targets.splice(targets.end(), deferredTargets);
        deferredTargets.resetFlags();
    }
}


bool
MSHR::checkFunctional(PacketPtr pkt)
{
    // For printing, we treat the MSHR as a whole as single entity.
    // For other requests, we iterate over the individual targets
    // since that's where the actual data lies.
    if (pkt->isPrint()) {
        pkt->checkFunctional(this, addr, isSecure, size, NULL);
        return false;
    } else {
        return (targets.checkFunctional(pkt) ||
                deferredTargets.checkFunctional(pkt));
    }
}


void
MSHR::print(std::ostream &os, int verbosity, const std::string &prefix) const
{
    ccprintf(os, "%s[%x:%x](%s) %s %s %s state: %s %s %s %s %s\n",
             prefix, addr, addr+size-1,
             isSecure ? "s" : "ns",
             isForward ? "Forward" : "",
             isForwardNoResponse() ? "ForwNoResp" : "",
             needsExclusive() ? "Excl" : "",
             _isUncacheable ? "Unc" : "",
             inService ? "InSvc" : "",
             downstreamPending ? "DwnPend" : "",
             hasPostInvalidate() ? "PostInv" : "",
             hasPostDowngrade() ? "PostDowngr" : "");

    ccprintf(os, "%s  Targets:\n", prefix);
    targets.print(os, verbosity, prefix + "    ");
    if (!deferredTargets.empty()) {
        ccprintf(os, "%s  Deferred Targets:\n", prefix);
        deferredTargets.print(os, verbosity, prefix + "      ");
    }
}

std::string
MSHR::print() const
{
    ostringstream str;
    print(str);
    return str.str();
}
