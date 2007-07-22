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
 */

/**
 * @file
 * Miss Status and Handling Register (MSHR) definitions.
 */

#include <assert.h>
#include <string>
#include <vector>
#include <algorithm>

#include "mem/cache/miss/mshr.hh"
#include "sim/core.hh" // for curTick
#include "sim/host.hh"
#include "base/misc.hh"
#include "mem/cache/cache.hh"

using namespace std;

MSHR::MSHR()
{
    inService = false;
    ntargets = 0;
    threadNum = -1;
    targets = new TargetList();
    deferredTargets = new TargetList();
}


MSHR::TargetList::TargetList()
    : needsExclusive(false), hasUpgrade(false)
{}


inline void
MSHR::TargetList::add(PacketPtr pkt, Tick readyTime, Counter order, bool cpuSide)
{
    if (cpuSide) {
        if (pkt->needsExclusive()) {
            needsExclusive = true;
        }

        if (pkt->cmd == MemCmd::UpgradeReq) {
            hasUpgrade = true;
        }
    }

    push_back(Target(pkt, readyTime, order, cpuSide));
}


void
MSHR::TargetList::replaceUpgrades()
{
    if (!hasUpgrade)
        return;

    Iterator end_i = end();
    for (Iterator i = begin(); i != end_i; ++i) {
        if (i->pkt->cmd == MemCmd::UpgradeReq) {
            i->pkt->cmd = MemCmd::ReadExReq;
            DPRINTF(Cache, "Replacing UpgradeReq with ReadExReq\n");
        }
    }

    hasUpgrade = false;
}


void
MSHR::allocate(Addr _addr, int _size, PacketPtr target,
               Tick whenReady, Counter _order)
{
    addr = _addr;
    size = _size;
    readyTime = whenReady;
    order = _order;
    assert(target);
    isCacheFill = false;
    _isUncacheable = target->req->isUncacheable();
    inService = false;
    threadNum = 0;
    ntargets = 1;
    // Don't know of a case where we would allocate a new MSHR for a
    // snoop (mem-side request), so set cpuSide to true here.
    assert(targets->isReset());
    targets->add(target, whenReady, _order, true);
    assert(deferredTargets->isReset());
    pendingInvalidate = false;
    pendingShared = false;
    data = NULL;
}

void
MSHR::deallocate()
{
    assert(targets->empty());
    targets->resetFlags();
    assert(deferredTargets->isReset());
    assert(ntargets == 0);
    inService = false;
    //allocIter = NULL;
    //readyIter = NULL;
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
    // - the outstanding request is for a non-exclusive block and this
    //   target requires an exclusive block
    if (inService &&
        (!deferredTargets->empty() || pendingInvalidate ||
         (!targets->needsExclusive && pkt->needsExclusive()))) {
        // need to put on deferred list
        deferredTargets->add(pkt, whenReady, _order, true);
    } else {
        // no request outstanding, or still OK to append to
        // outstanding request
        targets->add(pkt, whenReady, _order, true);
    }

    ++ntargets;
}

bool
MSHR::handleSnoop(PacketPtr pkt, Counter _order)
{
    if (!inService || (pkt->isExpressSnoop() && !pkt->isDeferredSnoop())) {
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
            targets->replaceUpgrades();
            deferredTargets->replaceUpgrades();
        }

        return false;
    }

    // From here on down, the request issued by this MSHR logically
    // precedes the request we're snooping.

    if (pkt->needsExclusive()) {
        // snooped request still precedes the re-request we'll have to
        // issue for deferred targets, if any...
        deferredTargets->replaceUpgrades();
    }

    if (pendingInvalidate) {
        // a prior snoop has already appended an invalidation, so
        // logically we don't have the block anymore; no need for
        // further snooping.
        return true;
    }

    if (targets->needsExclusive || pkt->needsExclusive()) {
        // actual target device (typ. PhysicalMemory) will delete the
        // packet on reception, so we need to save a copy here
        targets->add(new Packet(pkt), curTick, _order, false);
        ++ntargets;

        if (targets->needsExclusive) {
            // We're awaiting an exclusive copy, so ownership is pending.
            // It's up to us to respond once the data arrives.
            pkt->assertMemInhibit();
        }

        if (pkt->needsExclusive()) {
            // This transaction will take away our pending copy
            pendingInvalidate = true;
        }
    } else {
        // Read to a read: no conflict, so no need to record as
        // target, but make sure neither reader thinks he's getting an
        // exclusive copy
        pendingShared = true;
        pkt->assertShared();
    }

    return true;
}


bool
MSHR::promoteDeferredTargets()
{
    assert(targets->empty());
    if (deferredTargets->empty()) {
        return false;
    }

    // swap targets & deferredTargets lists
    TargetList *tmp = targets;
    targets = deferredTargets;
    deferredTargets = tmp;

    assert(targets->size() == ntargets);

    // clear deferredTargets flags
    deferredTargets->resetFlags();

    pendingInvalidate = false;
    pendingShared = false;
    order = targets->front().order;
    readyTime = std::max(curTick, targets->front().readyTime);

    return true;
}


void
MSHR::handleFill(Packet *pkt, CacheBlk *blk)
{
    if (pendingShared) {
        // we snooped another read while this read was in
        // service... assert shared line on its behalf
        pkt->assertShared();
    }
}


void
MSHR::dump()
{
    ccprintf(cerr,
             "inService: %d thread: %d\n"
             "Addr: %x ntargets %d\n"
             "Targets:\n",
             inService, threadNum, addr, ntargets);
#if 0
    TargetListIterator tar_it = targets->begin();
    for (int i = 0; i < ntargets; i++) {
        assert(tar_it != targets->end());

        ccprintf(cerr, "\t%d: Addr: %x cmd: %s\n",
                 i, tar_it->pkt->getAddr(), tar_it->pkt->cmdString());

        tar_it++;
    }
#endif
    ccprintf(cerr, "\n");
}

MSHR::~MSHR()
{
}
