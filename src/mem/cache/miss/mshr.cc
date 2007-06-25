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
}

void
MSHR::allocate(Addr _addr, int _size, PacketPtr target)
{
    addr = _addr;
    size = _size;
    assert(target);
    isCacheFill = false;
    needsExclusive = target->needsExclusive();
    _isUncacheable = target->req->isUncacheable();
    inService = false;
    threadNum = 0;
    ntargets = 1;
    // Don't know of a case where we would allocate a new MSHR for a
    // snoop (mem0-side request), so set cpuSide to true here.
    targets.push_back(Target(target, true));
    assert(deferredTargets.empty());
    deferredNeedsExclusive = false;
    pendingInvalidate = false;
}

void
MSHR::deallocate()
{
    assert(targets.empty());
    assert(deferredTargets.empty());
    assert(ntargets == 0);
    inService = false;
    //allocIter = NULL;
    //readyIter = NULL;
}

/*
 * Adds a target to an MSHR
 */
void
MSHR::allocateTarget(PacketPtr target)
{
    if (inService) {
        if (!deferredTargets.empty() || pendingInvalidate ||
            (!needsExclusive && target->needsExclusive())) {
            // need to put on deferred list
            deferredTargets.push_back(Target(target, true));
            if (target->needsExclusive()) {
                deferredNeedsExclusive = true;
            }
        } else {
            // still OK to append to outstanding request
            targets.push_back(Target(target, true));
        }
    } else {
        if (target->needsExclusive()) {
            needsExclusive = true;
        }

        targets.push_back(Target(target, true));
    }

    ++ntargets;
}

void
MSHR::allocateSnoopTarget(PacketPtr target)
{
    assert(inService); // don't bother to call otherwise

    if (pendingInvalidate) {
        // a prior snoop has already appended an invalidation, so
        // logically we don't have the block anymore...
        return;
    }

    if (needsExclusive) {
        // We're awaiting an exclusive copy, so ownership is pending.
        // It's up to us to respond once the data arrives.
        target->assertMemInhibit();
    } else if (target->needsExclusive()) {
        // This transaction will take away our pending copy
        pendingInvalidate = true;
    } else {
        // If we're not going to supply data or perform an
        // invalidation, we don't need to save this.
        return;
    }

    targets.push_back(Target(target, false));
    ++ntargets;
}


bool
MSHR::promoteDeferredTargets()
{
    if (deferredTargets.empty()) {
        return false;
    }

    assert(targets.empty());
    targets = deferredTargets;
    deferredTargets.clear();
    assert(targets.size() == ntargets);

    needsExclusive = deferredNeedsExclusive;
    pendingInvalidate = false;
    deferredNeedsExclusive = false;

    return true;
}


void
MSHR::dump()
{
    ccprintf(cerr,
             "inService: %d thread: %d\n"
             "Addr: %x ntargets %d\n"
             "Targets:\n",
             inService, threadNum, addr, ntargets);

    TargetListIterator tar_it = targets.begin();
    for (int i = 0; i < ntargets; i++) {
        assert(tar_it != targets.end());

        ccprintf(cerr, "\t%d: Addr: %x cmd: %s\n",
                 i, tar_it->pkt->getAddr(), tar_it->pkt->cmdString());

        tar_it++;
    }
    ccprintf(cerr, "\n");
}

MSHR::~MSHR()
{
}
