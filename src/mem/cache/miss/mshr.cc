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
}

void
MSHR::deallocate()
{
    assert(targets.empty());
    assert(ntargets == 0);
    inService = false;
    //allocIter = NULL;
    //readyIter = NULL;
}

/*
 * Adds a target to an MSHR
 */
void
MSHR::allocateTarget(PacketPtr target, bool cpuSide)
{
    //If we append an invalidate and we issued a read to the bus,
    //but now have some pending writes, we need to move
    //the invalidate to before the first non-read
    if (inService && !inServiceForExclusive && needsExclusive
        && !cpuSide && target->isInvalidate()) {
        std::list<Target> temp;

        while (!targets.empty()) {
            if (targets.front().pkt->needsExclusive()) break;
            //Place on top of temp stack
            temp.push_front(targets.front());
            //Remove from targets
            targets.pop_front();
        }

        //Now that we have all the reads off until first non-read, we can
        //place the invalidate on
        targets.push_front(Target(target, cpuSide));

        //Now we pop off the temp_stack and put them back
        while (!temp.empty()) {
            targets.push_front(temp.front());
            temp.pop_front();
        }
    }
    else {
        targets.push_back(Target(target, cpuSide));
    }

    ++ntargets;
    assert(targets.size() == ntargets);

    needsExclusive = needsExclusive || target->needsExclusive();
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
