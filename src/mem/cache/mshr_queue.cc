/*
 * Copyright (c) 2012-2013, 2015 ARM Limited
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
 *          Andreas Sandberg
 */

/** @file
 * Definition of MSHRQueue class functions.
 */

#include "base/trace.hh"
#include "mem/cache/mshr_queue.hh"
#include "debug/Drain.hh"

using namespace std;

MSHRQueue::MSHRQueue(const std::string &_label,
                     int num_entries, int reserve, int demand_reserve,
                     int _index)
    : label(_label), numEntries(num_entries + reserve - 1),
      numReserve(reserve), demandReserve(demand_reserve),
      registers(numEntries), allocated(0),
      inServiceEntries(0), index(_index)
{
    for (int i = 0; i < numEntries; ++i) {
        registers[i].queue = this;
        freeList.push_back(&registers[i]);
    }
}

MSHR *
MSHRQueue::findMatch(Addr blk_addr, bool is_secure) const
{
    for (const auto& mshr : allocatedList) {
        // we ignore any MSHRs allocated for uncacheable accesses and
        // simply ignore them when matching, in the cache we never
        // check for matches when adding new uncacheable entries, and
        // we do not want normal cacheable accesses being added to an
        // MSHR serving an uncacheable access
        if (!mshr->isUncacheable() && mshr->blkAddr == blk_addr &&
            mshr->isSecure == is_secure) {
            return mshr;
        }
    }
    return NULL;
}

bool
MSHRQueue::findMatches(Addr blk_addr, bool is_secure,
                       vector<MSHR*>& matches) const
{
    // Need an empty vector
    assert(matches.empty());
    bool retval = false;
    for (const auto& mshr : allocatedList) {
        if (!mshr->isUncacheable() && mshr->blkAddr == blk_addr &&
            mshr->isSecure == is_secure) {
            retval = true;
            matches.push_back(mshr);
        }
    }
    return retval;
}


bool
MSHRQueue::checkFunctional(PacketPtr pkt, Addr blk_addr)
{
    pkt->pushLabel(label);
    for (const auto& mshr : allocatedList) {
        if (mshr->blkAddr == blk_addr && mshr->checkFunctional(pkt)) {
            pkt->popLabel();
            return true;
        }
    }
    pkt->popLabel();
    return false;
}


MSHR *
MSHRQueue::findPending(Addr blk_addr, bool is_secure) const
{
    for (const auto& mshr : readyList) {
        if (mshr->blkAddr == blk_addr && mshr->isSecure == is_secure) {
            return mshr;
        }
    }
    return NULL;
}


MSHR::Iterator
MSHRQueue::addToReadyList(MSHR *mshr)
{
    if (readyList.empty() || readyList.back()->readyTime <= mshr->readyTime) {
        return readyList.insert(readyList.end(), mshr);
    }

    for (auto i = readyList.begin(); i != readyList.end(); ++i) {
        if ((*i)->readyTime > mshr->readyTime) {
            return readyList.insert(i, mshr);
        }
    }
    assert(false);
    return readyList.end();  // keep stupid compilers happy
}


MSHR *
MSHRQueue::allocate(Addr blk_addr, unsigned blk_size, PacketPtr pkt,
                    Tick when_ready, Counter order)
{
    assert(!freeList.empty());
    MSHR *mshr = freeList.front();
    assert(mshr->getNumTargets() == 0);
    freeList.pop_front();

    mshr->allocate(blk_addr, blk_size, pkt, when_ready, order);
    mshr->allocIter = allocatedList.insert(allocatedList.end(), mshr);
    mshr->readyIter = addToReadyList(mshr);

    allocated += 1;
    return mshr;
}


void
MSHRQueue::deallocate(MSHR *mshr)
{
    deallocateOne(mshr);
}

MSHR::Iterator
MSHRQueue::deallocateOne(MSHR *mshr)
{
    MSHR::Iterator retval = allocatedList.erase(mshr->allocIter);
    freeList.push_front(mshr);
    allocated--;
    if (mshr->inService) {
        inServiceEntries--;
    } else {
        readyList.erase(mshr->readyIter);
    }
    mshr->deallocate();
    if (drainState() == DrainState::Draining && allocated == 0) {
        // Notify the drain manager that we have completed draining if
        // there are no other outstanding requests in this MSHR queue.
        DPRINTF(Drain, "MSHRQueue now empty, signalling drained\n");
        signalDrainDone();
    }
    return retval;
}

void
MSHRQueue::moveToFront(MSHR *mshr)
{
    if (!mshr->inService) {
        assert(mshr == *(mshr->readyIter));
        readyList.erase(mshr->readyIter);
        mshr->readyIter = readyList.insert(readyList.begin(), mshr);
    }
}

void
MSHRQueue::markInService(MSHR *mshr, bool pending_dirty_resp)
{
    if (mshr->markInService(pending_dirty_resp)) {
        deallocate(mshr);
    } else {
        readyList.erase(mshr->readyIter);
        inServiceEntries += 1;
    }
}

void
MSHRQueue::markPending(MSHR *mshr)
{
    assert(mshr->inService);
    mshr->inService = false;
    --inServiceEntries;
    /**
     * @ todo might want to add rerequests to front of pending list for
     * performance.
     */
    mshr->readyIter = addToReadyList(mshr);
}

bool
MSHRQueue::forceDeallocateTarget(MSHR *mshr)
{
    bool was_full = isFull();
    assert(mshr->hasTargets());
    // Pop the prefetch off of the target list
    mshr->popTarget();
    // Delete mshr if no remaining targets
    if (!mshr->hasTargets() && !mshr->promoteDeferredTargets()) {
        deallocateOne(mshr);
    }

    // Notify if MSHR queue no longer full
    return was_full && !isFull();
}

void
MSHRQueue::squash(int threadNum)
{
    for (auto i = allocatedList.begin(); i != allocatedList.end();) {
        MSHR *mshr = *i;
        if (mshr->threadNum == threadNum) {
            while (mshr->hasTargets()) {
                mshr->popTarget();
                assert(0/*target->req->threadId()*/ == threadNum);
            }
            assert(!mshr->hasTargets());
            assert(mshr->getNumTargets()==0);
            if (!mshr->inService) {
                i = deallocateOne(mshr);
            } else {
                //mshr->pkt->flags &= ~CACHE_LINE_FILL;
                ++i;
            }
        } else {
            ++i;
        }
    }
}

DrainState
MSHRQueue::drain()
{
    return allocated == 0 ? DrainState::Drained : DrainState::Draining;
}
