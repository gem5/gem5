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
 */

/** @file
 * Definition of MSHRQueue class functions.
 */

#include "mem/cache/miss/mshr_queue.hh"

using namespace std;

MSHRQueue::MSHRQueue(int num_entries, int reserve, int _index)
    : numEntries(num_entries + reserve - 1), numReserve(reserve),
      index(_index)
{
    allocated = 0;
    inServiceEntries = 0;
    registers = new MSHR[numEntries];
    for (int i = 0; i < numEntries; ++i) {
        registers[i].queue = this;
        freeList.push_back(&registers[i]);
    }
}

MSHRQueue::~MSHRQueue()
{
    delete [] registers;
}

MSHR *
MSHRQueue::findMatch(Addr addr) const
{
    MSHR::ConstIterator i = allocatedList.begin();
    MSHR::ConstIterator end = allocatedList.end();
    for (; i != end; ++i) {
        MSHR *mshr = *i;
        if (mshr->addr == addr) {
            return mshr;
        }
    }
    return NULL;
}

bool
MSHRQueue::findMatches(Addr addr, vector<MSHR*>& matches) const
{
    // Need an empty vector
    assert(matches.empty());
    bool retval = false;
    MSHR::ConstIterator i = allocatedList.begin();
    MSHR::ConstIterator end = allocatedList.end();
    for (; i != end; ++i) {
        MSHR *mshr = *i;
        if (mshr->addr == addr) {
            retval = true;
            matches.push_back(mshr);
        }
    }
    return retval;

}

MSHR *
MSHRQueue::findPending(Addr addr, int size) const
{
    MSHR::ConstIterator i = readyList.begin();
    MSHR::ConstIterator end = readyList.end();
    for (; i != end; ++i) {
        MSHR *mshr = *i;
        if (mshr->addr < addr) {
            if (mshr->addr + mshr->size > addr) {
                return mshr;
            }
        } else {
            if (addr + size > mshr->addr) {
                return mshr;
            }
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

    MSHR::Iterator i = readyList.begin();
    MSHR::Iterator end = readyList.end();
    for (; i != end; ++i) {
        if ((*i)->readyTime > mshr->readyTime) {
            return readyList.insert(i, mshr);
        }
    }
    assert(false);
    return end;  // keep stupid compilers happy
}


MSHR *
MSHRQueue::allocate(Addr addr, int size, PacketPtr &pkt,
                    Tick when, Counter order)
{
    assert(!freeList.empty());
    MSHR *mshr = freeList.front();
    assert(mshr->getNumTargets() == 0);
    freeList.pop_front();

    mshr->allocate(addr, size, pkt, when, order);
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
MSHRQueue::markInService(MSHR *mshr)
{
    assert(!mshr->inService);
    if (mshr->isSimpleForward()) {
        // we just forwarded the request packet & don't expect a
        // response, so get rid of it
        assert(mshr->getNumTargets() == 1);
        mshr->popTarget();
        deallocate(mshr);
        return;
    }
    mshr->inService = true;
    readyList.erase(mshr->readyIter);
    //mshr->readyIter = NULL;
    inServiceEntries += 1;
    //readyList.pop_front();
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

void
MSHRQueue::squash(int threadNum)
{
    MSHR::Iterator i = allocatedList.begin();
    MSHR::Iterator end = allocatedList.end();
    for (; i != end;) {
        MSHR *mshr = *i;
        if (mshr->threadNum == threadNum) {
            while (mshr->hasTargets()) {
                mshr->popTarget();
                assert(0/*target->req->getThreadNum()*/ == threadNum);
            }
            assert(!mshr->hasTargets());
            assert(mshr->ntargets==0);
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
