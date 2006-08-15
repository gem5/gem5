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
 * Definition of the MSHRQueue.
 */

#include "mem/cache/miss/mshr_queue.hh"
#include "sim/eventq.hh"

using namespace std;

MSHRQueue::MSHRQueue(int num_mshrs, int reserve)
    : numMSHRs(num_mshrs + reserve - 1), numReserve(reserve)
{
    allocated = 0;
    inServiceMSHRs = 0;
    allocatedTargets = 0;
    registers = new MSHR[numMSHRs];
    for (int i = 0; i < numMSHRs; ++i) {
        freeList.push_back(&registers[i]);
    }
}

MSHRQueue::~MSHRQueue()
{
    delete [] registers;
}

MSHR*
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

MSHR*
MSHRQueue::findPending(Packet * &pkt) const
{
    MSHR::ConstIterator i = pendingList.begin();
    MSHR::ConstIterator end = pendingList.end();
    for (; i != end; ++i) {
        MSHR *mshr = *i;
        if (mshr->addr < pkt->getAddr()) {
            if (mshr->addr + mshr->pkt->getSize() > pkt->getAddr()) {
                return mshr;
            }
        } else {
            if (pkt->getAddr() + pkt->getSize() > mshr->addr) {
                return mshr;
            }
        }

        //need to check destination address for copies.
        //TEMP NOT DOING COPIES
#if 0
        if (mshr->pkt->cmd == Copy) {
            Addr dest = mshr->pkt->dest;
            if (dest < pkt->addr) {
                if (dest + mshr->pkt->size > pkt->addr) {
                    return mshr;
                }
            } else {
                if (pkt->addr + pkt->size > dest) {
                    return mshr;
                }
            }
        }
#endif
    }
    return NULL;
}

MSHR*
MSHRQueue::allocate(Packet * &pkt, int size)
{
    Addr aligned_addr = pkt->getAddr() & ~((Addr)size - 1);
    MSHR *mshr = freeList.front();
    assert(mshr->getNumTargets() == 0);
    freeList.pop_front();

    if (!pkt->needsResponse()) {
        mshr->allocateAsBuffer(pkt);
    } else {
        assert(size !=0);
        mshr->allocate(pkt->cmd, aligned_addr, size, pkt);
        allocatedTargets += 1;
    }
    mshr->allocIter = allocatedList.insert(allocatedList.end(), mshr);
    mshr->readyIter = pendingList.insert(pendingList.end(), mshr);

    allocated += 1;
    return mshr;
}

MSHR*
MSHRQueue::allocateFetch(Addr addr, int size, Packet * &target)
{
    MSHR *mshr = freeList.front();
    assert(mshr->getNumTargets() == 0);
    freeList.pop_front();
    mshr->allocate(Packet::ReadReq, addr, size, target);
    mshr->allocIter = allocatedList.insert(allocatedList.end(), mshr);
    mshr->readyIter = pendingList.insert(pendingList.end(), mshr);

    allocated += 1;
    return mshr;
}

MSHR*
MSHRQueue::allocateTargetList(Addr addr, int size)
{
    MSHR *mshr = freeList.front();
    assert(mshr->getNumTargets() == 0);
    freeList.pop_front();
    Packet * dummy;
    mshr->allocate(Packet::ReadReq, addr, size, dummy);
    mshr->allocIter = allocatedList.insert(allocatedList.end(), mshr);
    mshr->inService = true;
    ++inServiceMSHRs;
    ++allocated;
    return mshr;
}


void
MSHRQueue::deallocate(MSHR* mshr)
{
    deallocateOne(mshr);
}

MSHR::Iterator
MSHRQueue::deallocateOne(MSHR* mshr)
{
    MSHR::Iterator retval = allocatedList.erase(mshr->allocIter);
    freeList.push_front(mshr);
    allocated--;
    allocatedTargets -= mshr->getNumTargets();
    if (mshr->inService) {
        inServiceMSHRs--;
    } else {
        pendingList.erase(mshr->readyIter);
    }
    mshr->deallocate();
    return retval;
}

void
MSHRQueue::moveToFront(MSHR *mshr)
{
    if (!mshr->inService) {
        assert(mshr == *(mshr->readyIter));
        pendingList.erase(mshr->readyIter);
        mshr->readyIter = pendingList.insert(pendingList.begin(), mshr);
    }
}

void
MSHRQueue::markInService(MSHR* mshr)
{
    //assert(mshr == pendingList.front());
    if (!mshr->pkt->needsResponse()) {
        assert(mshr->getNumTargets() == 0);
        deallocate(mshr);
        return;
    }
    mshr->inService = true;
    pendingList.erase(mshr->readyIter);
    mshr->readyIter = NULL;
    inServiceMSHRs += 1;
    //pendingList.pop_front();
}

void
MSHRQueue::markPending(MSHR* mshr, Packet::Command cmd)
{
    assert(mshr->readyIter == NULL);
    mshr->pkt->cmd = cmd;
    mshr->pkt->flags &= ~SATISFIED;
    mshr->inService = false;
    --inServiceMSHRs;
    /**
     * @ todo might want to add rerequests to front of pending list for
     * performance.
     */
    mshr->readyIter = pendingList.insert(pendingList.end(), mshr);
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
                Packet * target = mshr->getTarget();
                mshr->popTarget();

                assert(target->req->getThreadNum() == threadNum);
                target = NULL;
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
