/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
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
 * Authors: Andreas Hansson
 */

#include "debug/BusAddrRanges.hh"
#include "mem/physical.hh"

using namespace std;

PhysicalMemory::PhysicalMemory(const vector<AbstractMemory*>& _memories) :
    size(0)
{
    for (vector<AbstractMemory*>::const_iterator m = _memories.begin();
         m != _memories.end(); ++m) {
        // only add the memory if it is part of the global address map
        if ((*m)->isInAddrMap()) {
            memories.push_back(*m);

            // calculate the total size once and for all
            size += (*m)->size();

            // add the range to our interval tree and make sure it does not
            // intersect an existing range
            if (addrMap.insert((*m)->getAddrRange(), *m) == addrMap.end())
                fatal("Memory address range for %s is overlapping\n",
                      (*m)->name());
        }
        DPRINTF(BusAddrRanges,
                "Skipping memory %s that is not in global address map\n",
                (*m)->name());
    }
    rangeCache.invalidate();
}

bool
PhysicalMemory::isMemAddr(Addr addr) const
{
    // see if the address is within the last matched range
    if (addr != rangeCache) {
        // lookup in the interval tree
        range_map<Addr, AbstractMemory*>::const_iterator r =
            addrMap.find(addr);
        if (r == addrMap.end()) {
            // not in the cache, and not in the tree
            return false;
        }
        // the range is in the tree, update the cache
        rangeCache = r->first;
    }

    assert(addrMap.find(addr) != addrMap.end());

    // either matched the cache or found in the tree
    return true;
}

AddrRangeList
PhysicalMemory::getConfAddrRanges() const
{
    // this could be done once in the constructor, but since it is unlikely to
    // be called more than once the iteration should not be a problem
    AddrRangeList ranges;
    for (vector<AbstractMemory*>::const_iterator m = memories.begin();
         m != memories.end(); ++m) {
        if ((*m)->isConfReported()) {
            ranges.push_back((*m)->getAddrRange());
        }
    }

    return ranges;
}

void
PhysicalMemory::access(PacketPtr pkt)
{
    assert(pkt->isRequest());
    Addr addr = pkt->getAddr();
    range_map<Addr, AbstractMemory*>::const_iterator m = addrMap.find(addr);
    assert(m != addrMap.end());
    m->second->access(pkt);
}

void
PhysicalMemory::functionalAccess(PacketPtr pkt)
{
    assert(pkt->isRequest());
    Addr addr = pkt->getAddr();
    range_map<Addr, AbstractMemory*>::const_iterator m = addrMap.find(addr);
    assert(m != addrMap.end());
    m->second->functionalAccess(pkt);
}
