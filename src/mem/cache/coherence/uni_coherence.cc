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

#include "mem/cache/coherence/uni_coherence.hh"
#include "mem/cache/base_cache.hh"

#include "base/trace.hh"

using namespace std;

UniCoherence::UniCoherence()
    : cshrs(50)
{
}

Packet *
UniCoherence::getPacket()
{
    bool unblock = cshrs.isFull();
    Packet* pkt = cshrs.getReq();
    cshrs.markInService((MSHR*)pkt->senderState);
    if (!cshrs.havePending()) {
        cache->clearSlaveRequest(Request_Coherence);
    }
    if (unblock) {
        //since CSHRs are always used as buffers, should always get rid of one
        assert(!cshrs.isFull());
        cache->clearBlocked(Blocked_Coherence);
    }
    return pkt;
}

/**
 * @todo add support for returning slave requests, not doing them here.
 */
bool
UniCoherence::handleBusRequest(Packet * &pkt, CacheBlk *blk, MSHR *mshr,
                               CacheBlk::State &new_state)
{
    new_state = 0;
    if (pkt->isInvalidate()) {
        DPRINTF(Cache, "snoop inval on blk %x (blk ptr %x)\n",
                pkt->getAddr(), blk);
        if (!cache->isTopLevel()) {
            // Forward to other caches
            Packet * tmp = new Packet(pkt->req, Packet::InvalidateReq, -1);
            cshrs.allocate(tmp);
            cache->setSlaveRequest(Request_Coherence, curTick);
            if (cshrs.isFull()) {
                cache->setBlockedForSnoop(Blocked_Coherence);
            }
        }
    } else {
        if (blk) {
            new_state = blk->status;
        }
    }
    return false;
}
