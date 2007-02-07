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

PacketPtr
UniCoherence::getPacket()
{
    PacketPtr pkt = cshrs.getReq();
    return pkt;
}

void
UniCoherence::sendResult(PacketPtr &pkt, MSHR* cshr, bool success)
{
    if (success)
    {
        bool unblock = cshrs.isFull();
//        cshrs.markInService(cshr);
        delete pkt->req;
        cshrs.deallocate(cshr);
        if (!cshrs.havePending()) {
            cache->clearSlaveRequest(Request_Coherence);
        }
        if (unblock) {
            //since CSHRs are always used as buffers, should always get rid of one
            assert(!cshrs.isFull());
            cache->clearBlocked(Blocked_Coherence);
        }
    }
}


/**
 * @todo add support for returning slave requests, not doing them here.
 */
bool
UniCoherence::handleBusRequest(PacketPtr &pkt, CacheBlk *blk, MSHR *mshr,
                               CacheBlk::State &new_state)
{
    new_state = 0;
    if (pkt->isInvalidate()) {
            DPRINTF(Cache, "snoop inval on blk %x (blk ptr %x)\n",
                    pkt->getAddr(), blk);
    }
    else if (blk) {
        new_state = blk->status;
        if (pkt->isRead()) {
            DPRINTF(Cache, "Uni-coherence snoops a read that hit in itself"
                    ". Should satisfy the packet\n");
            return true; //Satisfy Reads if we can
        }
    }
    return false;
}

bool
UniCoherence::propogateInvalidate(PacketPtr pkt, bool isTiming)
{
    if (pkt->isInvalidate()) {
/*  Temp Fix for now, forward all invalidates up as functional accesses */
        if (isTiming) {
            // Forward to other caches
            Request* req = new Request(pkt->req->getPaddr(), pkt->getSize(), 0);
            PacketPtr tmp = new Packet(req, MemCmd::InvalidateReq, -1);
            cshrs.allocate(tmp);
            cache->setSlaveRequest(Request_Coherence, curTick);
            if (cshrs.isFull())
                cache->setBlockedForSnoop(Blocked_Coherence);
        }
        else {
            PacketPtr tmp = new Packet(pkt->req, MemCmd::InvalidateReq, -1);
            cache->cpuSidePort->sendAtomic(tmp);
            delete tmp;
        }
/**/
/*            PacketPtr tmp = new Packet(pkt->req, MemCmd::InvalidateReq, -1);
            cache->cpuSidePort->sendFunctional(tmp);
            delete tmp;
*/
    }
    if (pkt->isRead()) {
        /*For now we will see if someone above us has the data by
          doing a functional access on reads.  Fix this later */
            PacketPtr tmp = new Packet(pkt->req, MemCmd::ReadReq, -1);
            tmp->allocate();
            cache->cpuSidePort->sendFunctional(tmp);
            bool hit = (tmp->result == Packet::Success);
            if (hit) {
                memcpy(pkt->getPtr<uint8_t>(), tmp->getPtr<uint8_t>(),
                       pkt->getSize());
                DPRINTF(Cache, "Uni-coherence snoops a read that hit in L1\n");
            }
            delete tmp;
            return hit;
    }
    return false;
}
