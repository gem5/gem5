/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#include "mem/tport.hh"

void
SimpleTimingPort::checkFunctional(PacketPtr pkt)
{
    DeferredPacketIterator i = transmitList.begin();
    DeferredPacketIterator end = transmitList.end();

    for (; i != end; ++i) {
        PacketPtr target = i->pkt;
        // If the target contains data, and it overlaps the
        // probed request, need to update data
        if (pkt->checkFunctional(target)) {
            return;
        }
    }
}

void
SimpleTimingPort::recvFunctional(PacketPtr pkt)
{
    checkFunctional(pkt);

    // Just do an atomic access and throw away the returned latency
    if (!pkt->isResponse())
        recvAtomic(pkt);
}

bool
SimpleTimingPort::recvTiming(PacketPtr pkt)
{
    // If the device is only a slave, it should only be sending
    // responses, which should never get nacked.  There used to be
    // code to hanldle nacks here, but I'm pretty sure it didn't work
    // correctly with the drain code, so that would need to be fixed
    // if we ever added it back.
    assert(pkt->isRequest());

    if (pkt->memInhibitAsserted()) {
        // snooper will supply based on copy of packet
        // still target's responsibility to delete packet
        delete pkt;
        return true;
    }

    bool needsResponse = pkt->needsResponse();
    Tick latency = recvAtomic(pkt);
    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());
        schedSendTiming(pkt, curTick + latency);
    } else {
        delete pkt->req;
        delete pkt;
    }

    return true;
}


void
SimpleTimingPort::schedSendTiming(PacketPtr pkt, Tick when)
{
    assert(when > curTick);

    // Nothing is on the list: add it and schedule an event
    if (transmitList.empty() || when < transmitList.front().tick) {
        transmitList.push_front(DeferredPacket(when, pkt));
        schedSendEvent(when);
        return;
    }

    // list is non-empty and this is not the head, so event should
    // already be scheduled
    assert(waitingOnRetry ||
           (sendEvent->scheduled() && sendEvent->when() <= when));

    // list is non-empty & this belongs at the end
    if (when >= transmitList.back().tick) {
        transmitList.push_back(DeferredPacket(when, pkt));
        return;
    }

    // this belongs in the middle somewhere
    DeferredPacketIterator i = transmitList.begin();
    i++; // already checked for insertion at front
    DeferredPacketIterator end = transmitList.end();

    for (; i != end; ++i) {
        if (when < i->tick) {
            transmitList.insert(i, DeferredPacket(when, pkt));
            return;
        }
    }
    assert(false); // should never get here
}


void
SimpleTimingPort::sendDeferredPacket()
{
    assert(deferredPacketReady());
    // take packet off list here; if recvTiming() on the other side
    // calls sendTiming() back on us (like SimpleTimingCpu does), then
    // we get confused by having a non-active packet on transmitList
    DeferredPacket dp = transmitList.front();
    transmitList.pop_front();
    bool success = sendTiming(dp.pkt);

    if (success) {
        if (!transmitList.empty() && !sendEvent->scheduled()) {
            Tick time = transmitList.front().tick;
            sendEvent->schedule(time <= curTick ? curTick+1 : time);
        }

        if (transmitList.empty() && drainEvent) {
            drainEvent->process();
            drainEvent = NULL;
        }
    } else {
        // Unsuccessful, need to put back on transmitList.  Callee
        // should not have messed with it (since it didn't accept that
        // packet), so we can just push it back on the front.
        assert(!sendEvent->scheduled());
        transmitList.push_front(dp);
    }

    waitingOnRetry = !success;

    if (waitingOnRetry) {
        DPRINTF(Bus, "Send failed, waiting on retry\n");
    }
}


void
SimpleTimingPort::recvRetry()
{
    DPRINTF(Bus, "Received retry\n");
    assert(waitingOnRetry);
    sendDeferredPacket();
}


void
SimpleTimingPort::processSendEvent()
{
    assert(!waitingOnRetry);
    sendDeferredPacket();
}


unsigned int
SimpleTimingPort::drain(Event *de)
{
    if (transmitList.size() == 0)
        return 0;
    drainEvent = de;
    return 1;
}
