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

    while (i != end) {
        PacketPtr target = i->pkt;
        // If the target contains data, and it overlaps the
        // probed request, need to update data
        if (target->intersect(pkt)) {
            if (!fixPacket(pkt, target)) {
                // fixPacket returns true for continue, false for done
                return;
            }
        }

        i++;
    }
}

void
SimpleTimingPort::recvFunctional(PacketPtr pkt)
{
    checkFunctional(pkt);

    // Just do an atomic access and throw away the returned latency
    if (pkt->result != Packet::Success)
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
    assert(pkt->result != Packet::Nacked);
    Tick latency = recvAtomic(pkt);
    // turn packet around to go back to requester if response expected
    if (pkt->needsResponse()) {
        pkt->makeTimingResponse();
        sendTiming(pkt, latency);
    }
    else if (pkt->cmd != MemCmd::UpgradeReq) {
        delete pkt->req;
        delete pkt;
    }
    return true;
}

void
SimpleTimingPort::recvRetry()
{
    assert(!transmitList.empty());
    if (Port::sendTiming(transmitList.front().pkt)) {
        transmitList.pop_front();
        DPRINTF(Bus, "No Longer waiting on retry\n");
        if (!transmitList.empty()) {
            Tick time = transmitList.front().tick;
            sendEvent->schedule(time <= curTick ? curTick+1 : time);
        }
    }

    if (transmitList.empty() && drainEvent) {
        drainEvent->process();
        drainEvent = NULL;
    }
}

void
SimpleTimingPort::sendTiming(PacketPtr pkt, Tick time)
{
    // Nothing is on the list: add it and schedule an event
    if (transmitList.empty()) {
        assert(!sendEvent->scheduled());
        sendEvent->schedule(curTick+time);
        transmitList.push_back(DeferredPacket(time+curTick, pkt));
        return;
    }

    // something is on the list and this belongs at the end
    if (time+curTick >= transmitList.back().tick) {
        transmitList.push_back(DeferredPacket(time+curTick, pkt));
        return;
    }
    // Something is on the list and this belongs somewhere else
    DeferredPacketIterator i = transmitList.begin();
    DeferredPacketIterator end = transmitList.end();
    bool done = false;

    while (i != end && !done) {
        if (time+curTick < i->tick) {
            if (i == transmitList.begin()) {
                //Inserting at begining, reschedule
                sendEvent->reschedule(time+curTick);
            }
            transmitList.insert(i, DeferredPacket(time+curTick, pkt));
            done = true;
        }
        i++;
    }
    assert(done);
}

void
SimpleTimingPort::processSendEvent()
{
    assert(transmitList.size());
    assert(transmitList.front().tick <= curTick);
    if (Port::sendTiming(transmitList.front().pkt)) {
        //send successful, remove packet
        transmitList.pop_front();
        if (!transmitList.empty()) {
            Tick time = transmitList.front().tick;
            sendEvent->schedule(time <= curTick ? curTick+1 : time);
        }
        if (transmitList.empty() && drainEvent) {
            drainEvent->process();
            drainEvent = NULL;
        }
        return;
    }
    // send unsuccessful (due to flow control).  Will get retry
    // callback later; save for then if not already
    DPRINTF(Bus, "Waiting on retry\n");
}


unsigned int
SimpleTimingPort::drain(Event *de)
{
    if (transmitList.size() == 0)
        return 0;
    drainEvent = de;
    return 1;
}
