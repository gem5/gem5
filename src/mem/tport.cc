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
SimpleTimingPort::recvFunctional(PacketPtr pkt)
{
    std::list<std::pair<Tick,PacketPtr> >::iterator i = transmitList.begin();
    std::list<std::pair<Tick,PacketPtr> >::iterator end = transmitList.end();
    bool done = false;

    while (i != end && !done) {
        PacketPtr target = i->second;
        // If the target contains data, and it overlaps the
        // probed request, need to update data
        if (target->intersect(pkt))
            done = fixPacket(pkt, target);

    }

    //Then just do an atomic access and throw away the returned latency
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
    else {
        if (pkt->cmd != Packet::UpgradeReq)
        {
            delete pkt->req;
            delete pkt;
        }
    }
    return true;
}

void
SimpleTimingPort::recvRetry()
{
    assert(!transmitList.empty());
    if (Port::sendTiming(transmitList.front().second)) {
        transmitList.pop_front();
        DPRINTF(Bus, "No Longer waiting on retry\n");
        if (!transmitList.empty()) {
            Tick time = transmitList.front().first;
            sendEvent.schedule(time <= curTick ? curTick+1 : time);
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
    if (transmitList.empty()) {
        assert(!sendEvent.scheduled());
        sendEvent.schedule(curTick+time);
    }
    transmitList.push_back(std::pair<Tick,PacketPtr>(time+curTick,pkt));
}

void
SimpleTimingPort::SendEvent::process()
{
    assert(port->transmitList.size());
    assert(port->transmitList.front().first <= curTick);
    if (port->Port::sendTiming(port->transmitList.front().second)) {
        //send successful, remove packet
        port->transmitList.pop_front();
        if (!port->transmitList.empty()) {
            Tick time = port->transmitList.front().first;
            schedule(time <= curTick ? curTick+1 : time);
        }
        if (port->transmitList.empty() && port->drainEvent) {
            port->drainEvent->process();
            port->drainEvent = NULL;
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
