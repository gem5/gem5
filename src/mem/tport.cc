/*
 * Copyright (c) 2012 ARM Limited
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
 *          Andreas Hansson
 */

#include "debug/Bus.hh"
#include "mem/mem_object.hh"
#include "mem/tport.hh"

using namespace std;

SimpleTimingPort::SimpleTimingPort(const string &_name, MemObject *_owner,
                                   const string _label)
    : Port(_name, _owner), label(_label), sendEvent(this), drainEvent(NULL),
      waitingOnRetry(false)
{
}

SimpleTimingPort::~SimpleTimingPort()
{
}

bool
SimpleTimingPort::checkFunctional(PacketPtr pkt)
{
    pkt->pushLabel(label);

    DeferredPacketIterator i = transmitList.begin();
    DeferredPacketIterator end = transmitList.end();
    bool found = false;

    while (!found && i != end) {
        // If the buffered packet contains data, and it overlaps the
        // current packet, then update data
        found = pkt->checkFunctional(i->pkt);
        ++i;
    }

    pkt->popLabel();

    return found;
}

void
SimpleTimingPort::recvFunctional(PacketPtr pkt)
{
    if (!checkFunctional(pkt)) {
        // Just do an atomic access and throw away the returned latency
        recvAtomic(pkt);
    }
}

bool
SimpleTimingPort::recvTiming(PacketPtr pkt)
{
    // If the device is only a slave, it should only be sending
    // responses, which should never get nacked.  There used to be
    // code to hanldle nacks here, but I'm pretty sure it didn't work
    // correctly with the drain code, so that would need to be fixed
    // if we ever added it back.

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
        schedSendTiming(pkt, curTick() + latency);
    } else {
        delete pkt;
    }

    return true;
}

void
SimpleTimingPort::schedSendEvent(Tick when)
{
    // if we are waiting on a retry, do not schedule a send event, and
    // instead rely on retry being called
    if (waitingOnRetry) {
        assert(!sendEvent.scheduled());
        return;
    }

    if (!sendEvent.scheduled()) {
        owner->schedule(&sendEvent, when);
    } else if (sendEvent.when() > when) {
        owner->reschedule(&sendEvent, when);
    }
}

void
SimpleTimingPort::schedSendTiming(PacketPtr pkt, Tick when)
{
    assert(when > curTick());
    assert(when < curTick() + SimClock::Int::ms);

    // Nothing is on the list: add it and schedule an event
    if (transmitList.empty() || when < transmitList.front().tick) {
        transmitList.push_front(DeferredPacket(when, pkt));
        schedSendEvent(when);
        return;
    }

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

void SimpleTimingPort::trySendTiming()
{
    assert(deferredPacketReady());
    // take the next packet off the list here, as we might return to
    // ourselves through the sendTiming call below
    DeferredPacket dp = transmitList.front();
    transmitList.pop_front();

    // attempt to send the packet and remember the outcome
    waitingOnRetry = !sendTiming(dp.pkt);

    if (waitingOnRetry) {
        // put the packet back at the front of the list (packet should
        // not have changed since it wasn't accepted)
        assert(!sendEvent.scheduled());
        transmitList.push_front(dp);
    }
}

void
SimpleTimingPort::scheduleSend(Tick time)
{
    // the next ready time is either determined by the next deferred packet,
    // or in the cache through the MSHR ready time
    Tick nextReady = std::min(deferredPacketReadyTime(), time);
    if (nextReady != MaxTick) {
        // if the sendTiming caused someone else to call our
        // recvTiming we could already have an event scheduled, check
        if (!sendEvent.scheduled())
            owner->schedule(&sendEvent, std::max(nextReady, curTick() + 1));
    } else {
        // no more to send, so if we're draining, we may be done
        if (drainEvent && !sendEvent.scheduled()) {
            drainEvent->process();
            drainEvent = NULL;
        }
    }
}

void
SimpleTimingPort::sendDeferredPacket()
{
    // try to send what is on the list
    trySendTiming();

    // if we succeeded and are not waiting for a retry, schedule the
    // next send
    if (!waitingOnRetry) {
        scheduleSend();
    }
}


void
SimpleTimingPort::recvRetry()
{
    DPRINTF(Bus, "Received retry\n");
    // note that in the cache we get a retry even though we may not
    // have a packet to retry (we could potentially decide on a new
    // packet every time we retry)
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
    if (transmitList.empty() && !sendEvent.scheduled())
        return 0;
    drainEvent = de;
    return 1;
}
