
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
 */

/**
 * @file Definition of a simple bus bridge without buffering.
 */


#include "base/trace.hh"
#include "mem/bridge.hh"
#include "sim/builder.hh"

void
Bridge::init()
{
    // Make sure that both sides are connected to.
    if (sideA == NULL || sideB == NULL)
        panic("Both ports of bus bridge are not connected to a bus.\n");
}


/** Function called by the port when the bus is recieving a Timing
 * transaction.*/
bool
Bridge::recvTiming(Packet &pkt, Side id)
{
    if (blockedA && id == SideA)
        return false;
    if (blockedB && id == SideB)
        return false;

    if (delay) {
        if (!sendEvent.scheduled())
            sendEvent.schedule(curTick + delay);
        if (id == SideA) {
            inboundA.push_back(std::make_pair<Packet*, Tick>(&pkt, curTick));
            blockCheck(SideA);
        } else {
            inboundB.push_back(std::make_pair<Packet*, Tick>(&pkt, curTick));
            blockCheck(SideB);
        }
    } else {
        if (id == SideB) {
            sideA->sendPkt(pkt);
            blockCheck(SideB);
        } else {
            sideB->sendPkt(pkt);
            blockCheck(SideA);
        }
    }
    return true;

}

void
Bridge::blockCheck(Side id)
{
    /* Check that we still have buffer space available. */
    if (id == SideB) {
        if (sideA->numQueued() + inboundB.size() >= queueSizeA && !blockedB) {
            sideB->sendStatusChange(Port::Blocked);
            blockedB = true;
        } else if (sideA->numQueued() + inboundB.size() < queueSizeA && blockedB) {
            sideB->sendStatusChange(Port::Unblocked);
            blockedB = false;
        }
    } else {
        if (sideB->numQueued() + inboundA.size() >= queueSizeB && !blockedA) {
            sideA->sendStatusChange(Port::Blocked);
            blockedA = true;
        } else if (sideB->numQueued() + inboundA.size() < queueSizeB && blockedA) {
            sideA->sendStatusChange(Port::Unblocked);
            blockedA = false;
        }
    }
}

void Bridge::timerEvent()
{
    Tick t = 0;

    assert(inboundA.size() || inboundB.size());
    if (inboundA.size()) {
        while (inboundA.front().second <= curTick + delay){
            sideB->sendPkt(inboundA.front());
            inboundA.pop_front();
        }
        if (inboundA.size())
            t = inboundA.front().second + delay;
    }
    if (inboundB.size()) {
        while (inboundB.front().second <= curTick + delay){
            sideB->sendPkt(inboundA.front());
            inboundB.pop_front();
        }
        if (inboundB.size())
            if (t == 0)
               t = inboundB.front().second + delay;
            else
               t = std::min(t,inboundB.front().second + delay);
    } else {
        panic("timerEvent() called but nothing to do?");
    }

    if (t != 0)
        sendEvent.schedule(t);
}


void
Bridge::BridgePort::sendPkt(Packet &pkt)
{
    if (!sendTiming(pkt))
        outbound.push_back(std::make_pair<Packet*,Tick>(&pkt, curTick));
}

void
Bridge::BridgePort::sendPkt(std::pair<Packet*, Tick> p)
{
    if (!sendTiming(*p.first))
        outbound.push_back(p);
}


Packet *
Bridge::BridgePort::recvRetry()
{
    Packet *pkt;
    assert(outbound.size() > 0);
    assert(outbound.front().second >= curTick + bridge->delay);
    pkt = outbound.front().first;
    outbound.pop_front();
    bridge->blockCheck(side);
    return pkt;
}

/** Function called by the port when the bus is recieving a Atomic
 * transaction.*/
Tick
Bridge::recvAtomic(Packet &pkt, Side id)
{
    pkt.time += delay;

    if (id == SideA)
        return sideB->sendAtomic(pkt);
    else
        return sideA->sendAtomic(pkt);
}

/** Function called by the port when the bus is recieving a Functional
 * transaction.*/
void
Bridge::recvFunctional(Packet &pkt, Side id)
{
    pkt.time += delay;
    std::list<std::pair<Packet*, Tick> >::iterator i;
    bool pktContinue = true;

    for(i = inboundA.begin();  i != inboundA.end(); ++i) {
        if (pkt.intersect(i->first)) {
            pktContinue &= fixPacket(pkt, *i->first);
        }
    }

    for(i = inboundB.begin();  i != inboundB.end(); ++i) {
        if (pkt.intersect(i->first)) {
            pktContinue &= fixPacket(pkt, *i->first);
        }
    }

    for(i = sideA->outbound.begin();  i != sideA->outbound.end(); ++i) {
        if (pkt.intersect(i->first)) {
            pktContinue &= fixPacket(pkt, *i->first);
        }
    }

    for(i = sideB->outbound.begin();  i != sideB->outbound.end(); ++i) {
        if (pkt.intersect(i->first)) {
            pktContinue &= fixPacket(pkt, *i->first);
        }
    }

    if (pktContinue) {
        if (id == SideA)
            sideB->sendFunctional(pkt);
        else
            sideA->sendFunctional(pkt);
    }
}

/** Function called by the port when the bus is recieving a status change.*/
void
Bridge::recvStatusChange(Port::Status status, Side id)
{
    if (status == Port::Blocked || status == Port::Unblocked)
        return ;

    if (id == SideA)
        sideB->sendStatusChange(status);
    else
        sideA->sendStatusChange(status);
}

void
Bridge::addressRanges(AddrRangeList &resp, AddrRangeList &snoop, Side id)
{
    if (id == SideA)
        sideB->getPeerAddressRanges(resp, snoop);
    else
        sideA->getPeerAddressRanges(resp, snoop);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Bridge)

   Param<int> queue_size_a;
   Param<int> queue_size_b;
   Param<Tick> delay;
   Param<bool> write_ack;

END_DECLARE_SIM_OBJECT_PARAMS(Bridge)

BEGIN_INIT_SIM_OBJECT_PARAMS(Bridge)

    INIT_PARAM(queue_size_a, "The size of the queue for data coming into side a"),
    INIT_PARAM(queue_size_b, "The size of the queue for data coming into side b"),
    INIT_PARAM(delay, "The miminum delay to cross this bridge"),
    INIT_PARAM(write_ack, "Acknowledge any writes that are received.")

END_INIT_SIM_OBJECT_PARAMS(Bridge)

CREATE_SIM_OBJECT(Bridge)
{
    return new Bridge(getInstanceName(), queue_size_a, queue_size_b, delay,
            write_ack);
}

REGISTER_SIM_OBJECT("Bridge", Bridge)
