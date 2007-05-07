
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
 *          Steve Reinhardt
 */

/**
 * @file
 * Definition of a simple bus bridge without buffering.
 */

#include <algorithm>

#include "base/trace.hh"
#include "mem/bridge.hh"
#include "sim/builder.hh"

Bridge::BridgePort::BridgePort(const std::string &_name,
                               Bridge *_bridge, BridgePort *_otherPort,
                               int _delay, int _queueLimit,
                               bool fix_partial_write)
    : Port(_name), bridge(_bridge), otherPort(_otherPort),
      delay(_delay), fixPartialWrite(fix_partial_write),
      outstandingResponses(0), queuedRequests(0),
      queueLimit(_queueLimit), sendEvent(this)
{
}

Bridge::Bridge(const std::string &n, int qsa, int qsb,
               Tick _delay, int write_ack, bool fix_partial_write_a,
               bool fix_partial_write_b)
    : MemObject(n),
      portA(n + "-portA", this, &portB, _delay, qsa, fix_partial_write_a),
      portB(n + "-portB", this, &portA, _delay, qsa, fix_partial_write_b),
      ackWrites(write_ack)
{
    if (ackWrites)
        panic("No support for acknowledging writes\n");
}

Port *
Bridge::getPort(const std::string &if_name, int idx)
{
    BridgePort *port;

    if (if_name == "side_a")
        port = &portA;
    else if (if_name == "side_b")
        port = &portB;
    else
        return NULL;

    if (port->getPeer() != NULL)
        panic("bridge side %s already connected to.", if_name);
    return port;
}


void
Bridge::init()
{
    // Make sure that both sides are connected to.
    if (portA.getPeer() == NULL || portB.getPeer() == NULL)
        fatal("Both ports of bus bridge are not connected to a bus.\n");

    if (portA.peerBlockSize() != portB.peerBlockSize())
        fatal("Busses don't have the same block size... Not supported.\n");
}

bool
Bridge::BridgePort::queueFull()
{
    // use >= here because sendQueue could get larger because of
    // nacks getting inserted
    return queuedRequests + outstandingResponses >= queueLimit;
}

/** Function called by the port when the bus is receiving a Timing
 * transaction.*/
bool
Bridge::BridgePort::recvTiming(PacketPtr pkt)
{
    if (!(pkt->flags & SNOOP_COMMIT))
        return true;


    DPRINTF(BusBridge, "recvTiming: src %d dest %d addr 0x%x\n",
                pkt->getSrc(), pkt->getDest(), pkt->getAddr());

    if (pkt->isRequest() && otherPort->queueFull()) {
        DPRINTF(BusBridge, "Remote queue full, nacking\n");
        nackRequest(pkt);
        return true;
    }

    if (pkt->needsResponse() && pkt->result != Packet::Nacked)
        if (queueFull()) {
            DPRINTF(BusBridge, "Local queue full, no space for response, nacking\n");
            DPRINTF(BusBridge, "queue size: %d outreq: %d outstanding resp: %d\n",
                    sendQueue.size(), queuedRequests, outstandingResponses);
            nackRequest(pkt);
            return true;
        } else {
            DPRINTF(BusBridge, "Request Needs response, reserving space\n");
            ++outstandingResponses;
        }

    otherPort->queueForSendTiming(pkt);

    return true;
}

void
Bridge::BridgePort::nackRequest(PacketPtr pkt)
{
    // Nack the packet
    pkt->result = Packet::Nacked;
    pkt->setDest(pkt->getSrc());

    //put it on the list to send
    Tick readyTime = curTick + delay;
    PacketBuffer *buf = new PacketBuffer(pkt, readyTime, true);
    if (sendQueue.empty()) {
        sendEvent.schedule(readyTime);
    }
    sendQueue.push_back(buf);
}


void
Bridge::BridgePort::queueForSendTiming(PacketPtr pkt)
{
   if (pkt->isResponse() || pkt->result == Packet::Nacked) {
        // This is a response for a request we forwarded earlier.  The
        // corresponding PacketBuffer should be stored in the packet's
        // senderState field.
        PacketBuffer *buf = dynamic_cast<PacketBuffer*>(pkt->senderState);
        assert(buf != NULL);
        // set up new packet dest & senderState based on values saved
        // from original request
        buf->fixResponse(pkt);

        // Check if this packet was expecting a response (this is either it or
        // its a nacked packet and we won't be seeing that response)
        if (buf->expectResponse)
            --outstandingResponses;


        DPRINTF(BusBridge, "restoring  sender state: %#X, from packet buffer: %#X\n",
                        pkt->senderState, buf);
        DPRINTF(BusBridge, "  is response, new dest %d\n", pkt->getDest());
        delete buf;
    }

    Tick readyTime = curTick + delay;
    PacketBuffer *buf = new PacketBuffer(pkt, readyTime);
    DPRINTF(BusBridge, "old sender state: %#X, new sender state: %#X\n",
            buf->origSenderState, buf);

    // If we're about to put this packet at the head of the queue, we
    // need to schedule an event to do the transmit.  Otherwise there
    // should already be an event scheduled for sending the head
    // packet.
    if (sendQueue.empty()) {
        sendEvent.schedule(readyTime);
    }
    ++queuedRequests;
    sendQueue.push_back(buf);
}

void
Bridge::BridgePort::trySend()
{
    assert(!sendQueue.empty());

    bool was_full = queueFull();
    int pbs = peerBlockSize();

    PacketBuffer *buf = sendQueue.front();

    assert(buf->ready <= curTick);

    PacketPtr pkt = buf->pkt;

    pkt->flags &= ~SNOOP_COMMIT; //CLear it if it was set

    if (pkt->cmd == MemCmd::WriteInvalidateReq && fixPartialWrite &&
            pkt->result != Packet::Nacked && pkt->getOffset(pbs) &&
            pkt->getSize() != pbs) {
        buf->partialWriteFix(this);
        pkt = buf->pkt;
    }

    DPRINTF(BusBridge, "trySend: origSrc %d dest %d addr 0x%x\n",
            buf->origSrc, pkt->getDest(), pkt->getAddr());


    if (sendTiming(pkt)) {
        // send successful
        sendQueue.pop_front();
        buf->pkt = NULL; // we no longer own packet, so it's not safe to look at it

        if (buf->expectResponse) {
            // Must wait for response
            DPRINTF(BusBridge, "  successful: awaiting response (%d)\n",
                    outstandingResponses);
        } else {
            // no response expected... deallocate packet buffer now.
            DPRINTF(BusBridge, "  successful: no response expected\n");
            delete buf;
        }

        if (!buf->nacked)
                --queuedRequests;

        // If there are more packets to send, schedule event to try again.
        if (!sendQueue.empty()) {
            buf = sendQueue.front();
            DPRINTF(BusBridge, "Scheduling next send\n");
            sendEvent.schedule(std::max(buf->ready, curTick + 1));
        }
        // Let things start sending again
        if (was_full && !queueFull()) {
          DPRINTF(BusBridge, "Queue was full, sending retry\n");
          otherPort->sendRetry();
        }

    } else {
        DPRINTF(BusBridge, "  unsuccessful\n");
        buf->undoPartialWriteFix();
    }
    DPRINTF(BusBridge, "trySend: queue size: %d outreq: %d outstanding resp: %d\n",
                    sendQueue.size(), queuedRequests, outstandingResponses);
}


void
Bridge::BridgePort::recvRetry()
{
    trySend();
}

/** Function called by the port when the bus is receiving a Atomic
 * transaction.*/
Tick
Bridge::BridgePort::recvAtomic(PacketPtr pkt)
{
    return otherPort->sendAtomic(pkt) + delay;
}

/** Function called by the port when the bus is receiving a Functional
 * transaction.*/
void
Bridge::BridgePort::recvFunctional(PacketPtr pkt)
{
    std::list<PacketBuffer*>::iterator i;
    bool pktContinue = true;

    for (i = sendQueue.begin();  i != sendQueue.end(); ++i) {
        if (pkt->intersect((*i)->pkt)) {
            pktContinue &= fixPacket(pkt, (*i)->pkt);
        }
    }

    if (pktContinue) {
        otherPort->sendFunctional(pkt);
    }
}

/** Function called by the port when the bus is receiving a status change.*/
void
Bridge::BridgePort::recvStatusChange(Port::Status status)
{
    otherPort->sendStatusChange(status);
}

void
Bridge::BridgePort::getDeviceAddressRanges(AddrRangeList &resp,
                                           AddrRangeList &snoop)
{
    otherPort->getPeerAddressRanges(resp, snoop);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Bridge)

   Param<int> queue_size_a;
   Param<int> queue_size_b;
   Param<Tick> delay;
   Param<bool> write_ack;
   Param<bool> fix_partial_write_a;
   Param<bool> fix_partial_write_b;

END_DECLARE_SIM_OBJECT_PARAMS(Bridge)

BEGIN_INIT_SIM_OBJECT_PARAMS(Bridge)

    INIT_PARAM(queue_size_a, "The size of the queue for data coming into side a"),
    INIT_PARAM(queue_size_b, "The size of the queue for data coming into side b"),
    INIT_PARAM(delay, "The miminum delay to cross this bridge"),
    INIT_PARAM(write_ack, "Acknowledge any writes that are received."),
    INIT_PARAM(fix_partial_write_a, "Fixup any partial block writes that are received"),
    INIT_PARAM(fix_partial_write_b, "Fixup any partial block writes that are received")

END_INIT_SIM_OBJECT_PARAMS(Bridge)

CREATE_SIM_OBJECT(Bridge)
{
    return new Bridge(getInstanceName(), queue_size_a, queue_size_b, delay,
            write_ack, fix_partial_write_a, fix_partial_write_b);
}

REGISTER_SIM_OBJECT("Bridge", Bridge)

