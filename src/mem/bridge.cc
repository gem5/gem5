
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
                               int _delay, int _nack_delay, int _req_limit,
                               int _resp_limit, bool fix_partial_write)
    : Port(_name), bridge(_bridge), otherPort(_otherPort),
      delay(_delay), nackDelay(_nack_delay), fixPartialWrite(fix_partial_write),
      outstandingResponses(0), queuedRequests(0), inRetry(false),
      reqQueueLimit(_req_limit), respQueueLimit(_resp_limit), sendEvent(this)
{
}

Bridge::Bridge(Params *p)
    : MemObject(p->name),
      portA(p->name + "-portA", this, &portB, p->delay, p->nack_delay,
              p->req_size_a, p->resp_size_a, p->fix_partial_write_a),
      portB(p->name + "-portB", this, &portA, p->delay, p->nack_delay,
              p->req_size_b, p->resp_size_b, p->fix_partial_write_b),
      ackWrites(p->write_ack), _params(p)
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
Bridge::BridgePort::respQueueFull()
{
    assert(outstandingResponses >= 0 && outstandingResponses <= respQueueLimit);
    return outstandingResponses >= respQueueLimit;
}

bool
Bridge::BridgePort::reqQueueFull()
{
    assert(queuedRequests >= 0 && queuedRequests <= reqQueueLimit);
    return queuedRequests >= reqQueueLimit;
}

/** Function called by the port when the bus is receiving a Timing
 * transaction.*/
bool
Bridge::BridgePort::recvTiming(PacketPtr pkt)
{
    DPRINTF(BusBridge, "recvTiming: src %d dest %d addr 0x%x\n",
                pkt->getSrc(), pkt->getDest(), pkt->getAddr());

    DPRINTF(BusBridge, "Local queue size: %d outreq: %d outresp: %d\n",
                    sendQueue.size(), queuedRequests, outstandingResponses);
    DPRINTF(BusBridge, "Remove queue size: %d outreq: %d outresp: %d\n",
                    otherPort->sendQueue.size(), otherPort->queuedRequests,
                    otherPort->outstandingResponses);

    if (pkt->isRequest() && otherPort->reqQueueFull() && pkt->result !=
            Packet::Nacked) {
        DPRINTF(BusBridge, "Remote queue full, nacking\n");
        nackRequest(pkt);
        return true;
    }

    if (pkt->needsResponse() && pkt->result != Packet::Nacked)
        if (respQueueFull()) {
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
    Tick readyTime = curTick + nackDelay;
    PacketBuffer *buf = new PacketBuffer(pkt, readyTime, true);

    // nothing on the list, add it and we're done
    if (sendQueue.empty()) {
        assert(!sendEvent.scheduled());
        sendEvent.schedule(readyTime);
        sendQueue.push_back(buf);
        return;
    }

    assert(sendEvent.scheduled() || inRetry);

    // does it go at the end?
    if (readyTime >= sendQueue.back()->ready) {
        sendQueue.push_back(buf);
        return;
    }

    // ok, somewhere in the middle, fun
    std::list<PacketBuffer*>::iterator i = sendQueue.begin();
    std::list<PacketBuffer*>::iterator end = sendQueue.end();
    std::list<PacketBuffer*>::iterator begin = sendQueue.begin();
    bool done = false;

    while (i != end && !done) {
        if (readyTime < (*i)->ready) {
            if (i == begin)
                sendEvent.reschedule(readyTime);
            sendQueue.insert(i,buf);
            done = true;
        }
        i++;
    }
    assert(done);
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

        // Check if this packet was expecting a response and it's a nacked
        // packet, in which case we will never being seeing it
        if (buf->expectResponse && pkt->result == Packet::Nacked)
            --outstandingResponses;


        DPRINTF(BusBridge, "restoring  sender state: %#X, from packet buffer: %#X\n",
                        pkt->senderState, buf);
        DPRINTF(BusBridge, "  is response, new dest %d\n", pkt->getDest());
        delete buf;
    }


    if (pkt->isRequest() && pkt->result != Packet::Nacked) {
        ++queuedRequests;
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
    sendQueue.push_back(buf);
}

void
Bridge::BridgePort::trySend()
{
    assert(!sendQueue.empty());

    int pbs = peerBlockSize();

    PacketBuffer *buf = sendQueue.front();

    assert(buf->ready <= curTick);

    PacketPtr pkt = buf->pkt;

    if (pkt->cmd == MemCmd::WriteInvalidateReq && fixPartialWrite &&
            pkt->result != Packet::Nacked && pkt->getOffset(pbs) &&
            pkt->getSize() != pbs) {
        buf->partialWriteFix(this);
        pkt = buf->pkt;
    }

    DPRINTF(BusBridge, "trySend: origSrc %d dest %d addr 0x%x\n",
            buf->origSrc, pkt->getDest(), pkt->getAddr());

    bool wasReq = pkt->isRequest();
    bool wasNacked = pkt->result == Packet::Nacked;

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

        if (!wasNacked) {
            if (wasReq)
                --queuedRequests;
            else
                --outstandingResponses;
        }

        // If there are more packets to send, schedule event to try again.
        if (!sendQueue.empty()) {
            buf = sendQueue.front();
            DPRINTF(BusBridge, "Scheduling next send\n");
            sendEvent.schedule(std::max(buf->ready, curTick + 1));
        }
    } else {
        DPRINTF(BusBridge, "  unsuccessful\n");
        buf->undoPartialWriteFix();
        inRetry = true;
    }
    DPRINTF(BusBridge, "trySend: queue size: %d outreq: %d outstanding resp: %d\n",
                    sendQueue.size(), queuedRequests, outstandingResponses);
}


void
Bridge::BridgePort::recvRetry()
{
    inRetry = false;
    Tick nextReady = sendQueue.front()->ready;
    if (nextReady <= curTick)
        trySend();
    else
        sendEvent.schedule(nextReady);
}

/** Function called by the port when the bus is receiving a Atomic
 * transaction.*/
Tick
Bridge::BridgePort::recvAtomic(PacketPtr pkt)
{
    int pbs = otherPort->peerBlockSize();
    Tick atomic_delay;
    // fix partial atomic writes... similar to the timing code that does the
    // same
    if (pkt->cmd == MemCmd::WriteInvalidateReq && fixPartialWrite &&
             pkt->getOffset(pbs) && pkt->getSize() != pbs) {
        PacketDataPtr data;
        data = new uint8_t[pbs];
        PacketPtr funcPkt = new Packet(pkt->req, MemCmd::ReadReq,
                         Packet::Broadcast, pbs);

        funcPkt->dataStatic(data);
        otherPort->sendFunctional(funcPkt);
        assert(funcPkt->result == Packet::Success);
        delete funcPkt;
        memcpy(data + pkt->getOffset(pbs), pkt->getPtr<uint8_t>(),
                         pkt->getSize());
        PacketPtr newPkt = new Packet(pkt->req, MemCmd::WriteInvalidateReq,
                Packet::Broadcast, pbs);
        pkt->dataDynamicArray(data);
        atomic_delay = otherPort->sendAtomic(newPkt);
        delete newPkt;
    } else {
        atomic_delay = otherPort->sendAtomic(pkt);
    }
    return atomic_delay + delay;
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

   Param<int> req_size_a;
   Param<int> req_size_b;
   Param<int> resp_size_a;
   Param<int> resp_size_b;
   Param<Tick> delay;
   Param<Tick> nack_delay;
   Param<bool> write_ack;
   Param<bool> fix_partial_write_a;
   Param<bool> fix_partial_write_b;

END_DECLARE_SIM_OBJECT_PARAMS(Bridge)

BEGIN_INIT_SIM_OBJECT_PARAMS(Bridge)

    INIT_PARAM(req_size_a, "The size of the queue for requests coming into side a"),
    INIT_PARAM(req_size_b, "The size of the queue for requests coming into side b"),
    INIT_PARAM(resp_size_a, "The size of the queue for responses coming into side a"),
    INIT_PARAM(resp_size_b, "The size of the queue for responses coming into side b"),
    INIT_PARAM(delay, "The miminum delay to cross this bridge"),
    INIT_PARAM(nack_delay, "The minimum delay to nack a packet"),
    INIT_PARAM(write_ack, "Acknowledge any writes that are received."),
    INIT_PARAM(fix_partial_write_a, "Fixup any partial block writes that are received"),
    INIT_PARAM(fix_partial_write_b, "Fixup any partial block writes that are received")

END_INIT_SIM_OBJECT_PARAMS(Bridge)

CREATE_SIM_OBJECT(Bridge)
{
    Bridge::Params *p = new Bridge::Params;
    p->name = getInstanceName();
    p->req_size_a = req_size_a;
    p->req_size_b = req_size_b;
    p->resp_size_a = resp_size_a;
    p->resp_size_b = resp_size_b;
    p->delay = delay;
    p->nack_delay = nack_delay;
    p->write_ack = write_ack;
    p->fix_partial_write_a = fix_partial_write_a;
    p->fix_partial_write_b = fix_partial_write_b;
    return new Bridge(p);
}

REGISTER_SIM_OBJECT("Bridge", Bridge)

