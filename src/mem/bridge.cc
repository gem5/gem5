/*
 * Copyright (c) 2011 ARM Limited
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
 *          Andreas Hansson
 */

/**
 * @file
 * Implementation of a memory-mapped bus bridge that connects a master
 * and a slave through a request and response queue.
 */

#include "base/trace.hh"
#include "debug/BusBridge.hh"
#include "mem/bridge.hh"
#include "params/Bridge.hh"

Bridge::BridgeSlavePort::BridgeSlavePort(const std::string &_name,
                                         Bridge* _bridge,
                                         BridgeMasterPort* _masterPort,
                                         int _delay, int _nack_delay,
                                         int _resp_limit,
                                         std::vector<Range<Addr> > _ranges)
    : Port(_name, _bridge), bridge(_bridge), masterPort(_masterPort),
      delay(_delay), nackDelay(_nack_delay),
      ranges(_ranges.begin(), _ranges.end()),
      outstandingResponses(0), inRetry(false),
      respQueueLimit(_resp_limit), sendEvent(this)
{
}

Bridge::BridgeMasterPort::BridgeMasterPort(const std::string &_name,
                                           Bridge* _bridge,
                                           BridgeSlavePort* _slavePort,
                                           int _delay, int _req_limit)
    : Port(_name, _bridge), bridge(_bridge), slavePort(_slavePort),
      delay(_delay), inRetry(false), reqQueueLimit(_req_limit), sendEvent(this)
{
}

Bridge::Bridge(Params *p)
    : MemObject(p),
      slavePort(p->name + "-slave", this, &masterPort, p->delay,
                p->nack_delay, p->resp_size, p->ranges),
      masterPort(p->name + "-master", this, &slavePort, p->delay, p->req_size),
      ackWrites(p->write_ack), _params(p)
{
    if (ackWrites)
        panic("No support for acknowledging writes\n");
}

Port*
Bridge::getPort(const std::string &if_name, int idx)
{
    Port* port;

    if (if_name == "slave")
        port = &slavePort;
    else if (if_name == "master")
        port = &masterPort;
    else
        return NULL;

    if (port->getPeer() != NULL)
        panic("bridge side %s already connected to %s.",
                if_name, port->getPeer()->name());
    return port;
}


void
Bridge::init()
{
    // make sure both sides are connected and have the same block size
    if (!slavePort.isConnected() || !masterPort.isConnected())
        fatal("Both ports of bus bridge are not connected to a bus.\n");

    if (slavePort.peerBlockSize() != masterPort.peerBlockSize())
        fatal("Slave port size %d, master port size %d \n " \
              "Busses don't have the same block size... Not supported.\n",
              slavePort.peerBlockSize(), masterPort.peerBlockSize());

    // notify the master side  of our address ranges
    slavePort.sendRangeChange();
}

bool
Bridge::BridgeSlavePort::respQueueFull()
{
    return outstandingResponses == respQueueLimit;
}

bool
Bridge::BridgeMasterPort::reqQueueFull()
{
    return requestQueue.size() == reqQueueLimit;
}

bool
Bridge::BridgeMasterPort::recvTiming(PacketPtr pkt)
{
    // should only see responses on the master side
    assert(pkt->isResponse());

    // all checks are done when the request is accepted on the slave
    // side, so we are guaranteed to have space for the response

    DPRINTF(BusBridge, "recvTiming: src %d dest %d addr 0x%x\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr());

    DPRINTF(BusBridge, "Request queue size: %d\n", requestQueue.size());

    slavePort->queueForSendTiming(pkt);

    return true;
}

bool
Bridge::BridgeSlavePort::recvTiming(PacketPtr pkt)
{
    // should only see requests on the slave side
    assert(pkt->isRequest());

    DPRINTF(BusBridge, "recvTiming: src %d dest %d addr 0x%x\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr());

    DPRINTF(BusBridge, "Response queue size: %d outresp: %d\n",
            responseQueue.size(), outstandingResponses);

    if (masterPort->reqQueueFull()) {
        DPRINTF(BusBridge, "Request queue full, nacking\n");
        nackRequest(pkt);
        return true;
    }

    if (pkt->needsResponse()) {
        if (respQueueFull()) {
            DPRINTF(BusBridge,
                    "Response queue full, no space for response, nacking\n");
            DPRINTF(BusBridge,
                    "queue size: %d outstanding resp: %d\n",
                    responseQueue.size(), outstandingResponses);
            nackRequest(pkt);
            return true;
        } else {
            DPRINTF(BusBridge, "Request Needs response, reserving space\n");
            assert(outstandingResponses != respQueueLimit);
            ++outstandingResponses;
        }
    }

    masterPort->queueForSendTiming(pkt);

    return true;
}

void
Bridge::BridgeSlavePort::nackRequest(PacketPtr pkt)
{
    // Nack the packet
    pkt->makeTimingResponse();
    pkt->setNacked();

    // The Nack packets are stored in the response queue just like any
    // other response, but they do not occupy any space as this is
    // tracked by the outstandingResponses, this guarantees space for
    // the Nack packets, but implicitly means we have an (unrealistic)
    // unbounded Nack queue.

    // put it on the list to send
    Tick readyTime = curTick() + nackDelay;
    PacketBuffer *buf = new PacketBuffer(pkt, readyTime, true);

    // nothing on the list, add it and we're done
    if (responseQueue.empty()) {
        assert(!sendEvent.scheduled());
        bridge->schedule(sendEvent, readyTime);
        responseQueue.push_back(buf);
        return;
    }

    assert(sendEvent.scheduled() || inRetry);

    // does it go at the end?
    if (readyTime >= responseQueue.back()->ready) {
        responseQueue.push_back(buf);
        return;
    }

    // ok, somewhere in the middle, fun
    std::list<PacketBuffer*>::iterator i = responseQueue.begin();
    std::list<PacketBuffer*>::iterator end = responseQueue.end();
    std::list<PacketBuffer*>::iterator begin = responseQueue.begin();
    bool done = false;

    while (i != end && !done) {
        if (readyTime < (*i)->ready) {
            if (i == begin)
                bridge->reschedule(sendEvent, readyTime);
            responseQueue.insert(i,buf);
            done = true;
        }
        i++;
    }
    assert(done);
}

void
Bridge::BridgeMasterPort::queueForSendTiming(PacketPtr pkt)
{
    Tick readyTime = curTick() + delay;
    PacketBuffer *buf = new PacketBuffer(pkt, readyTime);

    // If we're about to put this packet at the head of the queue, we
    // need to schedule an event to do the transmit.  Otherwise there
    // should already be an event scheduled for sending the head
    // packet.
    if (requestQueue.empty()) {
        bridge->schedule(sendEvent, readyTime);
    }

    assert(requestQueue.size() != reqQueueLimit);

    requestQueue.push_back(buf);
}


void
Bridge::BridgeSlavePort::queueForSendTiming(PacketPtr pkt)
{
    // This is a response for a request we forwarded earlier.  The
    // corresponding PacketBuffer should be stored in the packet's
    // senderState field.
    PacketBuffer *buf = dynamic_cast<PacketBuffer*>(pkt->senderState);
    assert(buf != NULL);
    // set up new packet dest & senderState based on values saved
    // from original request
    buf->fixResponse(pkt);

    DPRINTF(BusBridge, "response, new dest %d\n", pkt->getDest());
    delete buf;

    Tick readyTime = curTick() + delay;
    buf = new PacketBuffer(pkt, readyTime);

    // If we're about to put this packet at the head of the queue, we
    // need to schedule an event to do the transmit.  Otherwise there
    // should already be an event scheduled for sending the head
    // packet.
    if (responseQueue.empty()) {
        bridge->schedule(sendEvent, readyTime);
    }
    responseQueue.push_back(buf);
}

void
Bridge::BridgeMasterPort::trySend()
{
    assert(!requestQueue.empty());

    PacketBuffer *buf = requestQueue.front();

    assert(buf->ready <= curTick());

    PacketPtr pkt = buf->pkt;

    DPRINTF(BusBridge, "trySend: origSrc %d dest %d addr 0x%x\n",
            buf->origSrc, pkt->getDest(), pkt->getAddr());

    // If the send was successful, make sure sender state was set to NULL
    // otherwise we could get a NACK back of a packet that didn't expect a
    // response and we would try to use freed memory.

    Packet::SenderState *old_sender_state = pkt->senderState;
    if (!buf->expectResponse)
        pkt->senderState = NULL;

    if (sendTiming(pkt)) {
        // send successful
        requestQueue.pop_front();
        // we no longer own packet, so it's not safe to look at it
        buf->pkt = NULL;

        if (!buf->expectResponse) {
            // no response expected... deallocate packet buffer now.
            DPRINTF(BusBridge, "  successful: no response expected\n");
            delete buf;
        }

        // If there are more packets to send, schedule event to try again.
        if (!requestQueue.empty()) {
            buf = requestQueue.front();
            DPRINTF(BusBridge, "Scheduling next send\n");
            bridge->schedule(sendEvent, std::max(buf->ready, curTick() + 1));
        }
    } else {
        DPRINTF(BusBridge, "  unsuccessful\n");
        pkt->senderState = old_sender_state;
        inRetry = true;
    }

    DPRINTF(BusBridge, "trySend: request queue size: %d\n",
            requestQueue.size());
}

void
Bridge::BridgeSlavePort::trySend()
{
    assert(!responseQueue.empty());

    PacketBuffer *buf = responseQueue.front();

    assert(buf->ready <= curTick());

    PacketPtr pkt = buf->pkt;

    DPRINTF(BusBridge, "trySend: origSrc %d dest %d addr 0x%x\n",
            buf->origSrc, pkt->getDest(), pkt->getAddr());

    bool was_nacked_here = buf->nackedHere;

    // no need to worry about the sender state since we are not
    // modifying it

    if (sendTiming(pkt)) {
        DPRINTF(BusBridge, "  successful\n");
        // send successful
        responseQueue.pop_front();
        // this is a response... deallocate packet buffer now.
        delete buf;

        if (!was_nacked_here) {
            assert(outstandingResponses != 0);
            --outstandingResponses;
        }

        // If there are more packets to send, schedule event to try again.
        if (!responseQueue.empty()) {
            buf = responseQueue.front();
            DPRINTF(BusBridge, "Scheduling next send\n");
            bridge->schedule(sendEvent, std::max(buf->ready, curTick() + 1));
        }
    } else {
        DPRINTF(BusBridge, "  unsuccessful\n");
        inRetry = true;
    }

    DPRINTF(BusBridge, "trySend: queue size: %d outstanding resp: %d\n",
            responseQueue.size(), outstandingResponses);
}

void
Bridge::BridgeMasterPort::recvRetry()
{
    inRetry = false;
    Tick nextReady = requestQueue.front()->ready;
    if (nextReady <= curTick())
        trySend();
    else
        bridge->schedule(sendEvent, nextReady);
}

void
Bridge::BridgeSlavePort::recvRetry()
{
    inRetry = false;
    Tick nextReady = responseQueue.front()->ready;
    if (nextReady <= curTick())
        trySend();
    else
        bridge->schedule(sendEvent, nextReady);
}

Tick
Bridge::BridgeMasterPort::recvAtomic(PacketPtr pkt)
{
    // master port should never receive any atomic access (panic only
    // works once the other side, i.e. the busses, respects this)
    //
    //panic("Master port on %s got a recvAtomic\n", bridge->name());
    return 0;
}

Tick
Bridge::BridgeSlavePort::recvAtomic(PacketPtr pkt)
{
    return delay + masterPort->sendAtomic(pkt);
}

void
Bridge::BridgeMasterPort::recvFunctional(PacketPtr pkt)
{
    // master port should never receive any functional access (panic
    // only works once the other side, i.e. the busses, respect this)

    // panic("Master port on %s got a recvFunctional\n", bridge->name());
}

void
Bridge::BridgeSlavePort::recvFunctional(PacketPtr pkt)
{
    std::list<PacketBuffer*>::iterator i;

    pkt->pushLabel(name());

    // check the response queue
    for (i = responseQueue.begin();  i != responseQueue.end(); ++i) {
        if (pkt->checkFunctional((*i)->pkt)) {
            pkt->makeResponse();
            return;
        }
    }

    // also check the master port's request queue
    if (masterPort->checkFunctional(pkt)) {
        return;
    }

    pkt->popLabel();

    // fall through if pkt still not satisfied
    masterPort->sendFunctional(pkt);
}

bool
Bridge::BridgeMasterPort::checkFunctional(PacketPtr pkt)
{
    bool found = false;
    std::list<PacketBuffer*>::iterator i = requestQueue.begin();

    while(i != requestQueue.end() && !found) {
        if (pkt->checkFunctional((*i)->pkt)) {
            pkt->makeResponse();
            found = true;
        }
        ++i;
    }

    return found;
}

/** Function called by the port when the bridge is receiving a range change.*/
void
Bridge::BridgeMasterPort::recvRangeChange()
{
    // no need to forward as the bridge has a fixed set of ranges
}

void
Bridge::BridgeSlavePort::recvRangeChange()
{
    // is a slave port so do nothing
}

AddrRangeList
Bridge::BridgeSlavePort::getAddrRanges()
{
    return ranges;
}

Bridge *
BridgeParams::create()
{
    return new Bridge(this);
}
