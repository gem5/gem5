/*
 * Copyright (c) 2011-2013 ARM Limited
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
 * Copyright (c) 2015 The University of Bologna
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
 * @file
 * Implementation of the SerialLink Class, modeling Hybrid-Memory-Cube's
 * serial interface.
 */

#include "mem/serial_link.hh"

#include "base/trace.hh"
#include "debug/SerialLink.hh"
#include "params/SerialLink.hh"

namespace gem5
{

SerialLink::SerialLinkResponsePort::
SerialLinkResponsePort(const std::string& _name,
                                         SerialLink& _serial_link,
                                         SerialLinkRequestPort& _mem_side_port,
                                         Cycles _delay, int _resp_limit,
                                         const std::vector<AddrRange>&
                                         _ranges)
    : ResponsePort(_name, &_serial_link), serial_link(_serial_link),
      mem_side_port(_mem_side_port), delay(_delay),
      ranges(_ranges.begin(), _ranges.end()),
      outstandingResponses(0), retryReq(false),
      respQueueLimit(_resp_limit),
      sendEvent([this]{ trySendTiming(); }, _name)
{
}

SerialLink::SerialLinkRequestPort::SerialLinkRequestPort(const std::string&
                                           _name, SerialLink& _serial_link,
                                           SerialLinkResponsePort&
                                           _cpu_side_port, Cycles _delay,
                                           int _req_limit)
    : RequestPort(_name, &_serial_link), serial_link(_serial_link),
      cpu_side_port(_cpu_side_port), delay(_delay), reqQueueLimit(_req_limit),
      sendEvent([this]{ trySendTiming(); }, _name)
{
}

SerialLink::SerialLink(const SerialLinkParams &p)
    : ClockedObject(p),
      cpu_side_port(p.name + ".cpu_side_port", *this, mem_side_port,
                ticksToCycles(p.delay), p.resp_size, p.ranges),
      mem_side_port(p.name + ".mem_side_port", *this, cpu_side_port,
                 ticksToCycles(p.delay), p.req_size),
      num_lanes(p.num_lanes),
      link_speed(p.link_speed)
{
}

Port&
SerialLink::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side_port")
        return mem_side_port;
    else if (if_name == "cpu_side_port")
        return cpu_side_port;
    else
        // pass it along to our super class
        return ClockedObject::getPort(if_name, idx);
}

void
SerialLink::init()
{
    // make sure both sides are connected and have the same block size
    if (!cpu_side_port.isConnected() || !mem_side_port.isConnected())
        fatal("Both ports of a serial_link must be connected.\n");

    // notify the request side  of our address ranges
    cpu_side_port.sendRangeChange();
}

bool
SerialLink::SerialLinkResponsePort::respQueueFull() const
{
    return outstandingResponses == respQueueLimit;
}

bool
SerialLink::SerialLinkRequestPort::reqQueueFull() const
{
    return transmitList.size() == reqQueueLimit;
}

bool
SerialLink::SerialLinkRequestPort::recvTimingResp(PacketPtr pkt)
{
    // all checks are done when the request is accepted on the response
    // side, so we are guaranteed to have space for the response
    DPRINTF(SerialLink, "recvTimingResp: %s addr 0x%x\n",
            pkt->cmdString(), pkt->getAddr());

    DPRINTF(SerialLink, "Request queue size: %d\n", transmitList.size());

    // @todo: We need to pay for this and not just zero it out
    pkt->headerDelay = pkt->payloadDelay = 0;

    // This is similar to what happens for the request packets:
    // The serializer will start serialization as soon as it receives the
    // first flit, but the deserializer (at the host side in this case), will
    // have to wait to receive the whole packet. So we only account for the
    // deserialization latency.
    Cycles cycles = delay;
    cycles += Cycles(divCeil(pkt->getSize() * 8, serial_link.num_lanes
                * serial_link.link_speed));
     Tick t = serial_link.clockEdge(cycles);

    //@todo: If the processor sends two uncached requests towards HMC and the
    // second one is smaller than the first one. It may happen that the second
    // one crosses this link faster than the first one (because the packet
    // waits in the link based on its size). This can reorder the received
    // response.
    cpu_side_port.schedTimingResp(pkt, t);

    return true;
}

bool
SerialLink::SerialLinkResponsePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(SerialLink, "recvTimingReq: %s addr 0x%x\n",
            pkt->cmdString(), pkt->getAddr());

    // we should not see a timing request if we are already in a retry
    assert(!retryReq);

    DPRINTF(SerialLink, "Response queue size: %d outresp: %d\n",
            transmitList.size(), outstandingResponses);

    // if the request queue is full then there is no hope
    if (mem_side_port.reqQueueFull()) {
        DPRINTF(SerialLink, "Request queue full\n");
        retryReq = true;
    } else if ( !retryReq ) {
        // look at the response queue if we expect to see a response
        bool expects_response = pkt->needsResponse() &&
            !pkt->cacheResponding();
        if (expects_response) {
            if (respQueueFull()) {
                DPRINTF(SerialLink, "Response queue full\n");
                retryReq = true;
            } else {
                // ok to send the request with space for the response
                DPRINTF(SerialLink, "Reserving space for response\n");
                assert(outstandingResponses != respQueueLimit);
                ++outstandingResponses;

                // no need to set retryReq to false as this is already the
                // case
            }
        }

        if (!retryReq) {
            // @todo: We need to pay for this and not just zero it out
            pkt->headerDelay = pkt->payloadDelay = 0;

            // We assume that the serializer component at the transmitter side
            // does not need to receive the whole packet to start the
            // serialization (this assumption is consistent with the HMC
            // standard). But the deserializer waits for the complete packet
            // to check its integrity first. So everytime a packet crosses a
            // serial link, we should account for its deserialization latency
            // only.
            Cycles cycles = delay;
            cycles += Cycles(divCeil(pkt->getSize() * 8,
                    serial_link.num_lanes * serial_link.link_speed));
            Tick t = serial_link.clockEdge(cycles);

            //@todo: If the processor sends two uncached requests towards HMC
            // and the second one is smaller than the first one. It may happen
            // that the second one crosses this link faster than the first one
            // (because the packet waits in the link based on its size).
            // This can reorder the received response.
            mem_side_port.schedTimingReq(pkt, t);
        }
    }

    // remember that we are now stalling a packet and that we have to
    // tell the sending requestor to retry once space becomes available,
    // we make no distinction whether the stalling is due to the
    // request queue or response queue being full
    return !retryReq;
}

void
SerialLink::SerialLinkResponsePort::retryStalledReq()
{
    if (retryReq) {
        DPRINTF(SerialLink, "Request waiting for retry, now retrying\n");
        retryReq = false;
        sendRetryReq();
    }
}

void
SerialLink::SerialLinkRequestPort::schedTimingReq(PacketPtr pkt, Tick when)
{
    // If we're about to put this packet at the head of the queue, we
    // need to schedule an event to do the transmit.  Otherwise there
    // should already be an event scheduled for sending the head
    // packet.
    if (transmitList.empty()) {
        serial_link.schedule(sendEvent, when);
    }

    assert(transmitList.size() != reqQueueLimit);

    transmitList.emplace_back(DeferredPacket(pkt, when));
}


void
SerialLink::SerialLinkResponsePort::schedTimingResp(PacketPtr pkt, Tick when)
{
    // If we're about to put this packet at the head of the queue, we
    // need to schedule an event to do the transmit.  Otherwise there
    // should already be an event scheduled for sending the head
    // packet.
    if (transmitList.empty()) {
        serial_link.schedule(sendEvent, when);
    }

    transmitList.emplace_back(DeferredPacket(pkt, when));
}

void
SerialLink::SerialLinkRequestPort::trySendTiming()
{
    assert(!transmitList.empty());

    DeferredPacket req = transmitList.front();

    assert(req.tick <= curTick());

    PacketPtr pkt = req.pkt;

    DPRINTF(SerialLink, "trySend request addr 0x%x, queue size %d\n",
            pkt->getAddr(), transmitList.size());

    if (sendTimingReq(pkt)) {
        // send successful
        transmitList.pop_front();

        DPRINTF(SerialLink, "trySend request successful\n");

        // If there are more packets to send, schedule event to try again.
        if (!transmitList.empty()) {
            DeferredPacket next_req = transmitList.front();
            DPRINTF(SerialLink, "Scheduling next send\n");

            // Make sure bandwidth limitation is met
            Cycles cycles = Cycles(divCeil(pkt->getSize() * 8,
                serial_link.num_lanes * serial_link.link_speed));
            Tick t = serial_link.clockEdge(cycles);
            serial_link.schedule(sendEvent, std::max(next_req.tick, t));
        }

        // if we have stalled a request due to a full request queue,
        // then send a retry at this point, also note that if the
        // request we stalled was waiting for the response queue
        // rather than the request queue we might stall it again
        cpu_side_port.retryStalledReq();
    }

    // if the send failed, then we try again once we receive a retry,
    // and therefore there is no need to take any action
}

void
SerialLink::SerialLinkResponsePort::trySendTiming()
{
    assert(!transmitList.empty());

    DeferredPacket resp = transmitList.front();

    assert(resp.tick <= curTick());

    PacketPtr pkt = resp.pkt;

    DPRINTF(SerialLink, "trySend response addr 0x%x, outstanding %d\n",
            pkt->getAddr(), outstandingResponses);

    if (sendTimingResp(pkt)) {
        // send successful
        transmitList.pop_front();
        DPRINTF(SerialLink, "trySend response successful\n");

        assert(outstandingResponses != 0);
        --outstandingResponses;

        // If there are more packets to send, schedule event to try again.
        if (!transmitList.empty()) {
            DeferredPacket next_resp = transmitList.front();
            DPRINTF(SerialLink, "Scheduling next send\n");

            // Make sure bandwidth limitation is met
            Cycles cycles = Cycles(divCeil(pkt->getSize() * 8,
                serial_link.num_lanes * serial_link.link_speed));
            Tick t = serial_link.clockEdge(cycles);
            serial_link.schedule(sendEvent, std::max(next_resp.tick, t));
        }

        // if there is space in the request queue and we were stalling
        // a request, it will definitely be possible to accept it now
        // since there is guaranteed space in the response queue
        if (!mem_side_port.reqQueueFull() && retryReq) {
            DPRINTF(SerialLink, "Request waiting for retry, now retrying\n");
            retryReq = false;
            sendRetryReq();
        }
    }

    // if the send failed, then we try again once we receive a retry,
    // and therefore there is no need to take any action
}

void
SerialLink::SerialLinkRequestPort::recvReqRetry()
{
    trySendTiming();
}

void
SerialLink::SerialLinkResponsePort::recvRespRetry()
{
    trySendTiming();
}

Tick
SerialLink::SerialLinkResponsePort::recvAtomic(PacketPtr pkt)
{
    return delay * serial_link.clockPeriod() + mem_side_port.sendAtomic(pkt);
}

void
SerialLink::SerialLinkResponsePort::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    // check the response queue
    for (auto i = transmitList.begin();  i != transmitList.end(); ++i) {
        if (pkt->trySatisfyFunctional((*i).pkt)) {
            pkt->makeResponse();
            return;
        }
    }

    // also check the memory-side port's request queue
    if (mem_side_port.trySatisfyFunctional(pkt)) {
        return;
    }

    pkt->popLabel();

    // fall through if pkt still not satisfied
    mem_side_port.sendFunctional(pkt);
}

bool
SerialLink::SerialLinkRequestPort::trySatisfyFunctional(PacketPtr pkt)
{
    bool found = false;
    auto i = transmitList.begin();

    while (i != transmitList.end() && !found) {
        if (pkt->trySatisfyFunctional((*i).pkt)) {
            pkt->makeResponse();
            found = true;
        }
        ++i;
    }

    return found;
}

AddrRangeList
SerialLink::SerialLinkResponsePort::getAddrRanges() const
{
    return ranges;
}

} // namespace gem5
