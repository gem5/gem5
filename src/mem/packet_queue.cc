/*
 * Copyright (c) 2012,2015 ARM Limited
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

#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/PacketQueue.hh"
#include "mem/packet_queue.hh"

using namespace std;

PacketQueue::PacketQueue(EventManager& _em, const std::string& _label,
                         bool disable_sanity_check)
    : em(_em), sendEvent(this), _disableSanityCheck(disable_sanity_check),
      label(_label), waitingOnRetry(false)
{
}

PacketQueue::~PacketQueue()
{
}

void
PacketQueue::retry()
{
    DPRINTF(PacketQueue, "Queue %s received retry\n", name());
    assert(waitingOnRetry);
    waitingOnRetry = false;
    sendDeferredPacket();
}

bool
PacketQueue::hasAddr(Addr addr) const
{
    // caller is responsible for ensuring that all packets have the
    // same alignment
    for (const auto& p : transmitList) {
        if (p.pkt->getAddr() == addr)
            return true;
    }
    return false;
}

bool
PacketQueue::checkFunctional(PacketPtr pkt)
{
    pkt->pushLabel(label);

    auto i = transmitList.begin();
    bool found = false;

    while (!found && i != transmitList.end()) {
        // If the buffered packet contains data, and it overlaps the
        // current packet, then update data
        found = pkt->checkFunctional(i->pkt);
        ++i;
    }

    pkt->popLabel();

    return found;
}

void
PacketQueue::schedSendTiming(PacketPtr pkt, Tick when, bool force_order)
{
    DPRINTF(PacketQueue, "%s for %s address %x size %d when %lu ord: %i\n",
            __func__, pkt->cmdString(), pkt->getAddr(), pkt->getSize(), when,
            force_order);

    // we can still send a packet before the end of this tick
    assert(when >= curTick());

    // express snoops should never be queued
    assert(!pkt->isExpressSnoop());

    // add a very basic sanity check on the port to ensure the
    // invisible buffer is not growing beyond reasonable limits
    if (!_disableSanityCheck && transmitList.size() > 100) {
        panic("Packet queue %s has grown beyond 100 packets\n",
              name());
    }

    // nothing on the list
    if (transmitList.empty()) {
        transmitList.emplace_front(when, pkt);
        schedSendEvent(when);
        return;
    }

    // we should either have an outstanding retry, or a send event
    // scheduled, but there is an unfortunate corner case where the
    // x86 page-table walker and timing CPU send out a new request as
    // part of the receiving of a response (called by
    // PacketQueue::sendDeferredPacket), in which we end up calling
    // ourselves again before we had a chance to update waitingOnRetry
    // assert(waitingOnRetry || sendEvent.scheduled());

    // this belongs in the middle somewhere, so search from the end to
    // order by tick; however, if force_order is set, also make sure
    // not to re-order in front of some existing packet with the same
    // address
    auto i = transmitList.end();
    --i;
    while (i != transmitList.begin() && when < i->tick &&
           !(force_order && i->pkt->getAddr() == pkt->getAddr()))
        --i;

    // emplace inserts the element before the position pointed to by
    // the iterator, so advance it one step
    transmitList.emplace(++i, when, pkt);
}

void
PacketQueue::schedSendEvent(Tick when)
{
    // if we are waiting on a retry just hold off
    if (waitingOnRetry) {
        DPRINTF(PacketQueue, "Not scheduling send as waiting for retry\n");
        assert(!sendEvent.scheduled());
        return;
    }

    if (when != MaxTick) {
        // we cannot go back in time, and to be consistent we stick to
        // one tick in the future
        when = std::max(when, curTick() + 1);
        // @todo Revisit the +1

        if (!sendEvent.scheduled()) {
            em.schedule(&sendEvent, when);
        } else if (when < sendEvent.when()) {
            // if the new time is earlier than when the event
            // currently is scheduled, move it forward
            em.reschedule(&sendEvent, when);
        }
    } else {
        // we get a MaxTick when there is no more to send, so if we're
        // draining, we may be done at this point
        if (drainState() == DrainState::Draining &&
            transmitList.empty() && !sendEvent.scheduled()) {

            DPRINTF(Drain, "PacketQueue done draining,"
                    "processing drain event\n");
            signalDrainDone();
        }
    }
}

void
PacketQueue::sendDeferredPacket()
{
    // sanity checks
    assert(!waitingOnRetry);
    assert(deferredPacketReady());

    DeferredPacket dp = transmitList.front();

    // take the packet of the list before sending it, as sending of
    // the packet in some cases causes a new packet to be enqueued
    // (most notaly when responding to the timing CPU, leading to a
    // new request hitting in the L1 icache, leading to a new
    // response)
    transmitList.pop_front();

    // use the appropriate implementation of sendTiming based on the
    // type of queue
    waitingOnRetry = !sendTiming(dp.pkt);

    // if we succeeded and are not waiting for a retry, schedule the
    // next send
    if (!waitingOnRetry) {
        schedSendEvent(deferredPacketReadyTime());
    } else {
        // put the packet back at the front of the list
        transmitList.emplace_front(dp);
    }
}

void
PacketQueue::processSendEvent()
{
    assert(!waitingOnRetry);
    sendDeferredPacket();
}

DrainState
PacketQueue::drain()
{
    if (transmitList.empty()) {
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "PacketQueue not drained\n");
        return DrainState::Draining;
    }
}

ReqPacketQueue::ReqPacketQueue(EventManager& _em, MasterPort& _masterPort,
                               const std::string _label)
    : PacketQueue(_em, _label), masterPort(_masterPort)
{
}

bool
ReqPacketQueue::sendTiming(PacketPtr pkt)
{
    return masterPort.sendTimingReq(pkt);
}

SnoopRespPacketQueue::SnoopRespPacketQueue(EventManager& _em,
                                           MasterPort& _masterPort,
                                           const std::string _label)
    : PacketQueue(_em, _label), masterPort(_masterPort)
{
}

bool
SnoopRespPacketQueue::sendTiming(PacketPtr pkt)
{
    return masterPort.sendTimingSnoopResp(pkt);
}

RespPacketQueue::RespPacketQueue(EventManager& _em, SlavePort& _slavePort,
                                 const std::string _label)
    : PacketQueue(_em, _label), slavePort(_slavePort)
{
}

bool
RespPacketQueue::sendTiming(PacketPtr pkt)
{
    return slavePort.sendTimingResp(pkt);
}
