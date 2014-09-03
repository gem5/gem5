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

#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/PacketQueue.hh"
#include "mem/packet_queue.hh"

using namespace std;

PacketQueue::PacketQueue(EventManager& _em, const std::string& _label)
    : em(_em), sendEvent(this), drainManager(NULL), label(_label),
      waitingOnRetry(false)
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
    sendDeferredPacket();
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
PacketQueue::schedSendEvent(Tick when)
{
    // if we are waiting on a retry, do not schedule a send event, and
    // instead rely on retry being called
    if (waitingOnRetry) {
        assert(!sendEvent.scheduled());
        return;
    }

    if (!sendEvent.scheduled()) {
        em.schedule(&sendEvent, when);
    } else if (sendEvent.when() > when) {
        em.reschedule(&sendEvent, when);
    }
}

void
PacketQueue::schedSendTiming(PacketPtr pkt, Tick when, bool send_as_snoop)
{
    DPRINTF(PacketQueue, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    // we can still send a packet before the end of this tick
    assert(when >= curTick());

    // express snoops should never be queued
    assert(!pkt->isExpressSnoop());

    // add a very basic sanity check on the port to ensure the
    // invisible buffer is not growing beyond reasonable limits
    if (transmitList.size() > 100) {
        panic("Packet queue %s has grown beyond 100 packets\n",
              name());
    }

    // nothing on the list, or earlier than current front element,
    // schedule an event
    if (transmitList.empty() || when < transmitList.front().tick) {
        // note that currently we ignore a potentially outstanding retry
        // and could in theory put a new packet at the head of the
        // transmit list before retrying the existing packet
        transmitList.push_front(DeferredPacket(when, pkt, send_as_snoop));
        schedSendEvent(when);
        return;
    }

    // list is non-empty and this belongs at the end
    if (when >= transmitList.back().tick) {
        transmitList.push_back(DeferredPacket(when, pkt, send_as_snoop));
        return;
    }

    // this belongs in the middle somewhere, insertion sort
    auto i = transmitList.begin();
    ++i; // already checked for insertion at front
    while (i != transmitList.end() && when >= i->tick)
        ++i;
    transmitList.insert(i, DeferredPacket(when, pkt, send_as_snoop));
}

void PacketQueue::trySendTiming()
{
    assert(deferredPacketReady());

    DeferredPacket dp = transmitList.front();

    // use the appropriate implementation of sendTiming based on the
    // type of port associated with the queue, and whether the packet
    // is to be sent as a snoop or not
    waitingOnRetry = !sendTiming(dp.pkt, dp.sendAsSnoop);

    if (!waitingOnRetry) {
        // take the packet off the list
        transmitList.pop_front();
    }
}

void
PacketQueue::scheduleSend(Tick time)
{
    // the next ready time is either determined by the next deferred packet,
    // or in the cache through the MSHR ready time
    Tick nextReady = std::min(deferredPacketReadyTime(), time);

    if (nextReady != MaxTick) {
        // if the sendTiming caused someone else to call our
        // recvTiming we could already have an event scheduled, check
        if (!sendEvent.scheduled())
            em.schedule(&sendEvent, std::max(nextReady, curTick() + 1));
    } else {
        // no more to send, so if we're draining, we may be done
        if (drainManager && transmitList.empty() && !sendEvent.scheduled()) {
            DPRINTF(Drain, "PacketQueue done draining,"
                    "processing drain event\n");
            drainManager->signalDrainDone();
            drainManager = NULL;
        }
    }
}

void
PacketQueue::sendDeferredPacket()
{
    // try to send what is on the list, this will set waitingOnRetry
    // accordingly
    trySendTiming();

    // if we succeeded and are not waiting for a retry, schedule the
    // next send
    if (!waitingOnRetry) {
        scheduleSend();
    }
}

void
PacketQueue::processSendEvent()
{
    assert(!waitingOnRetry);
    sendDeferredPacket();
}

unsigned int
PacketQueue::drain(DrainManager *dm)
{
    if (transmitList.empty())
        return 0;
    DPRINTF(Drain, "PacketQueue not drained\n");
    drainManager = dm;
    return 1;
}

MasterPacketQueue::MasterPacketQueue(EventManager& _em, MasterPort& _masterPort,
                                     const std::string _label)
    : PacketQueue(_em, _label), masterPort(_masterPort)
{
}

bool
MasterPacketQueue::sendTiming(PacketPtr pkt, bool send_as_snoop)
{
    // attempt to send the packet and return according to the outcome
    if (!send_as_snoop)
        return masterPort.sendTimingReq(pkt);
    else
        return masterPort.sendTimingSnoopResp(pkt);
}

SlavePacketQueue::SlavePacketQueue(EventManager& _em, SlavePort& _slavePort,
                                   const std::string _label)
    : PacketQueue(_em, _label), slavePort(_slavePort)
{
}

bool
SlavePacketQueue::sendTiming(PacketPtr pkt, bool send_as_snoop)
{
    // we should never have queued snoop requests
    assert(!send_as_snoop);
    return slavePort.sendTimingResp(pkt);
}
