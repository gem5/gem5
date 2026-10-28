/*
 * Copyright (c) 2026 ZhangJian
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

#include "mem/timing_thread_bridge.hh"

#include "base/trace.hh"
#include "debug/Bridge.hh"
#include "sim/eventq.hh"

namespace gem5
{

TimingThreadBridge::TimingThreadBridge(const Params &p)
    : ClockedObject(p),
      inPort(p.name + ".in_port", *this, p.ranges),
      outPort(p.name + ".out_port", *this),
      delay(p.delay),
      peerQueueIdx(p.peer_queue),
      reqFreeSlots(p.req_size),
      respFreeSlots(p.resp_size),
      retryReq(false),
      reqNotifyPending(false)
{
}

Port &
TimingThreadBridge::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "in_port") {
        return inPort;
    } else if (if_name == "out_port") {
        return outPort;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

void
TimingThreadBridge::init()
{
    if (!inPort.isConnected() || !outPort.isConnected()) {
        fatal("Both ports of a TimingThreadBridge must be connected.\n");
    }

    inPort.sendRangeChange();
}

void
TimingThreadBridge::scheduleOnPeer(std::function<void()> fn, Tick when)
{
    EventQueue *peer = getEventQueue(peerQueueIdx);
    peer->schedule(new EventFunctionWrapper(fn, name() + ".peerEvent", true),
                   when);
}

void
TimingThreadBridge::enqueueReq(PacketPtr pkt)
{
    reqQueue.push_back(pkt);
    trySendReq();
}

void
TimingThreadBridge::trySendReq()
{
    while (!reqQueue.empty()) {
        PacketPtr pkt = reqQueue.front();
        if (outPort.sendTimingReq(pkt)) {
            reqQueue.pop_front();
            reqFreeSlots.fetch_add(1);
            notifyReqSpace();
        } else {
            break;
        }
    }
}

void
TimingThreadBridge::notifyReqSpace()
{
    bool expected = false;
    if (reqNotifyPending.compare_exchange_strong(expected, true)) {
        Tick when = curTick() + delay * clockPeriod();
        schedule(new EventFunctionWrapper([this] { drainReqNotify(); },
                                          name() + ".drainReqNotify", true),
                 when);
    }
}

void
TimingThreadBridge::drainReqNotify()
{
    reqNotifyPending.store(false);
    if (retryReq && reqFreeSlots.load() > 0 && respFreeSlots > 0) {
        retryReq = false;
        inPort.sendRetryReq();
    }
}

void
TimingThreadBridge::enqueueResp(PacketPtr pkt)
{
    respQueue.push_back(pkt);
    trySendResp();
}

void
TimingThreadBridge::trySendResp()
{
    while (!respQueue.empty()) {
        PacketPtr pkt = respQueue.front();
        if (inPort.sendTimingResp(pkt)) {
            respQueue.pop_front();
            ++respFreeSlots;

            if (retryReq && reqFreeSlots.load() > 0 && respFreeSlots > 0) {
                retryReq = false;
                inPort.sendRetryReq();
            }
        } else {
            break;
        }
    }
}

bool
TimingThreadBridge::BridgeResponsePort::recvTimingReq(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(),
             "Should not see packets where cache is responding");

    if (bridge.retryReq) {
        return false;
    }

    DPRINTF(Bridge, "TimingThreadBridge recvTimingReq %s addr %#x\n",
            pkt->cmdString(), pkt->getAddr());

    const bool expects_response = pkt->needsResponse();
    if (bridge.reqFreeSlots.load() == 0 ||
        (expects_response && bridge.respFreeSlots == 0)) {
        bridge.retryReq = true;
        return false;
    }

    bridge.reqFreeSlots.fetch_sub(1);
    if (expects_response) {
        --bridge.respFreeSlots;
    }

    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;

    Tick when = curTick() + bridge.delay * bridge.clockPeriod() +
                receive_delay;
    bridge.scheduleOnPeer([this, pkt] { bridge.enqueueReq(pkt); }, when);
    return true;
}

bool
TimingThreadBridge::BridgeRequestPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(Bridge, "TimingThreadBridge recvTimingResp %s addr %#x\n",
            pkt->cmdString(), pkt->getAddr());

    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;

    Tick when = curTick() + bridge.delay * bridge.clockPeriod() +
                receive_delay;
    bridge.schedule(
        new EventFunctionWrapper([this, pkt] { bridge.enqueueResp(pkt); },
                                 bridge.name() + ".enqueueResp", true),
        when);
    return true;
}

void
TimingThreadBridge::BridgeResponsePort::recvRespRetry()
{
    bridge.trySendResp();
}

void
TimingThreadBridge::BridgeRequestPort::recvReqRetry()
{
    bridge.trySendReq();
}

Tick
TimingThreadBridge::BridgeResponsePort::recvAtomic(PacketPtr pkt)
{
    EventQueue::ScopedMigration migrate(
        getEventQueue(bridge.peerQueueIdx));
    return bridge.delay * bridge.clockPeriod() +
           bridge.outPort.sendAtomic(pkt);
}

void
TimingThreadBridge::BridgeResponsePort::recvFunctional(PacketPtr pkt)
{
    EventQueue::ScopedMigration migrate(
        getEventQueue(bridge.peerQueueIdx));
    bridge.outPort.sendFunctional(pkt);
}

AddrRangeList
TimingThreadBridge::BridgeResponsePort::getAddrRanges() const
{
    return ranges;
}

} // namespace gem5
