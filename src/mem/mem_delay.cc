/*
 * Copyright (c) 2018, 2020 ARM Limited
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

#include "mem/mem_delay.hh"

#include "params/MemDelay.hh"
#include "params/SimpleMemDelay.hh"

MemDelay::MemDelay(const MemDelayParams *p)
    : ClockedObject(p),
      requestPort(name() + "-mem_side_port", *this),
      responsePort(name() + "-cpu_side_port", *this),
      reqQueue(*this, requestPort),
      respQueue(*this, responsePort),
      snoopRespQueue(*this, requestPort)
{
}

void
MemDelay::init()
{
    if (!responsePort.isConnected() || !requestPort.isConnected())
        fatal("Memory delay is not connected on both sides.\n");
}


Port &
MemDelay::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side_port") {
        return requestPort;
    } else if (if_name == "cpu_side_port") {
        return responsePort;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

bool
MemDelay::trySatisfyFunctional(PacketPtr pkt)
{
    return responsePort.trySatisfyFunctional(pkt) ||
        requestPort.trySatisfyFunctional(pkt);
}

MemDelay::RequestPort::RequestPort(const std::string &_name, MemDelay &_parent)
    : QueuedRequestPort(_name, &_parent,
                       _parent.reqQueue, _parent.snoopRespQueue),
      parent(_parent)
{
}

bool
MemDelay::RequestPort::recvTimingResp(PacketPtr pkt)
{
    // technically the packet only reaches us after the header delay,
    // and typically we also need to deserialise any payload
    const Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;

    const Tick when = curTick() + parent.delayResp(pkt) + receive_delay;

    parent.responsePort.schedTimingResp(pkt, when);

    return true;
}

void
MemDelay::RequestPort::recvFunctionalSnoop(PacketPtr pkt)
{
    if (parent.trySatisfyFunctional(pkt)) {
        pkt->makeResponse();
    } else {
        parent.responsePort.sendFunctionalSnoop(pkt);
    }
}

Tick
MemDelay::RequestPort::recvAtomicSnoop(PacketPtr pkt)
{
    const Tick delay = parent.delaySnoopResp(pkt);

    return delay + parent.responsePort.sendAtomicSnoop(pkt);
}

void
MemDelay::RequestPort::recvTimingSnoopReq(PacketPtr pkt)
{
    parent.responsePort.sendTimingSnoopReq(pkt);
}


MemDelay::ResponsePort::
ResponsePort(const std::string &_name, MemDelay &_parent)
    : QueuedResponsePort(_name, &_parent, _parent.respQueue),
      parent(_parent)
{
}

Tick
MemDelay::ResponsePort::recvAtomic(PacketPtr pkt)
{
    const Tick delay = parent.delayReq(pkt) + parent.delayResp(pkt);

    return delay + parent.requestPort.sendAtomic(pkt);
}

bool
MemDelay::ResponsePort::recvTimingReq(PacketPtr pkt)
{
    // technically the packet only reaches us after the header
    // delay, and typically we also need to deserialise any
    // payload
    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;

    const Tick when = curTick() + parent.delayReq(pkt) + receive_delay;

    parent.requestPort.schedTimingReq(pkt, when);

    return true;
}

void
MemDelay::ResponsePort::recvFunctional(PacketPtr pkt)
{
    if (parent.trySatisfyFunctional(pkt)) {
        pkt->makeResponse();
    } else {
        parent.requestPort.sendFunctional(pkt);
    }
}

bool
MemDelay::ResponsePort::recvTimingSnoopResp(PacketPtr pkt)
{
    const Tick when = curTick() + parent.delaySnoopResp(pkt);

    parent.requestPort.schedTimingSnoopResp(pkt, when);

    return true;
}



SimpleMemDelay::SimpleMemDelay(const SimpleMemDelayParams *p)
    : MemDelay(p),
      readReqDelay(p->read_req),
      readRespDelay(p->read_resp),
      writeReqDelay(p->write_req),
      writeRespDelay(p->write_resp)
{
}

Tick
SimpleMemDelay::delayReq(PacketPtr pkt)
{
    if (pkt->isRead()) {
        return readReqDelay;
    } else if (pkt->isWrite()) {
        return writeReqDelay;
    } else {
        return 0;
    }
}

Tick
SimpleMemDelay::delayResp(PacketPtr pkt)
{
    if (pkt->isRead()) {
        return readRespDelay;
    } else if (pkt->isWrite()) {
        return writeRespDelay;
    } else {
        return 0;
    }
}


SimpleMemDelay *
SimpleMemDelayParams::create()
{
    return new SimpleMemDelay(this);
}
