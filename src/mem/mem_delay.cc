/*
 * Copyright (c) 2018 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#include "mem/mem_delay.hh"

#include "params/MemDelay.hh"
#include "params/SimpleMemDelay.hh"

MemDelay::MemDelay(const MemDelayParams *p)
    : MemObject(p),
      masterPort(name() + "-master", *this),
      slavePort(name() + "-slave", *this),
      reqQueue(*this, masterPort),
      respQueue(*this, slavePort),
      snoopRespQueue(*this, masterPort)
{
}

void
MemDelay::init()
{
    if (!slavePort.isConnected() || !masterPort.isConnected())
        fatal("Memory delay is not connected on both sides.\n");
}


BaseMasterPort&
MemDelay::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "master") {
        return masterPort;
    } else {
        return MemObject::getMasterPort(if_name, idx);
    }
}

BaseSlavePort&
MemDelay::getSlavePort(const std::string& if_name, PortID idx)
{
    if (if_name == "slave") {
        return slavePort;
    } else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

bool
MemDelay::trySatisfyFunctional(PacketPtr pkt)
{
    return slavePort.trySatisfyFunctional(pkt) ||
        masterPort.trySatisfyFunctional(pkt);
}

MemDelay::MasterPort::MasterPort(const std::string &_name, MemDelay &_parent)
    : QueuedMasterPort(_name, &_parent,
                       _parent.reqQueue, _parent.snoopRespQueue),
      parent(_parent)
{
}

bool
MemDelay::MasterPort::recvTimingResp(PacketPtr pkt)
{
    const Tick when = curTick() + parent.delayResp(pkt);

    parent.slavePort.schedTimingResp(pkt, when);

    return true;
}

void
MemDelay::MasterPort::recvFunctionalSnoop(PacketPtr pkt)
{
    if (parent.trySatisfyFunctional(pkt)) {
        pkt->makeResponse();
    } else {
        parent.slavePort.sendFunctionalSnoop(pkt);
    }
}

Tick
MemDelay::MasterPort::recvAtomicSnoop(PacketPtr pkt)
{
    const Tick delay = parent.delaySnoopResp(pkt);

    return delay + parent.slavePort.sendAtomicSnoop(pkt);
}

void
MemDelay::MasterPort::recvTimingSnoopReq(PacketPtr pkt)
{
    parent.slavePort.sendTimingSnoopReq(pkt);
}


MemDelay::SlavePort::SlavePort(const std::string &_name, MemDelay &_parent)
    : QueuedSlavePort(_name, &_parent, _parent.respQueue),
      parent(_parent)
{
}

Tick
MemDelay::SlavePort::recvAtomic(PacketPtr pkt)
{
    const Tick delay = parent.delayReq(pkt) + parent.delayResp(pkt);

    return delay + parent.masterPort.sendAtomic(pkt);
}

bool
MemDelay::SlavePort::recvTimingReq(PacketPtr pkt)
{
    const Tick when = curTick() + parent.delayReq(pkt);

    parent.masterPort.schedTimingReq(pkt, when);

    return true;
}

void
MemDelay::SlavePort::recvFunctional(PacketPtr pkt)
{
    if (parent.trySatisfyFunctional(pkt)) {
        pkt->makeResponse();
    } else {
        parent.masterPort.sendFunctional(pkt);
    }
}

bool
MemDelay::SlavePort::recvTimingSnoopResp(PacketPtr pkt)
{
    const Tick when = curTick() + parent.delaySnoopResp(pkt);

    parent.masterPort.schedTimingSnoopResp(pkt, when);

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
