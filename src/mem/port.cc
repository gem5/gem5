/*
 * Copyright (c) 2012 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 *          Andreas Hansson
 *          William Wang
 */

/**
 * @file
 * Port object definitions.
 */
#include "base/trace.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"

Port::Port(const std::string &_name, MemObject& _owner, PortID _id)
    : portName(_name), id(_id), peer(NULL), owner(_owner)
{
}

Port::~Port()
{
}

/**
 * Master port
 */
MasterPort::MasterPort(const std::string& name, MemObject* owner, PortID _id)
    : Port(name, *owner, _id), _slavePort(NULL)
{
}

MasterPort::~MasterPort()
{
}

SlavePort&
MasterPort::getSlavePort() const
{
    if(_slavePort == NULL)
        panic("Cannot getSlavePort on master port %s that is not connected\n",
              name());

    return *_slavePort;
}

void
MasterPort::bind(SlavePort& slave_port)
{
    // master port keeps track of the slave port
    _slavePort = &slave_port;
    peer = &slave_port;

    // slave port also keeps track of master port
    _slavePort->bind(*this);
}

bool
MasterPort::isConnected() const
{
    return _slavePort != NULL;
}

unsigned
MasterPort::peerBlockSize() const
{
    return _slavePort->deviceBlockSize();
}

Tick
MasterPort::sendAtomic(PacketPtr pkt)
{
    assert(pkt->isRequest());
    return _slavePort->recvAtomic(pkt);
}

void
MasterPort::sendFunctional(PacketPtr pkt)
{
    assert(pkt->isRequest());
    return _slavePort->recvFunctional(pkt);
}

bool
MasterPort::sendTimingReq(PacketPtr pkt)
{
    assert(pkt->isRequest());
    return _slavePort->recvTimingReq(pkt);
}

bool
MasterPort::sendTimingSnoopResp(PacketPtr pkt)
{
    assert(pkt->isResponse());
    return _slavePort->recvTimingSnoopResp(pkt);
}

void
MasterPort::printAddr(Addr a)
{
    Request req(a, 1, 0, Request::funcMasterId);
    Packet pkt(&req, MemCmd::PrintReq);
    Packet::PrintReqState prs(std::cerr);
    pkt.senderState = &prs;

    sendFunctional(&pkt);
}

/**
 * Slave port
 */
SlavePort::SlavePort(const std::string& name, MemObject* owner, PortID id)
    : Port(name, *owner, id), _masterPort(NULL)
{
}

SlavePort::~SlavePort()
{
}

void
SlavePort::bind(MasterPort& master_port)
{
    _masterPort = &master_port;
    peer = &master_port;
}

MasterPort&
SlavePort::getMasterPort() const
{
    if (_masterPort == NULL)
        panic("Cannot getMasterPort on slave port %s that is not connected\n",
              name());

    return *_masterPort;
}

unsigned
SlavePort::peerBlockSize() const
{
    return _masterPort->deviceBlockSize();
}

bool
SlavePort::isConnected() const
{
    return _masterPort != NULL;
}

Tick
SlavePort::sendAtomicSnoop(PacketPtr pkt)
{
    assert(pkt->isRequest());
    return _masterPort->recvAtomicSnoop(pkt);
}

void
SlavePort::sendFunctionalSnoop(PacketPtr pkt)
{
    assert(pkt->isRequest());
    return _masterPort->recvFunctionalSnoop(pkt);
}

bool
SlavePort::sendTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());
    return _masterPort->recvTimingResp(pkt);
}

void
SlavePort::sendTimingSnoopReq(PacketPtr pkt)
{
    assert(pkt->isRequest());
    _masterPort->recvTimingSnoopReq(pkt);
}
