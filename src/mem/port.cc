/*
 * Copyright (c) 2012,2015 ARM Limited
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
#include "mem/port.hh"

#include "base/trace.hh"
#include "mem/mem_object.hh"

Port::Port(const std::string &_name, MemObject& _owner, PortID _id)
    : portName(_name), id(_id), owner(_owner)
{
}

Port::~Port()
{
}

BaseMasterPort::BaseMasterPort(const std::string& name, MemObject* owner,
                               PortID _id)
    : Port(name, *owner, _id), _baseSlavePort(NULL)
{
}

BaseMasterPort::~BaseMasterPort()
{
}

BaseSlavePort&
BaseMasterPort::getSlavePort() const
{
    if (_baseSlavePort == NULL)
        panic("Cannot getSlavePort on master port %s that is not connected\n",
              name());

    return *_baseSlavePort;
}

bool
BaseMasterPort::isConnected() const
{
    return _baseSlavePort != NULL;
}

BaseSlavePort::BaseSlavePort(const std::string& name, MemObject* owner,
                             PortID _id)
    : Port(name, *owner, _id), _baseMasterPort(NULL)
{
}

BaseSlavePort::~BaseSlavePort()
{
}

BaseMasterPort&
BaseSlavePort::getMasterPort() const
{
    if (_baseMasterPort == NULL)
        panic("Cannot getMasterPort on slave port %s that is not connected\n",
              name());

    return *_baseMasterPort;
}

bool
BaseSlavePort::isConnected() const
{
    return _baseMasterPort != NULL;
}

/**
 * Master port
 */
MasterPort::MasterPort(const std::string& name, MemObject* owner, PortID _id)
    : BaseMasterPort(name, owner, _id), _slavePort(NULL)
{
}

MasterPort::~MasterPort()
{
}

void
MasterPort::bind(BaseSlavePort& slave_port)
{
    // bind on the level of the base ports
    _baseSlavePort = &slave_port;

    // also attempt to base the slave to the appropriate type
    SlavePort* cast_slave_port = dynamic_cast<SlavePort*>(&slave_port);

    // if this port is compatible, then proceed with the binding
    if (cast_slave_port != NULL) {
        // master port keeps track of the slave port
        _slavePort = cast_slave_port;
        // slave port also keeps track of master port
        _slavePort->bind(*this);
    } else {
        fatal("Master port %s cannot bind to %s\n", name(),
              slave_port.name());
    }
}

void
MasterPort::unbind()
{
    if (_slavePort == NULL)
        panic("Attempting to unbind master port %s that is not connected\n",
              name());
    _slavePort->unbind();
    _slavePort = NULL;
    _baseSlavePort = NULL;
}

AddrRangeList
MasterPort::getAddrRanges() const
{
    return _slavePort->getAddrRanges();
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
MasterPort::sendRetryResp()
{
    _slavePort->recvRespRetry();
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
    : BaseSlavePort(name, owner, id), _masterPort(NULL)
{
}

SlavePort::~SlavePort()
{
}

void
SlavePort::unbind()
{
    _baseMasterPort = NULL;
    _masterPort = NULL;
}

void
SlavePort::bind(MasterPort& master_port)
{
    _baseMasterPort = &master_port;
    _masterPort = &master_port;
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

void
SlavePort::sendRetryReq()
{
    _masterPort->recvReqRetry();
}

void
SlavePort::sendRetrySnoopResp()
{
    _masterPort->recvRetrySnoopResp();
}
