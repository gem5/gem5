/*
 * Copyright (c) 2012,2015,2017 ARM Limited
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
#include "sim/sim_object.hh"

BaseMasterPort::BaseMasterPort(const std::string &name, PortID _id)
    : Port(name, _id), _baseSlavePort(NULL)
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

BaseSlavePort::BaseSlavePort(const std::string &name, PortID _id)
    : Port(name, _id), _baseMasterPort(NULL)
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

/**
 * Master port
 */
MasterPort::MasterPort(const std::string& name, SimObject* _owner, PortID _id)
    : BaseMasterPort(name, _id), _slavePort(NULL), owner(*_owner)
{
}

MasterPort::~MasterPort()
{
}

void
MasterPort::bind(Port &peer)
{
    auto *slave_port = dynamic_cast<SlavePort *>(&peer);
    if (!slave_port) {
        fatal("Attempt to bind port %s to non-slave port %s.",
                name(), peer.name());
    }
    // bind on the level of the base ports
    _baseSlavePort = slave_port;

    // master port keeps track of the slave port
    _slavePort = slave_port;
    _connected = true;
    // slave port also keeps track of master port
    _slavePort->slaveBind(*this);
}

void
MasterPort::unbind()
{
    if (_slavePort == NULL)
        panic("Attempting to unbind master port %s that is not connected\n",
              name());
    _slavePort->slaveUnbind();
    _slavePort = NULL;
    _connected = false;
    _baseSlavePort = NULL;
}

AddrRangeList
MasterPort::getAddrRanges() const
{
    return _slavePort->getAddrRanges();
}

void
MasterPort::printAddr(Addr a)
{
    auto req = std::make_shared<Request>(
        a, 1, 0, Request::funcMasterId);

    Packet pkt(req, MemCmd::PrintReq);
    Packet::PrintReqState prs(std::cerr);
    pkt.senderState = &prs;

    sendFunctional(&pkt);
}

/**
 * Slave port
 */
SlavePort::SlavePort(const std::string& name, SimObject* _owner, PortID id)
    : BaseSlavePort(name, id), _masterPort(NULL), defaultBackdoorWarned(false),
    owner(*_owner)
{
}

SlavePort::~SlavePort()
{
}

void
SlavePort::slaveUnbind()
{
    _baseMasterPort = NULL;
    _masterPort = NULL;
    _connected = false;
}

void
SlavePort::slaveBind(MasterPort& master_port)
{
    _baseMasterPort = &master_port;
    _masterPort = &master_port;
    _connected = true;
}

Tick
SlavePort::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor)
{
    if (!defaultBackdoorWarned) {
        warn("Port %s doesn't support requesting a back door.", name());
        defaultBackdoorWarned = true;
    }
    return recvAtomic(pkt);
}
