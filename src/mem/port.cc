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
 */

/**
 * @file
 * Port object definitions.
 */
#include "mem/port.hh"

#include "base/trace.hh"
#include "sim/sim_object.hh"

/**
 * Master port
 */
MasterPort::MasterPort(const std::string& name, SimObject* _owner, PortID _id)
    : Port(name, _id), _slavePort(NULL), owner(*_owner)
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
    // master port keeps track of the slave port
    _slavePort = slave_port;
    Port::bind(peer);
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
    _slavePort = nullptr;
    Port::unbind();
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
    : Port(name, id), _masterPort(NULL), defaultBackdoorWarned(false),
    owner(*_owner)
{
}

SlavePort::~SlavePort()
{
}

void
SlavePort::slaveUnbind()
{
    _masterPort = NULL;
    Port::unbind();
}

void
SlavePort::slaveBind(MasterPort& master_port)
{
    _masterPort = &master_port;
    Port::bind(master_port);
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
