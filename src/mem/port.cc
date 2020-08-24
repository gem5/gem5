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

namespace
{

class DefaultRequestPort : public RequestPort
{
  protected:
    [[noreturn]] void
    blowUp() const
    {
        throw UnboundPortException();
    }

  public:
    DefaultRequestPort() : RequestPort("default_request_port", nullptr) {}

    // Atomic protocol.
    Tick recvAtomicSnoop(PacketPtr) override { blowUp(); }

    // Timing protocol.
    bool recvTimingResp(PacketPtr) override { blowUp(); }
    void recvTimingSnoopReq(PacketPtr) override { blowUp(); }
    void recvReqRetry() override { blowUp(); }
    void recvRetrySnoopResp() override { blowUp(); }

    // Functional protocol.
    void recvFunctionalSnoop(PacketPtr) override { blowUp(); }
};

class DefaultResponsePort : public ResponsePort
{
  protected:
    [[noreturn]] void
    blowUp() const
    {
        throw UnboundPortException();
    }

  public:
    DefaultResponsePort() : ResponsePort("default_response_port", nullptr) {}

    // Atomic protocol.
    Tick recvAtomic(PacketPtr) override { blowUp(); }

    // Timing protocol.
    bool recvTimingReq(PacketPtr) override { blowUp(); }
    bool tryTiming(PacketPtr) override { blowUp(); }
    bool recvTimingSnoopResp(PacketPtr) override { blowUp(); }
    void recvRespRetry() override { blowUp(); }

    // Functional protocol.
    void recvFunctional(PacketPtr) override { blowUp(); }

    // General.
    AddrRangeList getAddrRanges() const override { return AddrRangeList(); }
};

DefaultRequestPort defaultRequestPort;
DefaultResponsePort defaultResponsePort;

} // anonymous namespace

/**
 * Request port
 */
RequestPort::RequestPort(const std::string& name, SimObject* _owner,
    PortID _id) : Port(name, _id), _responsePort(&defaultResponsePort),
    owner(*_owner)
{
}

RequestPort::~RequestPort()
{
}

void
RequestPort::bind(Port &peer)
{
    auto *response_port = dynamic_cast<ResponsePort *>(&peer);
    fatal_if(!response_port, "Can't bind port %s to non-response port %s.",
             name(), peer.name());
    // request port keeps track of the response port
    _responsePort = response_port;
    Port::bind(peer);
    // response port also keeps track of request port
    _responsePort->responderBind(*this);
}

void
RequestPort::unbind()
{
    panic_if(!isConnected(), "Can't unbind request port %s which is "
    "not bound.", name());
    _responsePort->responderUnbind();
    _responsePort = &defaultResponsePort;
    Port::unbind();
}

AddrRangeList
RequestPort::getAddrRanges() const
{
    return _responsePort->getAddrRanges();
}

void
RequestPort::printAddr(Addr a)
{
    auto req = std::make_shared<Request>(
        a, 1, 0, Request::funcRequestorId);

    Packet pkt(req, MemCmd::PrintReq);
    Packet::PrintReqState prs(std::cerr);
    pkt.senderState = &prs;

    sendFunctional(&pkt);
}

/**
 * Response port
 */
ResponsePort::ResponsePort(const std::string& name, SimObject* _owner,
    PortID id) : Port(name, id), _requestPort(&defaultRequestPort),
    defaultBackdoorWarned(false), owner(*_owner)
{
}

ResponsePort::~ResponsePort()
{
}

void
ResponsePort::responderUnbind()
{
    _requestPort = &defaultRequestPort;
    Port::unbind();
}

void
ResponsePort::responderBind(RequestPort& request_port)
{
    _requestPort = &request_port;
    Port::bind(request_port);
}

Tick
ResponsePort::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor)
{
    if (!defaultBackdoorWarned) {
        warn("Port %s doesn't support requesting a back door.", name());
        defaultBackdoorWarned = true;
    }
    return recvAtomic(pkt);
}
