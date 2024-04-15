// Copyright (c) 2021 The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "sst/outgoing_request_bridge.hh"

#include <cassert>
#include <iomanip>
#include <sstream>

#include "base/trace.hh"

namespace gem5
{

OutgoingRequestBridge::OutgoingRequestBridge(
    const OutgoingRequestBridgeParams &params)
    : SimObject(params),
      outgoingPort(std::string(name()), this),
      sstResponder(nullptr),
      physicalAddressRanges(params.physical_address_ranges.begin(),
                            params.physical_address_ranges.end())
{}

OutgoingRequestBridge::~OutgoingRequestBridge() {}

OutgoingRequestBridge::OutgoingRequestPort::OutgoingRequestPort(
    const std::string &name_, OutgoingRequestBridge *owner_)
    : ResponsePort(name_)
{
    owner = owner_;
}

OutgoingRequestBridge::OutgoingRequestPort::~OutgoingRequestPort() {}

void
OutgoingRequestBridge::init()
{
    if (outgoingPort.isConnected())
        outgoingPort.sendRangeChange();
}

Port &
OutgoingRequestBridge::getPort(const std::string &if_name, PortID idx)
{
    return outgoingPort;
}

AddrRangeList
OutgoingRequestBridge::getAddrRanges() const
{
    return outgoingPort.getAddrRanges();
}

std::vector<std::pair<Addr, std::vector<uint8_t>>>
OutgoingRequestBridge::getInitData() const
{
    return initData;
}

void
OutgoingRequestBridge::setResponder(SSTResponderInterface *responder)
{
    sstResponder = responder;
}

bool
OutgoingRequestBridge::sendTimingResp(gem5::PacketPtr pkt)
{
    return outgoingPort.sendTimingResp(pkt);
}

void
OutgoingRequestBridge::sendTimingSnoopReq(gem5::PacketPtr pkt)
{
    outgoingPort.sendTimingSnoopReq(pkt);
}

void
OutgoingRequestBridge::handleRecvFunctional(PacketPtr pkt)
{
    uint8_t *ptr = pkt->getPtr<uint8_t>();
    uint64_t size = pkt->getSize();
    std::vector<uint8_t> data(ptr, ptr + size);
    initData.push_back(std::make_pair(pkt->getAddr(), data));
}

Tick
OutgoingRequestBridge::OutgoingRequestPort::recvAtomic(PacketPtr pkt)
{
    assert(false && "OutgoingRequestPort::recvAtomic not implemented");
    return Tick();
}

void
OutgoingRequestBridge::OutgoingRequestPort::recvFunctional(PacketPtr pkt)
{
    owner->handleRecvFunctional(pkt);
}

bool
OutgoingRequestBridge::OutgoingRequestPort::recvTimingReq(PacketPtr pkt)
{
    owner->sstResponder->handleRecvTimingReq(pkt);
    return true;
}

void
OutgoingRequestBridge::OutgoingRequestPort::recvRespRetry()
{
    owner->sstResponder->handleRecvRespRetry();
}

AddrRangeList
OutgoingRequestBridge::OutgoingRequestPort::getAddrRanges() const
{
    return owner->physicalAddressRanges;
}

}; // namespace gem5
