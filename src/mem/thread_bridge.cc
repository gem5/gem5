/*
 * Copyright 2022 Google, LLC
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

#include "mem/thread_bridge.hh"

#include "base/trace.hh"
#include "sim/eventq.hh"

namespace gem5
{

ThreadBridge::ThreadBridge(const ThreadBridgeParams &p)
    : SimObject(p), in_port_("in_port", *this), out_port_("out_port", *this)
{
}

ThreadBridge::IncomingPort::IncomingPort(const std::string &name,
                                         ThreadBridge &device)
    : ResponsePort(name, &device), device_(device)
{
}

AddrRangeList
ThreadBridge::IncomingPort::getAddrRanges() const
{
    return device_.out_port_.getAddrRanges();
}

// TimingResponseProtocol
bool
ThreadBridge::IncomingPort::recvTimingReq(PacketPtr pkt)
{
    panic("ThreadBridge only supports atomic/functional access.");
}
void
ThreadBridge::IncomingPort::recvRespRetry()
{
    panic("ThreadBridge only supports atomic/functional access.");
}

// AtomicResponseProtocol
Tick
ThreadBridge::IncomingPort::recvAtomicBackdoor(PacketPtr pkt,
                                               MemBackdoorPtr &backdoor)
{
    panic("ThreadBridge only supports atomic/functional access.");
}
Tick
ThreadBridge::IncomingPort::recvAtomic(PacketPtr pkt)
{
    EventQueue::ScopedMigration migrate(device_.eventQueue());
    return device_.out_port_.sendAtomic(pkt);
}

// FunctionalResponseProtocol
void
ThreadBridge::IncomingPort::recvFunctional(PacketPtr pkt)
{
    EventQueue::ScopedMigration migrate(device_.eventQueue());
    device_.out_port_.sendFunctional(pkt);
}

void
ThreadBridge::IncomingPort::recvMemBackdoorReq(const MemBackdoorReq &req,
                                               MemBackdoorPtr &backdoor)
{
    EventQueue::ScopedMigration migrate(device_.eventQueue());
    device_.out_port_.sendMemBackdoorReq(req, backdoor);
}

ThreadBridge::OutgoingPort::OutgoingPort(const std::string &name,
                                         ThreadBridge &device)
    : RequestPort(name, &device), device_(device)
{
}

void
ThreadBridge::OutgoingPort::recvRangeChange()
{
    device_.in_port_.sendRangeChange();
}

// TimingRequestProtocol
bool
ThreadBridge::OutgoingPort::recvTimingResp(PacketPtr pkt)
{
    panic("ThreadBridge only supports atomic/functional access.");
}
void
ThreadBridge::OutgoingPort::recvReqRetry()
{
    panic("ThreadBridge only supports atomic/functional access.");
}

Port &
ThreadBridge::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "in_port")
        return in_port_;
    if (if_name == "out_port")
        return out_port_;
    return SimObject::getPort(if_name, idx);
}

}  // namespace gem5
