/*
 * Copyright (c) 2015, 2021, 2026 Arm Limited
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

#include "salam/testing/device_tester.hh"

#include "base/logging.hh"
#include "sim/system.hh"

namespace gem5
{

SALAMDeviceTester::SALAMDeviceTester(const Params &p)
    : ClockedObject(p),
      port(name() + ".port", *this),
      requestorId(p.system->getRequestorId(this)),
      responseRetryEvent([this] { retryResponse(); }, name())
{}

Port &
SALAMDeviceTester::getPort(const std::string &name, PortID idx)
{
    return name == "port" ? port : ClockedObject::getPort(name, idx);
}

PacketPtr
SALAMDeviceTester::packet(bool read, Addr addr, unsigned size, uint64_t data)
{
    panic_if(size == 0 || size > sizeof(data), "Invalid test access size");
    auto req = std::make_shared<Request>(addr, size, 0, requestorId);
    auto pkt = new Packet(req, read ? MemCmd::ReadReq : MemCmd::WriteReq);
    pkt->allocate();
    for (unsigned i = 0; i < size; ++i) {
        pkt->getPtr<uint8_t>()[i] = data >> (8 * i);
    }
    return pkt;
}

bool
SALAMDeviceTester::sendPacket(PacketPtr pkt)
{
    const auto index = attempts.size();
    attempts.push_back({curTick(), pkt->getAddr(), pkt->isRead(), 0});
    const bool accepted = port.sendTimingReq(pkt);
    attempts[index][3] = accepted;
    if (!accepted) {
        retryPkt = pkt;
    }
    return accepted;
}

bool
SALAMDeviceTester::send(bool read, Addr addr, unsigned size, uint64_t data)
{
    panic_if(retryPkt, "Sending while waiting for request retry");
    return sendPacket(packet(read, addr, size, data));
}

uint64_t
SALAMDeviceTester::functionalRead(Addr addr, unsigned size)
{
    auto pkt = packet(true, addr, size, 0);
    port.sendFunctional(pkt);
    uint64_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        value |= uint64_t(pkt->getConstPtr<uint8_t>()[i]) << (8 * i);
    }
    delete pkt;
    return value;
}

void
SALAMDeviceTester::functionalWrite(Addr addr, unsigned size, uint64_t data)
{
    auto pkt = packet(false, addr, size, data);
    port.sendFunctional(pkt);
    delete pkt;
}

void
SALAMDeviceTester::TestPort::recvReqRetry()
{
    panic_if(!tester.retryPkt, "Unexpected request retry");
    tester.requestRetries.push_back(curTick());
    auto pkt = tester.retryPkt;
    tester.retryPkt = nullptr;
    tester.sendPacket(pkt);
}

bool
SALAMDeviceTester::TestPort::recvTimingResp(PacketPtr pkt)
{
    if (curTick() < tester.rejectUntil) {
        tester.responseRejections.push_back(curTick());
        panic_if(tester.responseBlocked,
                 "Response sent again before the requester retried");
        tester.responseBlocked = true;
        tester.schedule(tester.responseRetryEvent, tester.rejectUntil);
        return false;
    }
    uint64_t value = 0;
    if (pkt->isRead()) {
        for (unsigned i = 0; i < pkt->getSize(); ++i) {
            value |= uint64_t(pkt->getConstPtr<uint8_t>()[i]) << (8 * i);
        }
    }
    tester.responses.push_back(
        {curTick(), pkt->getAddr(), pkt->getSize(), value});
    auto *reader = pkt->isWrite() ? tester.responseReader : nullptr;
    if (reader) {
        tester.responseReader = nullptr;
    }
    delete pkt;
    if (reader) {
        // Issue the dependent access in this response's call stack, just as
        // a requestor may do when a completed store releases its next load.
        reader->send(true, tester.responseReadAddress, tester.responseReadSize,
                     0);
    }
    return true;
}

void
SALAMDeviceTester::retryResponse()
{
    assert(responseBlocked);
    responseBlocked = false;
    port.sendRetryResp();
}

} // namespace gem5
