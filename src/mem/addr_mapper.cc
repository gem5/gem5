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

#include "mem/addr_mapper.hh"

namespace gem5
{

AddrMapper::AddrMapper(const AddrMapperParams &p)
    : SimObject(p),
      memSidePort(name() + "-mem_side_port", *this),
      cpuSidePort(name() + "-cpu_side_port", *this)
{}

void
AddrMapper::init()
{
    if (!cpuSidePort.isConnected() || !memSidePort.isConnected())
        fatal("Address mapper is not connected on both sides.\n");
}

Port &
AddrMapper::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side_port") {
        return memSidePort;
    } else if (if_name == "cpu_side_port") {
        return cpuSidePort;
    } else {
        return SimObject::getPort(if_name, idx);
    }
}

void
AddrMapper::recvFunctional(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    memSidePort.sendFunctional(pkt);
    pkt->setAddr(orig_addr);
}

void
AddrMapper::recvFunctionalSnoop(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    cpuSidePort.sendFunctionalSnoop(pkt);
    pkt->setAddr(orig_addr);
}

void
AddrMapper::recvMemBackdoorReq(const MemBackdoorReq &req,
                               MemBackdoorPtr &backdoor)
{
    AddrRange remapped_req_range = AddrRange(remapAddr(req.range().start()),
                                             remapAddr(req.range().end()));
    MemBackdoorReq remapped_req(remapped_req_range, req.flags());
    memSidePort.sendMemBackdoorReq(remapped_req, backdoor);
    if (backdoor != nullptr) {
        backdoor = getRevertedBackdoor(backdoor, req.range());
    }
}

Tick
AddrMapper::recvAtomic(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    Tick ret_tick = memSidePort.sendAtomic(pkt);
    pkt->setAddr(orig_addr);
    return ret_tick;
}

Tick
AddrMapper::recvAtomicSnoop(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    Tick ret_tick = cpuSidePort.sendAtomicSnoop(pkt);
    pkt->setAddr(orig_addr);
    return ret_tick;
}

Tick
AddrMapper::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    Tick ret_tick = memSidePort.sendAtomicBackdoor(pkt, backdoor);
    pkt->setAddr(orig_addr);
    if (backdoor != nullptr) {
        backdoor = getRevertedBackdoor(backdoor, pkt->getAddrRange());
    }
    return ret_tick;
}

bool
AddrMapper::recvTimingReq(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    bool needsResponse = pkt->needsResponse();
    bool cacheResponding = pkt->cacheResponding();

    if (needsResponse && !cacheResponding) {
        pkt->pushSenderState(new AddrMapperSenderState(orig_addr));
    }

    pkt->setAddr(remapAddr(orig_addr));

    // Attempt to send the packet
    bool successful = memSidePort.sendTimingReq(pkt);

    // If not successful, restore the address and sender state
    if (!successful) {
        pkt->setAddr(orig_addr);

        if (needsResponse) {
            delete pkt->popSenderState();
        }
    }

    return successful;
}

bool
AddrMapper::recvTimingResp(PacketPtr pkt)
{
    AddrMapperSenderState *receivedState =
        dynamic_cast<AddrMapperSenderState *>(pkt->senderState);

    // Restore initial sender state
    if (receivedState == NULL)
        panic("AddrMapper %s got a response without sender state\n", name());

    Addr remapped_addr = pkt->getAddr();

    // Restore the state and address
    pkt->senderState = receivedState->predecessor;
    pkt->setAddr(receivedState->origAddr);

    // Attempt to send the packet
    bool successful = cpuSidePort.sendTimingResp(pkt);

    // If packet successfully sent, delete the sender state, otherwise
    // restore state
    if (successful) {
        delete receivedState;
    } else {
        // Don't delete anything and let the packet look like we did
        // not touch it
        pkt->senderState = receivedState;
        pkt->setAddr(remapped_addr);
    }
    return successful;
}

void
AddrMapper::recvTimingSnoopReq(PacketPtr pkt)
{
    cpuSidePort.sendTimingSnoopReq(pkt);
}

bool
AddrMapper::recvTimingSnoopResp(PacketPtr pkt)
{
    return memSidePort.sendTimingSnoopResp(pkt);
}

bool
AddrMapper::isSnooping() const
{
    if (cpuSidePort.isSnooping())
        fatal("AddrMapper doesn't support remapping of snooping requests\n");
    return false;
}

void
AddrMapper::recvReqRetry()
{
    cpuSidePort.sendRetryReq();
}

void
AddrMapper::recvRespRetry()
{
    memSidePort.sendRetryResp();
}

void
AddrMapper::recvRangeChange()
{
    cpuSidePort.sendRangeChange();
}

RangeAddrMapper::RangeAddrMapper(const RangeAddrMapperParams &p)
    : AddrMapper(p),
      originalRanges(p.original_ranges),
      remappedRanges(p.remapped_ranges),
      backdoorManager(originalRanges, remappedRanges)
{
    if (originalRanges.size() != remappedRanges.size())
        fatal("AddrMapper: original and shadowed range list must "
              "be same size\n");

    for (size_t x = 0; x < originalRanges.size(); x++) {
        if (originalRanges[x].size() != remappedRanges[x].size())
            fatal("AddrMapper: original and shadowed range list elements"
                  " aren't all of the same size\n");
    }
}

Addr
RangeAddrMapper::remapAddr(Addr addr) const
{
    for (int i = 0; i < originalRanges.size(); ++i) {
        if (originalRanges[i].contains(addr)) {
            Addr offset = addr - originalRanges[i].start();
            return offset + remappedRanges[i].start();
        }
    }

    return addr;
}

MemBackdoorPtr
RangeAddrMapper::getRevertedBackdoor(MemBackdoorPtr &backdoor,
                                     const AddrRange &range)
{
    return backdoorManager.getRevertedBackdoor(backdoor, range);
}

AddrRangeList
RangeAddrMapper::getAddrRanges() const
{
    // Simply return the original ranges as given by the parameters
    AddrRangeList ranges(originalRanges.begin(), originalRanges.end());
    return ranges;
}

} // namespace gem5
