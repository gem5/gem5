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
 *
 * Authors: Andreas Hansson
 */

#include "mem/addr_mapper.hh"

AddrMapper::AddrMapper(const AddrMapperParams* p)
    : MemObject(p),
      masterPort(name() + "-master", *this),
      slavePort(name() + "-slave", *this)
{
}

void
AddrMapper::init()
{
    if (!slavePort.isConnected() || !masterPort.isConnected())
        fatal("Address mapper is not connected on both sides.\n");
}

BaseMasterPort&
AddrMapper::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "master") {
        return masterPort;
    } else {
        return MemObject::getMasterPort(if_name, idx);
    }
}

BaseSlavePort&
AddrMapper::getSlavePort(const std::string& if_name, PortID idx)
{
    if (if_name == "slave") {
        return slavePort;
    } else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

void
AddrMapper::recvFunctional(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    masterPort.sendFunctional(pkt);
    pkt->setAddr(orig_addr);
}

void
AddrMapper::recvFunctionalSnoop(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    slavePort.sendFunctionalSnoop(pkt);
    pkt->setAddr(orig_addr);
}

Tick
AddrMapper::recvAtomic(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    Tick ret_tick =  masterPort.sendAtomic(pkt);
    pkt->setAddr(orig_addr);
    return ret_tick;
}

Tick
AddrMapper::recvAtomicSnoop(PacketPtr pkt)
{
    Addr orig_addr = pkt->getAddr();
    pkt->setAddr(remapAddr(orig_addr));
    Tick ret_tick = slavePort.sendAtomicSnoop(pkt);
    pkt->setAddr(orig_addr);
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
    bool successful = masterPort.sendTimingReq(pkt);

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
    AddrMapperSenderState* receivedState =
        dynamic_cast<AddrMapperSenderState*>(pkt->senderState);

    // Restore initial sender state
    if (receivedState == NULL)
        panic("AddrMapper %s got a response without sender state\n",
              name());

    Addr remapped_addr = pkt->getAddr();

    // Restore the state and address
    pkt->senderState = receivedState->predecessor;
    pkt->setAddr(receivedState->origAddr);

    // Attempt to send the packet
    bool successful = slavePort.sendTimingResp(pkt);

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
    slavePort.sendTimingSnoopReq(pkt);
}

bool
AddrMapper::recvTimingSnoopResp(PacketPtr pkt)
{
    return masterPort.sendTimingSnoopResp(pkt);
}

bool
AddrMapper::isSnooping() const
{
    if (slavePort.isSnooping())
        fatal("AddrMapper doesn't support remapping of snooping requests\n");
    return false;
}

void
AddrMapper::recvReqRetry()
{
    slavePort.sendRetryReq();
}

void
AddrMapper::recvRespRetry()
{
    masterPort.sendRetryResp();
}

void
AddrMapper::recvRangeChange()
{
    slavePort.sendRangeChange();
}

RangeAddrMapper::RangeAddrMapper(const RangeAddrMapperParams* p) :
    AddrMapper(p),
    originalRanges(p->original_ranges),
    remappedRanges(p->remapped_ranges)
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

RangeAddrMapper*
RangeAddrMapperParams::create()
{
    return new RangeAddrMapper(this);
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

AddrRangeList
RangeAddrMapper::getAddrRanges() const
{
    // Simply return the original ranges as given by the parameters
    AddrRangeList ranges(originalRanges.begin(), originalRanges.end());
    return ranges;
}


