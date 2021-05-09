/*
 * Copyright (c) 2015 ARM Limited
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

#ifndef __SIM_PROBE_MEM_HH__
#define __SIM_PROBE_MEM_HH__

#include <memory>

#include "mem/packet.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(ProbePoints, probing);
namespace probing
{

/**
 * A struct to hold on to the essential fields from a packet, so that
 * the packet and underlying request can be safely passed on, and
 * consequently modified or even deleted.
 */
struct PacketInfo
{
    MemCmd cmd;
    Addr addr;
    uint32_t size;
    Request::FlagsType flags;
    Addr pc;
    RequestorID id;

    explicit PacketInfo(const PacketPtr& pkt) :
        cmd(pkt->cmd),
        addr(pkt->getAddr()),
        size(pkt->getSize()),
        flags(pkt->req->getFlags()),
        pc(pkt->req->hasPC() ? pkt->req->getPC() : 0),
        id(pkt->req->requestorId())  { }
};

/**
 * Packet probe point
 *
 * This probe point provides a unified interface for components that
 * want to instrument Packets in the memory system. Components should
 * when possible adhere to the following naming scheme:
 *
 * <ul>
 *
 *   <li>PktRequest: Requests sent out on the memory side of a normal
 *       components and incoming requests for memories. Packets should
 *       not be duplicated (i.e., a packet should only appear once
 *       irrespective of the receiving end requesting a retry).
 *
 *   <li>PktResponse: Response received from the memory side of a
 *       normal component or a response being sent out from a memory.
 *
 *   <li>PktRequestCPU: Incoming, accepted, memory request on the CPU
 *       side of a two-sided component. This probe point is primarily
 *       intended for components that cache or forward requests (e.g.,
 *       caches and XBars), single-sided components should use
 *       PktRequest instead. The probe point should only be called
 *       when a packet is accepted.
 *
 *   <li>PktResponseCPU: Outgoing response memory request on the CPU
 *       side of a two-sided component. This probe point is primarily
 *       intended for components that cache or forward requests (e.g.,
 *       caches and XBars), single-sided components should use
 *       PktRequest instead.
 *
 * </ul>
 *
 */
typedef ProbePointArg<PacketInfo> Packet;
typedef std::unique_ptr<Packet> PacketUPtr;

} // namespace probing

} // namespace gem5

#endif
