/*
 * Copyright (c) 2011-2015, 2018-2019 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Definition of a non-coherent crossbar object.
 */

#include "mem/noncoherent_xbar.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/NoncoherentXBar.hh"
#include "debug/XBar.hh"

namespace gem5
{

NoncoherentXBar::NoncoherentXBar(const NoncoherentXBarParams &p)
    : BaseXBar(p)
{
    // create the ports based on the size of the memory-side port and
    // CPU-side port vector ports, and the presence of the default port,
    // the ports are enumerated starting from zero
    for (int i = 0; i < p.port_mem_side_ports_connection_count; ++i) {
        std::string portName = csprintf("%s.mem_side_port[%d]", name(), i);
        RequestPort* bp = new NoncoherentXBarRequestPort(portName, *this, i);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this,
                                         csprintf("reqLayer%d", i)));
    }

    // see if we have a default CPU-side-port device connected and if so add
    // our corresponding memory-side port
    if (p.port_default_connection_count) {
        defaultPortID = memSidePorts.size();
        std::string portName = name() + ".default";
        RequestPort* bp = new NoncoherentXBarRequestPort(portName, *this,
                                                      defaultPortID);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this, csprintf("reqLayer%d",
                                                              defaultPortID)));
    }

    // create the CPU-side ports, once again starting at zero
    for (int i = 0; i < p.port_cpu_side_ports_connection_count; ++i) {
        std::string portName = csprintf("%s.cpu_side_ports[%d]", name(), i);
        QueuedResponsePort* bp = new NoncoherentXBarResponsePort(portName,
                                                                *this, i);
        cpuSidePorts.push_back(bp);
        respLayers.push_back(new RespLayer(*bp, *this,
                                           csprintf("respLayer%d", i)));
    }
}

NoncoherentXBar::~NoncoherentXBar()
{
    for (auto l: reqLayers)
        delete l;
    for (auto l: respLayers)
        delete l;
}

bool
NoncoherentXBar::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id)
{
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    // we should never see express snoops on a non-coherent crossbar
    assert(!pkt->isExpressSnoop());

    // determine the destination based on the address
    PortID mem_side_port_id = findPort(pkt->getAddrRange());

    // test if the layer should be considered occupied for the current
    // port
    if (!reqLayers[mem_side_port_id]->tryTiming(src_port)) {
        DPRINTF(NoncoherentXBar, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(NoncoherentXBar, "recvTimingReq: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // store the old header delay so we can restore it if needed
    Tick old_header_delay = pkt->headerDelay;

    // a request sees the frontend and forward latency
    Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // before forwarding the packet (and possibly altering it),
    // remember if we are expecting a response
    const bool expect_response = pkt->needsResponse() &&
        !pkt->cacheResponding();

    // since it is a normal request, attempt to send the packet
    bool success = memSidePorts[mem_side_port_id]->sendTimingReq(pkt);

    if (!success)  {
        DPRINTF(NoncoherentXBar, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        // restore the header delay as it is additive
        pkt->headerDelay = old_header_delay;

        // occupy until the header is sent
        reqLayers[mem_side_port_id]->failedTiming(src_port,
                                                clockEdge(Cycles(1)));

        return false;
    }

    // remember where to route the response to
    if (expect_response) {
        assert(routeTo.find(pkt->req) == routeTo.end());
        routeTo[pkt->req] = cpu_side_port_id;
    }

    reqLayers[mem_side_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

bool
NoncoherentXBar::recvTimingResp(PacketPtr pkt, PortID mem_side_port_id)
{
    // determine the source port based on the id
    RequestPort *src_port = memSidePorts[mem_side_port_id];

    // determine the destination
    const auto route_lookup = routeTo.find(pkt->req);
    assert(route_lookup != routeTo.end());
    const PortID cpu_side_port_id = route_lookup->second;
    assert(cpu_side_port_id != InvalidPortID);
    assert(cpu_side_port_id < respLayers.size());

    // test if the layer should be considered occupied for the current
    // port
    if (!respLayers[cpu_side_port_id]->tryTiming(src_port)) {
        DPRINTF(NoncoherentXBar, "recvTimingResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(NoncoherentXBar, "recvTimingResp: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // a response sees the response latency
    Tick xbar_delay = responseLatency * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // send the packet through the destination CPU-side port, and pay for
    // any outstanding latency
    Tick latency = pkt->headerDelay;
    pkt->headerDelay = 0;
    cpuSidePorts[cpu_side_port_id]->schedTimingResp(pkt,
                                        curTick() + latency);

    // remove the request from the routing table
    routeTo.erase(route_lookup);

    respLayers[cpu_side_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

void
NoncoherentXBar::recvReqRetry(PortID mem_side_port_id)
{
    // responses never block on forwarding them, so the retry will
    // always be coming from a port to which we tried to forward a
    // request
    reqLayers[mem_side_port_id]->recvRetry();
}

Tick
NoncoherentXBar::recvAtomicBackdoor(PacketPtr pkt, PortID cpu_side_port_id,
                                    MemBackdoorPtr *backdoor)
{
    DPRINTF(NoncoherentXBar, "recvAtomic: packet src %s addr 0x%x cmd %s\n",
            cpuSidePorts[cpu_side_port_id]->name(), pkt->getAddr(),
            pkt->cmdString());

    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // determine the destination port
    PortID mem_side_port_id = findPort(pkt->getAddrRange());

    // stats updates for the request
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    // forward the request to the appropriate destination
    auto mem_side_port = memSidePorts[mem_side_port_id];
    Tick response_latency = backdoor ?
        mem_side_port->sendAtomicBackdoor(pkt, *backdoor) :
        mem_side_port->sendAtomic(pkt);

    // add the response data
    if (pkt->isResponse()) {
        pkt_size = pkt->hasData() ? pkt->getSize() : 0;
        pkt_cmd = pkt->cmdToIndex();

        // stats updates
        pktCount[cpu_side_port_id][mem_side_port_id]++;
        pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
        transDist[pkt_cmd]++;
    }

    // @todo: Not setting first-word time
    pkt->payloadDelay = response_latency;
    return response_latency;
}

void
NoncoherentXBar::recvFunctional(PacketPtr pkt, PortID cpu_side_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(NoncoherentXBar,
                "recvFunctional: packet src %s addr 0x%x cmd %s\n",
                cpuSidePorts[cpu_side_port_id]->name(), pkt->getAddr(),
                pkt->cmdString());
    }

    // since our CPU-side ports are queued ports we need to check them as well
    for (const auto& p : cpuSidePorts) {
        // if we find a response that has the data, then the
        // downstream caches/memories may be out of date, so simply stop
        // here
        if (p->trySatisfyFunctional(pkt)) {
            if (pkt->needsResponse())
                pkt->makeResponse();
            return;
        }
    }

    // determine the destination port
    PortID dest_id = findPort(pkt->getAddrRange());

    // forward the request to the appropriate destination
    memSidePorts[dest_id]->sendFunctional(pkt);
}

} // namespace gem5
