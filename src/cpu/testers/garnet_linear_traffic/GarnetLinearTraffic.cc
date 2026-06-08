/*
 * Copyright (c) 2016 Georgia Institute of Technology
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

#include "cpu/testers/garnet_linear_traffic/GarnetLinearTraffic.hh"

#include <cmath>
#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/statistics.hh"
#include "debug/GarnetLinearTraffic.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

int TESTER_NETWORK_CUSTOM=0;

bool
GarnetLinearTraffic::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tester->completeRequest(pkt);
    return true;
}

void
GarnetLinearTraffic::CpuPort::recvReqRetry()
{
    tester->doRetry();
}

void
GarnetLinearTraffic::sendPkt(PacketPtr pkt)
{
    if (!cachePort.sendTimingReq(pkt)) {
        retryPkt = pkt; // RubyPort will retry sending
    }
    numPacketsSent++;
}

GarnetLinearTraffic::GarnetLinearTraffic(const Params &p)
    : ClockedObject(p),
      tickEvent([this]{ tick(); }, "GarnetLinearTraffic tick",
                false, Event::CPU_Tick_Pri),
      cachePort("GarnetLinearTraffic", this),
      retryPkt(NULL),
      size(p.memory_size),
      blockSizeBits(p.block_offset),
      simCycles(p.sim_cycles),
      injRate(p.packet_rate),
      destinations(p.destinations),
      rates(p.rates),
      responseLimit(p.response_limit),
      requestorId(p.system->getRequestorId(this))

{
    // set up counters
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    id = TESTER_NETWORK_CUSTOM++;

    DPRINTF(GarnetLinearTraffic,"Config Created: Name = %s , and id = %d\n",
            name(), id);
}

Port &
GarnetLinearTraffic::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "test")
        return cachePort;
    else
        return ClockedObject::getPort(if_name, idx);
}

void
GarnetLinearTraffic::init()
{
    numPacketsSent = 0;
}


void
GarnetLinearTraffic::completeRequest(PacketPtr pkt)
{
    DPRINTF(GarnetLinearTraffic,
            "Completed injection of %s packet for address %x\n",
            pkt->isWrite() ? "write" : "read\n",
            pkt->req->getPaddr());

    assert(pkt->isResponse());
    noResponseCycles = 0;
    delete pkt;
}


void
GarnetLinearTraffic::tick()
{
    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }

    if (curTick() * injRate > numPacketsSent)
        generatePkt();

    // Schedule wakeup
    if (curTick() >= simCycles)
        exitSimLoop("Network Tester completed simCycles");
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void
GarnetLinearTraffic::generatePkt()
{
    unsigned destination = id;
    // reset the polling system if the buckets are empty
    int pollsleft = 0;

    for (int i=0;i<poll.size();i++){
        pollsleft += poll[i];
    }
    if (pollsleft==0){
        poll = rates;
        tokensRemaining.assign((int)poll.size(),1.0);
    }
    // find the connection with most pending requests
    float max = 0;
    int pos = 0;
    for (int i=0;i<poll.size();i++){
        if (tokensRemaining[i]>max){
            max = tokensRemaining[i];
            pos = i;
        }
    }
    // reduce token in bucket and set destination
    poll[pos] -= 1;
    tokensRemaining[pos] = float(poll[pos])/float(rates[pos]);
    destination = destinations[pos];

    // The source of the packets is a cache.
    // The destination of the packets is a directory.
    // The destination bits are embedded in the address after byte-offset.
    Addr paddr =  destination;
    paddr <<= blockSizeBits;
    unsigned access_size = 1; // Does not affect Ruby simulation

    // Modeling different coherence msg types over different msg classes.
    //
    // GarnetLinearTraffic assumes the Garnet_standalone coherence protocol
    // which models three message classes/virtual networks.
    // These are: request, forward, response.
    // requests and forwards are "control" packets (typically 8 bytes),
    // while responses are "data" packets (typically 72 bytes).
    //
    // Life of a packet from the tester into the network:
    // (1) This function generatePkt() generates packets of one of the
    //     following 3 types (randomly) : ReadReq, INST_FETCH, WriteReq
    // (2) mem/ruby/system/RubyPort.cc converts these to RubyRequestType_LD,
    //     RubyRequestType_IFETCH, RubyRequestType_ST respectively
    // (3) mem/ruby/system/Sequencer.cc sends these to the cache controllers
    //     in the coherence protocol.
    // (4) Network_test-cache.sm tags RubyRequestType:LD,
    //     RubyRequestType:IFETCH and RubyRequestType:ST as
    //     Request, Forward, and Response events respectively;
    //     and injects them into virtual networks 0, 1 and 2 respectively.
    //     It immediately calls back the sequencer.
    // (5) The packet traverses the network (simple/garnet) and reaches its
    //     destination (Directory), and network stats are updated.
    // (6) Network_test-dir.sm simply drops the packet.
    //
    MemCmd::Command requestType;

    RequestPtr req = nullptr;
    Request::Flags flags;

    // Inject in specific Vnet
    // Vnet 0 and 1 are for control packets (1-flit)
    // Vnet 2 is for data packets (5-flit)
    // using only Vnet 2

    requestType = MemCmd::WriteReq;
    req = std::make_shared<Request>(paddr, access_size, flags,
                                        requestorId);

    req->setContext(id);

    //No need to do functional simulation
    //We just do timing simulation of the network

    DPRINTF(GarnetLinearTraffic,
            "Generated packet with destination %d, embedded in address %x, at %lu\n",
            destination, req->getPaddr(), curTick());

    PacketPtr pkt = new Packet(req, requestType);
    pkt->dataDynamic(new uint8_t[req->getSize()]);
    pkt->senderState = NULL;

    sendPkt(pkt);
}


void
GarnetLinearTraffic::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
GarnetLinearTraffic::printAddr(Addr a)
{
    cachePort.printAddr(a);
}

} // namespace gem5
