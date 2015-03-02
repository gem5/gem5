/*
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
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
 *
 * Authors: Tushar Krishna
 */

#include <cmath>
#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/random.hh"
#include "base/statistics.hh"
#include "cpu/testers/networktest/networktest.hh"
#include "debug/NetworkTest.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace std;

int TESTER_NETWORK=0;

bool
NetworkTest::CpuPort::recvTimingResp(PacketPtr pkt)
{
    networktest->completeRequest(pkt);
    return true;
}

void
NetworkTest::CpuPort::recvReqRetry()
{
    networktest->doRetry();
}

void
NetworkTest::sendPkt(PacketPtr pkt)
{
    if (!cachePort.sendTimingReq(pkt)) {
        retryPkt = pkt; // RubyPort will retry sending
    }
    numPacketsSent++;
}

NetworkTest::NetworkTest(const Params *p)
    : MemObject(p),
      tickEvent(this),
      cachePort("network-test", this),
      retryPkt(NULL),
      size(p->memory_size),
      blockSizeBits(p->block_offset),
      numMemories(p->num_memories),
      simCycles(p->sim_cycles),
      fixedPkts(p->fixed_pkts),
      maxPackets(p->max_packets),
      numPacketsSent(0),
      trafficType(p->traffic_type),
      injRate(p->inj_rate),
      precision(p->precision),
      masterId(p->system->getMasterId(name()))
{
    // set up counters
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    id = TESTER_NETWORK++;
    DPRINTF(NetworkTest,"Config Created: Name = %s , and id = %d\n",
            name(), id);
}

BaseMasterPort &
NetworkTest::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "test")
        return cachePort;
    else
        return MemObject::getMasterPort(if_name, idx);
}

void
NetworkTest::init()
{
    numPacketsSent = 0;
}


void
NetworkTest::completeRequest(PacketPtr pkt)
{
    Request *req = pkt->req;

    DPRINTF(NetworkTest, "Completed injection of %s packet for address %x\n",
            pkt->isWrite() ? "write" : "read\n",
            req->getPaddr());

    assert(pkt->isResponse());
    noResponseCycles = 0;
    delete req;
    delete pkt;
}


void
NetworkTest::tick()
{
    if (++noResponseCycles >= 500000) {
        cerr << name() << ": deadlocked at cycle " << curTick() << endl;
        fatal("");
    }

    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    bool send_this_cycle;
    double injRange = pow((double) 10, (double) precision);
    unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
    if (trySending < injRate*injRange)
        send_this_cycle = true;
    else
        send_this_cycle = false;

    // always generatePkt unless fixedPkts is enabled
    if (send_this_cycle) {
        if (fixedPkts) {
            if (numPacketsSent < maxPackets) {
                generatePkt();
            }
        } else {
            generatePkt();
        }
    }

    // Schedule wakeup
    if (curTick() >= simCycles)
        exitSimLoop("Network Tester completed simCycles");
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void
NetworkTest::generatePkt()
{
    unsigned destination = id;
    if (trafficType == 0) { // Uniform Random
        destination = random_mt.random<unsigned>(0, numMemories - 1);
    } else if (trafficType == 1) { // Tornado
        int networkDimension = (int) sqrt(numMemories);
        int my_x = id%networkDimension;
        int my_y = id/networkDimension;

        int dest_x = my_x + (int) ceil(networkDimension/2) - 1;
        dest_x = dest_x%networkDimension;
        int dest_y = my_y;

        destination = dest_y*networkDimension + dest_x;
    } else if (trafficType == 2) { // Bit Complement
        int networkDimension = (int) sqrt(numMemories);
        int my_x = id%networkDimension;
        int my_y = id/networkDimension;

        int dest_x = networkDimension - my_x - 1;
        int dest_y = networkDimension - my_y - 1;

        destination = dest_y*networkDimension + dest_x;
    }

    // The source of the packets is a cache.
    // The destination of the packets is a directory.
    // The destination bits are embedded in the address after byte-offset.
    Addr paddr =  destination;
    paddr <<= blockSizeBits;
    unsigned access_size = 1; // Does not affect Ruby simulation

    // Modeling different coherence msg types over different msg classes.
    //
    // networktest assumes the Network_test coherence protocol 
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

    Request *req = nullptr;
    Request::Flags flags;

    unsigned randomReqType = random_mt.random(0, 2);
    if (randomReqType == 0) {
        // generate packet for virtual network 0
        requestType = MemCmd::ReadReq;
        req = new Request(paddr, access_size, flags, masterId);
    } else if (randomReqType == 1) {
        // generate packet for virtual network 1
        requestType = MemCmd::ReadReq;
        flags.set(Request::INST_FETCH);
        req = new Request(0, 0x0, access_size, flags, masterId, 0x0, 0, 0);
        req->setPaddr(paddr);
    } else {  // if (randomReqType == 2)
        // generate packet for virtual network 2
        requestType = MemCmd::WriteReq;
        req = new Request(paddr, access_size, flags, masterId);
    }

    req->setThreadContext(id,0);

    //No need to do functional simulation
    //We just do timing simulation of the network

    DPRINTF(NetworkTest,
            "Generated packet with destination %d, embedded in address %x\n",
            destination, req->getPaddr());

    PacketPtr pkt = new Packet(req, requestType);
    pkt->dataDynamic(new uint8_t[req->getSize()]);
    pkt->senderState = NULL;

    sendPkt(pkt);
}

void
NetworkTest::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
NetworkTest::printAddr(Addr a)
{
    cachePort.printAddr(a);
}


NetworkTest *
NetworkTestParams::create()
{
    return new NetworkTest(this);
}
