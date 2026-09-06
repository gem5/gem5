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

#include "cpu/testers/betaar_synthetic_traffic/BetaarSyntheticTraffic.hh"

#include <cmath>
#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/statistics.hh"
#include "debug/BetaarSyntheticTraffic.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

int BETAAR_TESTER_NETWORK = 0;

bool
BetaarSyntheticTraffic::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tester->completeRequest(pkt);
    return true;
}

void
BetaarSyntheticTraffic::CpuPort::recvReqRetry()
{
    tester->doRetry();
}

void
BetaarSyntheticTraffic::sendPkt(PacketPtr pkt)
{
    if (!cachePort.sendTimingReq(pkt)) {
        retryPkt = pkt; // RubyPort will retry sending
    }
    numPacketsSent++;
}

BetaarSyntheticTraffic::BetaarSyntheticTraffic(const Params &p)
    : ClockedObject(p),
      tickEvent([this] { tick(); }, "BetaarSyntheticTraffic tick", false,
                Event::CPU_Tick_Pri),
      cachePort("BetaarSyntheticTraffic", this),
      retryPkt(NULL),
      size(p.memory_size),
      blockSizeBits(p.block_offset),
      numDestinations(p.num_dest),
      simCycles(p.sim_cycles),
      numPacketsMax(p.num_packets_max),
      numPacketsSent(0),
      singleSender(p.single_sender),
      singleDest(p.single_dest),
      trafficType(p.traffic_type),
      injRate(p.inj_rate),
      injVnet(p.inj_vnet),
      precision(p.precision),
      responseLimit(p.response_limit),
      requestorId(p.system->getRequestorId(this))
{
    // set up counters
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    initTrafficType();
    if (trafficStringToEnum.count(trafficType) == 0) {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }
    traffic = trafficStringToEnum[trafficType];

    id = BETAAR_TESTER_NETWORK++;
    // Initialize random generator with unique seed based on tester ID
    rng = Random::genRandom(Random::globalSeed + id);
    DPRINTF(BetaarSyntheticTraffic,
            "Config Created: Name = %s , and id = %d\n", name(), id);
}

Port &
BetaarSyntheticTraffic::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "test") {
        return cachePort;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

void
BetaarSyntheticTraffic::init()
{
    numPacketsSent = 0;
}

void
BetaarSyntheticTraffic::completeRequest(PacketPtr pkt)
{
    DPRINTF(BetaarSyntheticTraffic,
            "Completed injection of %s packet for address %x\n",
            pkt->isWrite() ? "write" : "read\n", pkt->req->getPaddr());

    assert(pkt->isResponse());
    noResponseCycles = 0;
    delete pkt;
}

void
BetaarSyntheticTraffic::tick()
{
    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }

    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    bool sendAllowedThisCycle;
    double injRange = pow((double)10, (double)precision);
    unsigned trySending = rng->random<unsigned>(0, (int)injRange);
    if (trySending < injRate * injRange) {
        sendAllowedThisCycle = true;
    } else {
        sendAllowedThisCycle = false;
    }

    // always generatePkt unless fixedPkts or singleSender is enabled
    if (sendAllowedThisCycle) {
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax) {
            senderEnable = false;
        }

        if (singleSender >= 0 && id != singleSender) {
            senderEnable = false;
        }

        if (senderEnable) {
            generatePkt();
        }
    }

    // Schedule wakeup
    if (curTick() >= simCycles) {
        exitSimLoop("Network Tester completed simCycles");
    } else {
        if (!tickEvent.scheduled()) {
            schedule(tickEvent, clockEdge(Cycles(1)));
        }
    }
}

void
BetaarSyntheticTraffic::generatePkt()
{
    int num_destinations = numDestinations;
    int radix = (int)sqrt(num_destinations);
    unsigned destination = id;
    int dest_x = -1;
    int dest_y = -1;
    int source = id;
    int src_x = id % radix;
    int src_y = id / radix;

    // Destination set for this packet. For every traffic pattern except
    // UNIFORM_RANDOM_ this holds exactly the single computed `destination`
    // (unchanged behavior). For UNIFORM_RANDOM_, this is a randomly sized
    // (2..num_destinations), randomly chosen set of distinct destinations --
    // the first building block towards multicast packet generation.
    std::vector<unsigned> destinations;

    if (singleDest >= 0) {
        destination = singleDest;
    } else if (traffic == UNIFORM_RANDOM_) {
        unsigned sz = rng->random<unsigned>(2, num_destinations);
        std::set<unsigned> destSet;
        while (destSet.size() < sz) {
            destSet.insert(rng->random<unsigned>(0, num_destinations - 1));
        }
        destinations.assign(destSet.begin(), destSet.end());
        destination = destinations[0];
    } else if (traffic == BIT_COMPLEMENT_) {
        dest_x = radix - src_x - 1;
        dest_y = radix - src_y - 1;
        destination = dest_y * radix + dest_x;
    } else if (traffic == BIT_REVERSE_) {
        unsigned int straight = source;
        unsigned int reverse = source & 1; // LSB

        int num_bits = (int)log2(num_destinations);

        for (int i = 1; i < num_bits; i++) {
            reverse <<= 1;
            straight >>= 1;
            reverse |= (straight & 1); // LSB
        }
        destination = reverse;
    } else if (traffic == BIT_ROTATION_) {
        if (source % 2 == 0) {
            destination = source / 2;
        } else { // (source%2 == 1)
            destination = ((source / 2) + (num_destinations / 2));
        }
    } else if (traffic == NEIGHBOR_) {
        dest_x = (src_x + 1) % radix;
        dest_y = src_y;
        destination = dest_y * radix + dest_x;
    } else if (traffic == SHUFFLE_) {
        if (source < num_destinations / 2) {
            destination = source * 2;
        } else {
            destination = (source * 2 - num_destinations + 1);
        }
    } else if (traffic == TRANSPOSE_) {
        dest_x = src_y;
        dest_y = src_x;
        destination = dest_y * radix + dest_x;
    } else if (traffic == TORNADO_) {
        dest_x = (src_x + (int)ceil(radix / 2) - 1) % radix;
        dest_y = src_y;
        destination = dest_y * radix + dest_x;
    } else {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }

    if (destinations.empty()) {
        destinations.push_back(destination);
    }

    DPRINTF(BetaarSyntheticTraffic,
            "Generated destination set of size %d for source %d\n",
            destinations.size(), source);

    for (unsigned dest : destinations) {
        // The source of the packets is a cache.
        // The destination of the packets is a directory.
        // The destination bits are embedded in the address after
        // byte-offset.
        Addr paddr = dest;
        paddr <<= blockSizeBits;
        unsigned access_size = 1; // Does not affect Ruby simulation

        // Modeling different coherence msg types over different msg
        // classes.
        //
        // BetaarSyntheticTraffic assumes the Garnet_standalone coherence
        // protocol which models three message classes/virtual networks.
        // These are: request, forward, response.
        // requests and forwards are "control" packets (typically 8 bytes),
        // while responses are "data" packets (typically 72 bytes).
        //
        // Life of a packet from the tester into the network:
        // (1) This function generatePkt() generates packets of one of the
        //     following 3 types (randomly) : ReadReq, INST_FETCH, WriteReq
        // (2) mem/ruby/system/RubyPort.cc converts these to
        //     RubyRequestType_LD, RubyRequestType_IFETCH, RubyRequestType_ST
        //     respectively
        // (3) mem/ruby/system/Sequencer.cc sends these to the cache
        //     controllers in the coherence protocol.
        // (4) Network_test-cache.sm tags RubyRequestType:LD,
        //     RubyRequestType:IFETCH and RubyRequestType:ST as
        //     Request, Forward, and Response events respectively;
        //     and injects them into virtual networks 0, 1 and 2
        //     respectively. It immediately calls back the sequencer.
        // (5) The packet traverses the network (simple/betaar) and reaches
        //     its destination (Directory), and network stats are updated.
        // (6) Network_test-dir.sm simply drops the packet.
        //
        MemCmd::Command requestType;

        RequestPtr req = nullptr;
        Request::Flags flags;

        // Inject in specific Vnet
        // Vnet 0 and 1 are for control packets (1-flit)
        // Vnet 2 is for data packets (5-flit)
        int injReqType = injVnet;

        if (injReqType < 0 || injReqType > 2) {
            // randomly inject in any vnet
            injReqType = rng->random(0, 2);
        }

        if (injReqType == 0) {
            // generate packet for virtual network 0
            requestType = MemCmd::ReadReq;
            req = std::make_shared<Request>(paddr, access_size, flags,
                                            requestorId);
        } else if (injReqType == 1) {
            // generate packet for virtual network 1
            requestType = MemCmd::ReadReq;
            flags.set(Request::INST_FETCH);
            req = std::make_shared<Request>(0x0, access_size, flags,
                                            requestorId, 0x0, 0);
            req->setPaddr(paddr);
        } else { // if (injReqType == 2)
            // generate packet for virtual network 2
            requestType = MemCmd::WriteReq;
            req = std::make_shared<Request>(paddr, access_size, flags,
                                            requestorId);
        }

        req->setContext(id);

        // No need to do functional simulation
        // We just do timing simulation of the network

        DPRINTF(BetaarSyntheticTraffic,
                "Generated packet with destination %d, embedded in "
                "address %x\n",
                dest, req->getPaddr());

        PacketPtr pkt = new Packet(req, requestType);
        pkt->dataDynamic(new uint8_t[req->getSize()]);
        pkt->senderState = NULL;

        sendPkt(pkt);
    }
}

void
BetaarSyntheticTraffic::initTrafficType()
{
    trafficStringToEnum["bit_complement"] = BIT_COMPLEMENT_;
    trafficStringToEnum["bit_reverse"] = BIT_REVERSE_;
    trafficStringToEnum["bit_rotation"] = BIT_ROTATION_;
    trafficStringToEnum["neighbor"] = NEIGHBOR_;
    trafficStringToEnum["shuffle"] = SHUFFLE_;
    trafficStringToEnum["tornado"] = TORNADO_;
    trafficStringToEnum["transpose"] = TRANSPOSE_;
    trafficStringToEnum["uniform_random"] = UNIFORM_RANDOM_;
}

void
BetaarSyntheticTraffic::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
BetaarSyntheticTraffic::printAddr(Addr a)
{
    cachePort.printAddr(a);
}

} // namespace gem5
