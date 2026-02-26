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

#include "cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTrafficV4.hh"

#include <cmath>
#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/statistics.hh"
#include "debug/MyGarnetSyntheticTraffic.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

// keshav: here we provide different seeds to all routers

namespace gem5
{

//int TESTER_NETWORK=0;		//kehav original
int TESTER_NETWORK_V4=0;

bool
GarnetSyntheticTrafficV4::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tester->completeRequest(pkt);
    return true;
}

void
GarnetSyntheticTrafficV4::CpuPort::recvReqRetry()
{
    tester->doRetry();
}

void
GarnetSyntheticTrafficV4::sendPkt(PacketPtr pkt)
{
    if (!cachePort.sendTimingReq(pkt)) {
        retryPkt = pkt; // RubyPort will retry sending
    }
    numPacketsSent++;
}

GarnetSyntheticTrafficV4::GarnetSyntheticTrafficV4(const Params &p)
    : ClockedObject(p),
      tickEvent([this]{ tick(); }, "GarnetSyntheticTrafficV2 tick",
                false, Event::CPU_Tick_Pri),
      cachePort("GarnetSyntheticTrafficV2", this),
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
      //tempTrafficType(p.temp_traffic_type),	//keshav newly added
      tempTrafficArgs(p.temp_traffic_args),	//keshav newly added
      injRate(p.inj_rate),
      injVnet(p.inj_vnet),
      precision(p.precision),
      responseLimit(p.response_limit),
      requestorId(p.system->getRequestorId(this)) {

    // set up counters
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    initTrafficType();
    if (trafficStringToEnum.count(trafficType) == 0) {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }
    traffic = trafficStringToEnum[trafficType];

    // keshav (newly added code) begins
    initTempTrafficType();
    initTempTrafficArgs();
    if (tempTrafficStringToEnum.count(tempTrafficType) == 0){
        fatal("Unknown Temporal traffic Type: %s!\n",tempTrafficType);
    }
    tempTraffic = tempTrafficStringToEnum[tempTrafficType];
    // keshav (newly code) ends


    id = TESTER_NETWORK_V4++;
    rng->init(static_cast<uint64_t>(TESTER_NETWORK_V4));
    //to give a diff seed to all routers
    //keshav
    // this line of code was not here originally
    DPRINTF(MyGarnetSyntheticTraffic,\
        "Config Created: Name = %s , and id = %d\n",name(), id);
}

Port &
GarnetSyntheticTrafficV4::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "test") {
        return cachePort; }
    else {
        return ClockedObject::getPort(if_name, idx); }
}

void
GarnetSyntheticTrafficV4::init()
{
    numPacketsSent = 0;
}


void
GarnetSyntheticTrafficV4::completeRequest(PacketPtr pkt)
{
    DPRINTF(MyGarnetSyntheticTraffic,
            "Completed injection of %s packet for address %x\n",
            pkt->isWrite() ? "write" : "read\n",
            pkt->req->getPaddr());

    assert(pkt->isResponse());
    noResponseCycles = 0;
    delete pkt;
}


void
GarnetSyntheticTrafficV4::tick()
{
    //if (++noResponseCycles >= responseLimit) {
    //keshav original
    // fatal("%s deadlocked at cycle %d with responseLimit %d and \
    // injection rate %f retryPkt %d\n",
    // name(), curTick(), responseLimit, injRate, retryPkt);
    //}
    if (retryPkt!=NULL){
        if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d with responseLimit %d and \
            injection rate %f retryPkt %d\n",
            name(), curTick(), responseLimit, injRate, retryPkt);
        }
    }

    bool sendAllowedThisCycle = false;
    if (tempTraffic == BY_DEFAULT_){
    //keshav newly added (initially only by default)
    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    double injRange = pow((double) 10, (double) precision);
    unsigned trySending = rng->random<unsigned>(0, (int) injRange);
    if (trySending < injRate*injRange)
        sendAllowedThisCycle = true;
    else
        sendAllowedThisCycle = false;
    } else if (tempTraffic == BERNOULLI_SHIFT_) {
        sendAllowedThisCycle = bernoulliShift();
        //sendAllowedThisCycle = true;
    } else if (tempTraffic == LIEBOVITCH_){
    sendAllowedThisCycle = liebovitch();
    } else if (tempTraffic == INTERMITTENCY_MAP_) {
    sendAllowedThisCycle = intermittencyMap();
    } else if (tempTraffic == ON_OFF_) {
    } else if (tempTraffic == ROUTER_STATE_BASED_) {
    }

    // always generatePkt unless fixedPkts or singleSender is enabled
    if (sendAllowedThisCycle) {
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
            senderEnable = false;

        if (singleSender >= 0 && id != singleSender)
            senderEnable = false;

        if (senderEnable)
            generatePkt();
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
GarnetSyntheticTrafficV4::generatePkt()
{
    int num_destinations = numDestinations;
    int radix = (int) sqrt(num_destinations);
    unsigned destination = id;
    int dest_x = -1;
    int dest_y = -1;
    int source = id;
    int src_x = id%radix;
    int src_y = id/radix;

    if (singleDest >= 0)
    {
        destination = singleDest;
    } else if (traffic == UNIFORM_RANDOM_) {
        destination = rng->random<unsigned>(0, num_destinations - 1);
    } else if (traffic == BIT_COMPLEMENT_) {
        dest_x = radix - src_x - 1;
        dest_y = radix - src_y - 1;
        destination = dest_y*radix + dest_x;
    } else if (traffic == BIT_REVERSE_) {
        unsigned int straight = source;
        unsigned int reverse = source & 1; // LSB

        int num_bits = (int) log2(num_destinations);

        for (int i = 1; i < num_bits; i++)
        {
            reverse <<= 1;
            straight >>= 1;
            reverse |= (straight & 1); // LSB
        }
        destination = reverse;
    } else if (traffic == BIT_ROTATION_) {
        if (source%2 == 0)
            destination = source/2;
        else // (source%2 == 1)
            destination = ((source/2) + (num_destinations/2));
    } else if (traffic == NEIGHBOR_) {
            dest_x = (src_x + 1) % radix;
            dest_y = src_y;
            destination = dest_y*radix + dest_x;
    } else if (traffic == SHUFFLE_) {
        if (source < num_destinations/2)
            destination = source*2;
        else
            destination = (source*2 - num_destinations + 1);
    } else if (traffic == TRANSPOSE_) {
            dest_x = src_y;
            dest_y = src_x;
            destination = dest_y*radix + dest_x;
    } else if (traffic == TORNADO_) {
        dest_x = (src_x + (int) ceil(radix/2) - 1) % radix;
        dest_y = src_y;
        destination = dest_y*radix + dest_x;
    }
    else {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }

    // The source of the packets is a cache.
    // The destination of the packets is a directory.
    // The destination bits are embedded in the address after byte-offset.
    Addr paddr =  destination;
    paddr <<= blockSizeBits;
    unsigned access_size = 1; // Does not affect Ruby simulation

    // Modeling different coherence msg types over different msg classes.
    //
    // GarnetSyntheticTraffic assumes the Garnet_standalone coherence protocol
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
    int injReqType = injVnet;

    if (injReqType < 0 || injReqType > 2)
    {
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
        req = std::make_shared<Request>(
            0x0, access_size, flags, requestorId, 0x0, 0);
        req->setPaddr(paddr);
    } else {  // if (injReqType == 2)
        // generate packet for virtual network 2
        requestType = MemCmd::WriteReq;
        req = std::make_shared<Request>(paddr, access_size, flags,
                                        requestorId);
    }

    req->setContext(id);

    //No need to do functional simulation
    //We just do timing simulation of the network

    DPRINTF(MyGarnetSyntheticTraffic,
            "Generated packet with destination %d, \
            embedded in address %x\n",
            destination, req->getPaddr());

    PacketPtr pkt = new Packet(req, requestType);
    pkt->dataDynamic(new uint8_t[req->getSize()]);
    pkt->senderState = NULL;

    sendPkt(pkt);
}

void
GarnetSyntheticTrafficV4::initTrafficType()
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
GarnetSyntheticTrafficV4::initTempTrafficType()
{
    tempTrafficStringToEnum["by_default"] = BY_DEFAULT_;
    tempTrafficStringToEnum["bernoulli_shift"] = BERNOULLI_SHIFT_;
    tempTrafficStringToEnum["liebovitch"] = LIEBOVITCH_;
    tempTrafficStringToEnum["intermittency_map"] = INTERMITTENCY_MAP_;
    tempTrafficStringToEnum["on_off"] = ON_OFF_;
    tempTrafficStringToEnum["router_state_based"] = ROUTER_STATE_BASED_;
}

void
GarnetSyntheticTrafficV4::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
GarnetSyntheticTrafficV4::printAddr(Addr a)
{
    cachePort.printAddr(a);
}

bool
GarnetSyntheticTrafficV4::bernoulliShift()
{
    if ((x_n > 0) && (x_n <= (1-lambda))) { x_n=x_n / (1-lambda);
    } else if ((x_n > (1-lambda)) && (x_n<1)) {
        x_n = (x_n - (1-lambda)) /lambda;
    } else {
    fatal("State variable in bernoulliShift went out of range x_n:%f!\n");
    }

    if (x_n <= (1-lambda)) { return false;
    } else { return true; }
}

bool
GarnetSyntheticTrafficV4::liebovitch()
{
   if (x_n<=d1) { x_n = alpha1*x_n;
   } else if ((x_n > d1) && (x_n <= d2)) {
    x_n = (d2-x_n) / (d2-d1);
   } else if ((x_n > d2) && (x_n < 1)) {
    x_n = 1 - (alpha2 * (1 - x_n));
   } else {
    fatal("State variable in liebovitch went out of range x_n:%f!\n",x_n);
}

   if (x_n <= d1) { return true;
   } else {return false; }
}

bool
GarnetSyntheticTrafficV4::intermittencyMap()
{
    if ((x_n > 0) && (x_n <= d)){
        x_n=epsilon+x_n+(((1-epsilon-d) / pow(d,m)) * pow(x_n,m));
    } else if ((x_n > d) && (x_n < 1)) { x_n = (x_n - d) / (1 - d);
    } else {
    fatal("State variable for intermittency_map is out of range");
    }

    if (x_n <= d) {return false;
    } else {return true; }
}

bool
GarnetSyntheticTrafficV4::onOff()
{
    return true;
}

bool
GarnetSyntheticTrafficV4::routerStateBased()
{
    return true;
}

void
GarnetSyntheticTrafficV4::initTempTrafficArgs()
{
    size_t pos;
    DPRINTF(MyGarnetSyntheticTraffic, "id:%d, tempTrafficArgs:%s\n",
            id,tempTrafficArgs);
    pos = tempTrafficArgs.find(',');
    tempTrafficType=tempTrafficArgs.substr(0,pos);
    tempTrafficArgs.erase(0,pos+1);
    float arr[10]; int i=0;
    while ((pos = tempTrafficArgs.find(',')) != std::string::npos) {
        arr[i] = std::stof(tempTrafficArgs.substr(0,pos));
        tempTrafficArgs.erase(0,pos+1);
        i=i+1;
    }
    if (tempTrafficType=="by_default") {
        //pass
    } else if (tempTrafficType == "bernoulli_shift") {
        // x_n, lambda
        x_n=arr[0]; lambda=arr[1];
            DPRINTF(MyGarnetSyntheticTraffic,
            "Temporal traffic %s, x_n:%f, lambda:%f\n",
            tempTrafficType, x_n, lambda);
    } else if (tempTrafficType == "liebovitch"){
        // x_n, alpha1, alpha2, d1, d2
        x_n=arr[0]; alpha1=arr[1]; alpha2=arr[2];
        d1=arr[3];  d2=arr[4];
        DPRINTF(MyGarnetSyntheticTraffic,
            "id:%d, Temporal traffic %s, x_n:%f,\
            alpha1:%f,alpha2:%f,d1:%f,d2:%f\n",
            id, tempTrafficType, x_n, alpha1, alpha2, d1, d2);
    } else if (tempTrafficType == "intermittency_map"){
        // x_n, epsilon, d, m
        x_n=arr[0]; epsilon=arr[1];
        d=arr[2];   m=arr[3];
            DPRINTF(MyGarnetSyntheticTraffic,
                "id:%d, Temporal traffic %s, x_n:%f,\
                epsilon:%f,d:%f,m:%d\n",
                id, tempTrafficType, x_n, epsilon, m, d);
    } else if (tempTrafficType == "router_state_based"){
    } else if (tempTrafficType == "on_off"){
    }
}

} // namespace gem5
