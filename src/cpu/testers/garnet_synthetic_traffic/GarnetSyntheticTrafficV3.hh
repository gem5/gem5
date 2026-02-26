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

#ifndef __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__
#define __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__

#include <set>

#include "base/random.hh"
#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/GarnetSyntheticTrafficV3.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

namespace gem5
{

//enum TrafficType {BIT_COMPLEMENT_ = 0,	//keshav origianl
enum TrafficTypeV3
{
                  BIT_COMPLEMENT_ = 0,
                  BIT_REVERSE_ = 1,
                  BIT_ROTATION_ = 2,
                  NEIGHBOR_ = 3,
                  SHUFFLE_ = 4,
                  TORNADO_ = 5,
                  TRANSPOSE_ = 6,
                  UNIFORM_RANDOM_ = 7,
                  NUM_TRAFFIC_PATTERNS_
  };

class Packet;
class GarnetSyntheticTrafficV3 : public ClockedObject
{
  public:
    // keshav code begins
    // keshav:added to model the waiting rather than packet loss
    PacketPtr packet_waiting_queue[50];	int rear=0;
    int push_in_pkt_waiting_queue(PacketPtr in_pkt) {
      if (rear < 50){packet_waiting_queue[rear]=in_pkt;rear=rear+1;
        return 0;}
      fatal("waiting queue of 50 full on %s at %d",name(),curTick());
      return 1;}
    PacketPtr pop_from_pkt_waiting_queue(void) {
      PacketPtr out_pkt;
      if (rear > 0){out_pkt=packet_waiting_queue[0];
        for (int i = 0; i < rear - 1;i++){
          packet_waiting_queue[i]= packet_waiting_queue[i + 1];}
          rear--; return out_pkt;} else {return nullptr;}}
    // keshav code neds
    typedef GarnetSyntheticTrafficV3Params Params;
    GarnetSyntheticTrafficV3(const Params &p);

    void init() override;

    // main simulation loop (one cycle)
    void tick();

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);

  protected:
    EventFunctionWrapper tickEvent;

    class CpuPort : public RequestPort
    {
        GarnetSyntheticTrafficV3 *tester;

      public:

        CpuPort(const std::string &_name, GarnetSyntheticTrafficV3 *_tester)
            : RequestPort(_name), tester(_tester)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvReqRetry();
    };

    CpuPort cachePort;

    class GarnetSyntheticTrafficSenderState : public Packet::SenderState
    {
      public:
        /** Constructor. */
        GarnetSyntheticTrafficSenderState(uint8_t *_data)
            : data(_data)
        { }

        // Hold onto data pointer
        uint8_t *data;
    };

    PacketPtr retryPkt;
    unsigned size;
    int id;

    std::map<std::string, TrafficType> trafficStringToEnum;

    unsigned blockSizeBits;

    Tick noResponseCycles;

    int numDestinations;
    Tick simCycles;
    int numPacketsMax;
    int numPacketsSent;
    int singleSender;
    int singleDest;

    std::string trafficType; // string
    TrafficType traffic; // enum from string
    double injRate;
    int injVnet;
    int precision;

    const Cycles responseLimit;

    RequestorID requestorId;

    Random::RandomPtr rng = Random::genRandom();

    void completeRequest(PacketPtr pkt);

    void generatePkt();
    void sendPkt(PacketPtr pkt);
    void initTrafficType();

    void doRetry();

    friend class MemCompleteEvent;
};

} // namespace gem5

#endif // __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__
