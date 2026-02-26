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
#include "params/GarnetSyntheticTrafficV4.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

namespace gem5 {
enum TempTrafficTypeV4
{
      BY_DEFAULT_ = 0,
      BERNOULLI_SHIFT_ = 1,
      LIEBOVITCH_ = 2,
      INTERMITTENCY_MAP_ = 3,
      ON_OFF_ = 4,
      ROUTER_STATE_BASED_ = 5,
};

//enum TrafficType {BIT_COMPLEMENT_ = 0,	//keshav
enum TrafficTypeV4
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
//class GarnetSyntheticTraffic : public ClockedObject	//keshav original
class GarnetSyntheticTrafficV4 : public ClockedObject
{
  public:
    //typedef GarnetSyntheticTrafficParams Params;	//keshav original
    typedef GarnetSyntheticTrafficV4Params Params;
    //GarnetSyntheticTraffic(const Params &p);		//kehsav original
    GarnetSyntheticTrafficV4(const Params &p);

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
        //GarnetSyntheticTraffic *tester;	//keshav original
        GarnetSyntheticTrafficV4 *tester;

      public:

  //CpuPort(const std::string &_name, GarnetSyntheticTraffic *_tester)
  //keshav original
        CpuPort(const std::string &_name, GarnetSyntheticTrafficV4 *_tester)
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

    //std::map<std::string, TrafficType> trafficStringToEnum;
    //keshav original
    std::map<std::string, TrafficTypeV4> trafficStringToEnum;
    std::map<std::string, TempTrafficTypeV4> tempTrafficStringToEnum;
    //keshav newly added

    unsigned blockSizeBits;

    Tick noResponseCycles;

    int numDestinations;
    Tick simCycles;
    int numPacketsMax;
    int numPacketsSent;
    int singleSender;
    int singleDest;

    std::string trafficType; // string
    std::string tempTrafficType; // string	//keshav newly added
    //TrafficType traffic; // enum from string	//keshav original
    TrafficTypeV4 traffic; // enum from string
    TempTrafficTypeV4 tempTraffic; // enum from string	// keshav newly added
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
    void initTempTrafficType();	//keshav it was not here

    void doRetry();

    friend class MemCompleteEvent;

    // keshav here upon temporal traffic types are added
    void initTempTrafficArgs();
    std::string tempTrafficArgs;
    float x_n;

    bool bernoulliShift();
    float lambda;

    bool liebovitch();
    float alpha1,alpha2,d1,d2;

    bool intermittencyMap();
    float epsilon,d;
    int m;

    bool onOff();
    bool routerStateBased();
};

} // namespace gem5

#endif // __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__
