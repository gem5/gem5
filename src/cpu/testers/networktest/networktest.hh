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

#ifndef __CPU_NETWORKTEST_NETWORKTEST_HH__
#define __CPU_NETWORKTEST_NETWORKTEST_HH__

#include <set>

#include "base/statistics.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"
#include "params/NetworkTest.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

class Packet;
class NetworkTest : public MemObject
{
  public:
    typedef NetworkTestParams Params;
    NetworkTest(const Params *p);

    virtual void init();

    // main simulation loop (one cycle)
    void tick();

    virtual BaseMasterPort &getMasterPort(const std::string &if_name,
                                          PortID idx = InvalidPortID);

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);

  protected:
    class TickEvent : public Event
    {
      private:
        NetworkTest *cpu;

      public:
        TickEvent(NetworkTest *c) : Event(CPU_Tick_Pri), cpu(c) {}
        void process() { cpu->tick(); }
        virtual const char *description() const { return "NetworkTest tick"; }
    };

    TickEvent tickEvent;

    class CpuPort : public MasterPort
    {
        NetworkTest *networktest;

      public:

        CpuPort(const std::string &_name, NetworkTest *_networktest)
            : MasterPort(_name, _networktest), networktest(_networktest)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvReqRetry();
    };

    CpuPort cachePort;

    class NetworkTestSenderState : public Packet::SenderState
    {
      public:
        /** Constructor. */
        NetworkTestSenderState(uint8_t *_data)
            : data(_data)
        { }

        // Hold onto data pointer
        uint8_t *data;
    };

    PacketPtr retryPkt;
    unsigned size;
    int id;

    unsigned blockSizeBits;

    Tick noResponseCycles;

    int numMemories;
    Tick simCycles;
    bool fixedPkts;
    int maxPackets;
    int numPacketsSent;

    int trafficType;
    double injRate;
    int precision;

    MasterID masterId;

    void completeRequest(PacketPtr pkt);

    void generatePkt();
    void sendPkt(PacketPtr pkt);

    void doRetry();

    friend class MemCompleteEvent;
};

#endif // __CPU_NETWORKTEST_NETWORKTEST_HH__



