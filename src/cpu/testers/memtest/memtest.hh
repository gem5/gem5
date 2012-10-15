/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 *          Steve Reinhardt
 */

#ifndef __CPU_MEMTEST_MEMTEST_HH__
#define __CPU_MEMTEST_MEMTEST_HH__

#include <set>

#include "base/statistics.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"
#include "mem/port_proxy.hh"
#include "params/MemTest.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

class Packet;
class MemTest : public MemObject
{
  public:
    typedef MemTestParams Params;
    MemTest(const Params *p);

    virtual void init();

    // register statistics
    virtual void regStats();

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
        MemTest *cpu;

      public:
        TickEvent(MemTest *c) : Event(CPU_Tick_Pri), cpu(c) {}
        void process() { cpu->tick(); }
        virtual const char *description() const { return "MemTest tick"; }
    };

    TickEvent tickEvent;

    class CpuPort : public MasterPort
    {
        MemTest *memtest;

      public:

        CpuPort(const std::string &_name, MemTest *_memtest)
            : MasterPort(_name, _memtest), memtest(_memtest)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvTimingSnoopReq(PacketPtr pkt) { }

        virtual Tick recvAtomicSnoop(PacketPtr pkt) { return 0; }

        virtual void recvFunctionalSnoop(PacketPtr pkt) { }

        virtual void recvRetry();
    };

    CpuPort cachePort;
    CpuPort funcPort;
    PortProxy funcProxy;

    class MemTestSenderState : public Packet::SenderState
    {
      public:
        /** Constructor. */
        MemTestSenderState(uint8_t *_data)
            : data(_data)
        { }

        // Hold onto data pointer
        uint8_t *data;
    };

    PacketPtr retryPkt;

    bool accessRetry;
    
    //
    // The dmaOustanding flag enforces only one dma at a time
    //
    bool dmaOutstanding;

    unsigned size;              // size of testing memory region

    unsigned percentReads;      // target percentage of read accesses
    unsigned percentFunctional; // target percentage of functional accesses
    unsigned percentUncacheable;

    bool issueDmas;

    /** Request id for all generated traffic */
    MasterID masterId;

    int id;

    std::set<unsigned> outstandingAddrs;

    unsigned blockSize;

    Addr blockAddrMask;

    Addr blockAddr(Addr addr)
    {
        return (addr & ~blockAddrMask);
    }

    Addr traceBlockAddr;

    Addr baseAddr1;             // fix this to option
    Addr baseAddr2;             // fix this to option
    Addr uncacheAddr;

    unsigned progressInterval;  // frequency of progress reports
    Tick nextProgressMessage;   // access # for next progress report

    unsigned percentSourceUnaligned;
    unsigned percentDestUnaligned;

    Tick noResponseCycles;

    uint64_t numReads;
    uint64_t numWrites;
    uint64_t maxLoads;

    bool atomic;
    bool suppress_func_warnings;

    Stats::Scalar numReadsStat;
    Stats::Scalar numWritesStat;
    Stats::Scalar numCopiesStat;

    // called by MemCompleteEvent::process()
    void completeRequest(PacketPtr pkt);

    void sendPkt(PacketPtr pkt);

    void doRetry();

    friend class MemCompleteEvent;
};

#endif // __CPU_MEMTEST_MEMTEST_HH__
