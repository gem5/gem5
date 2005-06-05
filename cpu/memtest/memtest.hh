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
 */

#ifndef __CPU_MEMTEST_MEMTEST_HH__
#define __CPU_MEMTEST_MEMTEST_HH__

#include <set>

#include "base/statistics.hh"
#include "mem/functional/functional.hh"
#include "mem/mem_interface.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

class ExecContext;
class MemTest : public SimObject
{
  public:

    MemTest(const std::string &name,
            MemInterface *_cache_interface,
            FunctionalMemory *main_mem,
            FunctionalMemory *check_mem,
            unsigned _memorySize,
            unsigned _percentReads,
            unsigned _percentCopies,
            unsigned _percentUncacheable,
            unsigned _progressInterval,
            unsigned _percentSourceUnaligned,
            unsigned _percentDestUnaligned,
            Addr _traceAddr,
            Counter _max_loads);

    // register statistics
    virtual void regStats();

    inline Tick cycles(int numCycles) const { return numCycles; }

    // main simulation loop (one cycle)
    void tick();

  protected:
    class TickEvent : public Event
    {
      private:
        MemTest *cpu;
      public:
        TickEvent(MemTest *c)
            : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c) {}
        void process() {cpu->tick();}
        virtual const char *description() { return "tick event"; }
    };

    TickEvent tickEvent;

    MemInterface *cacheInterface;
    FunctionalMemory *mainMem;
    FunctionalMemory *checkMem;
    ExecContext *xc;

    unsigned size;		// size of testing memory region

    unsigned percentReads;	// target percentage of read accesses
    unsigned percentCopies;	// target percentage of copy accesses
    unsigned percentUncacheable;

    int id;

    std::set<unsigned> outstandingAddrs;

    unsigned blockSize;

    Addr blockAddrMask;

    Addr blockAddr(Addr addr)
    {
        return (addr & ~blockAddrMask);
    }

    Addr traceBlockAddr;

    Addr baseAddr1;		// fix this to option
    Addr baseAddr2;		// fix this to option
    Addr uncacheAddr;

    unsigned progressInterval;	// frequency of progress reports
    Tick nextProgressMessage;	// access # for next progress report

    unsigned percentSourceUnaligned;
    unsigned percentDestUnaligned;

    Tick noResponseCycles;

    uint64_t numReads;
    uint64_t maxLoads;
    Stats::Scalar<> numReadsStat;
    Stats::Scalar<> numWritesStat;
    Stats::Scalar<> numCopiesStat;

    // called by MemCompleteEvent::process()
    void completeRequest(MemReqPtr &req, uint8_t *data);

    friend class MemCompleteEvent;
};


class MemCompleteEvent : public Event
{
    MemReqPtr req;
    uint8_t *data;
    MemTest *tester;

  public:

    MemCompleteEvent(MemReqPtr &_req, uint8_t *_data, MemTest *_tester)
        : Event(&mainEventQueue),
          req(_req), data(_data), tester(_tester)
    {
    }

    void process();

    virtual const char *description();
};

#endif // __CPU_MEMTEST_MEMTEST_HH__



