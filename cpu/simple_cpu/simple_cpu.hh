/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __SIMPLE_CPU_HH__
#define __SIMPLE_CPU_HH__

#include "cpu/base_cpu.hh"
#include "sim/eventq.hh"
#include "base/loader/symtab.hh"
#include "cpu/pc_event.hh"
#include "base/statistics.hh"


// forward declarations
#ifdef FULL_SYSTEM
class Processor;
class Kernel;
class AlphaItb;
class AlphaDtb;
class PhysicalMemory;

class RemoteGDB;
class GDBListener;
#endif // FULL_SYSTEM

class MemInterface;
class Checkpoint;

namespace Trace {
    class InstRecord;
}

class SimpleCPU : public BaseCPU
{
  public:
    // main simulation loop (one cycle)
    void tick();

  private:
    class TickEvent : public Event
    {
      private:
        SimpleCPU *cpu;

      public:
        TickEvent(SimpleCPU *c);
        void process();
        const char *description();
    };

    TickEvent tickEvent;

    /// Schedule tick event, regardless of its current state.
    void scheduleTickEvent(int delay)
    {
        if (tickEvent.squashed())
            tickEvent.reschedule(curTick + delay);
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick + delay);
    }

    /// Unschedule tick event, regardless of its current state.
    void unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

  private:
    Trace::InstRecord *traceData;
    template<typename T>
    void trace_data(T data) {
      if (traceData) {
        traceData->setData(data);
      }
    };

  public:
    //
    enum Status {
        Running,
        Idle,
        IcacheMissStall,
        IcacheMissComplete,
        DcacheMissStall,
        SwitchedOut
    };

  private:
    Status _status;

  public:
    void post_interrupt(int int_num, int index);

    void zero_fill_64(Addr addr) {
      static int warned = 0;
      if (!warned) {
        warn ("WH64 is not implemented");
        warned = 1;
      }
    };

#ifdef FULL_SYSTEM

    SimpleCPU(const std::string &_name,
              System *_system,
              Counter max_insts_any_thread, Counter max_insts_all_threads,
              Counter max_loads_any_thread, Counter max_loads_all_threads,
              AlphaItb *itb, AlphaDtb *dtb, FunctionalMemory *mem,
              MemInterface *icache_interface, MemInterface *dcache_interface,
              bool _def_reg, Tick freq);

#else

    SimpleCPU(const std::string &_name, Process *_process,
              Counter max_insts_any_thread,
              Counter max_insts_all_threads,
              Counter max_loads_any_thread,
              Counter max_loads_all_threads,
              MemInterface *icache_interface, MemInterface *dcache_interface,
              bool _def_reg);

#endif

    virtual ~SimpleCPU();
    virtual void init();

    // execution context
    ExecContext *xc;

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

#ifdef FULL_SYSTEM
    Addr dbg_vtophys(Addr addr);

    bool interval_stats;
#endif

    // L1 instruction cache
    MemInterface *icacheInterface;

    // L1 data cache
    MemInterface *dcacheInterface;

    bool defer_registration;

    // current instruction
    MachInst inst;

    // Refcounted pointer to the one memory request.
    MemReqPtr memReq;

    class CacheCompletionEvent : public Event
    {
      private:
        SimpleCPU *cpu;

      public:
        CacheCompletionEvent(SimpleCPU *_cpu);

        virtual void process();
        virtual const char *description();
    };

    CacheCompletionEvent cacheCompletionEvent;

    Status status() const { return _status; }

    virtual void activateContext(int thread_num, int delay);
    virtual void suspendContext(int thread_num);
    virtual void deallocateContext(int thread_num);
    virtual void haltContext(int thread_num);

    // statistics
    virtual void regStats();
    virtual void resetStats();

    // number of simulated instructions
    Counter numInst;
    Counter startNumInst;
    Statistics::Formula numInsts;

    // number of simulated memory references
    Statistics::Scalar<> numMemRefs;

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    // number of idle cycles
    Statistics::Average<> notIdleFraction;
    Statistics::Formula idleFraction;

    // number of cycles stalled for I-cache misses
    Statistics::Scalar<> icacheStallCycles;
    Counter lastIcacheStall;

    // number of cycles stalled for D-cache misses
    Statistics::Scalar<> dcacheStallCycles;
    Counter lastDcacheStall;

    void processCacheCompletion();

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags,
                        uint64_t *res);

    Fault prefetch(Addr addr, unsigned flags)
    {
        // need to do this...
        return No_Fault;
    }

    void writeHint(Addr addr, int size)
    {
        // need to do this...
    }
};

#endif // __SIMPLE_CPU_HH__
