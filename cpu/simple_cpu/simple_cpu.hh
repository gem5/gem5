/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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
#include "cpu/pc_event.hh"
#include "base/statistics.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"

// forward declarations
#ifdef FULL_SYSTEM
class Processor;
class AlphaITB;
class AlphaDTB;
class PhysicalMemory;

class RemoteGDB;
class GDBListener;

#else

class Process;

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
    struct TickEvent : public Event
    {
        SimpleCPU *cpu;
        int multiplier;

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

  public:
    void setTickMultiplier(int multiplier)
    {
        tickEvent.multiplier = multiplier;
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
              AlphaITB *itb, AlphaDTB *dtb, FunctionalMemory *mem,
              MemInterface *icache_interface, MemInterface *dcache_interface,
              bool _def_reg, Tick freq,
              bool _function_trace, Tick _function_trace_start);

#else

    SimpleCPU(const std::string &_name, Process *_process,
              Counter max_insts_any_thread,
              Counter max_insts_all_threads,
              Counter max_loads_any_thread,
              Counter max_loads_all_threads,
              MemInterface *icache_interface, MemInterface *dcache_interface,
              bool _def_reg,
              bool _function_trace, Tick _function_trace_start);

#endif

    virtual ~SimpleCPU();

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

    // current instruction
    MachInst inst;

    // Refcounted pointer to the one memory request.
    MemReqPtr memReq;

    StaticInstPtr<TheISA> curStaticInst;

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
    Stats::Scalar<> numInsts;

    virtual Counter totalInstructions() const
    {
        return numInst - startNumInst;
    }

    // number of simulated memory references
    Stats::Scalar<> numMemRefs;

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    // number of idle cycles
    Stats::Average<> notIdleFraction;
    Stats::Formula idleFraction;

    // number of cycles stalled for I-cache misses
    Stats::Scalar<> icacheStallCycles;
    Counter lastIcacheStall;

    // number of cycles stalled for D-cache misses
    Stats::Scalar<> dcacheStallCycles;
    Counter lastDcacheStall;

    void processCacheCompletion();

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags, uint64_t *res);

    void prefetch(Addr addr, unsigned flags)
    {
        // need to do this...
    }

    void writeHint(Addr addr, int size, unsigned flags)
    {
        // need to do this...
    }

    Fault copySrcTranslate(Addr src);

    Fault copy(Addr dest);

    // The register accessor methods provide the index of the
    // instruction's operand (e.g., 0 or 1), not the architectural
    // register index, to simplify the implementation of register
    // renaming.  We find the architectural register index by indexing
    // into the instruction's own operand index table.  Note that a
    // raw pointer to the StaticInst is provided instead of a
    // ref-counted StaticInstPtr to redice overhead.  This is fine as
    // long as these methods don't copy the pointer into any long-term
    // storage (which is pretty hard to imagine they would have reason
    // to do).

    uint64_t readIntReg(StaticInst<TheISA> *si, int idx)
    {
        return xc->readIntReg(si->srcRegIdx(idx));
    }

    float readFloatRegSingle(StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegSingle(reg_idx);
    }

    double readFloatRegDouble(StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegDouble(reg_idx);
    }

    uint64_t readFloatRegInt(StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegInt(reg_idx);
    }

    void setIntReg(StaticInst<TheISA> *si, int idx, uint64_t val)
    {
        xc->setIntReg(si->destRegIdx(idx), val);
    }

    void setFloatRegSingle(StaticInst<TheISA> *si, int idx, float val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        xc->setFloatRegSingle(reg_idx, val);
    }

    void setFloatRegDouble(StaticInst<TheISA> *si, int idx, double val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        xc->setFloatRegDouble(reg_idx, val);
    }

    void setFloatRegInt(StaticInst<TheISA> *si, int idx, uint64_t val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        xc->setFloatRegInt(reg_idx, val);
    }

    uint64_t readPC() { return xc->readPC(); }
    void setNextPC(uint64_t val) { xc->setNextPC(val); }

    uint64_t readUniq() { return xc->readUniq(); }
    void setUniq(uint64_t val) { xc->setUniq(val); }

    uint64_t readFpcr() { return xc->readFpcr(); }
    void setFpcr(uint64_t val) { xc->setFpcr(val); }

#ifdef FULL_SYSTEM
    uint64_t readIpr(int idx, Fault &fault) { return xc->readIpr(idx, fault); }
    Fault setIpr(int idx, uint64_t val) { return xc->setIpr(idx, val); }
    Fault hwrei() { return xc->hwrei(); }
    int readIntrFlag() { return xc->readIntrFlag(); }
    void setIntrFlag(int val) { xc->setIntrFlag(val); }
    bool inPalMode() { return xc->inPalMode(); }
    void ev5_trap(Fault fault) { xc->ev5_trap(fault); }
    bool simPalCheck(int palFunc) { return xc->simPalCheck(palFunc); }
#else
    void syscall() { xc->syscall(); }
#endif

    bool misspeculating() { return xc->misspeculating(); }
    ExecContext *xcBase() { return xc; }
};

#endif // __SIMPLE_CPU_HH__
