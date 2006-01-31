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

#ifndef __CPU_SIMPLE_CPU_SIMPLE_CPU_HH__
#define __CPU_SIMPLE_CPU_SIMPLE_CPU_HH__

#include "base/statistics.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/pc_event.hh"
#include "cpu/sampler/sampler.hh"
#include "cpu/static_inst.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"

// forward declarations
#if FULL_SYSTEM
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
    class CpuPort : public Port
    {

        SimpleCPU *cpu;

      public:

        CpuPort(SimpleCPU *_cpu)
            : cpu(_cpu)
        { }

      protected:

        virtual bool recvTiming(Packet &pkt)
        { return cpu->processCacheCompletion(pkt); }

        virtual Tick recvAtomic(Packet &pkt)
        { return cpu->processCacheCompletion(pkt); }

        virtual void recvFunctional(Packet &pkt)
        { cpu->processCacheCompletion(pkt); }

        virtual void recvStatusChange(Status status)
        { cpu->recvStatusChange(status); }

    };

    CpuPort icache_port;
    CpuPort dcache_port;

    bool recvTiming(Packet &pkt);
    Tick recvAtomic(Packet &pkt);
    void recvFunctional(Packet &pkt);

  public:
    // main simulation loop (one cycle)
    void tick();

  private:
    struct TickEvent : public Event
    {
        SimpleCPU *cpu;
        int width;

        TickEvent(SimpleCPU *c, int w);
        void process();
        const char *description();
    };

    TickEvent tickEvent;

    /// Schedule tick event, regardless of its current state.
    void scheduleTickEvent(int numCycles)
    {
        if (tickEvent.squashed())
            tickEvent.reschedule(curTick + cycles(numCycles));
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick + cycles(numCycles));
    }

    /// Unschedule tick event, regardless of its current state.
    void unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

  private:
    Trace::InstRecord *traceData;

  public:
    //
    enum Status {
        Running,
        Idle,
        IcacheMissStall,
        IcacheMissComplete,
        DcacheMissStall,
        DcacheMissSwitch,
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

  public:
    struct Params : public BaseCPU::Params
    {
        MemInterface *icache_interface;
        MemInterface *dcache_interface;
        int width;
#if FULL_SYSTEM
        AlphaITB *itb;
        AlphaDTB *dtb;
        FunctionalMemory *mem;
#else
        Process *process;
#endif
    };
    SimpleCPU(Params *params);
    virtual ~SimpleCPU();

  public:
    // execution context
    ExecContext *xc;

    void switchOut(Sampler *s);
    void takeOverFrom(BaseCPU *oldCPU);

#if FULL_SYSTEM
    Addr dbg_vtophys(Addr addr);

    bool interval_stats;
#endif

    // current instruction
    MachInst inst;

    CpuRequest *req;
    Packet *pkt;

    // Pointer to the sampler that is telling us to switchover.
    // Used to signal the completion of the pipe drain and schedule
    // the next switchover
    Sampler *sampler;

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

    // These functions are only used in CPU models that split
    // effective address computation from the actual memory access.
    void setEA(Addr EA) { panic("SimpleCPU::setEA() not implemented\n"); }
    Addr getEA() 	{ panic("SimpleCPU::getEA() not implemented\n"); }

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

    uint64_t readIntReg(const StaticInst<TheISA> *si, int idx)
    {
        return xc->readIntReg(si->srcRegIdx(idx));
    }

    float readFloatRegSingle(const StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegSingle(reg_idx);
    }

    double readFloatRegDouble(const StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegDouble(reg_idx);
    }

    uint64_t readFloatRegInt(const StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegInt(reg_idx);
    }

    void setIntReg(const StaticInst<TheISA> *si, int idx, uint64_t val)
    {
        xc->setIntReg(si->destRegIdx(idx), val);
    }

    void setFloatRegSingle(const StaticInst<TheISA> *si, int idx, float val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        xc->setFloatRegSingle(reg_idx, val);
    }

    void setFloatRegDouble(const StaticInst<TheISA> *si, int idx, double val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        xc->setFloatRegDouble(reg_idx, val);
    }

    void setFloatRegInt(const StaticInst<TheISA> *si, int idx, uint64_t val)
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

#if FULL_SYSTEM
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

#endif // __CPU_SIMPLE_CPU_SIMPLE_CPU_HH__
