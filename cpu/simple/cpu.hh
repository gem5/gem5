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
#include "cpu/cpu_exec_context.hh"
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
class MemObject;

class RemoteGDB;
class GDBListener;

#else

class Process;

#endif // FULL_SYSTEM

class ExecContext;
class Checkpoint;

namespace Trace {
    class InstRecord;
}


// Set exactly one of these symbols to 1 to set the memory access
// model.  Probably should make these template parameters, or even
// just fork the CPU models.
//
#define SIMPLE_CPU_MEM_TIMING    0
#define SIMPLE_CPU_MEM_ATOMIC    0
#define SIMPLE_CPU_MEM_IMMEDIATE 1


class SimpleCPU : public BaseCPU
{
  protected:
    typedef TheISA::MachInst MachInst;
    typedef TheISA::MiscReg MiscReg;
    class CpuPort : public Port
    {

        SimpleCPU *cpu;

      public:

        CpuPort(SimpleCPU *_cpu)
            : cpu(_cpu)
        { }

      protected:

        virtual bool recvTiming(Packet &pkt);

        virtual Tick recvAtomic(Packet &pkt);

        virtual void recvFunctional(Packet &pkt);

        virtual void recvStatusChange(Status status);

        virtual Packet *recvRetry();
    };

    CpuPort icachePort;
    CpuPort dcachePort;

  public:
    // main simulation loop (one cycle)
    void tick();
    virtual void init();

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
        IcacheRetry,
        IcacheWaitResponse,
        IcacheAccessComplete,
        DcacheRetry,
        DcacheWaitResponse,
        DcacheWaitSwitch,
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
        int width;
#if FULL_SYSTEM
        AlphaITB *itb;
        AlphaDTB *dtb;
#else
        MemObject *mem;
        Process *process;
#endif
    };
    SimpleCPU(Params *params);
    virtual ~SimpleCPU();

  public:
    // execution context
    CPUExecContext *cpuXC;

    ExecContext *xcProxy;

    void switchOut(Sampler *s);
    void takeOverFrom(BaseCPU *oldCPU);

#if FULL_SYSTEM
    Addr dbg_vtophys(Addr addr);

    bool interval_stats;
#endif

    // current instruction
    MachInst inst;

#if SIMPLE_CPU_MEM_TIMING
    Packet *retry_pkt;
#elif SIMPLE_CPU_MEM_ATOMIC || SIMPLE_CPU_MEM_IMMEDIATE
    CpuRequest *ifetch_req;
    Packet     *ifetch_pkt;
    CpuRequest *data_read_req;
    Packet     *data_read_pkt;
    CpuRequest *data_write_req;
    Packet     *data_write_pkt;
#endif

    // Pointer to the sampler that is telling us to switchover.
    // Used to signal the completion of the pipe drain and schedule
    // the next switchover
    Sampler *sampler;

    StaticInstPtr curStaticInst;

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

    // number of cycles stalled for I-cache responses
    Stats::Scalar<> icacheStallCycles;
    Counter lastIcacheStall;

    // number of cycles stalled for I-cache retries
    Stats::Scalar<> icacheRetryCycles;
    Counter lastIcacheRetry;

    // number of cycles stalled for D-cache responses
    Stats::Scalar<> dcacheStallCycles;
    Counter lastDcacheStall;

    // number of cycles stalled for D-cache retries
    Stats::Scalar<> dcacheRetryCycles;
    Counter lastDcacheRetry;

    void sendIcacheRequest(Packet *pkt);
    void sendDcacheRequest(Packet *pkt);
    void processResponse(Packet &response);

    Packet * processRetry();
    void recvStatusChange(Port::Status status) {}

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

    uint64_t readIntReg(const StaticInst *si, int idx)
    {
        return cpuXC->readIntReg(si->srcRegIdx(idx));
    }

    float readFloatRegSingle(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return cpuXC->readFloatRegSingle(reg_idx);
    }

    double readFloatRegDouble(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return cpuXC->readFloatRegDouble(reg_idx);
    }

    uint64_t readFloatRegInt(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return cpuXC->readFloatRegInt(reg_idx);
    }

    void setIntReg(const StaticInst *si, int idx, uint64_t val)
    {
        cpuXC->setIntReg(si->destRegIdx(idx), val);
    }

    void setFloatRegSingle(const StaticInst *si, int idx, float val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        cpuXC->setFloatRegSingle(reg_idx, val);
    }

    void setFloatRegDouble(const StaticInst *si, int idx, double val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        cpuXC->setFloatRegDouble(reg_idx, val);
    }

    void setFloatRegInt(const StaticInst *si, int idx, uint64_t val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        cpuXC->setFloatRegInt(reg_idx, val);
    }

    uint64_t readPC() { return cpuXC->readPC(); }
    uint64_t readNextPC() { return cpuXC->readNextPC(); }
    uint64_t readNextNPC() { return cpuXC->readNextNPC(); }

    void setPC(uint64_t val) { cpuXC->setPC(val); }
    void setNextPC(uint64_t val) { cpuXC->setNextPC(val); }
    void setNextNPC(uint64_t val) { cpuXC->setNextNPC(val); }

    MiscReg readMiscReg(int misc_reg)
    {
        return cpuXC->readMiscReg(misc_reg);
    }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    {
        return cpuXC->readMiscRegWithEffect(misc_reg, fault);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        return cpuXC->setMiscReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return cpuXC->setMiscRegWithEffect(misc_reg, val);
    }

#if FULL_SYSTEM
    Fault hwrei() { return cpuXC->hwrei(); }
    int readIntrFlag() { return cpuXC->readIntrFlag(); }
    void setIntrFlag(int val) { cpuXC->setIntrFlag(val); }
    bool inPalMode() { return cpuXC->inPalMode(); }
    void ev5_trap(Fault fault) { fault->invoke(xcProxy); }
    bool simPalCheck(int palFunc) { return cpuXC->simPalCheck(palFunc); }
#else
    void syscall() { cpuXC->syscall(); }
#endif

    bool misspeculating() { return cpuXC->misspeculating(); }
    ExecContext *xcBase() { return xcProxy; }
};

#endif // __CPU_SIMPLE_CPU_SIMPLE_CPU_HH__
