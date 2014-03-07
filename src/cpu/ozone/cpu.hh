/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#ifndef __CPU_OZONE_CPU_HH__
#define __CPU_OZONE_CPU_HH__

#include <set>

#include "arch/alpha/tlb.hh"
#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/ozone/rename_table.hh"
#include "cpu/ozone/thread_state.hh"
#include "cpu/base.hh"
#include "cpu/inst_seq.hh"
#include "cpu/pc_event.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "cpu/timebuf.hh"
#include "mem/page_table.hh"
#include "sim/eventq.hh"

// forward declarations

namespace TheISA {
    namespace Kernel {
        class Statistics;
    };
    class TLB;
};

class Checkpoint;
class EndQuiesceEvent;
class MemoryController;
class MemObject;
class Process;
class Request;

namespace Trace {
    class InstRecord;
}

template <class>
class Checker;

/**
 * Light weight out of order CPU model that approximates an out of
 * order CPU.  It is separated into a front end and a back end, with
 * the template parameter Impl describing the classes used for each.
 * The goal is to be able to specify through the Impl the class to use
 * for the front end and back end, with different classes used to
 * model different levels of detail.
 */
template <class Impl>
class OzoneCPU : public BaseCPU
{
  private:
    typedef typename Impl::FrontEnd FrontEnd;
    typedef typename Impl::BackEnd BackEnd;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::DynInstPtr DynInstPtr;

    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::MiscReg MiscReg;

  public:
    class OzoneTC : public ThreadContext {
      public:
        OzoneCPU<Impl> *cpu;

        OzoneThreadState<Impl> *thread;

        BaseCPU *getCpuPtr();

        TheISA::TLB *getITBPtr() { return cpu->itb; }

        TheISA::TLB * getDTBPtr() { return cpu->dtb; }

        System *getSystemPtr() { return cpu->system; }

        TheISA::Kernel::Statistics *getKernelStats()
        { return thread->getKernelStats(); }

        Process *getProcessPtr() { return thread->getProcessPtr(); }

        PortProxy &getPhysProxy() { return thread->getPhysProxy(); }

        FSTranslatingPortProxy &getVirtProxy()
        { return thread->getVirtProxy(); }

        SETranslatingPortProxy &getMemProxy() { return thread->getMemProxy(); }

        Status status() const { return thread->status(); }

        void setStatus(Status new_status);

        /// Set the status to Active.  Optional delay indicates number of
        /// cycles to wait before beginning execution.
        void activate(int delay = 1);

        /// Set the status to Suspended.
        void suspend();

        /// Set the status to Halted.
        void halt();

        void dumpFuncProfile();

        void takeOverFrom(ThreadContext *old_context);

        void regStats(const std::string &name);

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);

        EndQuiesceEvent *getQuiesceEvent();

        Tick readLastActivate();
        Tick readLastSuspend();

        void profileClear();
        void profileSample();

        int threadId() const;

        void copyArchRegs(ThreadContext *tc);

        void clearArchRegs();

        uint64_t readIntReg(int reg_idx);

        FloatReg readFloatReg(int reg_idx);

        FloatRegBits readFloatRegBits(int reg_idx);

        void setIntReg(int reg_idx, uint64_t val);

        void setFloatReg(int reg_idx, FloatReg val);

        void setFloatRegBits(int reg_idx, FloatRegBits val);

        uint64_t readPC() { return thread->PC; }
        void setPC(Addr val);

        uint64_t readNextPC() { return thread->nextPC; }
        void setNextPC(Addr val);

        uint64_t readNextNPC()
        {
#if ISA_HAS_DELAY_SLOT
            panic("Ozone needs to support nextNPC");
#else
            return thread->nextPC + sizeof(TheISA::MachInst);
#endif
        }

        void setNextNPC(uint64_t val)
        {
#if ISA_HAS_DELAY_SLOT
            panic("Ozone needs to support nextNPC");
#endif
        }

      public:
        // ISA stuff:
        MiscReg readMiscRegNoEffect(int misc_reg);

        MiscReg readMiscReg(int misc_reg);

        void setMiscRegNoEffect(int misc_reg, const MiscReg &val);

        void setMiscReg(int misc_reg, const MiscReg &val);

        unsigned readStCondFailures()
        { return thread->storeCondFailures; }

        void setStCondFailures(unsigned sc_failures)
        { thread->storeCondFailures = sc_failures; }

        bool misspeculating() { return false; }

        Counter readFuncExeInst() { return thread->funcExeInst; }

        void setFuncExeInst(Counter new_val)
        { thread->funcExeInst = new_val; }
    };

    // Ozone specific thread context
    OzoneTC ozoneTC;
    // Thread context to be used
    ThreadContext *tc;
    // Checker thread context; will wrap the OzoneTC if a checker is
    // being used.
    ThreadContext *checkerTC;

    typedef OzoneThreadState<Impl> ImplState;

  private:
    // Committed thread state for the OzoneCPU.
    OzoneThreadState<Impl> thread;

  public:
    // main simulation loop (one cycle)
    void tick();

#ifndef NDEBUG
    /** Count of total number of dynamic instructions in flight. */
    int instcount;
#endif

    std::set<InstSeqNum> snList;
    std::set<Addr> lockAddrList;
  private:
    struct TickEvent : public Event
    {
        OzoneCPU *cpu;
        int width;

        TickEvent(OzoneCPU *c, int w);
        void process();
        const char *description() const;
    };

    TickEvent tickEvent;

    /// Schedule tick event, regardless of its current state.
    void scheduleTickEvent(int delay)
    {
        if (tickEvent.squashed())
            tickEvent.reschedule(curTick() + ticks(delay));
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick() + ticks(delay));
    }

    /// Unschedule tick event, regardless of its current state.
    void unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

  public:
    enum Status {
        Running,
        Idle,
        SwitchedOut
    };

    Status _status;

  public:
    void wakeup();

    void zero_fill_64(Addr addr) {
        static int warned = 0;
        if (!warned) {
            warn ("WH64 is not implemented");
            warned = 1;
        }
    };

    typedef typename Impl::Params Params;

    OzoneCPU(Params *params);

    virtual ~OzoneCPU();

    void init();

  public:
    BaseCPU *getCpuPtr() { return this; }

    void switchOut();
    void signalSwitched();
    void takeOverFrom(BaseCPU *oldCPU);

    int switchCount;

    Addr dbg_vtophys(Addr addr);

    bool interval_stats;

    TheISA::TLB *itb;
    TheISA::TLB *dtb;
    System *system;

    FrontEnd *frontEnd;

    BackEnd *backEnd;

  private:
    Status status() const { return _status; }
    void setStatus(Status new_status) { _status = new_status; }

    virtual void activateContext(int thread_num, int delay);
    virtual void suspendContext(int thread_num);
    virtual void deallocateContext(int thread_num, int delay);
    virtual void haltContext(int thread_num);

    // statistics
    virtual void regStats();
    virtual void resetStats();

    // number of simulated instructions
  public:
    Counter numInst;
    Counter startNumInst;

    virtual Counter totalInstructions() const
    {
        return numInst - startNumInst;
    }

  private:
    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    // number of idle cycles
    Stats::Average notIdleFraction;
    Stats::Formula idleFraction;

  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    void demapPage(Addr vaddr, uint64_t asn)
    {
        cpu->itb->demap(vaddr, asn);
        cpu->dtb->demap(vaddr, asn);
    }

    void demapInstPage(Addr vaddr, uint64_t asn)
    {
        cpu->itb->demap(vaddr, asn);
    }

    void demapDataPage(Addr vaddr, uint64_t asn)
    {
        cpu->dtb->demap(vaddr, asn);
    }

    /** CPU read function, forwards read to LSQ. */
    template <class T>
    Fault read(Request *req, T &data, int load_idx)
    {
        return backEnd->read(req, data, load_idx);
    }

    /** CPU write function, forwards write to LSQ. */
    template <class T>
    Fault write(Request *req, T &data, int store_idx)
    {
        return backEnd->write(req, data, store_idx);
    }

  public:
    void squashFromTC();

    void dumpInsts() { frontEnd->dumpInsts(); }

    Fault hwrei();
    bool simPalCheck(int palFunc);
    void processInterrupts();
    void syscall(uint64_t &callnum);

    ThreadContext *tcBase() { return tc; }

    struct CommStruct {
        InstSeqNum doneSeqNum;
        InstSeqNum nonSpecSeqNum;
        bool uncached;
        unsigned lqIdx;

        bool stall;
    };

    InstSeqNum globalSeqNum;

    TimeBuffer<CommStruct> comm;

    bool decoupledFrontEnd;

    bool lockFlag;

    Stats::Scalar quiesceCycles;

    Checker<DynInstPtr> *checker;
};

#endif // __CPU_OZONE_CPU_HH__
