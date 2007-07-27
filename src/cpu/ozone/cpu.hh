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

#include "arch/regfile.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "cpu/inst_seq.hh"
#include "cpu/ozone/rename_table.hh"
#include "cpu/ozone/thread_state.hh"
#include "cpu/pc_event.hh"
#include "cpu/static_inst.hh"
#include "mem/page_table.hh"
#include "sim/eventq.hh"

// forward declarations
#if FULL_SYSTEM
#include "arch/alpha/tlb.hh"

namespace TheISA
{
    class ITB;
    class DTB;
}
class PhysicalMemory;
class MemoryController;

class RemoteGDB;
class GDBListener;

namespace TheISA {
    namespace Kernel {
        class Statistics;
    };
};

#else

class Process;

#endif // FULL_SYSTEM

class Checkpoint;
class EndQuiesceEvent;
class MemObject;
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

        void setCpuId(int id);

        int readCpuId() { return thread->readCpuId(); }

#if FULL_SYSTEM
        System *getSystemPtr() { return cpu->system; }

        PhysicalMemory *getPhysMemPtr() { return cpu->physmem; }

        TheISA::ITB *getITBPtr() { return cpu->itb; }

        TheISA::DTB * getDTBPtr() { return cpu->dtb; }

        TheISA::Kernel::Statistics *getKernelStats()
        { return thread->getKernelStats(); }

        FunctionalPort *getPhysPort() { return thread->getPhysPort(); }

        VirtualPort *getVirtPort(ThreadContext *tc = NULL)
        { return thread->getVirtPort(tc); }

        void delVirtPort(VirtualPort *vp);
#else
        TranslatingPort *getMemPort() { return thread->getMemPort(); }

        Process *getProcessPtr() { return thread->getProcessPtr(); }
#endif

        Status status() const { return thread->status(); }

        void setStatus(Status new_status);

        /// Set the status to Active.  Optional delay indicates number of
        /// cycles to wait before beginning execution.
        void activate(int delay = 1);

        /// Set the status to Suspended.
        void suspend();

        /// Set the status to Unallocated.
        void deallocate(int delay = 0);

        /// Set the status to Halted.
        void halt();

#if FULL_SYSTEM
        void dumpFuncProfile();
#endif

        void takeOverFrom(ThreadContext *old_context);

        void regStats(const std::string &name);

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);

#if FULL_SYSTEM
        EndQuiesceEvent *getQuiesceEvent();

        Tick readLastActivate();
        Tick readLastSuspend();

        void profileClear();
        void profileSample();
#endif

        int getThreadNum();

        // Also somewhat obnoxious.  Really only used for the TLB fault.
        TheISA::MachInst getInst();

        void copyArchRegs(ThreadContext *tc);

        void clearArchRegs();

        uint64_t readIntReg(int reg_idx);

        FloatReg readFloatReg(int reg_idx, int width);

        FloatReg readFloatReg(int reg_idx);

        FloatRegBits readFloatRegBits(int reg_idx, int width);

        FloatRegBits readFloatRegBits(int reg_idx);

        void setIntReg(int reg_idx, uint64_t val);

        void setFloatReg(int reg_idx, FloatReg val, int width);

        void setFloatReg(int reg_idx, FloatReg val);

        void setFloatRegBits(int reg_idx, FloatRegBits val, int width);

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

#if !FULL_SYSTEM
        TheISA::IntReg getSyscallArg(int i)
        {
            assert(i < TheISA::NumArgumentRegs);
            return thread->renameTable[TheISA::ArgumentReg[i]]->readIntResult();
        }

        // used to shift args for indirect syscall
        void setSyscallArg(int i, TheISA::IntReg val)
        {
            assert(i < TheISA::NumArgumentRegs);
            thread->renameTable[TheISA::ArgumentReg[i]]->setIntResult(i);
        }

        void setSyscallReturn(SyscallReturn return_value)
        { cpu->setSyscallReturn(return_value, thread->readTid()); }

        Counter readFuncExeInst() { return thread->funcExeInst; }

        void setFuncExeInst(Counter new_val)
        { thread->funcExeInst = new_val; }
#endif
        void changeRegFileContext(TheISA::RegContextParam param,
                                          TheISA::RegContextVal val)
        { panic("Not supported on Alpha!"); }
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

    std::set<InstSeqNum> snList;
    std::set<Addr> lockAddrList;
  private:
    struct TickEvent : public Event
    {
        OzoneCPU *cpu;
        int width;

        TickEvent(OzoneCPU *c, int w);
        void process();
        const char *description();
    };

    TickEvent tickEvent;

    /// Schedule tick event, regardless of its current state.
    void scheduleTickEvent(int delay)
    {
        if (tickEvent.squashed())
            tickEvent.reschedule(curTick + cycles(delay));
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick + cycles(delay));
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
    void post_interrupt(int int_num, int index);

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

    void setCpuId(int id) { cpuId = id; }

    int readCpuId() { return cpuId; }

    int cpuId;

    void switchOut();
    void signalSwitched();
    void takeOverFrom(BaseCPU *oldCPU);

    int switchCount;

#if FULL_SYSTEM
    Addr dbg_vtophys(Addr addr);

    bool interval_stats;

    TheISA::ITB *itb;
    TheISA::DTB *dtb;
    System *system;
    PhysicalMemory *physmem;
#endif

    virtual Port *getPort(const std::string &name, int idx);

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
    Stats::Average<> notIdleFraction;
    Stats::Formula idleFraction;

  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

#if FULL_SYSTEM
    /** Translates instruction requestion. */
    Fault translateInstReq(RequestPtr &req, OzoneThreadState<Impl> *thread)
    {
        return itb->translate(req, thread->getTC());
    }

    /** Translates data read request. */
    Fault translateDataReadReq(RequestPtr &req, OzoneThreadState<Impl> *thread)
    {
        return dtb->translate(req, thread->getTC(), false);
    }

    /** Translates data write request. */
    Fault translateDataWriteReq(RequestPtr &req, OzoneThreadState<Impl> *thread)
    {
        return dtb->translate(req, thread->getTC(), true);
    }

#else
    /** Translates instruction requestion in syscall emulation mode. */
    Fault translateInstReq(RequestPtr &req, OzoneThreadState<Impl> *thread)
    {
        return thread->getProcessPtr()->pTable->translate(req);
    }

    /** Translates data read request in syscall emulation mode. */
    Fault translateDataReadReq(RequestPtr &req, OzoneThreadState<Impl> *thread)
    {
        return thread->getProcessPtr()->pTable->translate(req);
    }

    /** Translates data write request in syscall emulation mode. */
    Fault translateDataWriteReq(RequestPtr &req, OzoneThreadState<Impl> *thread)
    {
        return thread->getProcessPtr()->pTable->translate(req);
    }
#endif

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

  public:
    void squashFromTC();

    void dumpInsts() { frontEnd->dumpInsts(); }

#if FULL_SYSTEM
    Fault hwrei();
    bool simPalCheck(int palFunc);
    void processInterrupts();
#else
    void syscall(uint64_t &callnum);
    void setSyscallReturn(SyscallReturn return_value, int tid);
#endif

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

    Stats::Scalar<> quiesceCycles;

    Checker<DynInstPtr> *checker;
};

#endif // __CPU_OZONE_CPU_HH__
