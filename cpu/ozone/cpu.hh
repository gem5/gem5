/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#ifndef __CPU_OZONE_CPU_HH__
#define __CPU_OZONE_CPU_HH__

#include <set>

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/inst_seq.hh"
#include "cpu/ozone/rename_table.hh"
#include "cpu/ozone/thread_state.hh"
#include "cpu/pc_event.hh"
#include "cpu/static_inst.hh"
#include "mem/mem_interface.hh"
#include "sim/eventq.hh"

// forward declarations
#if FULL_SYSTEM
#include "arch/alpha/tlb.hh"

class AlphaITB;
class AlphaDTB;
class PhysicalMemory;
class MemoryController;

class Sampler;
class RemoteGDB;
class GDBListener;

#else

class Process;

#endif // FULL_SYSTEM

class Checkpoint;
class EndQuiesceEvent;
class MemInterface;

namespace Trace {
    class InstRecord;
}

template <class>
class Checker;

/**
 * Declaration of Out-of-Order CPU class.  Basically it is a SimpleCPU with
 * simple out-of-order capabilities added to it.  It is still a 1 CPI machine
 * (?), but is capable of handling cache misses.  Basically it models having
 * a ROB/IQ by only allowing a certain amount of instructions to execute while
 * the cache miss is outstanding.
 */

template <class Impl>
class OzoneCPU : public BaseCPU
{
  private:
    typedef typename Impl::FrontEnd FrontEnd;
    typedef typename Impl::BackEnd BackEnd;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::DynInstPtr DynInstPtr;

    typedef TheISA::MiscReg MiscReg;

  public:
    class OzoneXC : public ExecContext {
      public:
        OzoneCPU<Impl> *cpu;

        OzoneThreadState<Impl> *thread;

        BaseCPU *getCpuPtr();

        void setCpuId(int id);

        int readCpuId() { return thread->cpuId; }

        FunctionalMemory *getMemPtr() { return thread->mem; }

#if FULL_SYSTEM
        System *getSystemPtr() { return cpu->system; }

        PhysicalMemory *getPhysMemPtr() { return cpu->physmem; }

        AlphaITB *getITBPtr() { return cpu->itb; }

        AlphaDTB * getDTBPtr() { return cpu->dtb; }
#else
        Process *getProcessPtr() { return thread->process; }
#endif

        Status status() const { return thread->_status; }

        void setStatus(Status new_status);

        /// Set the status to Active.  Optional delay indicates number of
        /// cycles to wait before beginning execution.
        void activate(int delay = 1);

        /// Set the status to Suspended.
        void suspend();

        /// Set the status to Unallocated.
        void deallocate();

        /// Set the status to Halted.
        void halt();

#if FULL_SYSTEM
        void dumpFuncProfile();
#endif

        void takeOverFrom(ExecContext *old_context);

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

        void copyArchRegs(ExecContext *xc);

        void clearArchRegs();

        uint64_t readIntReg(int reg_idx);

        float readFloatRegSingle(int reg_idx);

        double readFloatRegDouble(int reg_idx);

        uint64_t readFloatRegInt(int reg_idx);

        void setIntReg(int reg_idx, uint64_t val);

        void setFloatRegSingle(int reg_idx, float val);

        void setFloatRegDouble(int reg_idx, double val);

        void setFloatRegInt(int reg_idx, uint64_t val);

        uint64_t readPC() { return thread->PC; }
        void setPC(Addr val);

        uint64_t readNextPC() { return thread->nextPC; }
        void setNextPC(Addr val);

      public:
        // ISA stuff:
        MiscReg readMiscReg(int misc_reg);

        MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault);

        Fault setMiscReg(int misc_reg, const MiscReg &val);

        Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val);

        unsigned readStCondFailures()
        { return thread->storeCondFailures; }

        void setStCondFailures(unsigned sc_failures)
        { thread->storeCondFailures = sc_failures; }

#if FULL_SYSTEM
        bool inPalMode() { return cpu->inPalMode(); }
#endif

        bool misspeculating() { return false; }

#if !FULL_SYSTEM
        TheISA::IntReg getSyscallArg(int i)
        { return thread->renameTable[TheISA::ArgumentReg0 + i]->readIntResult(); }

        // used to shift args for indirect syscall
        void setSyscallArg(int i, TheISA::IntReg val)
        { thread->renameTable[TheISA::ArgumentReg0 + i]->setIntResult(i); }

        void setSyscallReturn(SyscallReturn return_value)
        { cpu->setSyscallReturn(return_value, thread->tid); }

        Counter readFuncExeInst() { return thread->funcExeInst; }

        void setFuncExeInst(Counter new_val)
        { thread->funcExeInst = new_val; }
#endif
    };

    // execution context proxy
    OzoneXC ozoneXC;
    ExecContext *xcProxy;
    ExecContext *checkerXC;

    typedef OzoneThreadState<Impl> ImplState;

  private:
    OzoneThreadState<Impl> thread;
/*
    // Squash event for when the XC needs to squash all inflight instructions.
    struct XCSquashEvent : public Event
    {
        void process();
        const char *description();
    };
*/
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

  private:
    Trace::InstRecord *traceData;

    template<typename T>
    void trace_data(T data);

  public:
    //
    enum Status {
        Running,
        Idle,
        SwitchedOut
    };

    Status _status;

  public:
    bool checkInterrupts;

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

//    FunctionalMemory *getMemPtr() { return mem; }

    int cpuId;

    void switchOut(Sampler *sampler);
    void signalSwitched();
    void takeOverFrom(BaseCPU *oldCPU);

    Sampler *sampler;

    int switchCount;

#if FULL_SYSTEM
    Addr dbg_vtophys(Addr addr);

    bool interval_stats;

    AlphaITB *itb;
    AlphaDTB *dtb;
    System *system;

    // the following two fields are redundant, since we can always
    // look them up through the system pointer, but we'll leave them
    // here for now for convenience
    MemoryController *memctrl;
    PhysicalMemory *physmem;
#endif

    // L1 instruction cache
    MemInterface *icacheInterface;

    // L1 data cache
    MemInterface *dcacheInterface;

    /** Pointer to memory. */
    FunctionalMemory *mem;

    FrontEnd *frontEnd;

    BackEnd *backEnd;
  private:
    Status status() const { return _status; }
    void setStatus(Status new_status) { _status = new_status; }

    // Not sure what an activate() call on the CPU's proxy XC would mean...

    virtual void activateContext(int thread_num, int delay);
    virtual void suspendContext(int thread_num);
    virtual void deallocateContext(int thread_num);
    virtual void haltContext(int thread_num);

    // statistics
    virtual void regStats();
    virtual void resetStats();

    // number of simulated instructions
  public:
    Counter numInst;
    Counter startNumInst;
//    Stats::Scalar<> numInsts;

    virtual Counter totalInstructions() const
    {
        return numInst - startNumInst;
    }

  private:
    // number of simulated memory references
//    Stats::Scalar<> numMemRefs;

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
    bool validInstAddr(Addr addr) { return true; }
    bool validDataAddr(Addr addr) { return true; }

    Fault translateInstReq(MemReqPtr &req)
    {
        return itb->translate(req);
    }

    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dtb->translate(req, false);
    }

    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dtb->translate(req, true);
    }

#else
    bool validInstAddr(Addr addr)
    { return true; }

    bool validDataAddr(Addr addr)
    { return true; }

    int getInstAsid() { return thread.asid; }
    int getDataAsid() { return thread.asid; }

    Fault dummyTranslation(MemReqPtr &req)
    {
#if 0
        assert((req->vaddr >> 48 & 0xffff) == 0);
#endif

        // put the asid in the upper 16 bits of the paddr
        req->paddr = req->vaddr & ~((Addr)0xffff << sizeof(Addr) * 8 - 16);
        req->paddr = req->paddr | (Addr)req->asid << sizeof(Addr) * 8 - 16;
        return NoFault;
    }

    /** Translates instruction requestion in syscall emulation mode. */
    Fault translateInstReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

    /** Translates data read request in syscall emulation mode. */
    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

    /** Translates data write request in syscall emulation mode. */
    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }
#endif

    /** Old CPU read from memory function. No longer used. */
    template <class T>
    Fault read(MemReqPtr &req, T &data)
    {
//	panic("CPU READ NOT IMPLEMENTED W/NEW MEMORY\n");
#if 0
#if FULL_SYSTEM && defined(TARGET_ALPHA)
        if (req->flags & LOCKED) {
            req->xc->setMiscReg(TheISA::Lock_Addr_DepTag, req->paddr);
            req->xc->setMiscReg(TheISA::Lock_Flag_DepTag, true);
        }
#endif
#endif
        Fault error;
        if (req->flags & LOCKED) {
//            lockAddr = req->paddr;
            lockAddrList.insert(req->paddr);
            lockFlag = true;
        }

        error = this->mem->read(req, data);
        data = gtoh(data);
        return error;
    }


    /** CPU read function, forwards read to LSQ. */
    template <class T>
    Fault read(MemReqPtr &req, T &data, int load_idx)
    {
        return backEnd->read(req, data, load_idx);
    }

    /** Old CPU write to memory function. No longer used. */
    template <class T>
    Fault write(MemReqPtr &req, T &data)
    {
#if 0
#if FULL_SYSTEM && defined(TARGET_ALPHA)
        ExecContext *xc;

        // If this is a store conditional, act appropriately
        if (req->flags & LOCKED) {
            xc = req->xc;

            if (req->flags & UNCACHEABLE) {
                // Don't update result register (see stq_c in isa_desc)
                req->result = 2;
                xc->setStCondFailures(0);//Needed? [RGD]
            } else {
                bool lock_flag = xc->readMiscReg(TheISA::Lock_Flag_DepTag);
                Addr lock_addr = xc->readMiscReg(TheISA::Lock_Addr_DepTag);
                req->result = lock_flag;
                if (!lock_flag ||
                    ((lock_addr & ~0xf) != (req->paddr & ~0xf))) {
                    xc->setMiscReg(TheISA::Lock_Flag_DepTag, false);
                    xc->setStCondFailures(xc->readStCondFailures() + 1);
                    if (((xc->readStCondFailures()) % 100000) == 0) {
                        std::cerr << "Warning: "
                                  << xc->readStCondFailures()
                                  << " consecutive store conditional failures "
                                  << "on cpu " << req->xc->readCpuId()
                                  << std::endl;
                    }
                    return NoFault;
                }
                else xc->setStCondFailures(0);
            }
        }

        // Need to clear any locked flags on other proccessors for
        // this address.  Only do this for succsful Store Conditionals
        // and all other stores (WH64?).  Unsuccessful Store
        // Conditionals would have returned above, and wouldn't fall
        // through.
        for (int i = 0; i < this->system->execContexts.size(); i++){
            xc = this->system->execContexts[i];
            if ((xc->readMiscReg(TheISA::Lock_Addr_DepTag) & ~0xf) ==
                (req->paddr & ~0xf)) {
                xc->setMiscReg(TheISA::Lock_Flag_DepTag, false);
            }
        }

#endif
#endif

        if (req->flags & LOCKED) {
            if (req->flags & UNCACHEABLE) {
                req->result = 2;
            } else {
                if (this->lockFlag/* && this->lockAddr == req->paddr*/) {
                    if (lockAddrList.find(req->paddr) !=
                        lockAddrList.end()) {
                        req->result = 1;
                    } else {
                        req->result = 0;
                        return NoFault;
                    }
                } else {
                    req->result = 0;
                    return NoFault;
                }
            }
        }

        return this->mem->write(req, (T)htog(data));
    }

    /** CPU write function, forwards write to LSQ. */
    template <class T>
    Fault write(MemReqPtr &req, T &data, int store_idx)
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

    InstSeqNum globalSeqNum;

  public:
    void squashFromXC();

    // @todo: This can be a useful debug function.  Implement it.
    void dumpInsts() { frontEnd->dumpInsts(); }

#if FULL_SYSTEM
    Fault hwrei();
    int readIntrFlag() { return thread.regs.intrflag; }
    void setIntrFlag(int val) { thread.regs.intrflag = val; }
    bool inPalMode() { return AlphaISA::PcPAL(thread.PC); }
    bool inPalMode(Addr pc) { return AlphaISA::PcPAL(pc); }
    bool simPalCheck(int palFunc);
    void processInterrupts();
#else
    void syscall();
    void setSyscallReturn(SyscallReturn return_value, int tid);
#endif

    ExecContext *xcBase() { return xcProxy; }

    bool decoupledFrontEnd;
    struct CommStruct {
        InstSeqNum doneSeqNum;
        InstSeqNum nonSpecSeqNum;
        bool uncached;
        unsigned lqIdx;

        bool stall;
    };
    TimeBuffer<CommStruct> comm;

    bool lockFlag;

    Stats::Scalar<> quiesceCycles;

    Checker<DynInstPtr> *checker;
};

#endif // __CPU_OZONE_CPU_HH__
