/*
 * Copyright (c) 2011-2012 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 * Authors: Steve Reinhardt
 *          Dave Greene
 *          Nathan Binkert
 */

#ifndef __CPU_SIMPLE_BASE_HH__
#define __CPU_SIMPLE_BASE_HH__

#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/pc_event.hh"
#include "cpu/simple_thread.hh"
#include "cpu/static_inst.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

// forward declarations
class Checkpoint;
class Process;
class Processor;
class ThreadContext;

namespace TheISA
{
    class DTB;
    class ITB;
}

namespace Trace {
    class InstRecord;
}

struct BaseSimpleCPUParams;
class BPredUnit;

class BaseSimpleCPU : public BaseCPU
{
  protected:
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::CCReg CCReg;

    BPredUnit *branchPred;

  protected:
    Trace::InstRecord *traceData;

    inline void checkPcEventQueue() {
        Addr oldpc, pc = thread->instAddr();
        do {
            oldpc = pc;
            system->pcEventQueue.service(tc);
            pc = thread->instAddr();
        } while (oldpc != pc);
    }

  public:
    void wakeup();

    void zero_fill_64(Addr addr) {
      static int warned = 0;
      if (!warned) {
        warn ("WH64 is not implemented");
        warned = 1;
      }
    };

  public:
    BaseSimpleCPU(BaseSimpleCPUParams *params);
    virtual ~BaseSimpleCPU();

  public:
    /** SimpleThread object, provides all the architectural state. */
    SimpleThread *thread;

    /** ThreadContext object, provides an interface for external
     * objects to modify this thread's state.
     */
    ThreadContext *tc;

    CheckerCPU *checker;

  protected:

    enum Status {
        Idle,
        Running,
        Faulting,
        ITBWaitResponse,
        IcacheRetry,
        IcacheWaitResponse,
        IcacheWaitSwitch,
        DTBWaitResponse,
        DcacheRetry,
        DcacheWaitResponse,
        DcacheWaitSwitch,
    };

    Status _status;

  public:

    Addr dbg_vtophys(Addr addr);

    bool interval_stats;

    // current instruction
    TheISA::MachInst inst;

    StaticInstPtr curStaticInst;
    StaticInstPtr curMacroStaticInst;

    //This is the offset from the current pc that fetch should be performed at
    Addr fetchOffset;
    //This flag says to stay at the current pc. This is useful for
    //instructions which go beyond MachInst boundaries.
    bool stayAtPC;

    void checkForInterrupts();
    void setupFetchRequest(Request *req);
    void preExecute();
    void postExecute();
    void advancePC(Fault fault);

    virtual void deallocateContext(ThreadID thread_num);
    virtual void haltContext(ThreadID thread_num);

    // statistics
    virtual void regStats();
    virtual void resetStats();

    virtual void startup();

    // number of simulated instructions
    Counter numInst;
    Counter startNumInst;
    Stats::Scalar numInsts;
    Counter numOp;
    Counter startNumOp;
    Stats::Scalar numOps;

    void countInst()
    {
        if (!curStaticInst->isMicroop() || curStaticInst->isLastMicroop()) {
            numInst++;
            numInsts++;
        }
        numOp++;
        numOps++;

        system->totalNumInsts++;
        thread->funcExeInst++;
    }

    virtual Counter totalInsts() const
    {
        return numInst - startNumInst;
    }

    virtual Counter totalOps() const
    {
        return numOp - startNumOp;
    }

    //number of integer alu accesses
    Stats::Scalar numIntAluAccesses;

    //number of float alu accesses
    Stats::Scalar numFpAluAccesses;

    //number of function calls/returns
    Stats::Scalar numCallsReturns;

    //conditional control instructions;
    Stats::Scalar numCondCtrlInsts;

    //number of int instructions
    Stats::Scalar numIntInsts;

    //number of float instructions
    Stats::Scalar numFpInsts;

    //number of integer register file accesses
    Stats::Scalar numIntRegReads;
    Stats::Scalar numIntRegWrites;

    //number of float register file accesses
    Stats::Scalar numFpRegReads;
    Stats::Scalar numFpRegWrites;

    //number of condition code register file accesses
    Stats::Scalar numCCRegReads;
    Stats::Scalar numCCRegWrites;

    // number of simulated memory references
    Stats::Scalar numMemRefs;
    Stats::Scalar numLoadInsts;
    Stats::Scalar numStoreInsts;

    // number of idle cycles
    Stats::Formula numIdleCycles;

    // number of busy cycles
    Stats::Formula numBusyCycles;

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    // number of idle cycles
    Stats::Average notIdleFraction;
    Stats::Formula idleFraction;

    // number of cycles stalled for I-cache responses
    Stats::Scalar icacheStallCycles;
    Counter lastIcacheStall;

    // number of cycles stalled for I-cache retries
    Stats::Scalar icacheRetryCycles;
    Counter lastIcacheRetry;

    // number of cycles stalled for D-cache responses
    Stats::Scalar dcacheStallCycles;
    Counter lastDcacheStall;

    // number of cycles stalled for D-cache retries
    Stats::Scalar dcacheRetryCycles;
    Counter lastDcacheRetry;

    /// @{
    /// Total number of branches fetched
    Stats::Scalar numBranches;
    /// Number of branches predicted as taken
    Stats::Scalar numPredictedBranches;
    /// Number of misprediced branches
    Stats::Scalar numBranchMispred;
    /// @}

    void serializeThread(std::ostream &os, ThreadID tid);
    void unserializeThread(Checkpoint *cp, const std::string &section,
                           ThreadID tid);

    // These functions are only used in CPU models that split
    // effective address computation from the actual memory access.
    void setEA(Addr EA) { panic("BaseSimpleCPU::setEA() not implemented\n"); }
    Addr getEA()        { panic("BaseSimpleCPU::getEA() not implemented\n");
        M5_DUMMY_RETURN}

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

    uint64_t readIntRegOperand(const StaticInst *si, int idx)
    {
        numIntRegReads++;
        return thread->readIntReg(si->srcRegIdx(idx));
    }

    FloatReg readFloatRegOperand(const StaticInst *si, int idx)
    {
        numFpRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Reg_Base;
        return thread->readFloatReg(reg_idx);
    }

    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx)
    {
        numFpRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Reg_Base;
        return thread->readFloatRegBits(reg_idx);
    }

    CCReg readCCRegOperand(const StaticInst *si, int idx)
    {
        numCCRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::CC_Reg_Base;
        return thread->readCCReg(reg_idx);
    }

    void setIntRegOperand(const StaticInst *si, int idx, uint64_t val)
    {
        numIntRegWrites++;
        thread->setIntReg(si->destRegIdx(idx), val);
    }

    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val)
    {
        numFpRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Reg_Base;
        thread->setFloatReg(reg_idx, val);
    }

    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val)
    {
        numFpRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Reg_Base;
        thread->setFloatRegBits(reg_idx, val);
    }

    void setCCRegOperand(const StaticInst *si, int idx, CCReg val)
    {
        numCCRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::CC_Reg_Base;
        thread->setCCReg(reg_idx, val);
    }

    bool readPredicate() { return thread->readPredicate(); }
    void setPredicate(bool val)
    {
        thread->setPredicate(val);
        if (traceData) {
            traceData->setPredicate(val);
        }
    }
    TheISA::PCState pcState() { return thread->pcState(); }
    void pcState(const TheISA::PCState &val) { thread->pcState(val); }
    Addr instAddr() { return thread->instAddr(); }
    Addr nextInstAddr() { return thread->nextInstAddr(); }
    MicroPC microPC() { return thread->microPC(); }

    MiscReg readMiscRegNoEffect(int misc_reg)
    {
        return thread->readMiscRegNoEffect(misc_reg);
    }

    MiscReg readMiscReg(int misc_reg)
    {
        numIntRegReads++;
        return thread->readMiscReg(misc_reg);
    }

    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        numIntRegWrites++;
        return thread->setMiscReg(misc_reg, val);
    }

    MiscReg readMiscRegOperand(const StaticInst *si, int idx)
    {
        numIntRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::Misc_Reg_Base;
        return thread->readMiscReg(reg_idx);
    }

    void setMiscRegOperand(
            const StaticInst *si, int idx, const MiscReg &val)
    {
        numIntRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::Misc_Reg_Base;
        return thread->setMiscReg(reg_idx, val);
    }

    void demapPage(Addr vaddr, uint64_t asn)
    {
        thread->demapPage(vaddr, asn);
    }

    void demapInstPage(Addr vaddr, uint64_t asn)
    {
        thread->demapInstPage(vaddr, asn);
    }

    void demapDataPage(Addr vaddr, uint64_t asn)
    {
        thread->demapDataPage(vaddr, asn);
    }

    unsigned readStCondFailures() {
        return thread->readStCondFailures();
    }

    void setStCondFailures(unsigned sc_failures) {
        thread->setStCondFailures(sc_failures);
    }

     MiscReg readRegOtherThread(int regIdx, ThreadID tid = InvalidThreadID)
     {
        panic("Simple CPU models do not support multithreaded "
              "register access.\n");
     }

     void setRegOtherThread(int regIdx, const MiscReg &val,
                            ThreadID tid = InvalidThreadID)
     {
        panic("Simple CPU models do not support multithreaded "
              "register access.\n");
     }

    //Fault CacheOp(uint8_t Op, Addr EA);

    Fault hwrei() { return thread->hwrei(); }
    bool simPalCheck(int palFunc) { return thread->simPalCheck(palFunc); }

    void
    syscall(int64_t callnum)
    {
        if (FullSystem)
            panic("Syscall emulation isn't available in FS mode.\n");

        thread->syscall(callnum);
    }

    bool misspeculating() { return thread->misspeculating(); }
    ThreadContext *tcBase() { return tc; }

  private:
    TheISA::PCState pred_pc;
};

#endif // __CPU_SIMPLE_BASE_HH__
