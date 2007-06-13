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
 * Authors: Steve Reinhardt
 *          Dave Greene
 *          Nathan Binkert
 */

#ifndef __CPU_SIMPLE_BASE_HH__
#define __CPU_SIMPLE_BASE_HH__

#include "arch/predecoder.hh"
#include "base/statistics.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/pc_event.hh"
#include "cpu/static_inst.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"

// forward declarations
#if FULL_SYSTEM
class Processor;
namespace TheISA
{
    class ITB;
    class DTB;
}
class MemObject;

#else

class Process;

#endif // FULL_SYSTEM

class RemoteGDB;
class GDBListener;

namespace TheISA
{
    class Predecoder;
}
class ThreadContext;
class Checkpoint;

namespace Trace {
    class InstRecord;
}


class BaseSimpleCPU : public BaseCPU
{
  protected:
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;

  protected:
    Trace::InstRecord *traceData;

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
#if FULL_SYSTEM
        TheISA::ITB *itb;
        TheISA::DTB *dtb;
#else
        Process *process;
#endif
    };
    BaseSimpleCPU(Params *params);
    virtual ~BaseSimpleCPU();

  public:
    /** SimpleThread object, provides all the architectural state. */
    SimpleThread *thread;

    /** ThreadContext object, provides an interface for external
     * objects to modify this thread's state.
     */
    ThreadContext *tc;

#if FULL_SYSTEM
    Addr dbg_vtophys(Addr addr);

    bool interval_stats;
#endif

    // current instruction
    TheISA::MachInst inst;

    // The predecoder
    TheISA::Predecoder predecoder;

    // Static data storage
    TheISA::LargestRead dataReg;

    StaticInstPtr curStaticInst;
    StaticInstPtr curMacroStaticInst;

    //This is the offset from the current pc that fetch should be performed at
    Addr fetchOffset;
    //This flag says to stay at the current pc. This is useful for
    //instructions which go beyond MachInst boundaries.
    bool stayAtPC;

    void checkForInterrupts();
    Fault setupFetchRequest(Request *req);
    void preExecute();
    void postExecute();
    void advancePC(Fault fault);

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

    // Mask to align PCs to MachInst sized boundaries
    static const Addr PCMask = ~((Addr)sizeof(TheISA::MachInst) - 1);

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

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    // These functions are only used in CPU models that split
    // effective address computation from the actual memory access.
    void setEA(Addr EA) { panic("BaseSimpleCPU::setEA() not implemented\n"); }
    Addr getEA() 	{ panic("BaseSimpleCPU::getEA() not implemented\n");
        M5_DUMMY_RETURN}

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

    uint64_t readIntRegOperand(const StaticInst *si, int idx)
    {
        return thread->readIntReg(si->srcRegIdx(idx));
    }

    FloatReg readFloatRegOperand(const StaticInst *si, int idx, int width)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return thread->readFloatReg(reg_idx, width);
    }

    FloatReg readFloatRegOperand(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return thread->readFloatReg(reg_idx);
    }

    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx,
                                         int width)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return thread->readFloatRegBits(reg_idx, width);
    }

    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return thread->readFloatRegBits(reg_idx);
    }

    void setIntRegOperand(const StaticInst *si, int idx, uint64_t val)
    {
        thread->setIntReg(si->destRegIdx(idx), val);
    }

    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val,
                            int width)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        thread->setFloatReg(reg_idx, val, width);
    }

    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        thread->setFloatReg(reg_idx, val);
    }

    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val, int width)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        thread->setFloatRegBits(reg_idx, val, width);
    }

    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        thread->setFloatRegBits(reg_idx, val);
    }

    uint64_t readPC() { return thread->readPC(); }
    uint64_t readNextPC() { return thread->readNextPC(); }
    uint64_t readNextNPC() { return thread->readNextNPC(); }

    void setPC(uint64_t val) { thread->setPC(val); }
    void setNextPC(uint64_t val) { thread->setNextPC(val); }
    void setNextNPC(uint64_t val) { thread->setNextNPC(val); }

    MiscReg readMiscRegNoEffect(int misc_reg)
    {
        return thread->readMiscRegNoEffect(misc_reg);
    }

    MiscReg readMiscReg(int misc_reg)
    {
        return thread->readMiscReg(misc_reg);
    }

    void setMiscRegNoEffect(int misc_reg, const MiscReg &val)
    {
        return thread->setMiscRegNoEffect(misc_reg, val);
    }

    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        return thread->setMiscReg(misc_reg, val);
    }

    MiscReg readMiscRegOperandNoEffect(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::Ctrl_Base_DepTag;
        return thread->readMiscRegNoEffect(reg_idx);
    }

    MiscReg readMiscRegOperand(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::Ctrl_Base_DepTag;
        return thread->readMiscReg(reg_idx);
    }

    void setMiscRegOperandNoEffect(const StaticInst *si, int idx, const MiscReg &val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::Ctrl_Base_DepTag;
        return thread->setMiscRegNoEffect(reg_idx, val);
    }

    void setMiscRegOperand(
            const StaticInst *si, int idx, const MiscReg &val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::Ctrl_Base_DepTag;
        return thread->setMiscReg(reg_idx, val);
    }

    unsigned readStCondFailures() {
        return thread->readStCondFailures();
    }

    void setStCondFailures(unsigned sc_failures) {
        thread->setStCondFailures(sc_failures);
    }

#if FULL_SYSTEM
    Fault hwrei() { return thread->hwrei(); }
    void ev5_trap(Fault fault) { fault->invoke(tc); }
    bool simPalCheck(int palFunc) { return thread->simPalCheck(palFunc); }
#else
    void syscall(int64_t callnum) { thread->syscall(callnum); }
#endif

    bool misspeculating() { return thread->misspeculating(); }
    ThreadContext *tcBase() { return tc; }
};

#endif // __CPU_SIMPLE_BASE_HH__
