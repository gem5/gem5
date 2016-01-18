/*
 * Copyright (c) 2014-2015 ARM Limited
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
 * Authors: Kevin Lim
 *          Andreas Sandberg
 *          Mitch Hayenga
 */

#ifndef __CPU_SIMPLE_EXEC_CONTEXT_HH__
#define __CPU_SIMPLE_EXEC_CONTEXT_HH__

#include "arch/registers.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/simple/base.hh"
#include "cpu/static_inst_fwd.hh"
#include "cpu/translation.hh"

class BaseSimpleCPU;

class SimpleExecContext : public ExecContext {
  protected:
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::CCReg CCReg;

  public:
    BaseSimpleCPU *cpu;
    SimpleThread* thread;

    // This is the offset from the current pc that fetch should be performed
    Addr fetchOffset;
    // This flag says to stay at the current pc. This is useful for
    // instructions which go beyond MachInst boundaries.
    bool stayAtPC;

    // Branch prediction
    TheISA::PCState predPC;

    /** PER-THREAD STATS */

    // Number of simulated instructions
    Counter numInst;
    Stats::Scalar numInsts;
    Counter numOp;
    Stats::Scalar numOps;

    // Number of integer alu accesses
    Stats::Scalar numIntAluAccesses;

    // Number of float alu accesses
    Stats::Scalar numFpAluAccesses;

    // Number of function calls/returns
    Stats::Scalar numCallsReturns;

    // Conditional control instructions;
    Stats::Scalar numCondCtrlInsts;

    // Number of int instructions
    Stats::Scalar numIntInsts;

    // Number of float instructions
    Stats::Scalar numFpInsts;

    // Number of integer register file accesses
    Stats::Scalar numIntRegReads;
    Stats::Scalar numIntRegWrites;

    // Number of float register file accesses
    Stats::Scalar numFpRegReads;
    Stats::Scalar numFpRegWrites;

    // Number of condition code register file accesses
    Stats::Scalar numCCRegReads;
    Stats::Scalar numCCRegWrites;

    // Number of simulated memory references
    Stats::Scalar numMemRefs;
    Stats::Scalar numLoadInsts;
    Stats::Scalar numStoreInsts;

    // Number of idle cycles
    Stats::Formula numIdleCycles;

    // Number of busy cycles
    Stats::Formula numBusyCycles;

    // Number of simulated loads
    Counter numLoad;

    // Number of idle cycles
    Stats::Average notIdleFraction;
    Stats::Formula idleFraction;

    // Number of cycles stalled for I-cache responses
    Stats::Scalar icacheStallCycles;
    Counter lastIcacheStall;

    // Number of cycles stalled for D-cache responses
    Stats::Scalar dcacheStallCycles;
    Counter lastDcacheStall;

    /// @{
    /// Total number of branches fetched
    Stats::Scalar numBranches;
    /// Number of branches predicted as taken
    Stats::Scalar numPredictedBranches;
    /// Number of misprediced branches
    Stats::Scalar numBranchMispred;
    /// @}

   // Instruction mix histogram by OpClass
   Stats::Vector statExecutedInstType;

  public:
    /** Constructor */
    SimpleExecContext(BaseSimpleCPU* _cpu, SimpleThread* _thread)
        : cpu(_cpu), thread(_thread), fetchOffset(0), stayAtPC(false),
        numInst(0), numOp(0), numLoad(0), lastIcacheStall(0), lastDcacheStall(0)
    { }

    /** Reads an integer register. */
    IntReg readIntRegOperand(const StaticInst *si, int idx) override
    {
        numIntRegReads++;
        return thread->readIntReg(si->srcRegIdx(idx));
    }

    /** Sets an integer register to a value. */
    void setIntRegOperand(const StaticInst *si, int idx, IntReg val) override
    {
        numIntRegWrites++;
        thread->setIntReg(si->destRegIdx(idx), val);
    }

    /** Reads a floating point register of single register width. */
    FloatReg readFloatRegOperand(const StaticInst *si, int idx) override
    {
        numFpRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Reg_Base;
        return thread->readFloatReg(reg_idx);
    }

    /** Reads a floating point register in its binary format, instead
     * of by value. */
    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx) override
    {
        numFpRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Reg_Base;
        return thread->readFloatRegBits(reg_idx);
    }

    /** Sets a floating point register of single width to a value. */
    void setFloatRegOperand(const StaticInst *si, int idx,
                            FloatReg val) override
    {
        numFpRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Reg_Base;
        thread->setFloatReg(reg_idx, val);
    }

    /** Sets the bits of a floating point register of single width
     * to a binary value. */
    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val) override
    {
        numFpRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Reg_Base;
        thread->setFloatRegBits(reg_idx, val);
    }

    CCReg readCCRegOperand(const StaticInst *si, int idx) override
    {
        numCCRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::CC_Reg_Base;
        return thread->readCCReg(reg_idx);
    }

    void setCCRegOperand(const StaticInst *si, int idx, CCReg val) override
    {
        numCCRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::CC_Reg_Base;
        thread->setCCReg(reg_idx, val);
    }

    MiscReg readMiscRegOperand(const StaticInst *si, int idx) override
    {
        numIntRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::Misc_Reg_Base;
        return thread->readMiscReg(reg_idx);
    }

    void setMiscRegOperand(const StaticInst *si, int idx,
                           const MiscReg &val) override
    {
        numIntRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::Misc_Reg_Base;
        thread->setMiscReg(reg_idx, val);
    }

    /**
     * Reads a miscellaneous register, handling any architectural
     * side effects due to reading that register.
     */
    MiscReg readMiscReg(int misc_reg) override
    {
        numIntRegReads++;
        return thread->readMiscReg(misc_reg);
    }

    /**
     * Sets a miscellaneous register, handling any architectural
     * side effects due to writing that register.
     */
    void setMiscReg(int misc_reg, const MiscReg &val) override
    {
        numIntRegWrites++;
        thread->setMiscReg(misc_reg, val);
    }

    PCState pcState() const override
    {
        return thread->pcState();
    }

    void pcState(const PCState &val) override
    {
        thread->pcState(val);
    }


    /**
     * Record the effective address of the instruction.
     *
     * @note Only valid for memory ops.
     */
    void setEA(Addr EA) override
    { panic("BaseSimpleCPU::setEA() not implemented\n"); }

    /**
     * Get the effective address of the instruction.
     *
     * @note Only valid for memory ops.
     */
    Addr getEA() const override
    { panic("BaseSimpleCPU::getEA() not implemented\n"); }

    Fault readMem(Addr addr, uint8_t *data, unsigned int size,
                  unsigned int flags) override
    {
        return cpu->readMem(addr, data, size, flags);
    }

    Fault initiateMemRead(Addr addr, unsigned int size,
                          unsigned int flags) override
    {
        return cpu->initiateMemRead(addr, size, flags);
    }

    Fault writeMem(uint8_t *data, unsigned int size, Addr addr,
                   unsigned int flags, uint64_t *res) override
    {
        return cpu->writeMem(data, size, addr, flags, res);
    }

    /**
     * Sets the number of consecutive store conditional failures.
     */
    void setStCondFailures(unsigned int sc_failures) override
    {
        thread->setStCondFailures(sc_failures);
    }

    /**
     * Returns the number of consecutive store conditional failures.
     */
    unsigned int readStCondFailures() const override
    {
        return thread->readStCondFailures();
    }

    /**
     * Executes a syscall specified by the callnum.
     */
    void syscall(int64_t callnum) override
    {
        if (FullSystem)
            panic("Syscall emulation isn't available in FS mode.");

        thread->syscall(callnum);
    }

    /** Returns a pointer to the ThreadContext. */
    ThreadContext *tcBase() override
    {
        return thread->getTC();
    }

    /**
     * Somewhat Alpha-specific function that handles returning from an
     * error or interrupt.
     */
    Fault hwrei() override
    {
        return thread->hwrei();
    }

    /**
     * Check for special simulator handling of specific PAL calls.  If
     * return value is false, actual PAL call will be suppressed.
     */
    bool simPalCheck(int palFunc) override
    {
        return thread->simPalCheck(palFunc);
    }

    bool readPredicate() override
    {
        return thread->readPredicate();
    }

    void setPredicate(bool val) override
    {
        thread->setPredicate(val);

        if (cpu->traceData) {
            cpu->traceData->setPredicate(val);
        }
    }

    /**
     * Invalidate a page in the DTLB <i>and</i> ITLB.
     */
    void demapPage(Addr vaddr, uint64_t asn) override
    {
        thread->demapPage(vaddr, asn);
    }

    void armMonitor(Addr address) override
    {
        cpu->armMonitor(thread->threadId(), address);
    }

    bool mwait(PacketPtr pkt) override
    {
        return cpu->mwait(thread->threadId(), pkt);
    }

    void mwaitAtomic(ThreadContext *tc) override
    {
        cpu->mwaitAtomic(thread->threadId(), tc, thread->dtb);
    }

    AddressMonitor *getAddrMonitor() override
    {
        return cpu->getCpuAddrMonitor(thread->threadId());
    }

#if THE_ISA == MIPS_ISA
    MiscReg readRegOtherThread(int regIdx, ThreadID tid = InvalidThreadID)
        override
    {
        panic("Simple CPU models do not support multithreaded "
              "register access.");
    }

    void setRegOtherThread(int regIdx, MiscReg val,
                           ThreadID tid = InvalidThreadID) override
    {
        panic("Simple CPU models do not support multithreaded "
              "register access.");
    }

#endif

};

#endif // __CPU_EXEC_CONTEXT_HH__
