/*
 * Copyright (c) 2014-2017 ARM Limited
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
#include "cpu/reg_class.hh"
#include "cpu/simple/base.hh"
#include "cpu/static_inst_fwd.hh"
#include "cpu/translation.hh"
#include "mem/request.hh"

class BaseSimpleCPU;

class SimpleExecContext : public ExecContext {
  protected:
    using VecRegContainer = TheISA::VecRegContainer;
    using VecElem = TheISA::VecElem;

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

    // Number of vector alu accesses
    Stats::Scalar numVecAluAccesses;

    // Number of function calls/returns
    Stats::Scalar numCallsReturns;

    // Conditional control instructions;
    Stats::Scalar numCondCtrlInsts;

    // Number of int instructions
    Stats::Scalar numIntInsts;

    // Number of float instructions
    Stats::Scalar numFpInsts;

    // Number of vector instructions
    Stats::Scalar numVecInsts;

    // Number of integer register file accesses
    Stats::Scalar numIntRegReads;
    Stats::Scalar numIntRegWrites;

    // Number of float register file accesses
    Stats::Scalar numFpRegReads;
    Stats::Scalar numFpRegWrites;

    // Number of vector register file accesses
    mutable Stats::Scalar numVecRegReads;
    Stats::Scalar numVecRegWrites;

    // Number of predicate register file accesses
    mutable Stats::Scalar numVecPredRegReads;
    Stats::Scalar numVecPredRegWrites;

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
    RegVal
    readIntRegOperand(const StaticInst *si, int idx) override
    {
        numIntRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isIntReg());
        return thread->readIntReg(reg.index());
    }

    /** Sets an integer register to a value. */
    void
    setIntRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        numIntRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isIntReg());
        thread->setIntReg(reg.index(), val);
    }

    /** Reads a floating point register in its binary format, instead
     * of by value. */
    RegVal
    readFloatRegOperandBits(const StaticInst *si, int idx) override
    {
        numFpRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isFloatReg());
        return thread->readFloatReg(reg.index());
    }

    /** Sets the bits of a floating point register of single width
     * to a binary value. */
    void
    setFloatRegOperandBits(const StaticInst *si, int idx, RegVal val) override
    {
        numFpRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isFloatReg());
        thread->setFloatReg(reg.index(), val);
    }

    /** Reads a vector register. */
    const VecRegContainer &
    readVecRegOperand(const StaticInst *si, int idx) const override
    {
        numVecRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecReg());
        return thread->readVecReg(reg);
    }

    /** Reads a vector register for modification. */
    VecRegContainer &
    getWritableVecRegOperand(const StaticInst *si, int idx) override
    {
        numVecRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecReg());
        return thread->getWritableVecReg(reg);
    }

    /** Sets a vector register to a value. */
    void
    setVecRegOperand(const StaticInst *si, int idx,
                     const VecRegContainer& val) override
    {
        numVecRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecReg());
        thread->setVecReg(reg, val);
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector lane. */
    template <typename VecElem>
    VecLaneT<VecElem, true>
    readVecLaneOperand(const StaticInst *si, int idx) const
    {
        numVecRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecReg());
        return thread->readVecLane<VecElem>(reg);
    }
    /** Reads source vector 8bit operand. */
    virtual ConstVecLane8
    readVec8BitLaneOperand(const StaticInst *si, int idx) const
                            override
    { return readVecLaneOperand<uint8_t>(si, idx); }

    /** Reads source vector 16bit operand. */
    virtual ConstVecLane16
    readVec16BitLaneOperand(const StaticInst *si, int idx) const
                            override
    { return readVecLaneOperand<uint16_t>(si, idx); }

    /** Reads source vector 32bit operand. */
    virtual ConstVecLane32
    readVec32BitLaneOperand(const StaticInst *si, int idx) const
                            override
    { return readVecLaneOperand<uint32_t>(si, idx); }

    /** Reads source vector 64bit operand. */
    virtual ConstVecLane64
    readVec64BitLaneOperand(const StaticInst *si, int idx) const
                            override
    { return readVecLaneOperand<uint64_t>(si, idx); }

    /** Write a lane of the destination vector operand. */
    template <typename LD>
    void
    setVecLaneOperandT(const StaticInst *si, int idx,
            const LD& val)
    {
        numVecRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecReg());
        return thread->setVecLane(reg, val);
    }
    /** Write a lane of the destination vector operand. */
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::Byte>& val) override
    { return setVecLaneOperandT(si, idx, val); }
    /** Write a lane of the destination vector operand. */
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::TwoByte>& val) override
    { return setVecLaneOperandT(si, idx, val); }
    /** Write a lane of the destination vector operand. */
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::FourByte>& val) override
    { return setVecLaneOperandT(si, idx, val); }
    /** Write a lane of the destination vector operand. */
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::EightByte>& val) override
    { return setVecLaneOperandT(si, idx, val); }
    /** @} */

    /** Reads an element of a vector register. */
    VecElem
    readVecElemOperand(const StaticInst *si, int idx) const override
    {
        numVecRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecElem());
        return thread->readVecElem(reg);
    }

    /** Sets an element of a vector register to a value. */
    void
    setVecElemOperand(const StaticInst *si, int idx,
                      const VecElem val) override
    {
        numVecRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecElem());
        thread->setVecElem(reg, val);
    }

    const VecPredRegContainer&
    readVecPredRegOperand(const StaticInst *si, int idx) const override
    {
        numVecPredRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecPredReg());
        return thread->readVecPredReg(reg);
    }

    VecPredRegContainer&
    getWritableVecPredRegOperand(const StaticInst *si, int idx) override
    {
        numVecPredRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecPredReg());
        return thread->getWritableVecPredReg(reg);
    }

    void
    setVecPredRegOperand(const StaticInst *si, int idx,
                         const VecPredRegContainer& val) override
    {
        numVecPredRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecPredReg());
        thread->setVecPredReg(reg, val);
    }

    RegVal
    readCCRegOperand(const StaticInst *si, int idx) override
    {
        numCCRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isCCReg());
        return thread->readCCReg(reg.index());
    }

    void
    setCCRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        numCCRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isCCReg());
        thread->setCCReg(reg.index(), val);
    }

    RegVal
    readMiscRegOperand(const StaticInst *si, int idx) override
    {
        numIntRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isMiscReg());
        return thread->readMiscReg(reg.index());
    }

    void
    setMiscRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        numIntRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isMiscReg());
        thread->setMiscReg(reg.index(), val);
    }

    /**
     * Reads a miscellaneous register, handling any architectural
     * side effects due to reading that register.
     */
    RegVal
    readMiscReg(int misc_reg) override
    {
        numIntRegReads++;
        return thread->readMiscReg(misc_reg);
    }

    /**
     * Sets a miscellaneous register, handling any architectural
     * side effects due to writing that register.
     */
    void
    setMiscReg(int misc_reg, RegVal val) override
    {
        numIntRegWrites++;
        thread->setMiscReg(misc_reg, val);
    }

    PCState
    pcState() const override
    {
        return thread->pcState();
    }

    void
    pcState(const PCState &val) override
    {
        thread->pcState(val);
    }


    Fault
    readMem(Addr addr, uint8_t *data, unsigned int size,
            Request::Flags flags) override
    {
        return cpu->readMem(addr, data, size, flags);
    }

    Fault
    initiateMemRead(Addr addr, unsigned int size,
                    Request::Flags flags) override
    {
        return cpu->initiateMemRead(addr, size, flags);
    }

    Fault
    writeMem(uint8_t *data, unsigned int size, Addr addr,
             Request::Flags flags, uint64_t *res) override
    {
        return cpu->writeMem(data, size, addr, flags, res);
    }

    Fault amoMem(Addr addr, uint8_t *data, unsigned int size,
                 Request::Flags flags, AtomicOpFunctor *amo_op) override
    {
        return cpu->amoMem(addr, data, size, flags, amo_op);
    }

    Fault initiateMemAMO(Addr addr, unsigned int size,
                         Request::Flags flags,
                         AtomicOpFunctor *amo_op) override
    {
        return cpu->initiateMemAMO(addr, size, flags, amo_op);
    }

    /**
     * Sets the number of consecutive store conditional failures.
     */
    void
    setStCondFailures(unsigned int sc_failures) override
    {
        thread->setStCondFailures(sc_failures);
    }

    /**
     * Returns the number of consecutive store conditional failures.
     */
    unsigned int
    readStCondFailures() const override
    {
        return thread->readStCondFailures();
    }

    /**
     * Executes a syscall specified by the callnum.
     */
    void
    syscall(int64_t callnum, Fault *fault) override
    {
        if (FullSystem)
            panic("Syscall emulation isn't available in FS mode.");

        thread->syscall(callnum, fault);
    }

    /** Returns a pointer to the ThreadContext. */
    ThreadContext *tcBase() override { return thread->getTC(); }

    /**
     * Somewhat Alpha-specific function that handles returning from an
     * error or interrupt.
     */
    Fault hwrei() override { return thread->hwrei(); }

    /**
     * Check for special simulator handling of specific PAL calls.  If
     * return value is false, actual PAL call will be suppressed.
     */
    bool
    simPalCheck(int palFunc) override
    {
        return thread->simPalCheck(palFunc);
    }

    bool
    readPredicate() const override
    {
        return thread->readPredicate();
    }

    void
    setPredicate(bool val) override
    {
        thread->setPredicate(val);

        if (cpu->traceData) {
            cpu->traceData->setPredicate(val);
        }
    }

    /**
     * Invalidate a page in the DTLB <i>and</i> ITLB.
     */
    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        thread->demapPage(vaddr, asn);
    }

    void
    armMonitor(Addr address) override
    {
        cpu->armMonitor(thread->threadId(), address);
    }

    bool
    mwait(PacketPtr pkt) override
    {
        return cpu->mwait(thread->threadId(), pkt);
    }

    void
    mwaitAtomic(ThreadContext *tc) override
    {
        cpu->mwaitAtomic(thread->threadId(), tc, thread->dtb);
    }

    AddressMonitor *
    getAddrMonitor() override
    {
        return cpu->getCpuAddrMonitor(thread->threadId());
    }

#if THE_ISA == MIPS_ISA
    RegVal
    readRegOtherThread(const RegId& reg, ThreadID tid=InvalidThreadID)
        override
    {
        panic("Simple CPU models do not support multithreaded "
              "register access.");
    }

    void
    setRegOtherThread(const RegId& reg, RegVal val,
                      ThreadID tid=InvalidThreadID) override
    {
        panic("Simple CPU models do not support multithreaded "
              "register access.");
    }
#endif

};

#endif // __CPU_EXEC_CONTEXT_HH__
