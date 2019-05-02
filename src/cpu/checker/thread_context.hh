/*
 * Copyright (c) 2011-2012, 2016-2018 ARM Limited
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

#ifndef __CPU_CHECKER_THREAD_CONTEXT_HH__
#define __CPU_CHECKER_THREAD_CONTEXT_HH__

#include "arch/types.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/Checker.hh"

class EndQuiesceEvent;
namespace Kernel {
    class Statistics;
};
namespace TheISA {
    class Decoder;
};

/**
 * Derived ThreadContext class for use with the Checker.  The template
 * parameter is the ThreadContext class used by the specific CPU being
 * verified.  This CheckerThreadContext is then used by the main CPU
 * in place of its usual ThreadContext class.  It handles updating the
 * checker's state any time state is updated externally through the
 * ThreadContext.
 */
template <class TC>
class CheckerThreadContext : public ThreadContext
{
  public:
    CheckerThreadContext(TC *actual_tc,
                         CheckerCPU *checker_cpu)
        : actualTC(actual_tc), checkerTC(checker_cpu->thread),
          checkerCPU(checker_cpu)
    { }

  private:
    /** The main CPU's ThreadContext, or class that implements the
     * ThreadContext interface. */
    TC *actualTC;
    /** The checker's own SimpleThread. Will be updated any time
     * anything uses this ThreadContext to externally update a
     * thread's state. */
    SimpleThread *checkerTC;
    /** Pointer to the checker CPU. */
    CheckerCPU *checkerCPU;

  public:

    BaseCPU *getCpuPtr() override { return actualTC->getCpuPtr(); }

    uint32_t socketId() const override { return actualTC->socketId(); }

    int cpuId() const override { return actualTC->cpuId(); }

    ContextID contextId() const override { return actualTC->contextId(); }

    void
    setContextId(ContextID id) override
    {
       actualTC->setContextId(id);
       checkerTC->setContextId(id);
    }

    /** Returns this thread's ID number. */
    int threadId() const override { return actualTC->threadId(); }
    void
    setThreadId(int id) override
    {
        checkerTC->setThreadId(id);
        actualTC->setThreadId(id);
    }

    BaseTLB *getITBPtr() override { return actualTC->getITBPtr(); }

    BaseTLB *getDTBPtr() override { return actualTC->getDTBPtr(); }

    CheckerCPU *
    getCheckerCpuPtr() override
    {
        return checkerCPU;
    }

    TheISA::ISA *getIsaPtr() override { return actualTC->getIsaPtr(); }

    TheISA::Decoder *
    getDecoderPtr() override
    {
        return actualTC->getDecoderPtr();
    }

    System *getSystemPtr() override { return actualTC->getSystemPtr(); }

    ::Kernel::Statistics *
    getKernelStats() override
    {
        return actualTC->getKernelStats();
    }

    Process *getProcessPtr() override { return actualTC->getProcessPtr(); }

    void setProcessPtr(Process *p) override { actualTC->setProcessPtr(p); }

    PortProxy &getPhysProxy() override { return actualTC->getPhysProxy(); }

    PortProxy &
    getVirtProxy() override
    {
        return actualTC->getVirtProxy();
    }

    void
    initMemProxies(ThreadContext *tc) override
    {
        actualTC->initMemProxies(tc);
    }

    void
    connectMemPorts(ThreadContext *tc)
    {
        actualTC->connectMemPorts(tc);
    }

    /** Executes a syscall in SE mode. */
    void
    syscall(int64_t callnum, Fault *fault) override
    {
        return actualTC->syscall(callnum, fault);
    }

    Status status() const override { return actualTC->status(); }

    void
    setStatus(Status new_status) override
    {
        actualTC->setStatus(new_status);
        checkerTC->setStatus(new_status);
    }

    /// Set the status to Active.
    void activate() override { actualTC->activate(); }

    /// Set the status to Suspended.
    void suspend() override { actualTC->suspend(); }

    /// Set the status to Halted.
    void halt() override { actualTC->halt(); }

    void dumpFuncProfile() override { actualTC->dumpFuncProfile(); }

    void
    takeOverFrom(ThreadContext *oldContext) override
    {
        actualTC->takeOverFrom(oldContext);
        checkerTC->copyState(oldContext);
    }

    void
    regStats(const std::string &name) override
    {
        actualTC->regStats(name);
        checkerTC->regStats(name);
    }

    EndQuiesceEvent *
    getQuiesceEvent() override
    {
        return actualTC->getQuiesceEvent();
    }

    Tick readLastActivate() override { return actualTC->readLastActivate(); }
    Tick readLastSuspend() override { return actualTC->readLastSuspend(); }

    void profileClear() override { return actualTC->profileClear(); }
    void profileSample() override { return actualTC->profileSample(); }

    // @todo: Do I need this?
    void
    copyArchRegs(ThreadContext *tc) override
    {
        actualTC->copyArchRegs(tc);
        checkerTC->copyArchRegs(tc);
    }

    void
    clearArchRegs() override
    {
        actualTC->clearArchRegs();
        checkerTC->clearArchRegs();
    }

    //
    // New accessors for new decoder.
    //
    RegVal
    readIntReg(RegIndex reg_idx) const override
    {
        return actualTC->readIntReg(reg_idx);
    }

    RegVal
    readFloatReg(RegIndex reg_idx) const override
    {
        return actualTC->readFloatReg(reg_idx);
    }

    const VecRegContainer &
    readVecReg (const RegId &reg) const override
    {
        return actualTC->readVecReg(reg);
    }

    /**
     * Read vector register for modification, hierarchical indexing.
     */
    VecRegContainer &
    getWritableVecReg (const RegId &reg) override
    {
        return actualTC->getWritableVecReg(reg);
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    ConstVecLane8
    readVec8BitLaneReg(const RegId &reg) const override
    {
        return actualTC->readVec8BitLaneReg(reg);
    }

    /** Reads source vector 16bit operand. */
    ConstVecLane16
    readVec16BitLaneReg(const RegId &reg) const override
    {
        return actualTC->readVec16BitLaneReg(reg);
    }

    /** Reads source vector 32bit operand. */
    ConstVecLane32
    readVec32BitLaneReg(const RegId &reg) const override
    {
        return actualTC->readVec32BitLaneReg(reg);
    }

    /** Reads source vector 64bit operand. */
    ConstVecLane64
    readVec64BitLaneReg(const RegId &reg) const override
    {
        return actualTC->readVec64BitLaneReg(reg);
    }

    /** Write a lane of the destination vector register. */
    virtual void
    setVecLane(const RegId &reg,
               const LaneData<LaneSize::Byte> &val) override
    {
        return actualTC->setVecLane(reg, val);
    }
    virtual void
    setVecLane(const RegId &reg,
               const LaneData<LaneSize::TwoByte> &val) override
    {
        return actualTC->setVecLane(reg, val);
    }
    virtual void
    setVecLane(const RegId &reg,
               const LaneData<LaneSize::FourByte> &val) override
    {
        return actualTC->setVecLane(reg, val);
    }
    virtual void
    setVecLane(const RegId &reg,
               const LaneData<LaneSize::EightByte> &val) override
    {
        return actualTC->setVecLane(reg, val);
    }
    /** @} */

    const VecElem &
    readVecElem(const RegId& reg) const override
    {
        return actualTC->readVecElem(reg);
    }

    const VecPredRegContainer &
    readVecPredReg(const RegId& reg) const override
    {
        return actualTC->readVecPredReg(reg);
    }

    VecPredRegContainer &
    getWritableVecPredReg(const RegId& reg) override
    {
        return actualTC->getWritableVecPredReg(reg);
    }

    RegVal
    readCCReg(RegIndex reg_idx) const override
    {
        return actualTC->readCCReg(reg_idx);
    }

    void
    setIntReg(RegIndex reg_idx, RegVal val) override
    {
        actualTC->setIntReg(reg_idx, val);
        checkerTC->setIntReg(reg_idx, val);
    }

    void
    setFloatReg(RegIndex reg_idx, RegVal val) override
    {
        actualTC->setFloatReg(reg_idx, val);
        checkerTC->setFloatReg(reg_idx, val);
    }

    void
    setVecReg(const RegId& reg, const VecRegContainer& val) override
    {
        actualTC->setVecReg(reg, val);
        checkerTC->setVecReg(reg, val);
    }

    void
    setVecElem(const RegId& reg, const VecElem& val) override
    {
        actualTC->setVecElem(reg, val);
        checkerTC->setVecElem(reg, val);
    }

    void
    setVecPredReg(const RegId& reg, const VecPredRegContainer& val) override
    {
        actualTC->setVecPredReg(reg, val);
        checkerTC->setVecPredReg(reg, val);
    }

    void
    setCCReg(RegIndex reg_idx, RegVal val) override
    {
        actualTC->setCCReg(reg_idx, val);
        checkerTC->setCCReg(reg_idx, val);
    }

    /** Reads this thread's PC state. */
    TheISA::PCState pcState() const override { return actualTC->pcState(); }

    /** Sets this thread's PC state. */
    void
    pcState(const TheISA::PCState &val) override
    {
        DPRINTF(Checker, "Changing PC to %s, old PC %s\n",
                         val, checkerTC->pcState());
        checkerTC->pcState(val);
        checkerCPU->recordPCChange(val);
        return actualTC->pcState(val);
    }

    void
    setNPC(Addr val)
    {
        checkerTC->setNPC(val);
        actualTC->setNPC(val);
    }

    void
    pcStateNoRecord(const TheISA::PCState &val) override
    {
        return actualTC->pcState(val);
    }

    /** Reads this thread's PC. */
    Addr instAddr() const override { return actualTC->instAddr(); }

    /** Reads this thread's next PC. */
    Addr nextInstAddr() const override { return actualTC->nextInstAddr(); }

    /** Reads this thread's next PC. */
    MicroPC microPC() const override { return actualTC->microPC(); }

    RegVal
    readMiscRegNoEffect(RegIndex misc_reg) const override
    {
        return actualTC->readMiscRegNoEffect(misc_reg);
    }

    RegVal
    readMiscReg(RegIndex misc_reg) override
    {
        return actualTC->readMiscReg(misc_reg);
    }

    void
    setMiscRegNoEffect(RegIndex misc_reg, RegVal val) override
    {
        DPRINTF(Checker, "Setting misc reg with no effect: %d to both Checker"
                         " and O3..\n", misc_reg);
        checkerTC->setMiscRegNoEffect(misc_reg, val);
        actualTC->setMiscRegNoEffect(misc_reg, val);
    }

    void
    setMiscReg(RegIndex misc_reg, RegVal val) override
    {
        DPRINTF(Checker, "Setting misc reg with effect: %d to both Checker"
                         " and O3..\n", misc_reg);
        checkerTC->setMiscReg(misc_reg, val);
        actualTC->setMiscReg(misc_reg, val);
    }

    RegId
    flattenRegId(const RegId& regId) const override
    {
        return actualTC->flattenRegId(regId);
    }

    unsigned
    readStCondFailures() const override
    {
        return actualTC->readStCondFailures();
    }

    void
    setStCondFailures(unsigned sc_failures) override
    {
        actualTC->setStCondFailures(sc_failures);
    }

    Counter
    readFuncExeInst() const override
    {
        return actualTC->readFuncExeInst();
    }

    RegVal
    readIntRegFlat(RegIndex idx) const override
    {
        return actualTC->readIntRegFlat(idx);
    }

    void
    setIntRegFlat(RegIndex idx, RegVal val) override
    {
        actualTC->setIntRegFlat(idx, val);
    }

    RegVal
    readFloatRegFlat(RegIndex idx) const override
    {
        return actualTC->readFloatRegFlat(idx);
    }

    void
    setFloatRegFlat(RegIndex idx, RegVal val) override
    {
        actualTC->setFloatRegFlat(idx, val);
    }

    const VecRegContainer &
    readVecRegFlat(RegIndex idx) const override
    {
        return actualTC->readVecRegFlat(idx);
    }

    /**
     * Read vector register for modification, flat indexing.
     */
    VecRegContainer &
    getWritableVecRegFlat(RegIndex idx) override
    {
        return actualTC->getWritableVecRegFlat(idx);
    }

    void
    setVecRegFlat(RegIndex idx, const VecRegContainer& val) override
    {
        actualTC->setVecRegFlat(idx, val);
    }

    const VecElem &
    readVecElemFlat(RegIndex idx, const ElemIndex& elem_idx) const override
    {
        return actualTC->readVecElemFlat(idx, elem_idx);
    }

    void
    setVecElemFlat(RegIndex idx,
                   const ElemIndex& elem_idx, const VecElem& val) override
    {
        actualTC->setVecElemFlat(idx, elem_idx, val);
    }

    const VecPredRegContainer &
    readVecPredRegFlat(RegIndex idx) const override
    {
        return actualTC->readVecPredRegFlat(idx);
    }

    VecPredRegContainer &
    getWritableVecPredRegFlat(RegIndex idx) override
    {
        return actualTC->getWritableVecPredRegFlat(idx);
    }

    void
    setVecPredRegFlat(RegIndex idx, const VecPredRegContainer& val) override
    {
        actualTC->setVecPredRegFlat(idx, val);
    }

    RegVal
    readCCRegFlat(RegIndex idx) const override
    {
        return actualTC->readCCRegFlat(idx);
    }

    void
    setCCRegFlat(RegIndex idx, RegVal val) override
    {
        actualTC->setCCRegFlat(idx, val);
    }
};

#endif // __CPU_CHECKER_EXEC_CONTEXT_HH__
