/*
 * Copyright (c) 2011-2012, 2016 ARM Limited
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
namespace TheISA {
    namespace Kernel {
        class Statistics;
    };
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

    BaseCPU *getCpuPtr() { return actualTC->getCpuPtr(); }

    uint32_t socketId() const { return actualTC->socketId(); }

    int cpuId() const { return actualTC->cpuId(); }

    ContextID contextId() const { return actualTC->contextId(); }

    void setContextId(ContextID id)
    {
       actualTC->setContextId(id);
       checkerTC->setContextId(id);
    }

    /** Returns this thread's ID number. */
    int threadId() const { return actualTC->threadId(); }
    void setThreadId(int id)
    {
        checkerTC->setThreadId(id);
        actualTC->setThreadId(id);
    }

    TheISA::TLB *getITBPtr() { return actualTC->getITBPtr(); }

    TheISA::TLB *getDTBPtr() { return actualTC->getDTBPtr(); }

    CheckerCPU *getCheckerCpuPtr()
    {
        return checkerCPU;
    }

    TheISA::Decoder *getDecoderPtr() { return actualTC->getDecoderPtr(); }

    System *getSystemPtr() { return actualTC->getSystemPtr(); }

    TheISA::Kernel::Statistics *getKernelStats()
    { return actualTC->getKernelStats(); }

    Process *getProcessPtr() { return actualTC->getProcessPtr(); }

    void setProcessPtr(Process *p) { actualTC->setProcessPtr(p); }

    PortProxy &getPhysProxy() { return actualTC->getPhysProxy(); }

    FSTranslatingPortProxy &getVirtProxy()
    { return actualTC->getVirtProxy(); }

    void initMemProxies(ThreadContext *tc)
    { actualTC->initMemProxies(tc); }

    void connectMemPorts(ThreadContext *tc)
    {
        actualTC->connectMemPorts(tc);
    }

    SETranslatingPortProxy &getMemProxy() { return actualTC->getMemProxy(); }

    /** Executes a syscall in SE mode. */
    void syscall(int64_t callnum, Fault *fault)
    { return actualTC->syscall(callnum, fault); }

    Status status() const { return actualTC->status(); }

    void setStatus(Status new_status)
    {
        actualTC->setStatus(new_status);
        checkerTC->setStatus(new_status);
    }

    /// Set the status to Active.
    void activate() { actualTC->activate(); }

    /// Set the status to Suspended.
    void suspend() { actualTC->suspend(); }

    /// Set the status to Halted.
    void halt() { actualTC->halt(); }

    void dumpFuncProfile() { actualTC->dumpFuncProfile(); }

    void takeOverFrom(ThreadContext *oldContext)
    {
        actualTC->takeOverFrom(oldContext);
        checkerTC->copyState(oldContext);
    }

    void regStats(const std::string &name)
    {
        actualTC->regStats(name);
        checkerTC->regStats(name);
    }

    EndQuiesceEvent *getQuiesceEvent() { return actualTC->getQuiesceEvent(); }

    Tick readLastActivate() { return actualTC->readLastActivate(); }
    Tick readLastSuspend() { return actualTC->readLastSuspend(); }

    void profileClear() { return actualTC->profileClear(); }
    void profileSample() { return actualTC->profileSample(); }

    // @todo: Do I need this?
    void copyArchRegs(ThreadContext *tc)
    {
        actualTC->copyArchRegs(tc);
        checkerTC->copyArchRegs(tc);
    }

    void clearArchRegs()
    {
        actualTC->clearArchRegs();
        checkerTC->clearArchRegs();
    }

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx)
    { return actualTC->readIntReg(reg_idx); }

    FloatReg readFloatReg(int reg_idx)
    { return actualTC->readFloatReg(reg_idx); }

    FloatRegBits readFloatRegBits(int reg_idx)
    { return actualTC->readFloatRegBits(reg_idx); }

    const VecRegContainer& readVecReg(const RegId& reg) const
    { return actualTC->readVecReg(reg); }

    /**
     * Read vector register for modification, hierarchical indexing.
     */
    VecRegContainer& getWritableVecReg(const RegId& reg)
    { return actualTC->getWritableVecReg(reg); }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    ConstVecLane8
    readVec8BitLaneReg(const RegId& reg) const
    { return actualTC->readVec8BitLaneReg(reg); }

    /** Reads source vector 16bit operand. */
    ConstVecLane16
    readVec16BitLaneReg(const RegId& reg) const
    { return actualTC->readVec16BitLaneReg(reg); }

    /** Reads source vector 32bit operand. */
    ConstVecLane32
    readVec32BitLaneReg(const RegId& reg) const
    { return actualTC->readVec32BitLaneReg(reg); }

    /** Reads source vector 64bit operand. */
    ConstVecLane64
    readVec64BitLaneReg(const RegId& reg) const
    { return actualTC->readVec64BitLaneReg(reg); }

    /** Write a lane of the destination vector register. */
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::Byte>& val)
    { return actualTC->setVecLane(reg, val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::TwoByte>& val)
    { return actualTC->setVecLane(reg, val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::FourByte>& val)
    { return actualTC->setVecLane(reg, val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::EightByte>& val)
    { return actualTC->setVecLane(reg, val); }
    /** @} */

    const VecElem& readVecElem(const RegId& reg) const
    { return actualTC->readVecElem(reg); }

    CCReg readCCReg(int reg_idx)
    { return actualTC->readCCReg(reg_idx); }

    void setIntReg(int reg_idx, uint64_t val)
    {
        actualTC->setIntReg(reg_idx, val);
        checkerTC->setIntReg(reg_idx, val);
    }

    void setFloatReg(int reg_idx, FloatReg val)
    {
        actualTC->setFloatReg(reg_idx, val);
        checkerTC->setFloatReg(reg_idx, val);
    }

    void setFloatRegBits(int reg_idx, FloatRegBits val)
    {
        actualTC->setFloatRegBits(reg_idx, val);
        checkerTC->setFloatRegBits(reg_idx, val);
    }

    void setVecReg(const RegId& reg, const VecRegContainer& val)
    {
        actualTC->setVecReg(reg, val);
        checkerTC->setVecReg(reg, val);
    }

    void setVecElem(const RegId& reg, const VecElem& val)
    {
        actualTC->setVecElem(reg, val);
        checkerTC->setVecElem(reg, val);
    }

    void setCCReg(int reg_idx, CCReg val)
    {
        actualTC->setCCReg(reg_idx, val);
        checkerTC->setCCReg(reg_idx, val);
    }

    /** Reads this thread's PC state. */
    TheISA::PCState pcState()
    { return actualTC->pcState(); }

    /** Sets this thread's PC state. */
    void pcState(const TheISA::PCState &val)
    {
        DPRINTF(Checker, "Changing PC to %s, old PC %s\n",
                         val, checkerTC->pcState());
        checkerTC->pcState(val);
        checkerCPU->recordPCChange(val);
        return actualTC->pcState(val);
    }

    void setNPC(Addr val)
    {
        checkerTC->setNPC(val);
        actualTC->setNPC(val);
    }

    void pcStateNoRecord(const TheISA::PCState &val)
    {
        return actualTC->pcState(val);
    }

    /** Reads this thread's PC. */
    Addr instAddr()
    { return actualTC->instAddr(); }

    /** Reads this thread's next PC. */
    Addr nextInstAddr()
    { return actualTC->nextInstAddr(); }

    /** Reads this thread's next PC. */
    MicroPC microPC()
    { return actualTC->microPC(); }

    MiscReg readMiscRegNoEffect(int misc_reg) const
    { return actualTC->readMiscRegNoEffect(misc_reg); }

    MiscReg readMiscReg(int misc_reg)
    { return actualTC->readMiscReg(misc_reg); }

    void setMiscRegNoEffect(int misc_reg, const MiscReg &val)
    {
        DPRINTF(Checker, "Setting misc reg with no effect: %d to both Checker"
                         " and O3..\n", misc_reg);
        checkerTC->setMiscRegNoEffect(misc_reg, val);
        actualTC->setMiscRegNoEffect(misc_reg, val);
    }

    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        DPRINTF(Checker, "Setting misc reg with effect: %d to both Checker"
                         " and O3..\n", misc_reg);
        checkerTC->setMiscReg(misc_reg, val);
        actualTC->setMiscReg(misc_reg, val);
    }

    RegId flattenRegId(const RegId& regId) const {
        return actualTC->flattenRegId(regId);
    }

    unsigned readStCondFailures()
    { return actualTC->readStCondFailures(); }

    void setStCondFailures(unsigned sc_failures)
    {
        actualTC->setStCondFailures(sc_failures);
    }

    Counter readFuncExeInst() { return actualTC->readFuncExeInst(); }

    uint64_t readIntRegFlat(int idx)
    { return actualTC->readIntRegFlat(idx); }

    void setIntRegFlat(int idx, uint64_t val)
    { actualTC->setIntRegFlat(idx, val); }

    FloatReg readFloatRegFlat(int idx)
    { return actualTC->readFloatRegFlat(idx); }

    void setFloatRegFlat(int idx, FloatReg val)
    { actualTC->setFloatRegFlat(idx, val); }

    FloatRegBits readFloatRegBitsFlat(int idx)
    { return actualTC->readFloatRegBitsFlat(idx); }

    void setFloatRegBitsFlat(int idx, FloatRegBits val)
    { actualTC->setFloatRegBitsFlat(idx, val); }

    const VecRegContainer& readVecRegFlat(int idx) const
    { return actualTC->readVecRegFlat(idx); }

    /**
     * Read vector register for modification, flat indexing.
     */
    VecRegContainer& getWritableVecRegFlat(int idx)
    { return actualTC->getWritableVecRegFlat(idx); }

    void setVecRegFlat(int idx, const VecRegContainer& val)
    { actualTC->setVecRegFlat(idx, val); }

    const VecElem& readVecElemFlat(const RegIndex& idx,
                                   const ElemIndex& elem_idx) const
    { return actualTC->readVecElemFlat(idx, elem_idx); }

    void setVecElemFlat(const RegIndex& idx,
                        const ElemIndex& elem_idx, const VecElem& val)
    { actualTC->setVecElemFlat(idx, elem_idx, val); }

    CCReg readCCRegFlat(int idx)
    { return actualTC->readCCRegFlat(idx); }

    void setCCRegFlat(int idx, CCReg val)
    { actualTC->setCCRegFlat(idx, val); }
};

#endif // __CPU_CHECKER_EXEC_CONTEXT_HH__
