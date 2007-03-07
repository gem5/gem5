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

#ifndef __CPU_CHECKER_THREAD_CONTEXT_HH__
#define __CPU_CHECKER_THREAD_CONTEXT_HH__

#include "arch/types.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"

class EndQuiesceEvent;
namespace TheISA {
    namespace Kernel {
        class Statistics;
    };
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

    void setCpuId(int id)
    {
        actualTC->setCpuId(id);
        checkerTC->setCpuId(id);
    }

    int readCpuId() { return actualTC->readCpuId(); }

#if FULL_SYSTEM
    System *getSystemPtr() { return actualTC->getSystemPtr(); }

    PhysicalMemory *getPhysMemPtr() { return actualTC->getPhysMemPtr(); }

    TheISA::ITB *getITBPtr() { return actualTC->getITBPtr(); }

    TheISA::DTB *getDTBPtr() { return actualTC->getDTBPtr(); }

    TheISA::Kernel::Statistics *getKernelStats()
    { return actualTC->getKernelStats(); }

    FunctionalPort *getPhysPort() { return actualTC->getPhysPort(); }

    VirtualPort *getVirtPort(ThreadContext *tc = NULL)
    { return actualTC->getVirtPort(); }

    void delVirtPort(VirtualPort *vp) { actualTC->delVirtPort(vp); }
#else
    TranslatingPort *getMemPort() { return actualTC->getMemPort(); }

    Process *getProcessPtr() { return actualTC->getProcessPtr(); }
#endif

    Status status() const { return actualTC->status(); }

    void setStatus(Status new_status)
    {
        actualTC->setStatus(new_status);
        checkerTC->setStatus(new_status);
    }

    /// Set the status to Active.  Optional delay indicates number of
    /// cycles to wait before beginning execution.
    void activate(int delay = 1) { actualTC->activate(delay); }

    /// Set the status to Suspended.
    void suspend() { actualTC->suspend(); }

    /// Set the status to Unallocated.
    void deallocate(int delay = 0) { actualTC->deallocate(delay); }

    /// Set the status to Halted.
    void halt() { actualTC->halt(); }

#if FULL_SYSTEM
    void dumpFuncProfile() { actualTC->dumpFuncProfile(); }
#endif

    void takeOverFrom(ThreadContext *oldContext)
    {
        actualTC->takeOverFrom(oldContext);
        checkerTC->copyState(oldContext);
    }

    void regStats(const std::string &name) { actualTC->regStats(name); }

    void serialize(std::ostream &os) { actualTC->serialize(os); }
    void unserialize(Checkpoint *cp, const std::string &section)
    { actualTC->unserialize(cp, section); }

#if FULL_SYSTEM
    EndQuiesceEvent *getQuiesceEvent() { return actualTC->getQuiesceEvent(); }

    Tick readLastActivate() { return actualTC->readLastActivate(); }
    Tick readLastSuspend() { return actualTC->readLastSuspend(); }

    void profileClear() { return actualTC->profileClear(); }
    void profileSample() { return actualTC->profileSample(); }
#endif

    int getThreadNum() { return actualTC->getThreadNum(); }

    // @todo: Do I need this?
    MachInst getInst() { return actualTC->getInst(); }

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

    FloatReg readFloatReg(int reg_idx, int width)
    { return actualTC->readFloatReg(reg_idx, width); }

    FloatReg readFloatReg(int reg_idx)
    { return actualTC->readFloatReg(reg_idx); }

    FloatRegBits readFloatRegBits(int reg_idx, int width)
    { return actualTC->readFloatRegBits(reg_idx, width); }

    FloatRegBits readFloatRegBits(int reg_idx)
    { return actualTC->readFloatRegBits(reg_idx); }

    void setIntReg(int reg_idx, uint64_t val)
    {
        actualTC->setIntReg(reg_idx, val);
        checkerTC->setIntReg(reg_idx, val);
    }

    void setFloatReg(int reg_idx, FloatReg val, int width)
    {
        actualTC->setFloatReg(reg_idx, val, width);
        checkerTC->setFloatReg(reg_idx, val, width);
    }

    void setFloatReg(int reg_idx, FloatReg val)
    {
        actualTC->setFloatReg(reg_idx, val);
        checkerTC->setFloatReg(reg_idx, val);
    }

    void setFloatRegBits(int reg_idx, FloatRegBits val, int width)
    {
        actualTC->setFloatRegBits(reg_idx, val, width);
        checkerTC->setFloatRegBits(reg_idx, val, width);
    }

    void setFloatRegBits(int reg_idx, FloatRegBits val)
    {
        actualTC->setFloatRegBits(reg_idx, val);
        checkerTC->setFloatRegBits(reg_idx, val);
    }

    uint64_t readPC() { return actualTC->readPC(); }

    void setPC(uint64_t val)
    {
        actualTC->setPC(val);
        checkerTC->setPC(val);
        checkerCPU->recordPCChange(val);
    }

    uint64_t readNextPC() { return actualTC->readNextPC(); }

    void setNextPC(uint64_t val)
    {
        actualTC->setNextPC(val);
        checkerTC->setNextPC(val);
        checkerCPU->recordNextPCChange(val);
    }

    uint64_t readNextNPC() { return actualTC->readNextNPC(); }

    void setNextNPC(uint64_t val)
    {
        actualTC->setNextNPC(val);
        checkerTC->setNextNPC(val);
        checkerCPU->recordNextPCChange(val);
    }

    MiscReg readMiscRegNoEffect(int misc_reg)
    { return actualTC->readMiscRegNoEffect(misc_reg); }

    MiscReg readMiscReg(int misc_reg)
    { return actualTC->readMiscReg(misc_reg); }

    void setMiscRegNoEffect(int misc_reg, const MiscReg &val)
    {
        checkerTC->setMiscRegNoEffect(misc_reg, val);
        actualTC->setMiscRegNoEffect(misc_reg, val);
    }

    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        checkerTC->setMiscReg(misc_reg, val);
        actualTC->setMiscReg(misc_reg, val);
    }

    unsigned readStCondFailures()
    { return actualTC->readStCondFailures(); }

    void setStCondFailures(unsigned sc_failures)
    {
        checkerTC->setStCondFailures(sc_failures);
        actualTC->setStCondFailures(sc_failures);
    }

    // @todo: Fix this!
    bool misspeculating() { return actualTC->misspeculating(); }

#if !FULL_SYSTEM
    IntReg getSyscallArg(int i) { return actualTC->getSyscallArg(i); }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, IntReg val)
    {
        checkerTC->setSyscallArg(i, val);
        actualTC->setSyscallArg(i, val);
    }

    void setSyscallReturn(SyscallReturn return_value)
    {
        checkerTC->setSyscallReturn(return_value);
        actualTC->setSyscallReturn(return_value);
    }

    Counter readFuncExeInst() { return actualTC->readFuncExeInst(); }
#endif
    void changeRegFileContext(TheISA::RegContextParam param,
            TheISA::RegContextVal val)
    {
        actualTC->changeRegFileContext(param, val);
        checkerTC->changeRegFileContext(param, val);
    }
};

#endif // __CPU_CHECKER_EXEC_CONTEXT_HH__
