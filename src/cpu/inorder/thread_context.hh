/*
 * Copyright (c) 2012 ARM Limited
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
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 *
 */

#ifndef __CPU_INORDER_THREAD_CONTEXT_HH__
#define __CPU_INORDER_THREAD_CONTEXT_HH__

#include "config/the_isa.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/thread_state.hh"
#include "cpu/exetrace.hh"
#include "cpu/thread_context.hh"
#include "arch/kernel_stats.hh"

class EndQuiesceEvent;
class CheckerCPU;
namespace Kernel {
    class Statistics;
};

/**
 * Derived ThreadContext class for use with the InOrderCPU.  It
 * provides the interface for any external objects to access a
 * single thread's state and some general CPU state.  Any time
 * external objects try to update state through this interface,
 * the CPU will create an event to squash all in-flight
 * instructions in order to ensure state is maintained correctly.
 * It must be defined specifically for the InOrderCPU because
 * not all architectural state is located within the O3ThreadState
 * (such as the commit PC, and registers), and specific actions
 * must be taken when using this interface (such as squashing all
 * in-flight instructions when doing a write to this interface).
 */
class InOrderThreadContext : public ThreadContext
{
  public:
    InOrderThreadContext() { }

    /** Pointer to the CPU. */
    InOrderCPU *cpu;

    /** Pointer to the thread state that this TC corrseponds to. */
    InOrderThreadState *thread;

    /** Returns a pointer to the ITB. */
    /** @TODO: PERF: Should we bind this to a pointer in constructor? */
    TheISA::TLB *getITBPtr() { return cpu->getITBPtr(); }

    /** Returns a pointer to the DTB. */
    /** @TODO: PERF: Should we bind this to a pointer in constructor? */
    TheISA::TLB *getDTBPtr() { return cpu->getDTBPtr(); }

    /** Currently InOrder model does not support CheckerCPU, this is
     *  merely here for supporting compilation of gem5 with the Checker
     *  as a runtime option
     */
    CheckerCPU *getCheckerCpuPtr() { return NULL; }

    TheISA::Decoder *
    getDecoderPtr()
    {
        return cpu->getDecoderPtr(thread->contextId());
    }

    System *getSystemPtr() { return cpu->system; }

    /** Returns a pointer to this CPU. */
    BaseCPU *getCpuPtr() { return cpu; }

    /** Returns a pointer to this CPU. */
    std::string getCpuName() { return cpu->name(); }

    /** Reads this CPU's ID. */
    int cpuId() { return cpu->cpuId(); }

    int contextId() { return thread->contextId(); }

    void setContextId(int id) { thread->setContextId(id); }

    /** Returns this thread's ID number. */
    int threadId() { return thread->threadId(); }
    void setThreadId(int id) { return thread->setThreadId(id); }

    uint64_t readMicroPC()
    { return 0; }

    void setMicroPC(uint64_t val) { };

    uint64_t readNextMicroPC()
    { return 0; }

    void setNextMicroPC(uint64_t val) { };

    /** Returns a pointer to this thread's kernel statistics. */
    TheISA::Kernel::Statistics *getKernelStats()
    { return thread->kernelStats; }

    PortProxy &getPhysProxy() { return thread->getPhysProxy(); }

    FSTranslatingPortProxy &getVirtProxy();

    void initMemProxies(ThreadContext *tc)
    { thread->initMemProxies(tc); }

    /** Dumps the function profiling information.
     * @todo: Implement.
     */
    void dumpFuncProfile();

    /** Reads the last tick that this thread was activated on. */
    Tick readLastActivate();
    /** Reads the last tick that this thread was suspended on. */
    Tick readLastSuspend();

    /** Clears the function profiling information. */
    void profileClear();

    /** Samples the function profiling information. */
    void profileSample();

    /** Returns pointer to the quiesce event. */
    EndQuiesceEvent *getQuiesceEvent()
    {
        return this->thread->quiesceEvent;
    }

    SETranslatingPortProxy &getMemProxy() { return thread->getMemProxy(); }

    /** Returns a pointer to this thread's process. */
    Process *getProcessPtr() { return thread->getProcessPtr(); }

    /** Returns this thread's status. */
    Status status() const { return thread->status(); }

    /** Sets this thread's status. */
    void setStatus(Status new_status)
    { thread->setStatus(new_status); }

    /** Set the status to Active.  Optional delay indicates number of
     * cycles to wait before beginning execution. */
    void activate(Cycles delay = Cycles(1));

    /** Set the status to Suspended. */
    void suspend(Cycles delay = Cycles(0));

    /** Set the status to Halted. */
    void halt(Cycles delay = Cycles(0));

    /** Takes over execution of a thread from another CPU. */
    void takeOverFrom(ThreadContext *old_context);

    /** Registers statistics associated with this TC. */
    void regStats(const std::string &name);

    /** Returns this thread's ID number. */
    int getThreadNum() { return thread->threadId(); }

    /** Copies the architectural registers from another TC into this TC. */
    void copyArchRegs(ThreadContext *src_tc);

    /** Resets all architectural registers to 0. */
    void clearArchRegs();

    /** Reads an integer register. */
    uint64_t readIntReg(int reg_idx);

    FloatReg readFloatReg(int reg_idx);

    FloatRegBits readFloatRegBits(int reg_idx);

    CCReg readCCReg(int reg_idx);

    uint64_t readRegOtherThread(int misc_reg, ThreadID tid);

    /** Sets an integer register to a value. */
    void setIntReg(int reg_idx, uint64_t val);

    void setFloatReg(int reg_idx, FloatReg val);

    void setFloatRegBits(int reg_idx, FloatRegBits val);

    void setCCReg(int reg_idx, CCReg val);

    void setRegOtherThread(int misc_reg,
                                   const MiscReg &val,
                                   ThreadID tid);

    /** Reads this thread's PC. */
    TheISA::PCState pcState()
    { return cpu->pcState(thread->threadId()); }

    /** Sets this thread's PC. */
    void pcState(const TheISA::PCState &val)
    { cpu->pcState(val, thread->threadId()); }

    /** Needs to be implemented for future CheckerCPU support.
     *  See O3CPU for examples on how to integrate Checker.
     */
    void pcStateNoRecord(const TheISA::PCState &val)
    {}

    Addr instAddr()
    { return cpu->instAddr(thread->threadId()); }

    Addr nextInstAddr()
    { return cpu->nextInstAddr(thread->threadId()); }

    MicroPC microPC()
    { return cpu->microPC(thread->threadId()); }

    /** Reads a miscellaneous register. */
    MiscReg readMiscRegNoEffect(int misc_reg)
    { return cpu->readMiscRegNoEffect(misc_reg, thread->threadId()); }

    /** Reads a misc. register, including any side-effects the
     * read might have as defined by the architecture. */
    MiscReg readMiscReg(int misc_reg)
    { return cpu->readMiscReg(misc_reg, thread->threadId()); }

    /** Sets a misc. register. */
    void setMiscRegNoEffect(int misc_reg, const MiscReg &val);

    /** Sets a misc. register, including any side-effects the
     * write might have as defined by the architecture. */
    void setMiscReg(int misc_reg, const MiscReg &val);

    int flattenIntIndex(int reg)
    { return cpu->isa[thread->threadId()]->flattenIntIndex(reg); }

    int flattenFloatIndex(int reg)
    { return cpu->isa[thread->threadId()]->flattenFloatIndex(reg); }

    int flattenCCIndex(int reg)
    { return cpu->isa[thread->threadId()]->flattenCCIndex(reg); }

    int flattenMiscIndex(int reg)
    { return cpu->isa[thread->threadId()]->flattenMiscIndex(reg); }

    void activateContext(Cycles delay)
    { cpu->activateContext(thread->threadId(), delay); }

    void deallocateContext()
    { cpu->deallocateContext(thread->threadId()); }

    /** Returns the number of consecutive store conditional failures. */
    // @todo: Figure out where these store cond failures should go.
    unsigned readStCondFailures()
    { return thread->storeCondFailures; }

    /** Sets the number of consecutive store conditional failures. */
    void setStCondFailures(unsigned sc_failures)
    { thread->storeCondFailures = sc_failures; }

    // Only really makes sense for old CPU model.  Lots of code
    // outside the CPU still checks this function, so it will
    // always return false to keep everything working.
    /** Checks if the thread is misspeculating.  Because it is
     * very difficult to determine if the thread is
     * misspeculating, this is set as false. */
    bool misspeculating() { return false; }

    /** Executes a syscall in SE mode. */
    void syscall(int64_t callnum)
    { return cpu->syscall(callnum, thread->threadId()); }

    /** Reads the funcExeInst counter. */
    Counter readFuncExeInst() { return thread->funcExeInst; }

    void changeRegFileContext(unsigned param,
                                      unsigned val)
    { panic("Not supported!"); }

    uint64_t readIntRegFlat(int idx);
    void setIntRegFlat(int idx, uint64_t val);

    FloatReg readFloatRegFlat(int idx);
    void setFloatRegFlat(int idx, FloatReg val);

    FloatRegBits readFloatRegBitsFlat(int idx);
    void setFloatRegBitsFlat(int idx, FloatRegBits val);

    CCReg readCCRegFlat(int idx);
    void setCCRegFlat(int idx, CCReg val);
};

#endif
