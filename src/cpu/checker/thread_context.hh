/*
 * Copyright (c) 2011-2012, 2016-2018, 2020-2021 Arm Limited
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
 */

#ifndef __CPU_CHECKER_THREAD_CONTEXT_HH__
#define __CPU_CHECKER_THREAD_CONTEXT_HH__

#include "arch/generic/pcstate.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/Checker.hh"

namespace gem5
{

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
    CheckerThreadContext(TC *actual_tc, CheckerCPU *checker_cpu)
        : actualTC(actual_tc),
          checkerTC(checker_cpu->thread),
          checkerCPU(checker_cpu)
    {}

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
    bool
    schedule(PCEvent *e) override
    {
        [[maybe_unused]] bool check_ret = checkerTC->schedule(e);
        bool actual_ret = actualTC->schedule(e);
        assert(actual_ret == check_ret);
        return actual_ret;
    }

    bool
    remove(PCEvent *e) override
    {
        [[maybe_unused]] bool check_ret = checkerTC->remove(e);
        bool actual_ret = actualTC->remove(e);
        assert(actual_ret == check_ret);
        return actual_ret;
    }

    void
    scheduleInstCountEvent(Event *event, Tick count) override
    {
        actualTC->scheduleInstCountEvent(event, count);
    }

    void
    descheduleInstCountEvent(Event *event) override
    {
        actualTC->descheduleInstCountEvent(event);
    }

    Tick
    getCurrentInstCount() override
    {
        return actualTC->getCurrentInstCount();
    }

    BaseCPU *
    getCpuPtr() override
    {
        return actualTC->getCpuPtr();
    }

    uint32_t
    socketId() const override
    {
        return actualTC->socketId();
    }

    int
    cpuId() const override
    {
        return actualTC->cpuId();
    }

    ContextID
    contextId() const override
    {
        return actualTC->contextId();
    }

    void
    setContextId(ContextID id) override
    {
        actualTC->setContextId(id);
        checkerTC->setContextId(id);
    }

    /** Returns this thread's ID number. */
    int
    threadId() const override
    {
        return actualTC->threadId();
    }

    void
    setThreadId(int id) override
    {
        checkerTC->setThreadId(id);
        actualTC->setThreadId(id);
    }

    BaseMMU *
    getMMUPtr() override
    {
        return actualTC->getMMUPtr();
    }

    CheckerCPU *
    getCheckerCpuPtr() override
    {
        return checkerCPU;
    }

    BaseISA *
    getIsaPtr() const override
    {
        return actualTC->getIsaPtr();
    }

    InstDecoder *
    getDecoderPtr() override
    {
        return actualTC->getDecoderPtr();
    }

    System *
    getSystemPtr() override
    {
        return actualTC->getSystemPtr();
    }

    Process *
    getProcessPtr() override
    {
        return actualTC->getProcessPtr();
    }

    void
    setProcessPtr(Process *p) override
    {
        actualTC->setProcessPtr(p);
    }

    void
    connectMemPorts(ThreadContext *tc)
    {
        actualTC->connectMemPorts(tc);
    }

    Status
    status() const override
    {
        return actualTC->status();
    }

    void
    setStatus(Status new_status) override
    {
        actualTC->setStatus(new_status);
        checkerTC->setStatus(new_status);
    }

    /// Set the status to Active.
    void
    activate() override
    {
        actualTC->activate();
    }

    /// Set the status to Suspended.
    void
    suspend() override
    {
        actualTC->suspend();
    }

    /// Set the status to Halted.
    void
    halt() override
    {
        actualTC->halt();
    }

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

    Tick
    readLastActivate() override
    {
        return actualTC->readLastActivate();
    }

    Tick
    readLastSuspend() override
    {
        return actualTC->readLastSuspend();
    }

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
    getReg(const RegId &reg) const override
    {
        return actualTC->getReg(reg);
    }

    void
    getReg(const RegId &reg, void *val) const override
    {
        actualTC->getReg(reg, val);
    }

    void *
    getWritableReg(const RegId &reg) override
    {
        return actualTC->getWritableReg(reg);
    }

    void
    setReg(const RegId &reg, RegVal val) override
    {
        actualTC->setReg(reg, val);
        checkerTC->setReg(reg, val);
    }

    void
    setReg(const RegId &reg, const void *val) override
    {
        actualTC->setReg(reg, val);
        checkerTC->setReg(reg, val);
    }

    /** Reads this thread's PC state. */
    const PCStateBase &
    pcState() const override
    {
        return actualTC->pcState();
    }

    /** Sets this thread's PC state. */
    void
    pcState(const PCStateBase &val) override
    {
        DPRINTF(Checker, "Changing PC to %s, old PC %s\n", val,
                checkerTC->pcState());
        checkerTC->pcState(val);
        checkerCPU->recordPCChange(val);
        return actualTC->pcState(val);
    }

    void
    pcStateNoRecord(const PCStateBase &val) override
    {
        return actualTC->pcState(val);
    }

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
        DPRINTF(Checker,
                "Setting misc reg with no effect: %d to both Checker"
                " and O3..\n",
                misc_reg);
        checkerTC->setMiscRegNoEffect(misc_reg, val);
        actualTC->setMiscRegNoEffect(misc_reg, val);
    }

    void
    setMiscReg(RegIndex misc_reg, RegVal val) override
    {
        DPRINTF(Checker,
                "Setting misc reg with effect: %d to both Checker"
                " and O3..\n",
                misc_reg);
        checkerTC->setMiscReg(misc_reg, val);
        actualTC->setMiscReg(misc_reg, val);
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

    // hardware transactional memory
    void
    htmAbortTransaction(uint64_t htm_uid, HtmFailureFaultCause cause) override
    {
        panic("function not implemented");
    }

    BaseHTMCheckpointPtr &
    getHtmCheckpointPtr() override
    {
        return actualTC->getHtmCheckpointPtr();
    }

    void
    setHtmCheckpointPtr(BaseHTMCheckpointPtr new_cpt) override
    {
        panic("function not implemented");
    }
};

} // namespace gem5

#endif // __CPU_CHECKER_EXEC_CONTEXT_HH__
