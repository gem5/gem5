/*
 * Copyright (c) 2011-2012, 2016-2018, 2020 ARM Limited
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
 * Copyright (c) 2001-2006 The Regents of The University of Michigan
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

#ifndef __CPU_SIMPLE_THREAD_HH__
#define __CPU_SIMPLE_THREAD_HH__

#include <algorithm>
#include <vector>

#include "arch/generic/htm.hh"
#include "arch/generic/mmu.hh"
#include "arch/generic/pcstate.hh"
#include "arch/generic/tlb.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/regfile.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "debug/CCRegs.hh"
#include "debug/FloatRegs.hh"
#include "debug/IntRegs.hh"
#include "debug/MatRegs.hh"
#include "debug/VecPredRegs.hh"
#include "debug/VecRegs.hh"
#include "mem/htm.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

namespace gem5
{

class BaseCPU;
class CheckerCPU;

/**
 * The SimpleThread object provides a combination of the ThreadState
 * object and the ThreadContext interface. It implements the
 * ThreadContext interface and adds to the ThreadState object by adding all
 * the objects needed for simple functional execution, including a
 * simple architectural register file, and pointers to the ITB and DTB
 * in full system mode. For CPU models that do not need more advanced
 * ways to hold state (i.e. a separate physical register file, or
 * separate fetch and commit PC's), this SimpleThread class provides
 * all the necessary state for full architecture-level functional
 * simulation.  See the AtomicSimpleCPU or TimingSimpleCPU for
 * examples.
 */

class SimpleThread : public ThreadState, public ThreadContext
{
  public:
    typedef ThreadContext::Status Status;

  protected:
    std::array<RegFile, CCRegClass + 1> regFiles;

    BaseISA *const isa; // one "instance" of the current ISA.

    std::unique_ptr<PCStateBase> _pcState;

    // hardware transactional memory
    std::unique_ptr<BaseHTMCheckpoint> _htmCheckpoint;

    /** Did this instruction execute or is it predicated false */
    bool predicate;

    /** True if the memory access should be skipped for this instruction */
    bool memAccPredicate;

  public:
    std::string
    name() const
    {
        return csprintf("%s.[tid:%i]", baseCpu->name(), threadId());
    }

    PCEventQueue pcEventQueue;
    /**
     * An instruction-based event queue. Used for scheduling events based on
     * number of instructions committed.
     */
    EventQueue comInstEventQueue;

    System *system;

    BaseMMU *mmu;

    InstDecoder *decoder;

    // hardware transactional memory
    int64_t htmTransactionStarts;
    int64_t htmTransactionStops;

    // constructor: initialize SimpleThread from given process structure
    // FS
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 BaseMMU *_mmu, BaseISA *_isa, InstDecoder *_decoder);
    // SE
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 Process *_process, BaseMMU *_mmu, BaseISA *_isa,
                 InstDecoder *_decoder);

    virtual ~SimpleThread() {}

    void takeOverFrom(ThreadContext *oldContext) override;

    void copyState(ThreadContext *oldContext);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /***************************************************************
     *  SimpleThread functions to provide CPU with access to various
     *  state.
     **************************************************************/

    /** Returns the pointer to this SimpleThread's ThreadContext. Used
     *  when a ThreadContext must be passed to objects outside of the
     *  CPU.
     */
    ThreadContext *
    getTC()
    {
        return this;
    }

    void
    demapPage(Addr vaddr, uint64_t asn)
    {
        mmu->demapPage(vaddr, asn);
    }

    /*******************************************
     * ThreadContext interface functions.
     ******************************************/

    bool
    schedule(PCEvent *e) override
    {
        return pcEventQueue.schedule(e);
    }

    bool
    remove(PCEvent *e) override
    {
        return pcEventQueue.remove(e);
    }

    void
    scheduleInstCountEvent(Event *event, Tick count) override
    {
        comInstEventQueue.schedule(event, count);
    }

    void
    descheduleInstCountEvent(Event *event) override
    {
        comInstEventQueue.deschedule(event);
    }

    Tick
    getCurrentInstCount() override
    {
        return comInstEventQueue.getCurTick();
    }

    BaseCPU *
    getCpuPtr() override
    {
        return baseCpu;
    }

    int
    cpuId() const override
    {
        return ThreadState::cpuId();
    }

    uint32_t
    socketId() const override
    {
        return ThreadState::socketId();
    }

    int
    threadId() const override
    {
        return ThreadState::threadId();
    }

    void
    setThreadId(int id) override
    {
        ThreadState::setThreadId(id);
    }

    ContextID
    contextId() const override
    {
        return ThreadState::contextId();
    }

    void
    setContextId(ContextID id) override
    {
        ThreadState::setContextId(id);
    }

    BaseMMU *
    getMMUPtr() override
    {
        return mmu;
    }

    CheckerCPU *
    getCheckerCpuPtr() override
    {
        return NULL;
    }

    BaseISA *
    getIsaPtr() const override
    {
        return isa;
    }

    InstDecoder *
    getDecoderPtr() override
    {
        return decoder;
    }

    System *
    getSystemPtr() override
    {
        return system;
    }

    Process *
    getProcessPtr() override
    {
        return ThreadState::getProcessPtr();
    }

    void
    setProcessPtr(Process *p) override
    {
        ThreadState::setProcessPtr(p);
    }

    Status
    status() const override
    {
        return _status;
    }

    void
    setStatus(Status newStatus) override
    {
        _status = newStatus;
    }

    /// Set the status to Active.
    void activate() override;

    /// Set the status to Suspended.
    void suspend() override;

    /// Set the status to Halted.
    void halt() override;

    Tick
    readLastActivate() override
    {
        return ThreadState::readLastActivate();
    }

    Tick
    readLastSuspend() override
    {
        return ThreadState::readLastSuspend();
    }

    void copyArchRegs(ThreadContext *tc) override;

    void
    clearArchRegs() override
    {
        set(_pcState, isa->newPCState());
        for (auto &rf : regFiles)
            rf.clear();
        isa->clear();
    }

    //
    // New accessors for new decoder.
    //
    const PCStateBase &
    pcState() const override
    {
        return *_pcState;
    }

    void
    pcState(const PCStateBase &val) override
    {
        set(_pcState, val);
    }

    void
    pcStateNoRecord(const PCStateBase &val) override
    {
        set(_pcState, val);
    }

    bool
    readPredicate() const
    {
        return predicate;
    }

    void
    setPredicate(bool val)
    {
        predicate = val;
    }

    RegVal
    readMiscRegNoEffect(RegIndex misc_reg) const override
    {
        return isa->readMiscRegNoEffect(misc_reg);
    }

    RegVal
    readMiscReg(RegIndex misc_reg) override
    {
        return isa->readMiscReg(misc_reg);
    }

    void
    setMiscRegNoEffect(RegIndex misc_reg, RegVal val) override
    {
        return isa->setMiscRegNoEffect(misc_reg, val);
    }

    void
    setMiscReg(RegIndex misc_reg, RegVal val) override
    {
        return isa->setMiscReg(misc_reg, val);
    }

    unsigned
    readStCondFailures() const override
    {
        return storeCondFailures;
    }

    bool
    readMemAccPredicate()
    {
        return memAccPredicate;
    }

    void
    setMemAccPredicate(bool val)
    {
        memAccPredicate = val;
    }

    void
    setStCondFailures(unsigned sc_failures) override
    {
        storeCondFailures = sc_failures;
    }

    RegVal
    getReg(const RegId &arch_reg) const override
    {
        const RegId reg = arch_reg.flatten(*isa);

        const RegIndex idx = reg.index();

        const auto &reg_file = regFiles[reg.classValue()];
        const auto &reg_class = reg_file.regClass;

        RegVal val = reg_file.reg(idx);
        DPRINTFV(reg_class.debug(), "Reading %s reg %s (%d) as %#x.\n",
                 reg.className(), reg_class.regName(arch_reg), idx, val);
        return val;
    }

    void
    getReg(const RegId &arch_reg, void *val) const override
    {
        const RegId reg = arch_reg.flatten(*isa);

        const RegIndex idx = reg.index();

        const auto &reg_file = regFiles[reg.classValue()];
        const auto &reg_class = reg_file.regClass;

        reg_file.get(idx, val);
        DPRINTFV(reg_class.debug(), "Reading %s register %s (%d) as %s.\n",
                 reg.className(), reg_class.regName(arch_reg), idx,
                 reg_class.valString(val));
    }

    void *
    getWritableReg(const RegId &arch_reg) override
    {
        const RegId reg = arch_reg.flatten(*isa);
        const RegIndex idx = reg.index();
        auto &reg_file = regFiles[reg.classValue()];

        return reg_file.ptr(idx);
    }

    void
    setReg(const RegId &arch_reg, RegVal val) override
    {
        const RegId reg = arch_reg.flatten(*isa);

        if (reg.is(InvalidRegClass))
            return;

        const RegIndex idx = reg.index();

        auto &reg_file = regFiles[reg.classValue()];
        const auto &reg_class = reg_file.regClass;

        DPRINTFV(reg_class.debug(), "Setting %s register %s (%d) to %#x.\n",
                 reg.className(), reg_class.regName(arch_reg), idx, val);
        reg_file.reg(idx) = val;
    }

    void
    setReg(const RegId &arch_reg, const void *val) override
    {
        const RegId reg = arch_reg.flatten(*isa);

        const RegIndex idx = reg.index();

        auto &reg_file = regFiles[reg.classValue()];
        const auto &reg_class = reg_file.regClass;

        DPRINTFV(reg_class.debug(), "Setting %s register %s (%d) to %s.\n",
                 reg.className(), reg_class.regName(arch_reg), idx,
                 reg_class.valString(val));
        reg_file.set(idx, val);
    }

    // hardware transactional memory
    void htmAbortTransaction(uint64_t htm_uid,
                             HtmFailureFaultCause cause) override;

    BaseHTMCheckpointPtr &getHtmCheckpointPtr() override;
    void setHtmCheckpointPtr(BaseHTMCheckpointPtr new_cpt) override;
};

} // namespace gem5

#endif // __CPU_SIMPLE_THREAD_HH__
