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

#include "arch/decoder.hh"
#include "arch/generic/htm.hh"
#include "arch/generic/mmu.hh"
#include "arch/generic/tlb.hh"
#include "arch/isa.hh"
#include "arch/pcstate.hh"
#include "arch/vecregs.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "debug/CCRegs.hh"
#include "debug/FloatRegs.hh"
#include "debug/IntRegs.hh"
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
    std::vector<RegVal> floatRegs;
    std::vector<RegVal> intRegs;
    std::vector<TheISA::VecRegContainer> vecRegs;
    std::vector<TheISA::VecPredRegContainer> vecPredRegs;
    std::vector<RegVal> ccRegs;
    TheISA::ISA *const isa;    // one "instance" of the current ISA.

    TheISA::PCState _pcState;

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

    TheISA::Decoder decoder;

    // hardware transactional memory
    int64_t htmTransactionStarts;
    int64_t htmTransactionStops;

    // constructor: initialize SimpleThread from given process structure
    // FS
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 BaseMMU *_mmu, BaseISA *_isa);
    // SE
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 Process *_process, BaseMMU *_mmu,
                 BaseISA *_isa);

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
    ThreadContext *getTC() { return this; }

    void
    demapPage(Addr vaddr, uint64_t asn)
    {
        mmu->demapPage(vaddr, asn);
    }

    /*******************************************
     * ThreadContext interface functions.
     ******************************************/

    bool schedule(PCEvent *e) override { return pcEventQueue.schedule(e); }
    bool remove(PCEvent *e) override { return pcEventQueue.remove(e); }

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

    BaseCPU *getCpuPtr() override { return baseCpu; }

    int cpuId() const override { return ThreadState::cpuId(); }
    uint32_t socketId() const override { return ThreadState::socketId(); }
    int threadId() const override { return ThreadState::threadId(); }
    void setThreadId(int id) override { ThreadState::setThreadId(id); }
    ContextID contextId() const override { return ThreadState::contextId(); }
    void setContextId(ContextID id) override { ThreadState::setContextId(id); }

    BaseMMU *getMMUPtr() override { return mmu; }

    CheckerCPU *getCheckerCpuPtr() override { return NULL; }

    BaseISA *getIsaPtr() override { return isa; }

    TheISA::Decoder *getDecoderPtr() override { return &decoder; }

    System *getSystemPtr() override { return system; }

    PortProxy &getVirtProxy() override { return ThreadState::getVirtProxy(); }

    void
    initMemProxies(ThreadContext *tc) override
    {
        ThreadState::initMemProxies(tc);
    }

    Process *getProcessPtr() override { return ThreadState::getProcessPtr(); }
    void setProcessPtr(Process *p) override { ThreadState::setProcessPtr(p); }

    Status status() const override { return _status; }

    void setStatus(Status newStatus) override { _status = newStatus; }

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
        _pcState = 0;
        std::fill(intRegs.begin(), intRegs.end(), 0);
        std::fill(floatRegs.begin(), floatRegs.end(), 0);
        for (auto &vec_reg: vecRegs)
            vec_reg.zero();
        for (auto &pred_reg: vecPredRegs)
            pred_reg.reset();
        std::fill(ccRegs.begin(), ccRegs.end(), 0);
        isa->clear();
    }

    //
    // New accessors for new decoder.
    //
    RegVal
    readIntReg(RegIndex reg_idx) const override
    {
        int flatIndex = isa->flattenIntIndex(reg_idx);
        assert(flatIndex < intRegs.size());
        uint64_t regVal = readIntRegFlat(flatIndex);
        DPRINTF(IntRegs, "Reading int reg %d (%d) as %#x.\n",
                reg_idx, flatIndex, regVal);
        return regVal;
    }

    RegVal
    readFloatReg(RegIndex reg_idx) const override
    {
        int flatIndex = isa->flattenFloatIndex(reg_idx);
        assert(flatIndex < floatRegs.size());
        RegVal regVal = readFloatRegFlat(flatIndex);
        DPRINTF(FloatRegs, "Reading float reg %d (%d) bits as %#x.\n",
                reg_idx, flatIndex, regVal);
        return regVal;
    }

    const TheISA::VecRegContainer&
    readVecReg(const RegId& reg) const override
    {
        int flatIndex = isa->flattenVecIndex(reg.index());
        assert(flatIndex < vecRegs.size());
        const TheISA::VecRegContainer& regVal = readVecRegFlat(flatIndex);
        DPRINTF(VecRegs, "Reading vector reg %d (%d) as %s.\n",
                reg.index(), flatIndex, regVal);
        return regVal;
    }

    TheISA::VecRegContainer&
    getWritableVecReg(const RegId& reg) override
    {
        int flatIndex = isa->flattenVecIndex(reg.index());
        assert(flatIndex < vecRegs.size());
        TheISA::VecRegContainer& regVal = getWritableVecRegFlat(flatIndex);
        DPRINTF(VecRegs, "Reading vector reg %d (%d) as %s for modify.\n",
                reg.index(), flatIndex, regVal);
        return regVal;
    }

    const TheISA::VecElem &
    readVecElem(const RegId &reg) const override
    {
        int flatIndex = isa->flattenVecElemIndex(reg.index());
        assert(flatIndex < vecRegs.size());
        const TheISA::VecElem& regVal =
            readVecElemFlat(flatIndex, reg.elemIndex());
        DPRINTF(VecRegs, "Reading element %d of vector reg %d (%d) as"
                " %#x.\n", reg.elemIndex(), reg.index(), flatIndex, regVal);
        return regVal;
    }

    const TheISA::VecPredRegContainer &
    readVecPredReg(const RegId &reg) const override
    {
        int flatIndex = isa->flattenVecPredIndex(reg.index());
        assert(flatIndex < vecPredRegs.size());
        const TheISA::VecPredRegContainer& regVal =
            readVecPredRegFlat(flatIndex);
        DPRINTF(VecPredRegs, "Reading predicate reg %d (%d) as %s.\n",
                reg.index(), flatIndex, regVal);
        return regVal;
    }

    TheISA::VecPredRegContainer &
    getWritableVecPredReg(const RegId &reg) override
    {
        int flatIndex = isa->flattenVecPredIndex(reg.index());
        assert(flatIndex < vecPredRegs.size());
        TheISA::VecPredRegContainer& regVal =
            getWritableVecPredRegFlat(flatIndex);
        DPRINTF(VecPredRegs,
                "Reading predicate reg %d (%d) as %s for modify.\n",
                reg.index(), flatIndex, regVal);
        return regVal;
    }

    RegVal
    readCCReg(RegIndex reg_idx) const override
    {
        int flatIndex = isa->flattenCCIndex(reg_idx);
        assert(0 <= flatIndex);
        assert(flatIndex < ccRegs.size());
        uint64_t regVal(readCCRegFlat(flatIndex));
        DPRINTF(CCRegs, "Reading CC reg %d (%d) as %#x.\n",
                reg_idx, flatIndex, regVal);
        return regVal;
    }

    void
    setIntReg(RegIndex reg_idx, RegVal val) override
    {
        int flatIndex = isa->flattenIntIndex(reg_idx);
        assert(flatIndex < intRegs.size());
        DPRINTF(IntRegs, "Setting int reg %d (%d) to %#x.\n",
                reg_idx, flatIndex, val);
        setIntRegFlat(flatIndex, val);
    }

    void
    setFloatReg(RegIndex reg_idx, RegVal val) override
    {
        int flatIndex = isa->flattenFloatIndex(reg_idx);
        assert(flatIndex < floatRegs.size());
        // XXX: Fix array out of bounds compiler error for gem5.fast
        // when checkercpu enabled
        if (flatIndex < floatRegs.size())
            setFloatRegFlat(flatIndex, val);
        DPRINTF(FloatRegs, "Setting float reg %d (%d) bits to %#x.\n",
                reg_idx, flatIndex, val);
    }

    void
    setVecReg(const RegId &reg, const TheISA::VecRegContainer &val) override
    {
        int flatIndex = isa->flattenVecIndex(reg.index());
        assert(flatIndex < vecRegs.size());
        setVecRegFlat(flatIndex, val);
        DPRINTF(VecRegs, "Setting vector reg %d (%d) to %s.\n",
                reg.index(), flatIndex, val);
    }

    void
    setVecElem(const RegId &reg, const TheISA::VecElem &val) override
    {
        int flatIndex = isa->flattenVecElemIndex(reg.index());
        assert(flatIndex < vecRegs.size());
        setVecElemFlat(flatIndex, reg.elemIndex(), val);
        DPRINTF(VecRegs, "Setting element %d of vector reg %d (%d) to"
                " %#x.\n", reg.elemIndex(), reg.index(), flatIndex, val);
    }

    void
    setVecPredReg(const RegId &reg,
            const TheISA::VecPredRegContainer &val) override
    {
        int flatIndex = isa->flattenVecPredIndex(reg.index());
        assert(flatIndex < vecPredRegs.size());
        setVecPredRegFlat(flatIndex, val);
        DPRINTF(VecPredRegs, "Setting predicate reg %d (%d) to %s.\n",
                reg.index(), flatIndex, val);
    }

    void
    setCCReg(RegIndex reg_idx, RegVal val) override
    {
        int flatIndex = isa->flattenCCIndex(reg_idx);
        assert(flatIndex < ccRegs.size());
        DPRINTF(CCRegs, "Setting CC reg %d (%d) to %#x.\n",
                reg_idx, flatIndex, val);
        setCCRegFlat(flatIndex, val);
    }

    TheISA::PCState pcState() const override { return _pcState; }
    void pcState(const TheISA::PCState &val) override { _pcState = val; }

    void
    pcStateNoRecord(const TheISA::PCState &val) override
    {
        _pcState = val;
    }

    Addr instAddr() const override  { return _pcState.instAddr(); }
    Addr nextInstAddr() const override { return _pcState.nextInstAddr(); }
    MicroPC microPC() const override { return _pcState.microPC(); }
    bool readPredicate() const { return predicate; }
    void setPredicate(bool val) { predicate = val; }

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

    RegId
    flattenRegId(const RegId& regId) const override
    {
        return isa->flattenRegId(regId);
    }

    unsigned readStCondFailures() const override { return storeCondFailures; }

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

    RegVal readIntRegFlat(RegIndex idx) const override { return intRegs[idx]; }
    void
    setIntRegFlat(RegIndex idx, RegVal val) override
    {
        intRegs[idx] = val;
    }

    RegVal
    readFloatRegFlat(RegIndex idx) const override
    {
        return floatRegs[idx];
    }
    void
    setFloatRegFlat(RegIndex idx, RegVal val) override
    {
        floatRegs[idx] = val;
    }

    const TheISA::VecRegContainer &
    readVecRegFlat(RegIndex reg) const override
    {
        return vecRegs[reg];
    }

    TheISA::VecRegContainer &
    getWritableVecRegFlat(RegIndex reg) override
    {
        return vecRegs[reg];
    }

    void
    setVecRegFlat(RegIndex reg, const TheISA::VecRegContainer &val) override
    {
        vecRegs[reg] = val;
    }

    const TheISA::VecElem &
    readVecElemFlat(RegIndex reg, const ElemIndex &elemIndex) const override
    {
        return vecRegs[reg].as<TheISA::VecElem>()[elemIndex];
    }

    void
    setVecElemFlat(RegIndex reg, const ElemIndex &elemIndex,
                   const TheISA::VecElem &val) override
    {
        vecRegs[reg].as<TheISA::VecElem>()[elemIndex] = val;
    }

    const TheISA::VecPredRegContainer &
    readVecPredRegFlat(RegIndex reg) const override
    {
        return vecPredRegs[reg];
    }

    TheISA::VecPredRegContainer &
    getWritableVecPredRegFlat(RegIndex reg) override
    {
        return vecPredRegs[reg];
    }

    void
    setVecPredRegFlat(RegIndex reg,
            const TheISA::VecPredRegContainer &val) override
    {
        vecPredRegs[reg] = val;
    }

    RegVal readCCRegFlat(RegIndex idx) const override { return ccRegs[idx]; }
    void setCCRegFlat(RegIndex idx, RegVal val) override { ccRegs[idx] = val; }

    // hardware transactional memory
    void htmAbortTransaction(uint64_t htm_uid,
                             HtmFailureFaultCause cause) override;

    BaseHTMCheckpointPtr& getHtmCheckpointPtr() override;
    void setHtmCheckpointPtr(BaseHTMCheckpointPtr new_cpt) override;
};

} // namespace gem5

#endif // __CPU_SIMPLE_THREAD_HH__
