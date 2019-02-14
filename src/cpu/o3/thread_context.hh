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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#ifndef __CPU_O3_THREAD_CONTEXT_HH__
#define __CPU_O3_THREAD_CONTEXT_HH__

#include "config/the_isa.hh"
#include "cpu/o3/isa_specific.hh"
#include "cpu/thread_context.hh"

class EndQuiesceEvent;
namespace Kernel {
    class Statistics;
}

/**
 * Derived ThreadContext class for use with the O3CPU.  It
 * provides the interface for any external objects to access a
 * single thread's state and some general CPU state.  Any time
 * external objects try to update state through this interface,
 * the CPU will create an event to squash all in-flight
 * instructions in order to ensure state is maintained correctly.
 * It must be defined specifically for the O3CPU because
 * not all architectural state is located within the O3ThreadState
 * (such as the commit PC, and registers), and specific actions
 * must be taken when using this interface (such as squashing all
 * in-flight instructions when doing a write to this interface).
 */
template <class Impl>
class O3ThreadContext : public ThreadContext
{
  public:
    typedef typename Impl::O3CPU O3CPU;

   /** Pointer to the CPU. */
    O3CPU *cpu;

    /** Pointer to the thread state that this TC corrseponds to. */
    O3ThreadState<Impl> *thread;

    /** Returns a pointer to the ITB. */
    BaseTLB *getITBPtr() override { return cpu->itb; }

    /** Returns a pointer to the DTB. */
    BaseTLB *getDTBPtr() override { return cpu->dtb; }

    CheckerCPU *getCheckerCpuPtr() override { return NULL; }

    TheISA::ISA *
    getIsaPtr() override
    {
        return cpu->isa[thread->threadId()];
    }

    TheISA::Decoder *
    getDecoderPtr() override
    {
        return cpu->fetch.decoder[thread->threadId()];
    }

    /** Returns a pointer to this CPU. */
    virtual BaseCPU *getCpuPtr() override { return cpu; }

    /** Reads this CPU's ID. */
    virtual int cpuId() const override { return cpu->cpuId(); }

    /** Reads this CPU's Socket ID. */
    virtual uint32_t socketId() const override { return cpu->socketId(); }

    virtual ContextID
    contextId() const override { return thread->contextId(); }

    virtual void setContextId(int id) override { thread->setContextId(id); }

    /** Returns this thread's ID number. */
    virtual int threadId() const override
    { return thread->threadId(); }
    virtual void setThreadId(int id) override
    { return thread->setThreadId(id); }

    /** Returns a pointer to the system. */
    virtual System *getSystemPtr() override { return cpu->system; }

    /** Returns a pointer to this thread's kernel statistics. */
    virtual TheISA::Kernel::Statistics *getKernelStats() override
    { return thread->kernelStats; }

    /** Returns a pointer to this thread's process. */
    virtual Process *getProcessPtr() override
    { return thread->getProcessPtr(); }

    virtual void setProcessPtr(Process *p) override
    { thread->setProcessPtr(p); }

    virtual PortProxy &getPhysProxy() override
    { return thread->getPhysProxy(); }

    virtual FSTranslatingPortProxy &getVirtProxy() override;

    virtual void initMemProxies(ThreadContext *tc) override
    { thread->initMemProxies(tc); }

    virtual SETranslatingPortProxy &getMemProxy() override
    { return thread->getMemProxy(); }

    /** Returns this thread's status. */
    virtual Status status() const override { return thread->status(); }

    /** Sets this thread's status. */
    virtual void setStatus(Status new_status) override
    { thread->setStatus(new_status); }

    /** Set the status to Active. */
    virtual void activate() override;

    /** Set the status to Suspended. */
    virtual void suspend() override;

    /** Set the status to Halted. */
    virtual void halt() override;

    /** Dumps the function profiling information.
     * @todo: Implement.
     */
    virtual void dumpFuncProfile() override;

    /** Takes over execution of a thread from another CPU. */
    virtual void takeOverFrom(ThreadContext *old_context) override;

    /** Registers statistics associated with this TC. */
    virtual void regStats(const std::string &name) override;

    /** Reads the last tick that this thread was activated on. */
    virtual Tick readLastActivate() override;
    /** Reads the last tick that this thread was suspended on. */
    virtual Tick readLastSuspend() override;

    /** Clears the function profiling information. */
    virtual void profileClear() override;
    /** Samples the function profiling information. */
    virtual void profileSample() override;

    /** Copies the architectural registers from another TC into this TC. */
    virtual void copyArchRegs(ThreadContext *tc) override;

    /** Resets all architectural registers to 0. */
    virtual void clearArchRegs() override;

    /** Reads an integer register. */
    virtual RegVal
    readReg(int reg_idx)
    {
        return readIntRegFlat(flattenRegId(RegId(IntRegClass,
                                                 reg_idx)).index());
    }
    virtual RegVal
    readIntReg(int reg_idx) override
    {
        return readIntRegFlat(flattenRegId(RegId(IntRegClass,
                                                 reg_idx)).index());
    }

    virtual RegVal
    readFloatReg(int reg_idx) override
    {
        return readFloatRegFlat(flattenRegId(RegId(FloatRegClass,
                                             reg_idx)).index());
    }

    virtual const VecRegContainer &
    readVecReg(const RegId& id) const override
    {
        return readVecRegFlat(flattenRegId(id).index());
    }

    /**
     * Read vector register operand for modification, hierarchical indexing.
     */
    virtual VecRegContainer &
    getWritableVecReg(const RegId& id) override
    {
        return getWritableVecRegFlat(flattenRegId(id).index());
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    virtual ConstVecLane8
    readVec8BitLaneReg(const RegId& id) const override
    {
        return readVecLaneFlat<uint8_t>(flattenRegId(id).index(),
                    id.elemIndex());
    }

    /** Reads source vector 16bit operand. */
    virtual ConstVecLane16
    readVec16BitLaneReg(const RegId& id) const override
    {
        return readVecLaneFlat<uint16_t>(flattenRegId(id).index(),
                    id.elemIndex());
    }

    /** Reads source vector 32bit operand. */
    virtual ConstVecLane32
    readVec32BitLaneReg(const RegId& id) const override
    {
        return readVecLaneFlat<uint32_t>(flattenRegId(id).index(),
                    id.elemIndex());
    }

    /** Reads source vector 64bit operand. */
    virtual ConstVecLane64
    readVec64BitLaneReg(const RegId& id) const override
    {
        return readVecLaneFlat<uint64_t>(flattenRegId(id).index(),
                    id.elemIndex());
    }

    /** Write a lane of the destination vector register. */
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::Byte>& val) override
    { return setVecLaneFlat(flattenRegId(reg).index(), reg.elemIndex(), val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::TwoByte>& val) override
    { return setVecLaneFlat(flattenRegId(reg).index(), reg.elemIndex(), val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::FourByte>& val) override
    { return setVecLaneFlat(flattenRegId(reg).index(), reg.elemIndex(), val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::EightByte>& val) override
    { return setVecLaneFlat(flattenRegId(reg).index(), reg.elemIndex(), val); }
    /** @} */

    virtual const VecElem& readVecElem(const RegId& reg) const override {
        return readVecElemFlat(flattenRegId(reg).index(), reg.elemIndex());
    }

    virtual const VecPredRegContainer&
    readVecPredReg(const RegId& id) const override {
        return readVecPredRegFlat(flattenRegId(id).index());
    }

    virtual VecPredRegContainer&
    getWritableVecPredReg(const RegId& id) override {
        return getWritableVecPredRegFlat(flattenRegId(id).index());
    }

    virtual RegVal
    readCCReg(int reg_idx) override
    {
        return readCCRegFlat(flattenRegId(RegId(CCRegClass,
                                                 reg_idx)).index());
    }

    /** Sets an integer register to a value. */
    virtual void
    setIntReg(int reg_idx, RegVal val) override
    {
        setIntRegFlat(flattenRegId(RegId(IntRegClass, reg_idx)).index(), val);
    }

    virtual void
    setFloatReg(int reg_idx, RegVal val) override
    {
        setFloatRegFlat(flattenRegId(RegId(FloatRegClass,
                                           reg_idx)).index(), val);
    }

    virtual void
    setVecReg(const RegId& reg, const VecRegContainer& val) override
    {
        setVecRegFlat(flattenRegId(reg).index(), val);
    }

    virtual void
    setVecElem(const RegId& reg, const VecElem& val) override
    {
        setVecElemFlat(flattenRegId(reg).index(), reg.elemIndex(), val);
    }

    virtual void
    setVecPredReg(const RegId& reg,
                  const VecPredRegContainer& val) override
    {
        setVecPredRegFlat(flattenRegId(reg).index(), val);
    }

    virtual void
    setCCReg(int reg_idx, RegVal val) override
    {
        setCCRegFlat(flattenRegId(RegId(CCRegClass, reg_idx)).index(), val);
    }

    /** Reads this thread's PC state. */
    virtual TheISA::PCState pcState() override
    { return cpu->pcState(thread->threadId()); }

    /** Sets this thread's PC state. */
    virtual void pcState(const TheISA::PCState &val) override;

    virtual void pcStateNoRecord(const TheISA::PCState &val) override;

    /** Reads this thread's PC. */
    virtual Addr instAddr() override
    { return cpu->instAddr(thread->threadId()); }

    /** Reads this thread's next PC. */
    virtual Addr nextInstAddr() override
    { return cpu->nextInstAddr(thread->threadId()); }

    /** Reads this thread's next PC. */
    virtual MicroPC microPC() override
    { return cpu->microPC(thread->threadId()); }

    /** Reads a miscellaneous register. */
    virtual RegVal readMiscRegNoEffect(int misc_reg) const override
    { return cpu->readMiscRegNoEffect(misc_reg, thread->threadId()); }

    /** Reads a misc. register, including any side-effects the
     * read might have as defined by the architecture. */
    virtual RegVal readMiscReg(int misc_reg) override
    { return cpu->readMiscReg(misc_reg, thread->threadId()); }

    /** Sets a misc. register. */
    virtual void setMiscRegNoEffect(int misc_reg, RegVal val) override;

    /** Sets a misc. register, including any side-effects the
     * write might have as defined by the architecture. */
    virtual void setMiscReg(int misc_reg, RegVal val) override;

    virtual RegId flattenRegId(const RegId& regId) const override;

    /** Returns the number of consecutive store conditional failures. */
    // @todo: Figure out where these store cond failures should go.
    virtual unsigned readStCondFailures() override
    { return thread->storeCondFailures; }

    /** Sets the number of consecutive store conditional failures. */
    virtual void setStCondFailures(unsigned sc_failures) override
    { thread->storeCondFailures = sc_failures; }

    /** Executes a syscall in SE mode. */
    virtual void syscall(int64_t callnum, Fault *fault) override
    { return cpu->syscall(callnum, thread->threadId(), fault); }

    /** Reads the funcExeInst counter. */
    virtual Counter readFuncExeInst() override { return thread->funcExeInst; }

    /** Returns pointer to the quiesce event. */
    virtual EndQuiesceEvent *
    getQuiesceEvent() override
    {
        return this->thread->quiesceEvent;
    }
    /** check if the cpu is currently in state update mode and squash if not.
     * This function will return true if a trap is pending or if a fault or
     * similar is currently writing to the thread context and doesn't want
     * reset all the state (see noSquashFromTC).
     */
    inline void
    conditionalSquash()
    {
        if (!thread->trapPending && !thread->noSquashFromTC)
            cpu->squashFromTC(thread->threadId());
    }

    virtual RegVal readIntRegFlat(int idx) override;
    virtual void setIntRegFlat(int idx, RegVal val) override;

    virtual RegVal readFloatRegFlat(int idx) override;
    virtual void setFloatRegFlat(int idx, RegVal val) override;

    virtual const VecRegContainer& readVecRegFlat(int idx) const override;
    /** Read vector register operand for modification, flat indexing. */
    virtual VecRegContainer& getWritableVecRegFlat(int idx) override;
    virtual void setVecRegFlat(int idx, const VecRegContainer& val) override;

    template <typename VecElem>
    VecLaneT<VecElem, true>
    readVecLaneFlat(int idx, int lId) const
    {
        return cpu->template readArchVecLane<VecElem>(idx, lId,
                thread->threadId());
    }

    template <typename LD>
    void setVecLaneFlat(int idx, int lId, const LD& val)
    {
        cpu->template setArchVecLane(idx, lId, thread->threadId(), val);
    }

    virtual const VecElem& readVecElemFlat(
        const RegIndex& idx,
        const ElemIndex& elemIndex) const override;
    virtual void setVecElemFlat(
        const RegIndex& idx,
        const ElemIndex& elemIdx, const VecElem& val) override;

    virtual const VecPredRegContainer& readVecPredRegFlat(int idx)
        const override;
    virtual VecPredRegContainer& getWritableVecPredRegFlat(int idx) override;
    virtual void setVecPredRegFlat(int idx,
                                   const VecPredRegContainer& val) override;

    virtual RegVal readCCRegFlat(int idx) override;
    virtual void setCCRegFlat(int idx, RegVal val) override;
};

#endif
