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
    TheISA::TLB *getITBPtr() { return cpu->itb; }

    /** Returns a pointer to the DTB. */
    TheISA::TLB *getDTBPtr() { return cpu->dtb; }

    CheckerCPU *getCheckerCpuPtr() { return NULL; }

    TheISA::Decoder *
    getDecoderPtr()
    {
        return cpu->fetch.decoder[thread->threadId()];
    }

    /** Returns a pointer to this CPU. */
    virtual BaseCPU *getCpuPtr() { return cpu; }

    /** Reads this CPU's ID. */
    virtual int cpuId() const { return cpu->cpuId(); }

    /** Reads this CPU's Socket ID. */
    virtual uint32_t socketId() const { return cpu->socketId(); }

    virtual ContextID contextId() const { return thread->contextId(); }

    virtual void setContextId(int id) { thread->setContextId(id); }

    /** Returns this thread's ID number. */
    virtual int threadId() const { return thread->threadId(); }
    virtual void setThreadId(int id) { return thread->setThreadId(id); }

    /** Returns a pointer to the system. */
    virtual System *getSystemPtr() { return cpu->system; }

    /** Returns a pointer to this thread's kernel statistics. */
    virtual TheISA::Kernel::Statistics *getKernelStats()
    { return thread->kernelStats; }

    /** Returns a pointer to this thread's process. */
    virtual Process *getProcessPtr() { return thread->getProcessPtr(); }

    virtual void setProcessPtr(Process *p) { thread->setProcessPtr(p); }

    virtual PortProxy &getPhysProxy() { return thread->getPhysProxy(); }

    virtual FSTranslatingPortProxy &getVirtProxy();

    virtual void initMemProxies(ThreadContext *tc)
    { thread->initMemProxies(tc); }

    virtual SETranslatingPortProxy &getMemProxy()
    { return thread->getMemProxy(); }

    /** Returns this thread's status. */
    virtual Status status() const { return thread->status(); }

    /** Sets this thread's status. */
    virtual void setStatus(Status new_status)
    { thread->setStatus(new_status); }

    /** Set the status to Active. */
    virtual void activate();

    /** Set the status to Suspended. */
    virtual void suspend();

    /** Set the status to Halted. */
    virtual void halt();

    /** Dumps the function profiling information.
     * @todo: Implement.
     */
    virtual void dumpFuncProfile();

    /** Takes over execution of a thread from another CPU. */
    virtual void takeOverFrom(ThreadContext *old_context);

    /** Registers statistics associated with this TC. */
    virtual void regStats(const std::string &name);

    /** Reads the last tick that this thread was activated on. */
    virtual Tick readLastActivate();
    /** Reads the last tick that this thread was suspended on. */
    virtual Tick readLastSuspend();

    /** Clears the function profiling information. */
    virtual void profileClear();
    /** Samples the function profiling information. */
    virtual void profileSample();

    /** Copies the architectural registers from another TC into this TC. */
    virtual void copyArchRegs(ThreadContext *tc);

    /** Resets all architectural registers to 0. */
    virtual void clearArchRegs();

    /** Reads an integer register. */
    virtual uint64_t readReg(int reg_idx) {
        return readIntRegFlat(flattenRegId(RegId(IntRegClass,
                                                 reg_idx)).index());
    }
    virtual uint64_t readIntReg(int reg_idx) {
        return readIntRegFlat(flattenRegId(RegId(IntRegClass,
                                                 reg_idx)).index());
    }

    virtual FloatReg readFloatReg(int reg_idx) {
        return readFloatRegFlat(flattenRegId(RegId(FloatRegClass,
                                                 reg_idx)).index());
    }

    virtual FloatRegBits readFloatRegBits(int reg_idx) {
        return readFloatRegBitsFlat(flattenRegId(RegId(FloatRegClass,
                                                 reg_idx)).index());
    }

    virtual const VecRegContainer& readVecReg(const RegId& id) const {
        return readVecRegFlat(flattenRegId(id).index());
    }

    /**
     * Read vector register operand for modification, hierarchical indexing.
     */
    virtual VecRegContainer& getWritableVecReg(const RegId& id) {
        return getWritableVecRegFlat(flattenRegId(id).index());
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    virtual ConstVecLane8
    readVec8BitLaneReg(const RegId& id) const
    {
        return readVecLaneFlat<uint8_t>(flattenRegId(id).index(),
                    id.elemIndex());
    }

    /** Reads source vector 16bit operand. */
    virtual ConstVecLane16
    readVec16BitLaneReg(const RegId& id) const
    {
        return readVecLaneFlat<uint16_t>(flattenRegId(id).index(),
                    id.elemIndex());
    }

    /** Reads source vector 32bit operand. */
    virtual ConstVecLane32
    readVec32BitLaneReg(const RegId& id) const
    {
        return readVecLaneFlat<uint32_t>(flattenRegId(id).index(),
                    id.elemIndex());
    }

    /** Reads source vector 64bit operand. */
    virtual ConstVecLane64
    readVec64BitLaneReg(const RegId& id) const
    {
        return readVecLaneFlat<uint64_t>(flattenRegId(id).index(),
                    id.elemIndex());
    }

    /** Write a lane of the destination vector register. */
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::Byte>& val)
    { return setVecLaneFlat(flattenRegId(reg).index(), reg.elemIndex(), val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::TwoByte>& val)
    { return setVecLaneFlat(flattenRegId(reg).index(), reg.elemIndex(), val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::FourByte>& val)
    { return setVecLaneFlat(flattenRegId(reg).index(), reg.elemIndex(), val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::EightByte>& val)
    { return setVecLaneFlat(flattenRegId(reg).index(), reg.elemIndex(), val); }
    /** @} */

    virtual const VecElem& readVecElem(const RegId& reg) const {
        return readVecElemFlat(flattenRegId(reg).index(), reg.elemIndex());
    }

    virtual CCReg readCCReg(int reg_idx) {
        return readCCRegFlat(flattenRegId(RegId(CCRegClass,
                                                 reg_idx)).index());
    }

    /** Sets an integer register to a value. */
    virtual void setIntReg(int reg_idx, uint64_t val) {
        setIntRegFlat(flattenRegId(RegId(IntRegClass, reg_idx)).index(), val);
    }

    virtual void setFloatReg(int reg_idx, FloatReg val) {
        setFloatRegFlat(flattenRegId(RegId(FloatRegClass,
                                           reg_idx)).index(), val);
    }

    virtual void setFloatRegBits(int reg_idx, FloatRegBits val) {
        setFloatRegBitsFlat(flattenRegId(RegId(FloatRegClass,
                                               reg_idx)).index(), val);
    }

    virtual void setVecReg(const RegId& reg, const VecRegContainer& val) {
        setVecRegFlat(flattenRegId(reg).index(), val);
    }

    virtual void setVecElem(const RegId& reg, const VecElem& val) {
        setVecElemFlat(flattenRegId(reg).index(), reg.elemIndex(), val);
    }

    virtual void setCCReg(int reg_idx, CCReg val) {
        setCCRegFlat(flattenRegId(RegId(CCRegClass, reg_idx)).index(), val);
    }

    /** Reads this thread's PC state. */
    virtual TheISA::PCState pcState()
    { return cpu->pcState(thread->threadId()); }

    /** Sets this thread's PC state. */
    virtual void pcState(const TheISA::PCState &val);

    virtual void pcStateNoRecord(const TheISA::PCState &val);

    /** Reads this thread's PC. */
    virtual Addr instAddr()
    { return cpu->instAddr(thread->threadId()); }

    /** Reads this thread's next PC. */
    virtual Addr nextInstAddr()
    { return cpu->nextInstAddr(thread->threadId()); }

    /** Reads this thread's next PC. */
    virtual MicroPC microPC()
    { return cpu->microPC(thread->threadId()); }

    /** Reads a miscellaneous register. */
    virtual MiscReg readMiscRegNoEffect(int misc_reg) const
    { return cpu->readMiscRegNoEffect(misc_reg, thread->threadId()); }

    /** Reads a misc. register, including any side-effects the
     * read might have as defined by the architecture. */
    virtual MiscReg readMiscReg(int misc_reg)
    { return cpu->readMiscReg(misc_reg, thread->threadId()); }

    /** Sets a misc. register. */
    virtual void setMiscRegNoEffect(int misc_reg, const MiscReg &val);

    /** Sets a misc. register, including any side-effects the
     * write might have as defined by the architecture. */
    virtual void setMiscReg(int misc_reg, const MiscReg &val);

    virtual RegId flattenRegId(const RegId& regId) const;

    /** Returns the number of consecutive store conditional failures. */
    // @todo: Figure out where these store cond failures should go.
    virtual unsigned readStCondFailures()
    { return thread->storeCondFailures; }

    /** Sets the number of consecutive store conditional failures. */
    virtual void setStCondFailures(unsigned sc_failures)
    { thread->storeCondFailures = sc_failures; }

    /** Executes a syscall in SE mode. */
    virtual void syscall(int64_t callnum, Fault *fault)
    { return cpu->syscall(callnum, thread->threadId(), fault); }

    /** Reads the funcExeInst counter. */
    virtual Counter readFuncExeInst() { return thread->funcExeInst; }

    /** Returns pointer to the quiesce event. */
    virtual EndQuiesceEvent *getQuiesceEvent()
    {
        return this->thread->quiesceEvent;
    }
    /** check if the cpu is currently in state update mode and squash if not.
     * This function will return true if a trap is pending or if a fault or
     * similar is currently writing to the thread context and doesn't want
     * reset all the state (see noSquashFromTC).
     */
    inline void conditionalSquash()
    {
        if (!thread->trapPending && !thread->noSquashFromTC)
            cpu->squashFromTC(thread->threadId());
    }

    virtual uint64_t readIntRegFlat(int idx);
    virtual void setIntRegFlat(int idx, uint64_t val);

    virtual FloatReg readFloatRegFlat(int idx);
    virtual void setFloatRegFlat(int idx, FloatReg val);

    virtual FloatRegBits readFloatRegBitsFlat(int idx);
    virtual void setFloatRegBitsFlat(int idx, FloatRegBits val);

    virtual const VecRegContainer& readVecRegFlat(int idx) const;
    /** Read vector register operand for modification, flat indexing. */
    virtual VecRegContainer& getWritableVecRegFlat(int idx);
    virtual void setVecRegFlat(int idx, const VecRegContainer& val);

    template <typename VecElem>
    VecLaneT<VecElem, true> readVecLaneFlat(int idx, int lId) const
    {
        return cpu->template readArchVecLane<VecElem>(idx, lId,
                thread->threadId());
    }

    template <typename LD>
    void setVecLaneFlat(int idx, int lId, const LD& val)
    {
        cpu->template setArchVecLane(idx, lId, thread->threadId(), val);
    }

    virtual const VecElem& readVecElemFlat(const RegIndex& idx,
                                           const ElemIndex& elemIndex) const;
    virtual void setVecElemFlat(const RegIndex& idx, const ElemIndex& elemIdx,
                                const VecElem& val);

    virtual CCReg readCCRegFlat(int idx);
    virtual void setCCRegFlat(int idx, CCReg val);
};

#endif
