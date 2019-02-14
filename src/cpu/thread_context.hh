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

#ifndef __CPU_THREAD_CONTEXT_HH__
#define __CPU_THREAD_CONTEXT_HH__

#include <iostream>
#include <string>

#include "arch/registers.hh"
#include "arch/types.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/reg_class.hh"

// @todo: Figure out a more architecture independent way to obtain the ITB and
// DTB pointers.
namespace TheISA
{
    class ISA;
    class Decoder;
}
class BaseCPU;
class BaseTLB;
class CheckerCPU;
class Checkpoint;
class EndQuiesceEvent;
class SETranslatingPortProxy;
class FSTranslatingPortProxy;
class PortProxy;
class Process;
class System;
namespace TheISA {
    namespace Kernel {
        class Statistics;
    }
}

/**
 * ThreadContext is the external interface to all thread state for
 * anything outside of the CPU. It provides all accessor methods to
 * state that might be needed by external objects, ranging from
 * register values to things such as kernel stats. It is an abstract
 * base class; the CPU can create its own ThreadContext by either
 * deriving from it, or using the templated ProxyThreadContext.
 *
 * The ThreadContext is slightly different than the ExecContext.  The
 * ThreadContext provides access to an individual thread's state; an
 * ExecContext provides ISA access to the CPU (meaning it is
 * implicitly multithreaded on SMT systems).  Additionally the
 * ThreadState is an abstract class that exactly defines the
 * interface; the ExecContext is a more implicit interface that must
 * be implemented so that the ISA can access whatever state it needs.
 */
class ThreadContext
{
  protected:
    typedef TheISA::MachInst MachInst;
    using VecRegContainer = TheISA::VecRegContainer;
    using VecElem = TheISA::VecElem;
    using VecPredRegContainer = TheISA::VecPredRegContainer;

  public:

    enum Status
    {
        /// Running.  Instructions should be executed only when
        /// the context is in this state.
        Active,

        /// Temporarily inactive.  Entered while waiting for
        /// synchronization, etc.
        Suspended,

        /// Trying to exit and waiting for an event to completely exit.
        /// Entered when target executes an exit syscall.
        Halting,

        /// Permanently shut down.  Entered when target executes
        /// m5exit pseudo-instruction.  When all contexts enter
        /// this state, the simulation will terminate.
        Halted
    };

    virtual ~ThreadContext() { };

    virtual BaseCPU *getCpuPtr() = 0;

    virtual int cpuId() const = 0;

    virtual uint32_t socketId() const = 0;

    virtual int threadId() const = 0;

    virtual void setThreadId(int id) = 0;

    virtual int contextId() const = 0;

    virtual void setContextId(int id) = 0;

    virtual BaseTLB *getITBPtr() = 0;

    virtual BaseTLB *getDTBPtr() = 0;

    virtual CheckerCPU *getCheckerCpuPtr() = 0;

    virtual TheISA::ISA *getIsaPtr() = 0;

    virtual TheISA::Decoder *getDecoderPtr() = 0;

    virtual System *getSystemPtr() = 0;

    virtual TheISA::Kernel::Statistics *getKernelStats() = 0;

    virtual PortProxy &getPhysProxy() = 0;

    virtual FSTranslatingPortProxy &getVirtProxy() = 0;

    /**
     * Initialise the physical and virtual port proxies and tie them to
     * the data port of the CPU.
     *
     * tc ThreadContext for the virtual-to-physical translation
     */
    virtual void initMemProxies(ThreadContext *tc) = 0;

    virtual SETranslatingPortProxy &getMemProxy() = 0;

    virtual Process *getProcessPtr() = 0;

    virtual void setProcessPtr(Process *p) = 0;

    virtual Status status() const = 0;

    virtual void setStatus(Status new_status) = 0;

    /// Set the status to Active.
    virtual void activate() = 0;

    /// Set the status to Suspended.
    virtual void suspend() = 0;

    /// Set the status to Halted.
    virtual void halt() = 0;

    /// Quiesce thread context
    void quiesce();

    /// Quiesce, suspend, and schedule activate at resume
    void quiesceTick(Tick resume);

    virtual void dumpFuncProfile() = 0;

    virtual void takeOverFrom(ThreadContext *old_context) = 0;

    virtual void regStats(const std::string &name) = 0;

    virtual EndQuiesceEvent *getQuiesceEvent() = 0;

    // Not necessarily the best location for these...
    // Having an extra function just to read these is obnoxious
    virtual Tick readLastActivate() = 0;
    virtual Tick readLastSuspend() = 0;

    virtual void profileClear() = 0;
    virtual void profileSample() = 0;

    virtual void copyArchRegs(ThreadContext *tc) = 0;

    virtual void clearArchRegs() = 0;

    //
    // New accessors for new decoder.
    //
    virtual RegVal readIntReg(int reg_idx) = 0;

    virtual RegVal readFloatReg(int reg_idx) = 0;

    virtual const VecRegContainer& readVecReg(const RegId& reg) const = 0;
    virtual VecRegContainer& getWritableVecReg(const RegId& reg) = 0;

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    virtual ConstVecLane8
    readVec8BitLaneReg(const RegId& reg) const = 0;

    /** Reads source vector 16bit operand. */
    virtual ConstVecLane16
    readVec16BitLaneReg(const RegId& reg) const = 0;

    /** Reads source vector 32bit operand. */
    virtual ConstVecLane32
    readVec32BitLaneReg(const RegId& reg) const = 0;

    /** Reads source vector 64bit operand. */
    virtual ConstVecLane64
    readVec64BitLaneReg(const RegId& reg) const = 0;

    /** Write a lane of the destination vector register. */
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::Byte>& val) = 0;
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::TwoByte>& val) = 0;
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::FourByte>& val) = 0;
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::EightByte>& val) = 0;
    /** @} */

    virtual const VecElem& readVecElem(const RegId& reg) const = 0;

    virtual const VecPredRegContainer& readVecPredReg(const RegId& reg)
        const = 0;
    virtual VecPredRegContainer& getWritableVecPredReg(const RegId& reg) = 0;

    virtual RegVal readCCReg(int reg_idx) = 0;

    virtual void setIntReg(int reg_idx, RegVal val) = 0;

    virtual void setFloatReg(int reg_idx, RegVal val) = 0;

    virtual void setVecReg(const RegId& reg, const VecRegContainer& val) = 0;

    virtual void setVecElem(const RegId& reg, const VecElem& val) = 0;

    virtual void setVecPredReg(const RegId& reg,
                               const VecPredRegContainer& val) = 0;

    virtual void setCCReg(int reg_idx, RegVal val) = 0;

    virtual TheISA::PCState pcState() = 0;

    virtual void pcState(const TheISA::PCState &val) = 0;

    void
    setNPC(Addr val)
    {
        TheISA::PCState pc_state = pcState();
        pc_state.setNPC(val);
        pcState(pc_state);
    }

    virtual void pcStateNoRecord(const TheISA::PCState &val) = 0;

    virtual Addr instAddr() = 0;

    virtual Addr nextInstAddr() = 0;

    virtual MicroPC microPC() = 0;

    virtual RegVal readMiscRegNoEffect(int misc_reg) const = 0;

    virtual RegVal readMiscReg(int misc_reg) = 0;

    virtual void setMiscRegNoEffect(int misc_reg, RegVal val) = 0;

    virtual void setMiscReg(int misc_reg, RegVal val) = 0;

    virtual RegId flattenRegId(const RegId& regId) const = 0;

    virtual RegVal
    readRegOtherThread(const RegId& misc_reg, ThreadID tid)
    {
        return 0;
    }

    virtual void
    setRegOtherThread(const RegId& misc_reg, RegVal val, ThreadID tid)
    {
    }

    // Also not necessarily the best location for these two.  Hopefully will go
    // away once we decide upon where st cond failures goes.
    virtual unsigned readStCondFailures() = 0;

    virtual void setStCondFailures(unsigned sc_failures) = 0;

    // Same with st cond failures.
    virtual Counter readFuncExeInst() = 0;

    virtual void syscall(int64_t callnum, Fault *fault) = 0;

    // This function exits the thread context in the CPU and returns
    // 1 if the CPU has no more active threads (meaning it's OK to exit);
    // Used in syscall-emulation mode when a  thread calls the exit syscall.
    virtual int exit() { return 1; };

    /** function to compare two thread contexts (for debugging) */
    static void compare(ThreadContext *one, ThreadContext *two);

    /** @{ */
    /**
     * Flat register interfaces
     *
     * Some architectures have different registers visible in
     * different modes. Such architectures "flatten" a register (see
     * flattenRegId()) to map it into the
     * gem5 register file. This interface provides a flat interface to
     * the underlying register file, which allows for example
     * serialization code to access all registers.
     */

    virtual RegVal readIntRegFlat(int idx) = 0;
    virtual void setIntRegFlat(int idx, RegVal val) = 0;

    virtual RegVal readFloatRegFlat(int idx) = 0;
    virtual void setFloatRegFlat(int idx, RegVal val) = 0;

    virtual const VecRegContainer& readVecRegFlat(int idx) const = 0;
    virtual VecRegContainer& getWritableVecRegFlat(int idx) = 0;
    virtual void setVecRegFlat(int idx, const VecRegContainer& val) = 0;

    virtual const VecElem& readVecElemFlat(const RegIndex& idx,
                                           const ElemIndex& elemIdx) const = 0;
    virtual void setVecElemFlat(const RegIndex& idx, const ElemIndex& elemIdx,
                                const VecElem& val) = 0;

    virtual const VecPredRegContainer& readVecPredRegFlat(int idx) const = 0;
    virtual VecPredRegContainer& getWritableVecPredRegFlat(int idx) = 0;
    virtual void setVecPredRegFlat(int idx,
                                   const VecPredRegContainer& val) = 0;

    virtual RegVal readCCRegFlat(int idx) = 0;
    virtual void setCCRegFlat(int idx, RegVal val) = 0;
    /** @} */

};

/**
 * ProxyThreadContext class that provides a way to implement a
 * ThreadContext without having to derive from it. ThreadContext is an
 * abstract class, so anything that derives from it and uses its
 * interface will pay the overhead of virtual function calls.  This
 * class is created to enable a user-defined Thread object to be used
 * wherever ThreadContexts are used, without paying the overhead of
 * virtual function calls when it is used by itself.  See
 * simple_thread.hh for an example of this.
 */
template <class TC>
class ProxyThreadContext : public ThreadContext
{
  public:
    ProxyThreadContext(TC *actual_tc)
    { actualTC = actual_tc; }

  private:
    TC *actualTC;

  public:

    BaseCPU *getCpuPtr() { return actualTC->getCpuPtr(); }

    int cpuId() const { return actualTC->cpuId(); }

    uint32_t socketId() const { return actualTC->socketId(); }

    int threadId() const { return actualTC->threadId(); }

    void setThreadId(int id) { actualTC->setThreadId(id); }

    int contextId() const { return actualTC->contextId(); }

    void setContextId(int id) { actualTC->setContextId(id); }

    BaseTLB *getITBPtr() { return actualTC->getITBPtr(); }

    BaseTLB *getDTBPtr() { return actualTC->getDTBPtr(); }

    CheckerCPU *getCheckerCpuPtr() { return actualTC->getCheckerCpuPtr(); }

    TheISA::ISA *getIsaPtr() { return actualTC->getIsaPtr(); }

    TheISA::Decoder *getDecoderPtr() { return actualTC->getDecoderPtr(); }

    System *getSystemPtr() { return actualTC->getSystemPtr(); }

    TheISA::Kernel::Statistics *getKernelStats()
    { return actualTC->getKernelStats(); }

    PortProxy &getPhysProxy() { return actualTC->getPhysProxy(); }

    FSTranslatingPortProxy &getVirtProxy() { return actualTC->getVirtProxy(); }

    void initMemProxies(ThreadContext *tc) { actualTC->initMemProxies(tc); }

    SETranslatingPortProxy &getMemProxy() { return actualTC->getMemProxy(); }

    Process *getProcessPtr() { return actualTC->getProcessPtr(); }

    void setProcessPtr(Process *p) { actualTC->setProcessPtr(p); }

    Status status() const { return actualTC->status(); }

    void setStatus(Status new_status) { actualTC->setStatus(new_status); }

    /// Set the status to Active.
    void activate() { actualTC->activate(); }

    /// Set the status to Suspended.
    void suspend() { actualTC->suspend(); }

    /// Set the status to Halted.
    void halt() { actualTC->halt(); }

    /// Quiesce thread context
    void quiesce() { actualTC->quiesce(); }

    /// Quiesce, suspend, and schedule activate at resume
    void quiesceTick(Tick resume) { actualTC->quiesceTick(resume); }

    void dumpFuncProfile() { actualTC->dumpFuncProfile(); }

    void takeOverFrom(ThreadContext *oldContext)
    { actualTC->takeOverFrom(oldContext); }

    void regStats(const std::string &name) { actualTC->regStats(name); }

    EndQuiesceEvent *getQuiesceEvent() { return actualTC->getQuiesceEvent(); }

    Tick readLastActivate() { return actualTC->readLastActivate(); }
    Tick readLastSuspend() { return actualTC->readLastSuspend(); }

    void profileClear() { return actualTC->profileClear(); }
    void profileSample() { return actualTC->profileSample(); }

    // @todo: Do I need this?
    void copyArchRegs(ThreadContext *tc) { actualTC->copyArchRegs(tc); }

    void clearArchRegs() { actualTC->clearArchRegs(); }

    //
    // New accessors for new decoder.
    //
    RegVal readIntReg(int reg_idx)
    { return actualTC->readIntReg(reg_idx); }

    RegVal readFloatReg(int reg_idx)
    { return actualTC->readFloatReg(reg_idx); }

    const VecRegContainer& readVecReg(const RegId& reg) const
    { return actualTC->readVecReg(reg); }

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

    const VecPredRegContainer& readVecPredReg(const RegId& reg) const
    { return actualTC->readVecPredReg(reg); }

    VecPredRegContainer& getWritableVecPredReg(const RegId& reg)
    { return actualTC->getWritableVecPredReg(reg); }

    RegVal readCCReg(int reg_idx)
    { return actualTC->readCCReg(reg_idx); }

    void setIntReg(int reg_idx, RegVal val)
    { actualTC->setIntReg(reg_idx, val); }

    void setFloatReg(int reg_idx, RegVal val)
    { actualTC->setFloatReg(reg_idx, val); }

    void setVecReg(const RegId& reg, const VecRegContainer& val)
    { actualTC->setVecReg(reg, val); }

    void setVecPredReg(const RegId& reg, const VecPredRegContainer& val)
    { actualTC->setVecPredReg(reg, val); }

    void setVecElem(const RegId& reg, const VecElem& val)
    { actualTC->setVecElem(reg, val); }

    void setCCReg(int reg_idx, RegVal val)
    { actualTC->setCCReg(reg_idx, val); }

    TheISA::PCState pcState() { return actualTC->pcState(); }

    void pcState(const TheISA::PCState &val) { actualTC->pcState(val); }

    void pcStateNoRecord(const TheISA::PCState &val) { actualTC->pcState(val); }

    Addr instAddr() { return actualTC->instAddr(); }
    Addr nextInstAddr() { return actualTC->nextInstAddr(); }
    MicroPC microPC() { return actualTC->microPC(); }

    bool readPredicate() { return actualTC->readPredicate(); }

    void setPredicate(bool val)
    { actualTC->setPredicate(val); }

    RegVal readMiscRegNoEffect(int misc_reg) const
    { return actualTC->readMiscRegNoEffect(misc_reg); }

    RegVal readMiscReg(int misc_reg)
    { return actualTC->readMiscReg(misc_reg); }

    void setMiscRegNoEffect(int misc_reg, RegVal val)
    { return actualTC->setMiscRegNoEffect(misc_reg, val); }

    void setMiscReg(int misc_reg, RegVal val)
    { return actualTC->setMiscReg(misc_reg, val); }

    RegId flattenRegId(const RegId& regId) const
    { return actualTC->flattenRegId(regId); }

    unsigned readStCondFailures()
    { return actualTC->readStCondFailures(); }

    void setStCondFailures(unsigned sc_failures)
    { actualTC->setStCondFailures(sc_failures); }

    void syscall(int64_t callnum, Fault *fault)
    { actualTC->syscall(callnum, fault); }

    Counter readFuncExeInst() { return actualTC->readFuncExeInst(); }

    RegVal readIntRegFlat(int idx)
    { return actualTC->readIntRegFlat(idx); }

    void setIntRegFlat(int idx, RegVal val)
    { actualTC->setIntRegFlat(idx, val); }

    RegVal readFloatRegFlat(int idx)
    { return actualTC->readFloatRegFlat(idx); }

    void setFloatRegFlat(int idx, RegVal val)
    { actualTC->setFloatRegFlat(idx, val); }

    const VecRegContainer& readVecRegFlat(int id) const
    { return actualTC->readVecRegFlat(id); }

    VecRegContainer& getWritableVecRegFlat(int id)
    { return actualTC->getWritableVecRegFlat(id); }

    void setVecRegFlat(int idx, const VecRegContainer& val)
    { actualTC->setVecRegFlat(idx, val); }

    const VecElem& readVecElemFlat(const RegIndex& id,
                                   const ElemIndex& elemIndex) const
    { return actualTC->readVecElemFlat(id, elemIndex); }

    void setVecElemFlat(const RegIndex& id, const ElemIndex& elemIndex,
                        const VecElem& val)
    { actualTC->setVecElemFlat(id, elemIndex, val); }

    const VecPredRegContainer& readVecPredRegFlat(int id) const
    { return actualTC->readVecPredRegFlat(id); }

    VecPredRegContainer& getWritableVecPredRegFlat(int id)
    { return actualTC->getWritableVecPredRegFlat(id); }

    void setVecPredRegFlat(int idx, const VecPredRegContainer& val)
    { actualTC->setVecPredRegFlat(idx, val); }

    RegVal readCCRegFlat(int idx)
    { return actualTC->readCCRegFlat(idx); }

    void setCCRegFlat(int idx, RegVal val)
    { actualTC->setCCRegFlat(idx, val); }
};

/** @{ */
/**
 * Thread context serialization helpers
 *
 * These helper functions provide a way to the data in a
 * ThreadContext. They are provided as separate helper function since
 * implementing them as members of the ThreadContext interface would
 * be confusing when the ThreadContext is exported via a proxy.
 */

void serialize(ThreadContext &tc, CheckpointOut &cp);
void unserialize(ThreadContext &tc, CheckpointIn &cp);

/** @} */


/**
 * Copy state between thread contexts in preparation for CPU handover.
 *
 * @note This method modifies the old thread contexts as well as the
 * new thread context. The old thread context will have its quiesce
 * event descheduled if it is scheduled and its status set to halted.
 *
 * @param new_tc Destination ThreadContext.
 * @param old_tc Source ThreadContext.
 */
void takeOverFrom(ThreadContext &new_tc, ThreadContext &old_tc);

#endif
