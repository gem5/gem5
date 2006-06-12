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

#ifndef __CPU_THREAD_CONTEXT_HH__
#define __CPU_THREAD_CONTEXT_HH__

#include "config/full_system.hh"
#include "mem/request.hh"
#include "sim/faults.hh"
#include "sim/host.hh"
#include "sim/serialize.hh"
#include "sim/byteswap.hh"

// @todo: Figure out a more architecture independent way to obtain the ITB and
// DTB pointers.
class AlphaDTB;
class AlphaITB;
class BaseCPU;
class EndQuiesceEvent;
class Event;
class TranslatingPort;
class FunctionalPort;
class VirtualPort;
class Process;
class System;
namespace Kernel {
    class Statistics;
};

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
    typedef TheISA::RegFile RegFile;
    typedef TheISA::MachInst MachInst;
    typedef TheISA::IntReg IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::MiscRegFile MiscRegFile;
    typedef TheISA::MiscReg MiscReg;
  public:
    enum Status
    {
        /// Initialized but not running yet.  All CPUs start in
        /// this state, but most transition to Active on cycle 1.
        /// In MP or SMT systems, non-primary contexts will stay
        /// in this state until a thread is assigned to them.
        Unallocated,

        /// Running.  Instructions should be executed only when
        /// the context is in this state.
        Active,

        /// Temporarily inactive.  Entered while waiting for
        /// synchronization, etc.
        Suspended,

        /// Permanently shut down.  Entered when target executes
        /// m5exit pseudo-instruction.  When all contexts enter
        /// this state, the simulation will terminate.
        Halted
    };

    virtual ~ThreadContext() { };

    virtual BaseCPU *getCpuPtr() = 0;

    virtual void setCpuId(int id) = 0;

    virtual int readCpuId() = 0;

#if FULL_SYSTEM
    virtual System *getSystemPtr() = 0;

    virtual AlphaITB *getITBPtr() = 0;

    virtual AlphaDTB * getDTBPtr() = 0;

    virtual Kernel::Statistics *getKernelStats() = 0;

    virtual FunctionalPort *getPhysPort() = 0;

    virtual VirtualPort *getVirtPort(ThreadContext *tc = NULL) = 0;

    virtual void delVirtPort(VirtualPort *vp) = 0;
#else
    virtual TranslatingPort *getMemPort() = 0;

    virtual Process *getProcessPtr() = 0;
#endif

    virtual Status status() const = 0;

    virtual void setStatus(Status new_status) = 0;

    /// Set the status to Active.  Optional delay indicates number of
    /// cycles to wait before beginning execution.
    virtual void activate(int delay = 1) = 0;

    /// Set the status to Suspended.
    virtual void suspend() = 0;

    /// Set the status to Unallocated.
    virtual void deallocate() = 0;

    /// Set the status to Halted.
    virtual void halt() = 0;

#if FULL_SYSTEM
    virtual void dumpFuncProfile() = 0;
#endif

    virtual void takeOverFrom(ThreadContext *old_context) = 0;

    virtual void regStats(const std::string &name) = 0;

    virtual void serialize(std::ostream &os) = 0;
    virtual void unserialize(Checkpoint *cp, const std::string &section) = 0;

#if FULL_SYSTEM
    virtual EndQuiesceEvent *getQuiesceEvent() = 0;

    // Not necessarily the best location for these...
    // Having an extra function just to read these is obnoxious
    virtual Tick readLastActivate() = 0;
    virtual Tick readLastSuspend() = 0;

    virtual void profileClear() = 0;
    virtual void profileSample() = 0;
#endif

    virtual int getThreadNum() = 0;

    // Also somewhat obnoxious.  Really only used for the TLB fault.
    // However, may be quite useful in SPARC.
    virtual TheISA::MachInst getInst() = 0;

    virtual void copyArchRegs(ThreadContext *tc) = 0;

    virtual void clearArchRegs() = 0;

    //
    // New accessors for new decoder.
    //
    virtual uint64_t readIntReg(int reg_idx) = 0;

    virtual FloatReg readFloatReg(int reg_idx, int width) = 0;

    virtual FloatReg readFloatReg(int reg_idx) = 0;

    virtual FloatRegBits readFloatRegBits(int reg_idx, int width) = 0;

    virtual FloatRegBits readFloatRegBits(int reg_idx) = 0;

    virtual void setIntReg(int reg_idx, uint64_t val) = 0;

    virtual void setFloatReg(int reg_idx, FloatReg val, int width) = 0;

    virtual void setFloatReg(int reg_idx, FloatReg val) = 0;

    virtual void setFloatRegBits(int reg_idx, FloatRegBits val) = 0;

    virtual void setFloatRegBits(int reg_idx, FloatRegBits val, int width) = 0;

    virtual uint64_t readPC() = 0;

    virtual void setPC(uint64_t val) = 0;

    virtual uint64_t readNextPC() = 0;

    virtual void setNextPC(uint64_t val) = 0;

    virtual uint64_t readNextNPC() = 0;

    virtual void setNextNPC(uint64_t val) = 0;

    virtual MiscReg readMiscReg(int misc_reg) = 0;

    virtual MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault) = 0;

    virtual Fault setMiscReg(int misc_reg, const MiscReg &val) = 0;

    virtual Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val) = 0;

    // Also not necessarily the best location for these two.  Hopefully will go
    // away once we decide upon where st cond failures goes.
    virtual unsigned readStCondFailures() = 0;

    virtual void setStCondFailures(unsigned sc_failures) = 0;

#if FULL_SYSTEM
    virtual bool inPalMode() = 0;
#endif

    // Only really makes sense for old CPU model.  Still could be useful though.
    virtual bool misspeculating() = 0;

#if !FULL_SYSTEM
    virtual IntReg getSyscallArg(int i) = 0;

    // used to shift args for indirect syscall
    virtual void setSyscallArg(int i, IntReg val) = 0;

    virtual void setSyscallReturn(SyscallReturn return_value) = 0;

    // Same with st cond failures.
    virtual Counter readFuncExeInst() = 0;
#endif

    virtual void changeRegFileContext(RegFile::ContextParam param,
            RegFile::ContextVal val) = 0;
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

    void setCpuId(int id) { actualTC->setCpuId(id); }

    int readCpuId() { return actualTC->readCpuId(); }

#if FULL_SYSTEM
    System *getSystemPtr() { return actualTC->getSystemPtr(); }

    AlphaITB *getITBPtr() { return actualTC->getITBPtr(); }

    AlphaDTB *getDTBPtr() { return actualTC->getDTBPtr(); }

    Kernel::Statistics *getKernelStats() { return actualTC->getKernelStats(); }

    FunctionalPort *getPhysPort() { return actualTC->getPhysPort(); }

    VirtualPort *getVirtPort(ThreadContext *tc = NULL) { return actualTC->getVirtPort(tc); }

    void delVirtPort(VirtualPort *vp) { return actualTC->delVirtPort(vp); }
#else
    TranslatingPort *getMemPort() { return actualTC->getMemPort(); }

    Process *getProcessPtr() { return actualTC->getProcessPtr(); }
#endif

    Status status() const { return actualTC->status(); }

    void setStatus(Status new_status) { actualTC->setStatus(new_status); }

    /// Set the status to Active.  Optional delay indicates number of
    /// cycles to wait before beginning execution.
    void activate(int delay = 1) { actualTC->activate(delay); }

    /// Set the status to Suspended.
    void suspend() { actualTC->suspend(); }

    /// Set the status to Unallocated.
    void deallocate() { actualTC->deallocate(); }

    /// Set the status to Halted.
    void halt() { actualTC->halt(); }

#if FULL_SYSTEM
    void dumpFuncProfile() { actualTC->dumpFuncProfile(); }
#endif

    void takeOverFrom(ThreadContext *oldContext)
    { actualTC->takeOverFrom(oldContext); }

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
    void copyArchRegs(ThreadContext *tc) { actualTC->copyArchRegs(tc); }

    void clearArchRegs() { actualTC->clearArchRegs(); }

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
    { actualTC->setIntReg(reg_idx, val); }

    void setFloatReg(int reg_idx, FloatReg val, int width)
    { actualTC->setFloatReg(reg_idx, val, width); }

    void setFloatReg(int reg_idx, FloatReg val)
    { actualTC->setFloatReg(reg_idx, val); }

    void setFloatRegBits(int reg_idx, FloatRegBits val, int width)
    { actualTC->setFloatRegBits(reg_idx, val, width); }

    void setFloatRegBits(int reg_idx, FloatRegBits val)
    { actualTC->setFloatRegBits(reg_idx, val); }

    uint64_t readPC() { return actualTC->readPC(); }

    void setPC(uint64_t val) { actualTC->setPC(val); }

    uint64_t readNextPC() { return actualTC->readNextPC(); }

    void setNextPC(uint64_t val) { actualTC->setNextPC(val); }

    uint64_t readNextNPC() { return actualTC->readNextNPC(); }

    void setNextNPC(uint64_t val) { actualTC->setNextNPC(val); }

    MiscReg readMiscReg(int misc_reg)
    { return actualTC->readMiscReg(misc_reg); }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    { return actualTC->readMiscRegWithEffect(misc_reg, fault); }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    { return actualTC->setMiscReg(misc_reg, val); }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    { return actualTC->setMiscRegWithEffect(misc_reg, val); }

    unsigned readStCondFailures()
    { return actualTC->readStCondFailures(); }

    void setStCondFailures(unsigned sc_failures)
    { actualTC->setStCondFailures(sc_failures); }
#if FULL_SYSTEM
    bool inPalMode() { return actualTC->inPalMode(); }
#endif

    // @todo: Fix this!
    bool misspeculating() { return actualTC->misspeculating(); }

#if !FULL_SYSTEM
    IntReg getSyscallArg(int i) { return actualTC->getSyscallArg(i); }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, IntReg val)
    { actualTC->setSyscallArg(i, val); }

    void setSyscallReturn(SyscallReturn return_value)
    { actualTC->setSyscallReturn(return_value); }

    Counter readFuncExeInst() { return actualTC->readFuncExeInst(); }
#endif

    void changeRegFileContext(RegFile::ContextParam param,
            RegFile::ContextVal val)
    {
        actualTC->changeRegFileContext(param, val);
    }
};

#endif
