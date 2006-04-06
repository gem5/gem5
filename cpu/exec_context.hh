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
 */

#ifndef __CPU_EXEC_CONTEXT_HH__
#define __CPU_EXEC_CONTEXT_HH__

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
class Event;
class TranslatingPort;
class FunctionalPort;
class VirtualPort;
class Process;
class System;

class ExecContext
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

    virtual ~ExecContext() { };

    virtual BaseCPU *getCpuPtr() = 0;

    virtual void setCpuId(int id) = 0;

    virtual int readCpuId() = 0;

#if FULL_SYSTEM
    virtual System *getSystemPtr() = 0;

    virtual AlphaITB *getITBPtr() = 0;

    virtual AlphaDTB * getDTBPtr() = 0;

    virtual FunctionalPort *getPhysPort() = 0;

    virtual VirtualPort *getVirtPort(ExecContext *xc = NULL) = 0;

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

    virtual void takeOverFrom(ExecContext *old_context) = 0;

    virtual void regStats(const std::string &name) = 0;

    virtual void serialize(std::ostream &os) = 0;
    virtual void unserialize(Checkpoint *cp, const std::string &section) = 0;

#if FULL_SYSTEM
    virtual Event *getQuiesceEvent() = 0;

    // Not necessarily the best location for these...
    // Having an extra function just to read these is obnoxious
    virtual Tick readLastActivate() = 0;
    virtual Tick readLastSuspend() = 0;

    virtual void profileClear() = 0;
    virtual void profileSample() = 0;
#endif

    virtual int getThreadNum() = 0;

    virtual int getInstAsid() = 0;
    virtual int getDataAsid() = 0;

    virtual Fault translateInstReq(CpuRequestPtr &req) = 0;

    virtual Fault translateDataReadReq(CpuRequestPtr &req) = 0;

    virtual Fault translateDataWriteReq(CpuRequestPtr &req) = 0;

    // Also somewhat obnoxious.  Really only used for the TLB fault.
    // However, may be quite useful in SPARC.
    virtual TheISA::MachInst getInst() = 0;

    virtual void copyArchRegs(ExecContext *xc) = 0;

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
    virtual int readIntrFlag() = 0;
    virtual void setIntrFlag(int val) = 0;
    virtual Fault hwrei() = 0;
    virtual bool inPalMode() = 0;
    virtual bool simPalCheck(int palFunc) = 0;
#endif

    // Only really makes sense for old CPU model.  Still could be useful though.
    virtual bool misspeculating() = 0;

#if !FULL_SYSTEM
    virtual IntReg getSyscallArg(int i) = 0;

    // used to shift args for indirect syscall
    virtual void setSyscallArg(int i, IntReg val) = 0;

    virtual void setSyscallReturn(SyscallReturn return_value) = 0;

    virtual void syscall() = 0;

    // Same with st cond failures.
    virtual Counter readFuncExeInst() = 0;

    virtual void setFuncExeInst(Counter new_val) = 0;
#endif

    virtual void changeRegFileContext(RegFile::ContextParam param,
            RegFile::ContextVal val) = 0;
};

template <class XC>
class ProxyExecContext : public ExecContext
{
  public:
    ProxyExecContext(XC *actual_xc)
    { actualXC = actual_xc; }

  private:
    XC *actualXC;

  public:

    BaseCPU *getCpuPtr() { return actualXC->getCpuPtr(); }

    void setCpuId(int id) { actualXC->setCpuId(id); }

    int readCpuId() { return actualXC->readCpuId(); }

#if FULL_SYSTEM
    System *getSystemPtr() { return actualXC->getSystemPtr(); }

    AlphaITB *getITBPtr() { return actualXC->getITBPtr(); }

    AlphaDTB *getDTBPtr() { return actualXC->getDTBPtr(); }

    FunctionalPort *getPhysPort() { return actualXC->getPhysPort(); }

    VirtualPort *getVirtPort(ExecContext *xc = NULL) { return actualXC->getVirtPort(xc); }

    void delVirtPort(VirtualPort *vp) { return actualXC->delVirtPort(vp); }
#else
    TranslatingPort *getMemPort() { return actualXC->getMemPort(); }

    Process *getProcessPtr() { return actualXC->getProcessPtr(); }
#endif

    Status status() const { return actualXC->status(); }

    void setStatus(Status new_status) { actualXC->setStatus(new_status); }

    /// Set the status to Active.  Optional delay indicates number of
    /// cycles to wait before beginning execution.
    void activate(int delay = 1) { actualXC->activate(delay); }

    /// Set the status to Suspended.
    void suspend() { actualXC->suspend(); }

    /// Set the status to Unallocated.
    void deallocate() { actualXC->deallocate(); }

    /// Set the status to Halted.
    void halt() { actualXC->halt(); }

#if FULL_SYSTEM
    void dumpFuncProfile() { actualXC->dumpFuncProfile(); }
#endif

    void takeOverFrom(ExecContext *oldContext)
    { actualXC->takeOverFrom(oldContext); }

    void regStats(const std::string &name) { actualXC->regStats(name); }

    void serialize(std::ostream &os) { actualXC->serialize(os); }
    void unserialize(Checkpoint *cp, const std::string &section)
    { actualXC->unserialize(cp, section); }

#if FULL_SYSTEM
    Event *getQuiesceEvent() { return actualXC->getQuiesceEvent(); }

    Tick readLastActivate() { return actualXC->readLastActivate(); }
    Tick readLastSuspend() { return actualXC->readLastSuspend(); }

    void profileClear() { return actualXC->profileClear(); }
    void profileSample() { return actualXC->profileSample(); }
#endif

    int getThreadNum() { return actualXC->getThreadNum(); }

    int getInstAsid() { return actualXC->getInstAsid(); }
    int getDataAsid() { return actualXC->getDataAsid(); }

    Fault translateInstReq(CpuRequestPtr &req)
    { return actualXC->translateInstReq(req); }

    Fault translateDataReadReq(CpuRequestPtr &req)
    { return actualXC->translateDataReadReq(req); }

    Fault translateDataWriteReq(CpuRequestPtr &req)
    { return actualXC->translateDataWriteReq(req); }

    // @todo: Do I need this?
    MachInst getInst() { return actualXC->getInst(); }

    // @todo: Do I need this?
    void copyArchRegs(ExecContext *xc) { actualXC->copyArchRegs(xc); }

    void clearArchRegs() { actualXC->clearArchRegs(); }

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx)
    { return actualXC->readIntReg(reg_idx); }

    FloatReg readFloatReg(int reg_idx, int width)
    { return actualXC->readFloatReg(reg_idx, width); }

    FloatReg readFloatReg(int reg_idx)
    { return actualXC->readFloatReg(reg_idx); }

    FloatRegBits readFloatRegBits(int reg_idx, int width)
    { return actualXC->readFloatRegBits(reg_idx, width); }

    FloatRegBits readFloatRegBits(int reg_idx)
    { return actualXC->readFloatRegBits(reg_idx); }

    void setIntReg(int reg_idx, uint64_t val)
    { actualXC->setIntReg(reg_idx, val); }

    void setFloatReg(int reg_idx, FloatReg val, int width)
    { actualXC->setFloatReg(reg_idx, val, width); }

    void setFloatReg(int reg_idx, FloatReg val)
    { actualXC->setFloatReg(reg_idx, val); }

    void setFloatRegBits(int reg_idx, FloatRegBits val, int width)
    { actualXC->setFloatRegBits(reg_idx, val, width); }

    void setFloatRegBits(int reg_idx, FloatRegBits val)
    { actualXC->setFloatRegBits(reg_idx, val); }

    uint64_t readPC() { return actualXC->readPC(); }

    void setPC(uint64_t val) { actualXC->setPC(val); }

    uint64_t readNextPC() { return actualXC->readNextPC(); }

    void setNextPC(uint64_t val) { actualXC->setNextPC(val); }

    uint64_t readNextNPC() { return actualXC->readNextNPC(); }

    void setNextNPC(uint64_t val) { actualXC->setNextNPC(val); }

    MiscReg readMiscReg(int misc_reg)
    { return actualXC->readMiscReg(misc_reg); }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    { return actualXC->readMiscRegWithEffect(misc_reg, fault); }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    { return actualXC->setMiscReg(misc_reg, val); }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    { return actualXC->setMiscRegWithEffect(misc_reg, val); }

    unsigned readStCondFailures()
    { return actualXC->readStCondFailures(); }

    void setStCondFailures(unsigned sc_failures)
    { actualXC->setStCondFailures(sc_failures); }

#if FULL_SYSTEM
    int readIntrFlag() { return actualXC->readIntrFlag(); }

    void setIntrFlag(int val) { actualXC->setIntrFlag(val); }

    Fault hwrei() { return actualXC->hwrei(); }

    bool inPalMode() { return actualXC->inPalMode(); }

    bool simPalCheck(int palFunc) { return actualXC->simPalCheck(palFunc); }
#endif

    // @todo: Fix this!
    bool misspeculating() { return actualXC->misspeculating(); }

#if !FULL_SYSTEM
    IntReg getSyscallArg(int i) { return actualXC->getSyscallArg(i); }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, IntReg val)
    { actualXC->setSyscallArg(i, val); }

    void setSyscallReturn(SyscallReturn return_value)
    { actualXC->setSyscallReturn(return_value); }

    void syscall() { actualXC->syscall(); }

    Counter readFuncExeInst() { return actualXC->readFuncExeInst(); }

    void setFuncExeInst(Counter new_val)
    { return actualXC->setFuncExeInst(new_val); }
#endif

    void changeRegFileContext(RegFile::ContextParam param,
            RegFile::ContextVal val)
    {
        actualXC->changeRegFileContext(param, val);
    }
};

#endif
