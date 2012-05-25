/*
 * Copyright (c) 2011 ARM Limited
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
 *
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#ifndef __CPU_SIMPLE_THREAD_HH__
#define __CPU_SIMPLE_THREAD_HH__

#include "arch/decoder.hh"
#include "arch/isa.hh"
#include "arch/isa_traits.hh"
#include "arch/registers.hh"
#include "arch/tlb.hh"
#include "arch/types.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "debug/FloatRegs.hh"
#include "debug/IntRegs.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/eventq.hh"
#include "sim/process.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

class BaseCPU;
class CheckerCPU;

class FunctionProfile;
class ProfileNode;

namespace TheISA {
    namespace Kernel {
        class Statistics;
    }
}

/**
 * The SimpleThread object provides a combination of the ThreadState
 * object and the ThreadContext interface. It implements the
 * ThreadContext interface so that a ProxyThreadContext class can be
 * made using SimpleThread as the template parameter (see
 * thread_context.hh). It adds to the ThreadState object by adding all
 * the objects needed for simple functional execution, including a
 * simple architectural register file, and pointers to the ITB and DTB
 * in full system mode. For CPU models that do not need more advanced
 * ways to hold state (i.e. a separate physical register file, or
 * separate fetch and commit PC's), this SimpleThread class provides
 * all the necessary state for full architecture-level functional
 * simulation.  See the AtomicSimpleCPU or TimingSimpleCPU for
 * examples.
 */

class SimpleThread : public ThreadState
{
  protected:
    typedef TheISA::MachInst MachInst;
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
  public:
    typedef ThreadContext::Status Status;

  protected:
    union {
        FloatReg f[TheISA::NumFloatRegs];
        FloatRegBits i[TheISA::NumFloatRegs];
    } floatRegs;
    TheISA::IntReg intRegs[TheISA::NumIntRegs];
    TheISA::ISA isa;    // one "instance" of the current ISA.

    TheISA::PCState _pcState;

    /** Did this instruction execute or is it predicated false */
    bool predicate;

  public:
    std::string name() const
    {
        return csprintf("%s.[tid:%i]", baseCpu->name(), tc->threadId());
    }

    ProxyThreadContext<SimpleThread> *tc;

    System *system;

    TheISA::TLB *itb;
    TheISA::TLB *dtb;

    TheISA::Decoder decoder;

    // constructor: initialize SimpleThread from given process structure
    // FS
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 TheISA::TLB *_itb, TheISA::TLB *_dtb,
                 bool use_kernel_stats = true);
    // SE
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 Process *_process, TheISA::TLB *_itb, TheISA::TLB *_dtb);

    SimpleThread();

    virtual ~SimpleThread();

    virtual void takeOverFrom(ThreadContext *oldContext);

    void regStats(const std::string &name);

    void copyTC(ThreadContext *context);

    void copyState(ThreadContext *oldContext);

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

    /***************************************************************
     *  SimpleThread functions to provide CPU with access to various
     *  state.
     **************************************************************/

    /** Returns the pointer to this SimpleThread's ThreadContext. Used
     *  when a ThreadContext must be passed to objects outside of the
     *  CPU.
     */
    ThreadContext *getTC() { return tc; }

    void demapPage(Addr vaddr, uint64_t asn)
    {
        itb->demapPage(vaddr, asn);
        dtb->demapPage(vaddr, asn);
    }

    void demapInstPage(Addr vaddr, uint64_t asn)
    {
        itb->demapPage(vaddr, asn);
    }

    void demapDataPage(Addr vaddr, uint64_t asn)
    {
        dtb->demapPage(vaddr, asn);
    }

    void dumpFuncProfile();

    Fault hwrei();

    bool simPalCheck(int palFunc);

    /*******************************************
     * ThreadContext interface functions.
     ******************************************/

    BaseCPU *getCpuPtr() { return baseCpu; }

    TheISA::TLB *getITBPtr() { return itb; }

    TheISA::TLB *getDTBPtr() { return dtb; }

    CheckerCPU *getCheckerCpuPtr() { return NULL; }

    TheISA::Decoder *getDecoderPtr() { return &decoder; }

    System *getSystemPtr() { return system; }

    Status status() const { return _status; }

    void setStatus(Status newStatus) { _status = newStatus; }

    /// Set the status to Active.  Optional delay indicates number of
    /// cycles to wait before beginning execution.
    void activate(int delay = 1);

    /// Set the status to Suspended.
    void suspend();

    /// Set the status to Halted.
    void halt();

    virtual bool misspeculating();

    void copyArchRegs(ThreadContext *tc);

    void clearArchRegs()
    {
        _pcState = 0;
        memset(intRegs, 0, sizeof(intRegs));
        memset(floatRegs.i, 0, sizeof(floatRegs.i));
        isa.clear();
    }

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx)
    {
        int flatIndex = isa.flattenIntIndex(reg_idx);
        assert(flatIndex < TheISA::NumIntRegs);
        uint64_t regVal = intRegs[flatIndex];
        DPRINTF(IntRegs, "Reading int reg %d (%d) as %#x.\n",
                reg_idx, flatIndex, regVal);
        return regVal;
    }

    FloatReg readFloatReg(int reg_idx)
    {
        int flatIndex = isa.flattenFloatIndex(reg_idx);
        assert(flatIndex < TheISA::NumFloatRegs);
        FloatReg regVal = floatRegs.f[flatIndex];
        DPRINTF(FloatRegs, "Reading float reg %d (%d) as %f, %#x.\n",
                reg_idx, flatIndex, regVal, floatRegs.i[flatIndex]);
        return regVal;
    }

    FloatRegBits readFloatRegBits(int reg_idx)
    {
        int flatIndex = isa.flattenFloatIndex(reg_idx);
        assert(flatIndex < TheISA::NumFloatRegs);
        FloatRegBits regVal = floatRegs.i[flatIndex];
        DPRINTF(FloatRegs, "Reading float reg %d (%d) bits as %#x, %f.\n",
                reg_idx, flatIndex, regVal, floatRegs.f[flatIndex]);
        return regVal;
    }

    void setIntReg(int reg_idx, uint64_t val)
    {
        int flatIndex = isa.flattenIntIndex(reg_idx);
        assert(flatIndex < TheISA::NumIntRegs);
        DPRINTF(IntRegs, "Setting int reg %d (%d) to %#x.\n",
                reg_idx, flatIndex, val);
        intRegs[flatIndex] = val;
    }

    void setFloatReg(int reg_idx, FloatReg val)
    {
        int flatIndex = isa.flattenFloatIndex(reg_idx);
        assert(flatIndex < TheISA::NumFloatRegs);
        floatRegs.f[flatIndex] = val;
        DPRINTF(FloatRegs, "Setting float reg %d (%d) to %f, %#x.\n",
                reg_idx, flatIndex, val, floatRegs.i[flatIndex]);
    }

    void setFloatRegBits(int reg_idx, FloatRegBits val)
    {
        int flatIndex = isa.flattenFloatIndex(reg_idx);
        assert(flatIndex < TheISA::NumFloatRegs);
        // XXX: Fix array out of bounds compiler error for gem5.fast
        // when checkercpu enabled
        if (flatIndex < TheISA::NumFloatRegs)
            floatRegs.i[flatIndex] = val;
        DPRINTF(FloatRegs, "Setting float reg %d (%d) bits to %#x, %#f.\n",
                reg_idx, flatIndex, val, floatRegs.f[flatIndex]);
    }

    TheISA::PCState
    pcState()
    {
        return _pcState;
    }

    void
    pcState(const TheISA::PCState &val)
    {
        _pcState = val;
    }

    void
    pcStateNoRecord(const TheISA::PCState &val)
    {
        _pcState = val;
    }

    Addr
    instAddr()
    {
        return _pcState.instAddr();
    }

    Addr
    nextInstAddr()
    {
        return _pcState.nextInstAddr();
    }

    MicroPC
    microPC()
    {
        return _pcState.microPC();
    }

    bool readPredicate()
    {
        return predicate;
    }

    void setPredicate(bool val)
    {
        predicate = val;
    }

    MiscReg
    readMiscRegNoEffect(int misc_reg, ThreadID tid = 0)
    {
        return isa.readMiscRegNoEffect(misc_reg);
    }

    MiscReg
    readMiscReg(int misc_reg, ThreadID tid = 0)
    {
        return isa.readMiscReg(misc_reg, tc);
    }

    void
    setMiscRegNoEffect(int misc_reg, const MiscReg &val, ThreadID tid = 0)
    {
        return isa.setMiscRegNoEffect(misc_reg, val);
    }

    void
    setMiscReg(int misc_reg, const MiscReg &val, ThreadID tid = 0)
    {
        return isa.setMiscReg(misc_reg, val, tc);
    }

    int
    flattenIntIndex(int reg)
    {
        return isa.flattenIntIndex(reg);
    }

    int
    flattenFloatIndex(int reg)
    {
        return isa.flattenFloatIndex(reg);
    }

    unsigned readStCondFailures() { return storeCondFailures; }

    void setStCondFailures(unsigned sc_failures)
    { storeCondFailures = sc_failures; }

    void syscall(int64_t callnum)
    {
        process->syscall(callnum, tc);
    }
};


// for non-speculative execution context, spec_mode is always false
inline bool
SimpleThread::misspeculating()
{
    return false;
}

#endif // __CPU_CPU_EXEC_CONTEXT_HH__
