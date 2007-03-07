/*
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

#include "arch/isa_traits.hh"
#include "arch/regfile.hh"
#include "arch/syscallreturn.hh"
#include "config/full_system.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "mem/physical.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/eventq.hh"
#include "sim/host.hh"
#include "sim/serialize.hh"

class BaseCPU;

#if FULL_SYSTEM

#include "sim/system.hh"
#include "arch/tlb.hh"

class FunctionProfile;
class ProfileNode;
class FunctionalPort;
class PhysicalPort;

namespace TheISA {
    namespace Kernel {
        class Statistics;
    };
};

#else // !FULL_SYSTEM

#include "sim/process.hh"
#include "mem/page_table.hh"
class TranslatingPort;

#endif // FULL_SYSTEM

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
    typedef TheISA::RegFile RegFile;
    typedef TheISA::MachInst MachInst;
    typedef TheISA::MiscRegFile MiscRegFile;
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
  public:
    typedef ThreadContext::Status Status;

  protected:
    RegFile regs;	// correct-path register context

  public:
    // pointer to CPU associated with this SimpleThread
    BaseCPU *cpu;

    ProxyThreadContext<SimpleThread> *tc;

    System *system;

#if FULL_SYSTEM
    TheISA::ITB *itb;
    TheISA::DTB *dtb;
#endif

    // constructor: initialize SimpleThread from given process structure
#if FULL_SYSTEM
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 TheISA::ITB *_itb, TheISA::DTB *_dtb,
                 bool use_kernel_stats = true);
#else
    SimpleThread(BaseCPU *_cpu, int _thread_num, Process *_process, int _asid);
#endif

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
     *  state, and to provide address translation methods.
     **************************************************************/

    /** Returns the pointer to this SimpleThread's ThreadContext. Used
     *  when a ThreadContext must be passed to objects outside of the
     *  CPU.
     */
    ThreadContext *getTC() { return tc; }

#if FULL_SYSTEM
    int getInstAsid() { return regs.instAsid(); }
    int getDataAsid() { return regs.dataAsid(); }

    Fault translateInstReq(RequestPtr &req)
    {
        return itb->translate(req, tc);
    }

    Fault translateDataReadReq(RequestPtr &req)
    {
        return dtb->translate(req, tc, false);
    }

    Fault translateDataWriteReq(RequestPtr &req)
    {
        return dtb->translate(req, tc, true);
    }

    void dumpFuncProfile();

    Fault hwrei();

    bool simPalCheck(int palFunc);
#else

    Fault translateInstReq(RequestPtr &req)
    {
        return process->pTable->translate(req);
    }

    Fault translateDataReadReq(RequestPtr &req)
    {
        return process->pTable->translate(req);
    }

    Fault translateDataWriteReq(RequestPtr &req)
    {
        return process->pTable->translate(req);
    }
#endif

    /*******************************************
     * ThreadContext interface functions.
     ******************************************/

    BaseCPU *getCpuPtr() { return cpu; }

    int getThreadNum() { return tid; }

#if FULL_SYSTEM
    System *getSystemPtr() { return system; }

    TheISA::ITB *getITBPtr() { return itb; }

    TheISA::DTB *getDTBPtr() { return dtb; }

    FunctionalPort *getPhysPort() { return physPort; }

    /** Return a virtual port. If no thread context is specified then a static
     * port is returned. Otherwise a port is created and returned. It must be
     * deleted by deleteVirtPort(). */
    VirtualPort *getVirtPort(ThreadContext *tc);

    void delVirtPort(VirtualPort *vp);
#endif

    Status status() const { return _status; }

    void setStatus(Status newStatus) { _status = newStatus; }

    /// Set the status to Active.  Optional delay indicates number of
    /// cycles to wait before beginning execution.
    void activate(int delay = 1);

    /// Set the status to Suspended.
    void suspend();

    /// Set the status to Unallocated.
    void deallocate();

    /// Set the status to Halted.
    void halt();

    virtual bool misspeculating();

    Fault instRead(RequestPtr &req)
    {
        panic("instRead not implemented");
        // return funcPhysMem->read(req, inst);
        return NoFault;
    }

    void copyArchRegs(ThreadContext *tc);

    void clearArchRegs() { regs.clear(); }

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx)
    {
        return regs.readIntReg(TheISA::flattenIntIndex(getTC(), reg_idx));
    }

    FloatReg readFloatReg(int reg_idx, int width)
    {
        return regs.readFloatReg(reg_idx, width);
    }

    FloatReg readFloatReg(int reg_idx)
    {
        return regs.readFloatReg(reg_idx);
    }

    FloatRegBits readFloatRegBits(int reg_idx, int width)
    {
        return regs.readFloatRegBits(reg_idx, width);
    }

    FloatRegBits readFloatRegBits(int reg_idx)
    {
        return regs.readFloatRegBits(reg_idx);
    }

    void setIntReg(int reg_idx, uint64_t val)
    {
        regs.setIntReg(TheISA::flattenIntIndex(getTC(), reg_idx), val);
    }

    void setFloatReg(int reg_idx, FloatReg val, int width)
    {
        regs.setFloatReg(reg_idx, val, width);
    }

    void setFloatReg(int reg_idx, FloatReg val)
    {
        regs.setFloatReg(reg_idx, val);
    }

    void setFloatRegBits(int reg_idx, FloatRegBits val, int width)
    {
        regs.setFloatRegBits(reg_idx, val, width);
    }

    void setFloatRegBits(int reg_idx, FloatRegBits val)
    {
        regs.setFloatRegBits(reg_idx, val);
    }

    uint64_t readPC()
    {
        return regs.readPC();
    }

    void setPC(uint64_t val)
    {
        regs.setPC(val);
    }

    uint64_t readMicroPC()
    {
        return microPC;
    }

    void setMicroPC(uint64_t val)
    {
        microPC = val;
    }

    uint64_t readNextPC()
    {
        return regs.readNextPC();
    }

    void setNextPC(uint64_t val)
    {
        regs.setNextPC(val);
    }

    uint64_t readNextMicroPC()
    {
        return nextMicroPC;
    }

    void setNextMicroPC(uint64_t val)
    {
        nextMicroPC = val;
    }

    uint64_t readNextNPC()
    {
        return regs.readNextNPC();
    }

    void setNextNPC(uint64_t val)
    {
        regs.setNextNPC(val);
    }

    MiscReg readMiscRegNoEffect(int misc_reg)
    {
        return regs.readMiscRegNoEffect(misc_reg);
    }

    MiscReg readMiscReg(int misc_reg)
    {
        return regs.readMiscReg(misc_reg, tc);
    }

    void setMiscRegNoEffect(int misc_reg, const MiscReg &val)
    {
        return regs.setMiscRegNoEffect(misc_reg, val);
    }

    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        return regs.setMiscReg(misc_reg, val, tc);
    }

    unsigned readStCondFailures() { return storeCondFailures; }

    void setStCondFailures(unsigned sc_failures)
    { storeCondFailures = sc_failures; }

#if !FULL_SYSTEM
    TheISA::IntReg getSyscallArg(int i)
    {
        return regs.readIntReg(TheISA::flattenIntIndex(getTC(),
                    TheISA::ArgumentReg0 + i));
    }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, TheISA::IntReg val)
    {
        regs.setIntReg(TheISA::flattenIntIndex(getTC(),
                    TheISA::ArgumentReg0 + i), val);
    }

    void setSyscallReturn(SyscallReturn return_value)
    {
        TheISA::setSyscallReturn(return_value, getTC());
    }

    void syscall(int64_t callnum)
    {
        process->syscall(callnum, tc);
    }
#endif

    void changeRegFileContext(TheISA::RegContextParam param,
            TheISA::RegContextVal val)
    {
        regs.changeContext(param, val);
    }
};


// for non-speculative execution context, spec_mode is always false
inline bool
SimpleThread::misspeculating()
{
    return false;
}

#endif // __CPU_CPU_EXEC_CONTEXT_HH__
