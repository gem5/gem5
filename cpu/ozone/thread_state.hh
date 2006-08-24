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

#ifndef __CPU_OZONE_THREAD_STATE_HH__
#define __CPU_OZONE_THREAD_STATE_HH__

#include "arch/faults.hh"
#include "arch/isa_traits.hh"
#include "base/callback.hh"
#include "base/output.hh"
#include "cpu/exec_context.hh"
#include "cpu/thread_state.hh"
#include "sim/process.hh"
#include "sim/sim_exit.hh"

class Event;
//class Process;

#if FULL_SYSTEM
class EndQuiesceEvent;
class FunctionProfile;
class ProfileNode;
#else
class Process;
class FunctionalMemory;
#endif

// Maybe this ozone thread state should only really have committed state?
// I need to think about why I'm using this and what it's useful for.  Clearly
// has benefits for SMT; basically serves same use as CPUExecContext.
// Makes the ExecContext proxy easier.  Gives organization/central access point
// to state of a thread that can be accessed normally (i.e. not in-flight
// stuff within a OoO processor).  Does this need an XC proxy within it?
template <class Impl>
struct OzoneThreadState : public ThreadState {
    typedef typename ExecContext::Status Status;
    typedef typename Impl::FullCPU FullCPU;
    typedef TheISA::MiscReg MiscReg;

#if FULL_SYSTEM
    OzoneThreadState(FullCPU *_cpu, int _thread_num, FunctionalMemory *_mem)
        : ThreadState(-1, _thread_num, _mem),
          cpu(_cpu), inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
        if (cpu->params->profile) {
            profile = new FunctionProfile(cpu->params->system->kernelSymtab);
            Callback *cb =
                new MakeCallback<OzoneThreadState,
                &OzoneThreadState::dumpFuncProfile>(this);
            registerExitCallback(cb);
        }

        // let's fill with a dummy node for now so we don't get a segfault
        // on the first cycle when there's no node available.
        static ProfileNode dummyNode;
        profileNode = &dummyNode;
        profilePC = 3;
    }
#else
    OzoneThreadState(FullCPU *_cpu, int _thread_num, Process *_process, int _asid)
        : ThreadState(-1, _thread_num, _process->getMemory(), _process, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
    }

    OzoneThreadState(FullCPU *_cpu, int _thread_num, FunctionalMemory *_mem,
                     int _asid)
        : ThreadState(-1, _thread_num, _mem, NULL, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
    }
#endif

    Status _status;

    Status status() const { return _status; }

    void setStatus(Status new_status) { _status = new_status; }

    RenameTable<Impl> renameTable;
    Addr PC;
    Addr nextPC;

    // Current instruction
    TheISA::MachInst inst;

    TheISA::RegFile regs;

    typename Impl::FullCPU *cpu;

    bool inSyscall;

    bool trapPending;

    ExecContext *xcProxy;

    ExecContext *getXCProxy() { return xcProxy; }

#if !FULL_SYSTEM

    Fault dummyTranslation(MemReqPtr &req)
    {
#if 0
        assert((req->vaddr >> 48 & 0xffff) == 0);
#endif

        // put the asid in the upper 16 bits of the paddr
        req->paddr = req->vaddr & ~((Addr)0xffff << sizeof(Addr) * 8 - 16);
        req->paddr = req->paddr | (Addr)req->asid << sizeof(Addr) * 8 - 16;
        return NoFault;
    }
    Fault translateInstReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }
    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }
    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }
#else
    Fault translateInstReq(MemReqPtr &req)
    {
        return cpu->itb->translate(req);
    }

    Fault translateDataReadReq(MemReqPtr &req)
    {
        return cpu->dtb->translate(req, false);
    }

    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return cpu->dtb->translate(req, true);
    }
#endif

    MiscReg readMiscReg(int misc_reg)
    {
        return regs.miscRegs.readReg(misc_reg);
    }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    {
        return regs.miscRegs.readRegWithEffect(misc_reg, fault, xcProxy);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        return regs.miscRegs.setReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return regs.miscRegs.setRegWithEffect(misc_reg, val, xcProxy);
    }

    uint64_t readPC()
    { return PC; }

    void setPC(uint64_t val)
    { PC = val; }

    uint64_t readNextPC()
    { return nextPC; }

    void setNextPC(uint64_t val)
    { nextPC = val; }

    void setInst(TheISA::MachInst _inst) { inst = _inst; }

    Counter readFuncExeInst() { return funcExeInst; }

    void setFuncExeInst(Counter new_val) { funcExeInst = new_val; }

#if FULL_SYSTEM
    void dumpFuncProfile()
    {
        std::ostream *os = simout.create(csprintf("profile.%s.dat", cpu->name()));
        profile->dump(xcProxy, *os);
    }
#endif
};

#endif // __CPU_OZONE_THREAD_STATE_HH__
