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

#ifndef __CPU_OZONE_THREAD_STATE_HH__
#define __CPU_OZONE_THREAD_STATE_HH__

#include "arch/faults.hh"
#include "arch/isa_traits.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "sim/process.hh"

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
// has benefits for SMT; basically serves same use as SimpleThread.
// Makes the ExecContext proxy easier.  Gives organization/central access point
// to state of a thread that can be accessed normally (i.e. not in-flight
// stuff within a OoO processor).  Does this need an TC proxy within it?
template <class Impl>
struct OzoneThreadState : public ThreadState {
    typedef typename ThreadContext::Status Status;
    typedef typename Impl::FullCPU FullCPU;
    typedef TheISA::MiscReg MiscReg;

#if FULL_SYSTEM
    OzoneThreadState(FullCPU *_cpu, int _thread_num)
        : ThreadState(-1, _thread_num),
          inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
    }
#else
    OzoneThreadState(FullCPU *_cpu, int _thread_num, Process *_process, int _asid)
        : ThreadState(-1, _thread_num, NULL, _process, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
    }

    OzoneThreadState(FullCPU *_cpu, int _thread_num,
                     int _asid)
        : ThreadState(-1, _thread_num, NULL, NULL, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
    }
#endif

    RenameTable<Impl> renameTable;

    Addr PC;

    Addr nextPC;

    TheISA::RegFile regs;

    typename Impl::FullCPU *cpu;

    bool inSyscall;

    bool trapPending;

    ThreadContext *tc;

    ThreadContext *getTC() { return tc; }

#if !FULL_SYSTEM
    Fault translateInstReq(Request *req)
    {
        return process->pTable->translate(req);
    }
    Fault translateDataReadReq(Request *req)
    {
        return process->pTable->translate(req);
    }
    Fault translateDataWriteReq(Request *req)
    {
        return process->pTable->translate(req);
    }
#else
    Fault translateInstReq(Request *req)
    {
        return cpu->itb->translate(req);
    }

    Fault translateDataReadReq(Request *req)
    {
        return cpu->dtb->translate(req, false);
    }

    Fault translateDataWriteReq(Request *req)
    {
        return cpu->dtb->translate(req, true);
    }
#endif

    MiscReg readMiscReg(int misc_reg)
    {
        return regs.readMiscReg(misc_reg);
    }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    {
        return regs.readMiscRegWithEffect(misc_reg, fault, tc);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        return regs.setMiscReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return regs.setMiscRegWithEffect(misc_reg, val, tc);
    }

    uint64_t readPC()
    { return PC; }

    void setPC(uint64_t val)
    { PC = val; }

    uint64_t readNextPC()
    { return nextPC; }

    void setNextPC(uint64_t val)
    { nextPC = val; }
};

#endif // __CPU_OZONE_THREAD_STATE_HH__
