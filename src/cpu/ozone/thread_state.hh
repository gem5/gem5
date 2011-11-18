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

#include "arch/regfile.hh"
#include "arch/types.hh"
#include "base/callback.hh"
#include "base/output.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "sim/faults.hh"
#include "sim/process.hh"
#include "sim/sim_exit.hh"

class Event;
//class Process;

class EndQuiesceEvent;
class FunctionalMemory;
class FunctionProfile;
class Process;
class ProfileNode;

// Maybe this ozone thread state should only really have committed state?
// I need to think about why I'm using this and what it's useful for.  Clearly
// has benefits for SMT; basically serves same use as SimpleThread.
// Makes the ExecContext proxy easier.  Gives organization/central access point
// to state of a thread that can be accessed normally (i.e. not in-flight
// stuff within a OoO processor).  Does this need an TC proxy within it?
template <class Impl>
struct OzoneThreadState : public ThreadState {
    typedef typename ThreadContext::Status Status;
    typedef typename Impl::CPUType CPUType;
    typedef TheISA::MiscReg MiscReg;

    OzoneThreadState(CPUType *_cpu, int _thread_num)
        : ThreadState(_cpu, -1, _thread_num),
          intrflag(0), cpu(_cpu), inSyscall(0), trapPending(0)
    {
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
        miscRegFile.clear();
    }

    OzoneThreadState(CPUType *_cpu, int _thread_num, Process *_process)
        : ThreadState(_cpu, -1, _thread_num, _process),
          cpu(_cpu), inSyscall(0), trapPending(0)
    {
        miscRegFile.clear();
    }

    RenameTable<Impl> renameTable;

    Addr PC;

    Addr nextPC;

    TheISA::MiscRegFile miscRegFile;

    int intrflag;

    typename Impl::CPUType *cpu;

    bool inSyscall;

    bool trapPending;

    ThreadContext *tc;

    ThreadContext *getTC() { return tc; }

    MiscReg readMiscRegNoEffect(int misc_reg)
    {
        return miscRegFile.readRegNoEffect(misc_reg);
    }

    MiscReg readMiscReg(int misc_reg)
    {
        return miscRegFile.readReg(misc_reg, tc);
    }

    void setMiscRegNoEffect(int misc_reg, const MiscReg &val)
    {
        miscRegFile.setRegNoEffect(misc_reg, val);
    }

    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        miscRegFile.setReg(misc_reg, val, tc);
    }

    uint64_t readPC()
    { return PC; }

    void setPC(uint64_t val)
    { PC = val; }

    uint64_t readNextPC()
    { return nextPC; }

    void setNextPC(uint64_t val)
    { nextPC = val; }

    void dumpFuncProfile()
    {
        std::ostream *os = simout.create(csprintf("profile.%s.dat", cpu->name()));
        profile->dump(tc, *os);
    }
};

#endif // __CPU_OZONE_THREAD_STATE_HH__
