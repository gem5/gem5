/*
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
 * Authors: Gabe Black
 */

#include "config/use_checker.hh"

#include "arch/sparc/faults.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/miscregfile.hh"
#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/checker/thread_context.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"

#include "cpu/o3/sparc/cpu.hh"
#include "cpu/o3/sparc/params.hh"
#include "cpu/o3/sparc/thread_context.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/thread_state.hh"

#if FULL_SYSTEM
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/kernel_stats.hh"
#include "cpu/quiesce_event.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"
#endif

template <class Impl>
SparcO3CPU<Impl>::SparcO3CPU(Params *params)
#if FULL_SYSTEM
    : FullO3CPU<Impl>(params), itb(params->itb), dtb(params->dtb)
#else
    : FullO3CPU<Impl>(params)
#endif
{
    DPRINTF(O3CPU, "Creating SparcO3CPU object.\n");

    // Setup any thread state.
    this->thread.resize(this->numThreads);

    for (int i = 0; i < this->numThreads; ++i) {
#if FULL_SYSTEM
        // SMT is not supported in FS mode yet.
        assert(this->numThreads == 1);
        this->thread[i] = new Thread(this, 0);
        this->thread[i]->setStatus(ThreadContext::Suspended);
#else
        if (i < params->workload.size()) {
            DPRINTF(O3CPU, "Workload[%i] process is %#x",
                    i, this->thread[i]);
            this->thread[i] = new Thread(this, i, params->workload[i], i);

            this->thread[i]->setStatus(ThreadContext::Suspended);

            //usedTids[i] = true;
            //threadMap[i] = i;
        } else {
            //Allocate Empty thread so M5 can use later
            //when scheduling threads to CPU
            Process* dummy_proc = NULL;

            this->thread[i] = new Thread(this, i, dummy_proc, i);
            //usedTids[i] = false;
        }
#endif // !FULL_SYSTEM

        ThreadContext *tc;

        // Setup the TC that will serve as the interface to the threads/CPU.
        SparcTC<Impl> *sparc_tc = new SparcTC<Impl>;

        tc = sparc_tc;

        // If we're using a checker, then the TC should be the
        // CheckerThreadContext.
#if USE_CHECKER
        if (params->checker) {
            tc = new CheckerThreadContext<SparcTC<Impl> >(
                sparc_tc, this->checker);
        }
#endif

        sparc_tc->cpu = this;
        sparc_tc->thread = this->thread[i];

#if FULL_SYSTEM
        // Setup quiesce event.
        this->thread[i]->quiesceEvent = new EndQuiesceEvent(tc);
#endif
        // Give the thread the TC.
        this->thread[i]->tc = tc;

        // Add the TC to the CPU's list of TC's.
        this->threadContexts.push_back(tc);
    }

    for (int i=0; i < this->numThreads; i++) {
        this->thread[i]->setFuncExeInst(0);
    }

    // Sets CPU pointers. These must be set at this level because the CPU
    // pointers are defined to be the highest level of CPU class.
    this->fetch.setCPU(this);
    this->decode.setCPU(this);
    this->rename.setCPU(this);
    this->iew.setCPU(this);
    this->commit.setCPU(this);

    this->rob.setCPU(this);
    this->regFile.setCPU(this);

    lockAddr = 0;
    lockFlag = false;
}

template <class Impl>
void
SparcO3CPU<Impl>::regStats()
{
    // Register stats for everything that has stats.
    this->fullCPURegStats();
    this->fetch.regStats();
    this->decode.regStats();
    this->rename.regStats();
    this->iew.regStats();
    this->commit.regStats();
}


template <class Impl>
TheISA::MiscReg
SparcO3CPU<Impl>::readMiscReg(int misc_reg, unsigned tid)
{
    return this->regFile.readMiscReg(misc_reg, tid);
}

template <class Impl>
TheISA::MiscReg
SparcO3CPU<Impl>::readMiscRegWithEffect(int misc_reg, unsigned tid)
{
    return this->regFile.readMiscRegWithEffect(misc_reg, tid);
}

template <class Impl>
void
SparcO3CPU<Impl>::setMiscReg(int misc_reg, const MiscReg &val, unsigned tid)
{
    this->regFile.setMiscReg(misc_reg, val, tid);
}

template <class Impl>
void
SparcO3CPU<Impl>::setMiscRegWithEffect(int misc_reg, const MiscReg &val,
                                       unsigned tid)
{
    this->regFile.setMiscRegWithEffect(misc_reg, val, tid);
}

template <class Impl>
void
SparcO3CPU<Impl>::squashFromTC(unsigned tid)
{
    this->thread[tid]->inSyscall = true;
    this->commit.generateTCEvent(tid);
}

#if FULL_SYSTEM

template <class Impl>
void
SparcO3CPU<Impl>::post_interrupt(int int_num, int index)
{
    BaseCPU::post_interrupt(int_num, index);

    if (this->thread[0]->status() == ThreadContext::Suspended) {
        DPRINTF(IPI,"Suspended Processor awoke\n");
        this->threadContexts[0]->activate();
    }
}

template <class Impl>
Fault
SparcO3CPU<Impl>::hwrei(unsigned tid)
{
    panic("This doesn't make sense for SPARC\n");
    return NoFault;
}

template <class Impl>
bool
SparcO3CPU<Impl>::simPalCheck(int palFunc, unsigned tid)
{
    panic("This doesn't make sense for SPARC\n");
    return true;
}

template <class Impl>
Fault
SparcO3CPU<Impl>::getInterrupts()
{
    // Check if there are any outstanding interrupts
    return this->interrupts.getInterrupt(this->threadContexts[0]);
}

template <class Impl>
void
SparcO3CPU<Impl>::processInterrupts(Fault interrupt)
{
    // Check for interrupts here.  For now can copy the code that
    // exists within isa_fullsys_traits.hh.  Also assume that thread 0
    // is the one that handles the interrupts.
    // @todo: Possibly consolidate the interrupt checking code.
    // @todo: Allow other threads to handle interrupts.

    assert(interrupt != NoFault);
    this->interrupts.updateIntrInfo(this->threadContexts[0]);

    DPRINTF(O3CPU, "Interrupt %s being handled\n", interrupt->name());
    this->checkInterrupts = false;
    this->trap(interrupt, 0);
}

#endif // FULL_SYSTEM

template <class Impl>
void
SparcO3CPU<Impl>::trap(Fault fault, unsigned tid)
{
    // Pass the thread's TC into the invoke method.
    fault->invoke(this->threadContexts[tid]);
}

#if !FULL_SYSTEM

template <class Impl>
void
SparcO3CPU<Impl>::syscall(int64_t callnum, int tid)
{
    DPRINTF(O3CPU, "[tid:%i] Executing syscall().\n\n", tid);

    DPRINTF(Activity,"Activity: syscall() called.\n");

    // Temporarily increase this by one to account for the syscall
    // instruction.
    ++(this->thread[tid]->funcExeInst);

    // Execute the actual syscall.
    this->thread[tid]->syscall(callnum);

    // Decrease funcExeInst by one as the normal commit will handle
    // incrementing it.
    --(this->thread[tid]->funcExeInst);
}

template <class Impl>
TheISA::IntReg
SparcO3CPU<Impl>::getSyscallArg(int i, int tid)
{
    IntReg idx = TheISA::flattenIntIndex(this->tcBase(tid),
            SparcISA::ArgumentReg0 + i);
    return this->readArchIntReg(idx, tid);
}

template <class Impl>
void
SparcO3CPU<Impl>::setSyscallArg(int i, IntReg val, int tid)
{
    IntReg idx = TheISA::flattenIntIndex(this->tcBase(tid),
            SparcISA::ArgumentReg0 + i);
    this->setArchIntReg(idx, val, tid);
}

template <class Impl>
void
SparcO3CPU<Impl>::setSyscallReturn(SyscallReturn return_value, int tid)
{
    TheISA::setSyscallReturn(return_value, this->tcBase(tid));
}
#endif
