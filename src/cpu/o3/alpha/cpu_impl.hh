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
 * Authors: Kevin Lim
 */

#include "config/use_checker.hh"

#include "arch/alpha/faults.hh"
#include "arch/alpha/isa_traits.hh"
#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/checker/thread_context.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"

#include "cpu/o3/alpha/cpu.hh"
#include "cpu/o3/alpha/params.hh"
#include "cpu/o3/alpha/thread_context.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/thread_state.hh"

#if FULL_SYSTEM
#include "arch/alpha/osfpal.hh"
#include "arch/isa_traits.hh"
#include "cpu/quiesce_event.hh"
#include "kern/kernel_stats.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"
#endif

template <class Impl>
AlphaO3CPU<Impl>::AlphaO3CPU(Params *params)
#if FULL_SYSTEM
    : FullO3CPU<Impl>(params), itb(params->itb), dtb(params->dtb)
#else
    : FullO3CPU<Impl>(params)
#endif
{
    DPRINTF(O3CPU, "Creating AlphaO3CPU object.\n");

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
            this->thread[i] = new Thread(this, i, params->workload[i],
                                         i, params->mem);

            this->thread[i]->setStatus(ThreadContext::Suspended);

#if !FULL_SYSTEM
            /* Use this port to for syscall emulation writes to memory. */
            Port *mem_port;
            TranslatingPort *trans_port;
            trans_port = new TranslatingPort(csprintf("%s-%d-funcport",
                                                      name(), i),
                                             params->workload[i]->pTable,
                                             false);
            mem_port = params->mem->getPort("functional");
            mem_port->setPeer(trans_port);
            trans_port->setPeer(mem_port);
            this->thread[i]->setMemPort(trans_port);
#endif
            //usedTids[i] = true;
            //threadMap[i] = i;
        } else {
            //Allocate Empty thread so M5 can use later
            //when scheduling threads to CPU
            Process* dummy_proc = NULL;

            this->thread[i] = new Thread(this, i, dummy_proc, i, params->mem);
            //usedTids[i] = false;
        }
#endif // !FULL_SYSTEM

        ThreadContext *tc;

        // Setup the TC that will serve as the interface to the threads/CPU.
        AlphaTC<Impl> *alpha_tc =
            new AlphaTC<Impl>;

        tc = alpha_tc;

        // If we're using a checker, then the TC should be the
        // CheckerThreadContext.
#if USE_CHECKER
        if (params->checker) {
            tc = new CheckerThreadContext<AlphaTC<Impl> >(
                alpha_tc, this->checker);
        }
#endif

        alpha_tc->cpu = this;
        alpha_tc->thread = this->thread[i];

#if FULL_SYSTEM
        // Setup quiesce event.
        this->thread[i]->quiesceEvent = new EndQuiesceEvent(tc);

        Port *mem_port;
        FunctionalPort *phys_port;
        VirtualPort *virt_port;
        phys_port = new FunctionalPort(csprintf("%s-%d-funcport",
                                                name(), i));
        mem_port = this->system->physmem->getPort("functional");
        mem_port->setPeer(phys_port);
        phys_port->setPeer(mem_port);

        virt_port = new VirtualPort(csprintf("%s-%d-vport",
                                             name(), i));
        mem_port = this->system->physmem->getPort("functional");
        mem_port->setPeer(virt_port);
        virt_port->setPeer(mem_port);

        this->thread[i]->setPhysPort(phys_port);
        this->thread[i]->setVirtPort(virt_port);
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
AlphaO3CPU<Impl>::regStats()
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
AlphaO3CPU<Impl>::readMiscReg(int misc_reg, unsigned tid)
{
    return this->regFile.readMiscReg(misc_reg, tid);
}

template <class Impl>
TheISA::MiscReg
AlphaO3CPU<Impl>::readMiscRegWithEffect(int misc_reg, Fault &fault,
                                        unsigned tid)
{
    return this->regFile.readMiscRegWithEffect(misc_reg, fault, tid);
}

template <class Impl>
Fault
AlphaO3CPU<Impl>::setMiscReg(int misc_reg, const MiscReg &val, unsigned tid)
{
    return this->regFile.setMiscReg(misc_reg, val, tid);
}

template <class Impl>
Fault
AlphaO3CPU<Impl>::setMiscRegWithEffect(int misc_reg, const MiscReg &val,
                                       unsigned tid)
{
    return this->regFile.setMiscRegWithEffect(misc_reg, val, tid);
}

template <class Impl>
void
AlphaO3CPU<Impl>::squashFromTC(unsigned tid)
{
    this->thread[tid]->inSyscall = true;
    this->commit.generateTCEvent(tid);
}

#if FULL_SYSTEM

template <class Impl>
void
AlphaO3CPU<Impl>::post_interrupt(int int_num, int index)
{
    BaseCPU::post_interrupt(int_num, index);

    if (this->thread[0]->status() == ThreadContext::Suspended) {
        DPRINTF(IPI,"Suspended Processor awoke\n");
        this->threadContexts[0]->activate();
    }
}

template <class Impl>
int
AlphaO3CPU<Impl>::readIntrFlag()
{
    return this->regFile.readIntrFlag();
}

template <class Impl>
void
AlphaO3CPU<Impl>::setIntrFlag(int val)
{
    this->regFile.setIntrFlag(val);
}

template <class Impl>
Fault
AlphaO3CPU<Impl>::hwrei(unsigned tid)
{
    // Need to clear the lock flag upon returning from an interrupt.
    this->setMiscReg(TheISA::Lock_Flag_DepTag, false, tid);

    this->thread[tid]->kernelStats->hwrei();

    this->checkInterrupts = true;

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
bool
AlphaO3CPU<Impl>::simPalCheck(int palFunc, unsigned tid)
{
    if (this->thread[tid]->kernelStats)
        this->thread[tid]->kernelStats->callpal(palFunc,
                                                this->threadContexts[tid]);

    switch (palFunc) {
      case PAL::halt:
        halt();
        if (--System::numSystemsRunning == 0)
            exitSimLoop("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (this->system->breakpoint())
            return false;
        break;
    }

    return true;
}

template <class Impl>
void
AlphaO3CPU<Impl>::processInterrupts()
{
    using namespace TheISA;
    // Check for interrupts here.  For now can copy the code that
    // exists within isa_fullsys_traits.hh.  Also assume that thread 0
    // is the one that handles the interrupts.
    // @todo: Possibly consolidate the interrupt checking code.
    // @todo: Allow other threads to handle interrupts.

    // Check if there are any outstanding interrupts
    //Handle the interrupts
    int ipl = 0;
    int summary = 0;

    this->checkInterrupts = false;

    if (this->readMiscReg(IPR_ASTRR, 0))
        panic("asynchronous traps not implemented\n");

    if (this->readMiscReg(IPR_SIRR, 0)) {
        for (int i = INTLEVEL_SOFTWARE_MIN;
             i < INTLEVEL_SOFTWARE_MAX; i++) {
            if (this->readMiscReg(IPR_SIRR, 0) & (ULL(1) << i)) {
                // See table 4-19 of the 21164 hardware reference
                ipl = (i - INTLEVEL_SOFTWARE_MIN) + 1;
                summary |= (ULL(1) << i);
            }
        }
    }

    uint64_t interrupts = this->intr_status();

    if (interrupts) {
        for (int i = INTLEVEL_EXTERNAL_MIN;
             i < INTLEVEL_EXTERNAL_MAX; i++) {
            if (interrupts & (ULL(1) << i)) {
                // See table 4-19 of the 21164 hardware reference
                ipl = i;
                summary |= (ULL(1) << i);
            }
        }
    }

    if (ipl && ipl > this->readMiscReg(IPR_IPLR, 0)) {
        this->setMiscReg(IPR_ISR, summary, 0);
        this->setMiscReg(IPR_INTID, ipl, 0);
        // Checker needs to know these two registers were updated.
#if USE_CHECKER
        if (this->checker) {
            this->checker->threadBase()->setMiscReg(IPR_ISR, summary);
            this->checker->threadBase()->setMiscReg(IPR_INTID, ipl);
        }
#endif
        this->trap(Fault(new InterruptFault), 0);
        DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                this->readMiscReg(IPR_IPLR, 0), ipl, summary);
    }
}

#endif // FULL_SYSTEM

template <class Impl>
void
AlphaO3CPU<Impl>::trap(Fault fault, unsigned tid)
{
    // Pass the thread's TC into the invoke method.
    fault->invoke(this->threadContexts[tid]);
}

#if !FULL_SYSTEM

template <class Impl>
void
AlphaO3CPU<Impl>::syscall(int64_t callnum, int tid)
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
AlphaO3CPU<Impl>::getSyscallArg(int i, int tid)
{
    return this->readArchIntReg(AlphaISA::ArgumentReg0 + i, tid);
}

template <class Impl>
void
AlphaO3CPU<Impl>::setSyscallArg(int i, IntReg val, int tid)
{
    this->setArchIntReg(AlphaISA::ArgumentReg0 + i, val, tid);
}

template <class Impl>
void
AlphaO3CPU<Impl>::setSyscallReturn(SyscallReturn return_value, int tid)
{
    // check for error condition.  Alpha syscall convention is to
    // indicate success/failure in reg a3 (r19) and put the
    // return value itself in the standard return value reg (v0).
    if (return_value.successful()) {
        // no error
        this->setArchIntReg(TheISA::SyscallSuccessReg, 0, tid);
        this->setArchIntReg(TheISA::ReturnValueReg, return_value.value(), tid);
    } else {
        // got an error, return details
        this->setArchIntReg(TheISA::SyscallSuccessReg, (IntReg) -1, tid);
        this->setArchIntReg(TheISA::ReturnValueReg, -return_value.value(), tid);
    }
}
#endif
