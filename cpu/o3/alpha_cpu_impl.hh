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
 */

#include "arch/alpha/faults.hh"
#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/checker/exec_context.hh"
#include "cpu/quiesce_event.hh"
#include "mem/mem_interface.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"

#include "cpu/o3/alpha_cpu.hh"
#include "cpu/o3/alpha_params.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/thread_state.hh"

#if FULL_SYSTEM
#include "arch/alpha/osfpal.hh"
#include "arch/isa_traits.hh"
#endif

using namespace TheISA;

template <class Impl>
AlphaFullCPU<Impl>::AlphaFullCPU(Params *params)
#if FULL_SYSTEM
    : FullO3CPU<Impl>(params), itb(params->itb), dtb(params->dtb)
#else
    : FullO3CPU<Impl>(params)
#endif
{
    DPRINTF(FullCPU, "AlphaFullCPU: Creating AlphaFullCPU object.\n");

    this->thread.resize(this->numThreads);

    for (int i = 0; i < this->numThreads; ++i) {
#if FULL_SYSTEM
        assert(this->numThreads == 1);
        this->thread[i] = new Thread(this, 0, params->mem);
        this->thread[i]->setStatus(ExecContext::Suspended);
#else
        if (i < params->workload.size()) {
            DPRINTF(FullCPU, "FullCPU: Workload[%i]'s starting PC is %#x, "
                    "process is %#x",
                    i, params->workload[i]->prog_entry, this->thread[i]);
            this->thread[i] = new Thread(this, i, params->workload[i], i);
            assert(params->workload[i]->getMemory() != NULL);

            this->thread[i]->setStatus(ExecContext::Suspended);
            //usedTids[i] = true;
            //threadMap[i] = i;
        } else {
            //Allocate Empty execution context so M5 can use later
            //when scheduling threads to CPU
            Process* dummy_proc = NULL;

            this->thread[i] = new Thread(this, i, dummy_proc, i);
            //usedTids[i] = false;
        }
#endif // !FULL_SYSTEM

        this->thread[i]->numInst = 0;

        ExecContext *xc_proxy;

        AlphaXC *alpha_xc_proxy = new AlphaXC;

        if (params->checker) {
            xc_proxy = new CheckerExecContext<AlphaXC>(alpha_xc_proxy, this->checker);
        } else {
            xc_proxy = alpha_xc_proxy;
        }

        alpha_xc_proxy->cpu = this;
        alpha_xc_proxy->thread = this->thread[i];

        alpha_xc_proxy->quiesceEvent =
            new EndQuiesceEvent(xc_proxy);
        alpha_xc_proxy->lastActivate = 0;
        alpha_xc_proxy->lastSuspend = 0;

        this->thread[i]->xcProxy = xc_proxy;

        this->execContexts.push_back(xc_proxy);
    }


    for (int i=0; i < this->numThreads; i++) {
        this->thread[i]->funcExeInst = 0;
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
AlphaFullCPU<Impl>::regStats()
{
    // Register stats for everything that has stats.
    this->fullCPURegStats();
    this->fetch.regStats();
    this->decode.regStats();
    this->rename.regStats();
    this->iew.regStats();
    this->commit.regStats();
}

#if FULL_SYSTEM
template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::dumpFuncProfile()
{
    // Currently not supported
}
#endif

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::takeOverFrom(ExecContext *old_context)
{
    // some things should already be set up
    assert(getMemPtr() == old_context->getMemPtr());
#if FULL_SYSTEM
    assert(getSystemPtr() == old_context->getSystemPtr());
#else
    assert(getProcessPtr() == old_context->getProcessPtr());
#endif

    // copy over functional state
    setStatus(old_context->status());
    copyArchRegs(old_context);
    setCpuId(old_context->readCpuId());
#if !FULL_SYSTEM
    thread->funcExeInst = old_context->readFuncExeInst();
#else
    EndQuiesceEvent *other_quiesce = old_context->getQuiesceEvent();
    if (other_quiesce) {
        // Point the quiesce event's XC at this XC so that it wakes up
        // the proper CPU.
        other_quiesce->xc = this;
    }
    if (thread->quiesceEvent) {
        thread->quiesceEvent->xc = this;
    }
//    storeCondFailures = 0;
    cpu->lockFlag = false;
#endif

    old_context->setStatus(ExecContext::Unallocated);

    thread->inSyscall = false;
    thread->trapPending = false;
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::activate(int delay)
{
    DPRINTF(FullCPU, "Calling activate on AlphaXC\n");

    if (thread->status() == ExecContext::Active)
        return;

    lastActivate = curTick;

    if (thread->status() == ExecContext::Unallocated) {
        cpu->activateWhenReady(thread->tid);
        return;
    }

    thread->setStatus(ExecContext::Active);

    // status() == Suspended
    cpu->activateContext(thread->tid, delay);
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::suspend()
{
    DPRINTF(FullCPU, "Calling suspend on AlphaXC\n");

    if (thread->status() == ExecContext::Suspended)
        return;

    lastActivate = curTick;
    lastSuspend = curTick;
/*
#if FULL_SYSTEM
    // Don't change the status from active if there are pending interrupts
    if (cpu->check_interrupts()) {
        assert(status() == ExecContext::Active);
        return;
    }
#endif
*/
    thread->setStatus(ExecContext::Suspended);
    cpu->suspendContext(thread->tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::deallocate()
{
    DPRINTF(FullCPU, "Calling deallocate on AlphaXC\n");

    if (thread->status() == ExecContext::Unallocated)
        return;

    thread->setStatus(ExecContext::Unallocated);
    cpu->deallocateContext(thread->tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::halt()
{
    DPRINTF(FullCPU, "Calling halt on AlphaXC\n");

    if (thread->status() == ExecContext::Halted)
        return;

    thread->setStatus(ExecContext::Halted);
    cpu->haltContext(thread->tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::regStats(const std::string &name)
{}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::serialize(std::ostream &os)
{}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::unserialize(Checkpoint *cp, const std::string &section)
{}

#if FULL_SYSTEM
template <class Impl>
EndQuiesceEvent *
AlphaFullCPU<Impl>::AlphaXC::getQuiesceEvent()
{
    return quiesceEvent;
}

template <class Impl>
Tick
AlphaFullCPU<Impl>::AlphaXC::readLastActivate()
{
    return lastActivate;
}

template <class Impl>
Tick
AlphaFullCPU<Impl>::AlphaXC::readLastSuspend()
{
    return lastSuspend;
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::profileClear()
{}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::profileSample()
{}
#endif

template <class Impl>
TheISA::MachInst
AlphaFullCPU<Impl>::AlphaXC:: getInst()
{
    return thread->inst;
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::copyArchRegs(ExecContext *xc)
{
    // This function will mess things up unless the ROB is empty and
    // there are no instructions in the pipeline.
    unsigned tid = thread->tid;
    PhysRegIndex renamed_reg;

    // First loop through the integer registers.
    for (int i = 0; i < AlphaISA::NumIntRegs; ++i) {
        renamed_reg = cpu->renameMap[tid].lookup(i);

        DPRINTF(FullCPU, "FullCPU: Copying over register %i, had data %lli, "
                "now has data %lli.\n",
                renamed_reg, cpu->readIntReg(renamed_reg),
                xc->readIntReg(i));

        cpu->setIntReg(renamed_reg, xc->readIntReg(i));
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < AlphaISA::NumFloatRegs; ++i) {
        renamed_reg = cpu->renameMap[tid].lookup(i + AlphaISA::FP_Base_DepTag);
        cpu->setFloatRegDouble(renamed_reg,
                               xc->readFloatRegDouble(i));
        cpu->setFloatRegInt(renamed_reg,
                            xc->readFloatRegInt(i));
    }

    // Copy the misc regs.
    cpu->regFile.miscRegs[tid].copyMiscRegs(xc);

    // Then finally set the PC and the next PC.
    cpu->setPC(xc->readPC(), tid);
    cpu->setNextPC(xc->readNextPC(), tid);
#if !FULL_SYSTEM
    this->thread->funcExeInst = xc->readFuncExeInst();
#endif
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::clearArchRegs()
{}

template <class Impl>
uint64_t
AlphaFullCPU<Impl>::AlphaXC::readIntReg(int reg_idx)
{
    DPRINTF(Fault, "Reading int register through the XC!\n");
    return cpu->readArchIntReg(reg_idx, thread->tid);
}

template <class Impl>
float
AlphaFullCPU<Impl>::AlphaXC::readFloatRegSingle(int reg_idx)
{
    DPRINTF(Fault, "Reading float register through the XC!\n");
    return cpu->readArchFloatRegSingle(reg_idx, thread->tid);
}

template <class Impl>
double
AlphaFullCPU<Impl>::AlphaXC::readFloatRegDouble(int reg_idx)
{
    DPRINTF(Fault, "Reading float register through the XC!\n");
    return cpu->readArchFloatRegDouble(reg_idx, thread->tid);
}

template <class Impl>
uint64_t
AlphaFullCPU<Impl>::AlphaXC::readFloatRegInt(int reg_idx)
{
    DPRINTF(Fault, "Reading floatint register through the XC!\n");
    return cpu->readArchFloatRegInt(reg_idx, thread->tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::setIntReg(int reg_idx, uint64_t val)
{
    DPRINTF(Fault, "Setting int register through the XC!\n");
    cpu->setArchIntReg(reg_idx, val, thread->tid);

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromXC(thread->tid);
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::setFloatRegSingle(int reg_idx, float val)
{
    DPRINTF(Fault, "Setting float register through the XC!\n");
    cpu->setArchFloatRegSingle(reg_idx, val, thread->tid);

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromXC(thread->tid);
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::setFloatRegDouble(int reg_idx, double val)
{
    DPRINTF(Fault, "Setting float register through the XC!\n");
    cpu->setArchFloatRegDouble(reg_idx, val, thread->tid);

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromXC(thread->tid);
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::setFloatRegInt(int reg_idx, uint64_t val)
{
    DPRINTF(Fault, "Setting floatint register through the XC!\n");
    cpu->setArchFloatRegInt(reg_idx, val, thread->tid);

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromXC(thread->tid);
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::setPC(uint64_t val)
{
    cpu->setPC(val, thread->tid);

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromXC(thread->tid);
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::setNextPC(uint64_t val)
{
    cpu->setNextPC(val, thread->tid);

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromXC(thread->tid);
    }
}

template <class Impl>
Fault
AlphaFullCPU<Impl>::AlphaXC::setMiscReg(int misc_reg, const MiscReg &val)
{
    DPRINTF(Fault, "Setting misc register through the XC!\n");

    Fault ret_fault = cpu->setMiscReg(misc_reg, val, thread->tid);

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromXC(thread->tid);
    }

    return ret_fault;
}

template <class Impl>
Fault
AlphaFullCPU<Impl>::AlphaXC::setMiscRegWithEffect(int misc_reg, const MiscReg &val)
{
    DPRINTF(Fault, "Setting misc register through the XC!\n");

    Fault ret_fault = cpu->setMiscRegWithEffect(misc_reg, val, thread->tid);

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromXC(thread->tid);
    }

    return ret_fault;
}

#if !FULL_SYSTEM

template <class Impl>
TheISA::IntReg
AlphaFullCPU<Impl>::AlphaXC::getSyscallArg(int i)
{
    return cpu->getSyscallArg(i, thread->tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::setSyscallArg(int i, IntReg val)
{
    cpu->setSyscallArg(i, val, thread->tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaXC::setSyscallReturn(SyscallReturn return_value)
{
    cpu->setSyscallReturn(return_value, thread->tid);
}

#endif // FULL_SYSTEM

template <class Impl>
MiscReg
AlphaFullCPU<Impl>::readMiscReg(int misc_reg, unsigned tid)
{
    return this->regFile.readMiscReg(misc_reg, tid);
}

template <class Impl>
MiscReg
AlphaFullCPU<Impl>::readMiscRegWithEffect(int misc_reg, Fault &fault,
                                          unsigned tid)
{
    return this->regFile.readMiscRegWithEffect(misc_reg, fault, tid);
}

template <class Impl>
Fault
AlphaFullCPU<Impl>::setMiscReg(int misc_reg, const MiscReg &val, unsigned tid)
{
    return this->regFile.setMiscReg(misc_reg, val, tid);
}

template <class Impl>
Fault
AlphaFullCPU<Impl>::setMiscRegWithEffect(int misc_reg, const MiscReg &val,
                                         unsigned tid)
{
    return this->regFile.setMiscRegWithEffect(misc_reg, val, tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::squashFromXC(unsigned tid)
{
    this->thread[tid]->inSyscall = true;
    this->commit.generateXCEvent(tid);
}

#if FULL_SYSTEM

template <class Impl>
void
AlphaFullCPU<Impl>::post_interrupt(int int_num, int index)
{
    BaseCPU::post_interrupt(int_num, index);

    if (this->thread[0]->status() == ExecContext::Suspended) {
        DPRINTF(IPI,"Suspended Processor awoke\n");
//	xcProxies[0]->activate();
        this->execContexts[0]->activate();
    }
}

template <class Impl>
int
AlphaFullCPU<Impl>::readIntrFlag()
{
    return this->regFile.readIntrFlag();
}

template <class Impl>
void
AlphaFullCPU<Impl>::setIntrFlag(int val)
{
    this->regFile.setIntrFlag(val);
}

template <class Impl>
Fault
AlphaFullCPU<Impl>::hwrei(unsigned tid)
{
    // Need to clear the lock flag upon returning from an interrupt.
    this->lockFlag = false;

    this->kernelStats->hwrei();

    this->checkInterrupts = true;

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
bool
AlphaFullCPU<Impl>::simPalCheck(int palFunc, unsigned tid)
{
    if (this->kernelStats)
        this->kernelStats->callpal(palFunc,
                                   this->execContexts[tid]);

    switch (palFunc) {
      case PAL::halt:
        halt();
        if (--System::numSystemsRunning == 0)
            new SimExitEvent("all cpus halted");
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
AlphaFullCPU<Impl>::trap(Fault fault, unsigned tid)
{
    fault->invoke(this->execContexts[tid]);
}

template <class Impl>
void
AlphaFullCPU<Impl>::processInterrupts()
{
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
        if (this->checker) {
            this->checker->cpuXCBase()->setMiscReg(IPR_ISR, summary);
            this->checker->cpuXCBase()->setMiscReg(IPR_INTID, ipl);
        }
        this->trap(Fault(new InterruptFault), 0);
        DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                this->readMiscReg(IPR_IPLR, 0), ipl, summary);
    }
}

#endif // FULL_SYSTEM

#if !FULL_SYSTEM

template <class Impl>
void
AlphaFullCPU<Impl>::syscall(int tid)
{
    DPRINTF(FullCPU, "AlphaFullCPU: [tid:%i] Executing syscall().\n\n", tid);

    DPRINTF(Activity,"Activity: syscall() called.\n");

    // Temporarily increase this by one to account for the syscall
    // instruction.
    ++(this->thread[tid]->funcExeInst);

    // Execute the actual syscall.
    this->thread[tid]->syscall();

    // Decrease funcExeInst by one as the normal commit will handle
    // incrementing it.
    --(this->thread[tid]->funcExeInst);
}

template <class Impl>
TheISA::IntReg
AlphaFullCPU<Impl>::getSyscallArg(int i, int tid)
{
    return this->readArchIntReg(AlphaISA::ArgumentReg0 + i, tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::setSyscallArg(int i, IntReg val, int tid)
{
    this->setArchIntReg(AlphaISA::ArgumentReg0 + i, val, tid);
}

template <class Impl>
void
AlphaFullCPU<Impl>::setSyscallReturn(SyscallReturn return_value, int tid)
{
    // check for error condition.  Alpha syscall convention is to
    // indicate success/failure in reg a3 (r19) and put the
    // return value itself in the standard return value reg (v0).
    if (return_value.successful()) {
        // no error
        this->setArchIntReg(SyscallSuccessReg, 0, tid);
        this->setArchIntReg(ReturnValueReg, return_value.value(), tid);
    } else {
        // got an error, return details
        this->setArchIntReg(SyscallSuccessReg, (IntReg) -1, tid);
        this->setArchIntReg(ReturnValueReg, -return_value.value(), tid);
    }
}
#endif
