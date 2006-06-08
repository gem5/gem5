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

#include "arch/alpha/faults.hh"
#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/checker/thread_context.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"

#include "cpu/o3/alpha_cpu.hh"
#include "cpu/o3/alpha_params.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/thread_state.hh"

#if FULL_SYSTEM
#include "arch/alpha/osfpal.hh"
#include "arch/isa_traits.hh"
#include "cpu/quiesce_event.hh"
#include "kern/kernel_stats.hh"
#include "sim/system.hh"
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
            DPRINTF(FullCPU, "FullCPU: Workload[%i] process is %#x",
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
        AlphaTC *alpha_tc = new AlphaTC;

        // If we're using a checker, then the TC should be the
        // CheckerThreadContext.
        if (params->checker) {
            tc = new CheckerThreadContext<AlphaTC>(
                alpha_tc, this->checker);
        } else {
            tc = alpha_tc;
        }

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
VirtualPort *
AlphaFullCPU<Impl>::AlphaTC::getVirtPort(ThreadContext *src_tc)
{
    if (!src_tc)
        return thread->getVirtPort();

    VirtualPort *vp;
    Port *mem_port;

    vp = new VirtualPort("tc-vport", src_tc);
    mem_port = cpu->system->physmem->getPort("functional");
    mem_port->setPeer(vp);
    vp->setPeer(mem_port);
    return vp;
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::dumpFuncProfile()
{
    // Currently not supported
}
#endif

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::takeOverFrom(ThreadContext *old_context)
{
    // some things should already be set up
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
        // Point the quiesce event's TC at this TC so that it wakes up
        // the proper CPU.
        other_quiesce->tc = this;
    }
    if (thread->quiesceEvent) {
        thread->quiesceEvent->tc = this;
    }

    // Transfer kernel stats from one CPU to the other.
    thread->kernelStats = old_context->getKernelStats();
//    storeCondFailures = 0;
    cpu->lockFlag = false;
#endif

    old_context->setStatus(ThreadContext::Unallocated);

    thread->inSyscall = false;
    thread->trapPending = false;
}

#if FULL_SYSTEM
template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::delVirtPort(VirtualPort *vp)
{
    delete vp->getPeer();
    delete vp;
}
#endif

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::activate(int delay)
{
    DPRINTF(FullCPU, "Calling activate on AlphaTC\n");

    if (thread->status() == ThreadContext::Active)
        return;

#if FULL_SYSTEM
    thread->lastActivate = curTick;
#endif

    if (thread->status() == ThreadContext::Unallocated) {
        cpu->activateWhenReady(thread->readTid());
        return;
    }

    thread->setStatus(ThreadContext::Active);

    // status() == Suspended
    cpu->activateContext(thread->readTid(), delay);
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::suspend()
{
    DPRINTF(FullCPU, "Calling suspend on AlphaTC\n");

    if (thread->status() == ThreadContext::Suspended)
        return;

#if FULL_SYSTEM
    thread->lastActivate = curTick;
    thread->lastSuspend = curTick;
#endif
/*
#if FULL_SYSTEM
    // Don't change the status from active if there are pending interrupts
    if (cpu->check_interrupts()) {
        assert(status() == ThreadContext::Active);
        return;
    }
#endif
*/
    thread->setStatus(ThreadContext::Suspended);
    cpu->suspendContext(thread->readTid());
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::deallocate()
{
    DPRINTF(FullCPU, "Calling deallocate on AlphaTC\n");

    if (thread->status() == ThreadContext::Unallocated)
        return;

    thread->setStatus(ThreadContext::Unallocated);
    cpu->deallocateContext(thread->readTid());
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::halt()
{
    DPRINTF(FullCPU, "Calling halt on AlphaTC\n");

    if (thread->status() == ThreadContext::Halted)
        return;

    thread->setStatus(ThreadContext::Halted);
    cpu->haltContext(thread->readTid());
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::regStats(const std::string &name)
{
#if FULL_SYSTEM
    thread->kernelStats = new Kernel::Statistics(cpu->system);
    thread->kernelStats->regStats(name + ".kern");
#endif
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::serialize(std::ostream &os)
{
#if FULL_SYSTEM
    if (thread->kernelStats)
        thread->kernelStats->serialize(os);
#endif

}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::unserialize(Checkpoint *cp, const std::string &section)
{
#if FULL_SYSTEM
    if (thread->kernelStats)
        thread->kernelStats->unserialize(cp, section);
#endif

}

#if FULL_SYSTEM
template <class Impl>
EndQuiesceEvent *
AlphaFullCPU<Impl>::AlphaTC::getQuiesceEvent()
{
    return thread->quiesceEvent;
}

template <class Impl>
Tick
AlphaFullCPU<Impl>::AlphaTC::readLastActivate()
{
    return thread->lastActivate;
}

template <class Impl>
Tick
AlphaFullCPU<Impl>::AlphaTC::readLastSuspend()
{
    return thread->lastSuspend;
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::profileClear()
{}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::profileSample()
{}
#endif

template <class Impl>
TheISA::MachInst
AlphaFullCPU<Impl>::AlphaTC:: getInst()
{
    return thread->getInst();
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::copyArchRegs(ThreadContext *tc)
{
    // This function will mess things up unless the ROB is empty and
    // there are no instructions in the pipeline.
    unsigned tid = thread->readTid();
    PhysRegIndex renamed_reg;

    // First loop through the integer registers.
    for (int i = 0; i < AlphaISA::NumIntRegs; ++i) {
        renamed_reg = cpu->renameMap[tid].lookup(i);

        DPRINTF(FullCPU, "FullCPU: Copying over register %i, had data %lli, "
                "now has data %lli.\n",
                renamed_reg, cpu->readIntReg(renamed_reg),
                tc->readIntReg(i));

        cpu->setIntReg(renamed_reg, tc->readIntReg(i));
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < AlphaISA::NumFloatRegs; ++i) {
        renamed_reg = cpu->renameMap[tid].lookup(i + AlphaISA::FP_Base_DepTag);
        cpu->setFloatRegBits(renamed_reg,
                             tc->readFloatRegBits(i));
    }

    // Copy the misc regs.
    copyMiscRegs(tc, this);

    // Then finally set the PC and the next PC.
    cpu->setPC(tc->readPC(), tid);
    cpu->setNextPC(tc->readNextPC(), tid);
#if !FULL_SYSTEM
    this->thread->funcExeInst = tc->readFuncExeInst();
#endif
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::clearArchRegs()
{}

template <class Impl>
uint64_t
AlphaFullCPU<Impl>::AlphaTC::readIntReg(int reg_idx)
{
    return cpu->readArchIntReg(reg_idx, thread->readTid());
}

template <class Impl>
FloatReg
AlphaFullCPU<Impl>::AlphaTC::readFloatReg(int reg_idx, int width)
{
    switch(width) {
      case 32:
        return cpu->readArchFloatRegSingle(reg_idx, thread->readTid());
      case 64:
        return cpu->readArchFloatRegDouble(reg_idx, thread->readTid());
      default:
        panic("Unsupported width!");
        return 0;
    }
}

template <class Impl>
FloatReg
AlphaFullCPU<Impl>::AlphaTC::readFloatReg(int reg_idx)
{
    return cpu->readArchFloatRegSingle(reg_idx, thread->readTid());
}

template <class Impl>
FloatRegBits
AlphaFullCPU<Impl>::AlphaTC::readFloatRegBits(int reg_idx, int width)
{
    DPRINTF(Fault, "Reading floatint register through the TC!\n");
    return cpu->readArchFloatRegInt(reg_idx, thread->readTid());
}

template <class Impl>
FloatRegBits
AlphaFullCPU<Impl>::AlphaTC::readFloatRegBits(int reg_idx)
{
    return cpu->readArchFloatRegInt(reg_idx, thread->readTid());
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setIntReg(int reg_idx, uint64_t val)
{
    cpu->setArchIntReg(reg_idx, val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setFloatReg(int reg_idx, FloatReg val, int width)
{
    switch(width) {
      case 32:
        cpu->setArchFloatRegSingle(reg_idx, val, thread->readTid());
        break;
      case 64:
        cpu->setArchFloatRegDouble(reg_idx, val, thread->readTid());
        break;
    }

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setFloatReg(int reg_idx, FloatReg val)
{
    cpu->setArchFloatRegSingle(reg_idx, val, thread->readTid());

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setFloatRegBits(int reg_idx, FloatRegBits val,
                                             int width)
{
    DPRINTF(Fault, "Setting floatint register through the TC!\n");
    cpu->setArchFloatRegInt(reg_idx, val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setFloatRegBits(int reg_idx, FloatRegBits val)
{
    cpu->setArchFloatRegInt(reg_idx, val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setPC(uint64_t val)
{
    cpu->setPC(val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setNextPC(uint64_t val)
{
    cpu->setNextPC(val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
Fault
AlphaFullCPU<Impl>::AlphaTC::setMiscReg(int misc_reg, const MiscReg &val)
{
    Fault ret_fault = cpu->setMiscReg(misc_reg, val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }

    return ret_fault;
}

template <class Impl>
Fault
AlphaFullCPU<Impl>::AlphaTC::setMiscRegWithEffect(int misc_reg,
                                                  const MiscReg &val)
{
    Fault ret_fault = cpu->setMiscRegWithEffect(misc_reg, val,
                                                thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }

    return ret_fault;
}

#if !FULL_SYSTEM

template <class Impl>
TheISA::IntReg
AlphaFullCPU<Impl>::AlphaTC::getSyscallArg(int i)
{
    return cpu->getSyscallArg(i, thread->readTid());
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setSyscallArg(int i, IntReg val)
{
    cpu->setSyscallArg(i, val, thread->readTid());
}

template <class Impl>
void
AlphaFullCPU<Impl>::AlphaTC::setSyscallReturn(SyscallReturn return_value)
{
    cpu->setSyscallReturn(return_value, thread->readTid());
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
AlphaFullCPU<Impl>::squashFromTC(unsigned tid)
{
    this->thread[tid]->inSyscall = true;
    this->commit.generateTCEvent(tid);
}

#if FULL_SYSTEM

template <class Impl>
void
AlphaFullCPU<Impl>::post_interrupt(int int_num, int index)
{
    BaseCPU::post_interrupt(int_num, index);

    if (this->thread[0]->status() == ThreadContext::Suspended) {
        DPRINTF(IPI,"Suspended Processor awoke\n");
        this->threadContexts[0]->activate();
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

    this->thread[tid]->kernelStats->hwrei();

    this->checkInterrupts = true;

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
bool
AlphaFullCPU<Impl>::simPalCheck(int palFunc, unsigned tid)
{
    if (this->thread[tid]->kernelStats)
        this->thread[tid]->kernelStats->callpal(palFunc,
                                                this->threadContexts[tid]);

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
    // Pass the thread's TC into the invoke method.
    fault->invoke(this->threadContexts[tid]);
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
        // Checker needs to know these two registers were updated.
        if (this->checker) {
            this->checker->threadBase()->setMiscReg(IPR_ISR, summary);
            this->checker->threadBase()->setMiscReg(IPR_INTID, ipl);
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
AlphaFullCPU<Impl>::syscall(int64_t callnum, int tid)
{
    DPRINTF(FullCPU, "AlphaFullCPU: [tid:%i] Executing syscall().\n\n", tid);

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
