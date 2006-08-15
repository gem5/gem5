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
 *          Korey Sewell
 */

#include "cpu/o3/thread_context.hh"
#include "cpu/quiesce_event.hh"

#if FULL_SYSTEM
template <class Impl>
VirtualPort *
O3ThreadContext<Impl>::getVirtPort(ThreadContext *src_tc)
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
O3ThreadContext<Impl>::dumpFuncProfile()
{
    // Currently not supported
}
#endif

template <class Impl>
void
O3ThreadContext<Impl>::takeOverFrom(ThreadContext *old_context)
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
O3ThreadContext<Impl>::delVirtPort(VirtualPort *vp)
{
    delete vp->getPeer();
    delete vp;
}
#endif

template <class Impl>
void
O3ThreadContext<Impl>::activate(int delay)
{
    DPRINTF(O3CPU, "Calling activate on Thread Context %d\n",
            getThreadNum());

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
O3ThreadContext<Impl>::suspend()
{
    DPRINTF(O3CPU, "Calling suspend on Thread Context %d\n",
            getThreadNum());

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
O3ThreadContext<Impl>::deallocate(int delay)
{
    DPRINTF(O3CPU, "Calling deallocate on Thread Context %d\n",
            getThreadNum());

    if (thread->status() == ThreadContext::Unallocated)
        return;

    thread->setStatus(ThreadContext::Unallocated);
    cpu->deallocateContext(thread->readTid(), delay);
}

template <class Impl>
void
O3ThreadContext<Impl>::halt()
{
    DPRINTF(O3CPU, "Calling halt on Thread Context %d\n",
            getThreadNum());

    if (thread->status() == ThreadContext::Halted)
        return;

    thread->setStatus(ThreadContext::Halted);
    cpu->haltContext(thread->readTid());
}

template <class Impl>
void
O3ThreadContext<Impl>::regStats(const std::string &name)
{
#if FULL_SYSTEM
    thread->kernelStats = new Kernel::Statistics(cpu->system);
    thread->kernelStats->regStats(name + ".kern");
#endif
}

template <class Impl>
void
O3ThreadContext<Impl>::serialize(std::ostream &os)
{
#if FULL_SYSTEM
    if (thread->kernelStats)
        thread->kernelStats->serialize(os);
#endif

}

template <class Impl>
void
O3ThreadContext<Impl>::unserialize(Checkpoint *cp, const std::string &section)
{
#if FULL_SYSTEM
    if (thread->kernelStats)
        thread->kernelStats->unserialize(cp, section);
#endif

}

#if FULL_SYSTEM
template <class Impl>
Tick
O3ThreadContext<Impl>::readLastActivate()
{
    return thread->lastActivate;
}

template <class Impl>
Tick
O3ThreadContext<Impl>::readLastSuspend()
{
    return thread->lastSuspend;
}

template <class Impl>
void
O3ThreadContext<Impl>::profileClear()
{}

template <class Impl>
void
O3ThreadContext<Impl>::profileSample()
{}
#endif

template <class Impl>
TheISA::MachInst
O3ThreadContext<Impl>:: getInst()
{
    return thread->getInst();
}

template <class Impl>
void
O3ThreadContext<Impl>::copyArchRegs(ThreadContext *tc)
{
    // This function will mess things up unless the ROB is empty and
    // there are no instructions in the pipeline.
    unsigned tid = thread->readTid();
    PhysRegIndex renamed_reg;

    // First loop through the integer registers.
    for (int i = 0; i < TheISA::NumIntRegs; ++i) {
        renamed_reg = cpu->renameMap[tid].lookup(i);

        DPRINTF(O3CPU, "Copying over register %i, had data %lli, "
                "now has data %lli.\n",
                renamed_reg, cpu->readIntReg(renamed_reg),
                tc->readIntReg(i));

        cpu->setIntReg(renamed_reg, tc->readIntReg(i));
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < TheISA::NumFloatRegs; ++i) {
        renamed_reg = cpu->renameMap[tid].lookup(i + TheISA::FP_Base_DepTag);
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
O3ThreadContext<Impl>::clearArchRegs()
{}

template <class Impl>
uint64_t
O3ThreadContext<Impl>::readIntReg(int reg_idx)
{
    return cpu->readArchIntReg(reg_idx, thread->readTid());
}

template <class Impl>
FloatReg
O3ThreadContext<Impl>::readFloatReg(int reg_idx, int width)
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
O3ThreadContext<Impl>::readFloatReg(int reg_idx)
{
    return cpu->readArchFloatRegSingle(reg_idx, thread->readTid());
}

template <class Impl>
FloatRegBits
O3ThreadContext<Impl>::readFloatRegBits(int reg_idx, int width)
{
    DPRINTF(Fault, "Reading floatint register through the TC!\n");
    return cpu->readArchFloatRegInt(reg_idx, thread->readTid());
}

template <class Impl>
FloatRegBits
O3ThreadContext<Impl>::readFloatRegBits(int reg_idx)
{
    return cpu->readArchFloatRegInt(reg_idx, thread->readTid());
}

template <class Impl>
void
O3ThreadContext<Impl>::setIntReg(int reg_idx, uint64_t val)
{
    cpu->setArchIntReg(reg_idx, val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
O3ThreadContext<Impl>::setFloatReg(int reg_idx, FloatReg val, int width)
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
O3ThreadContext<Impl>::setFloatReg(int reg_idx, FloatReg val)
{
    cpu->setArchFloatRegSingle(reg_idx, val, thread->readTid());

    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
O3ThreadContext<Impl>::setFloatRegBits(int reg_idx, FloatRegBits val,
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
O3ThreadContext<Impl>::setFloatRegBits(int reg_idx, FloatRegBits val)
{
    cpu->setArchFloatRegInt(reg_idx, val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
O3ThreadContext<Impl>::setPC(uint64_t val)
{
    cpu->setPC(val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
void
O3ThreadContext<Impl>::setNextPC(uint64_t val)
{
    cpu->setNextPC(val, thread->readTid());

    // Squash if we're not already in a state update mode.
    if (!thread->trapPending && !thread->inSyscall) {
        cpu->squashFromTC(thread->readTid());
    }
}

template <class Impl>
Fault
O3ThreadContext<Impl>::setMiscReg(int misc_reg, const MiscReg &val)
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
O3ThreadContext<Impl>::setMiscRegWithEffect(int misc_reg,
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
O3ThreadContext<Impl>::getSyscallArg(int i)
{
    return cpu->getSyscallArg(i, thread->readTid());
}

template <class Impl>
void
O3ThreadContext<Impl>::setSyscallArg(int i, IntReg val)
{
    cpu->setSyscallArg(i, val, thread->readTid());
}

template <class Impl>
void
O3ThreadContext<Impl>::setSyscallReturn(SyscallReturn return_value)
{
    cpu->setSyscallReturn(return_value, thread->readTid());
}

#endif // FULL_SYSTEM

