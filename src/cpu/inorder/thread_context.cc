/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 *
 */

#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/thread_context.hh"
#include "cpu/exetrace.hh"
#include "debug/InOrderCPU.hh"
#include "sim/full_system.hh"

using namespace TheISA;

FSTranslatingPortProxy&
InOrderThreadContext::getVirtProxy()
{
    return thread->getVirtProxy();
}

void
InOrderThreadContext::dumpFuncProfile()
{
    thread->dumpFuncProfile();
}


Tick
InOrderThreadContext::readLastActivate()
{
    return thread->lastActivate;
}


Tick
InOrderThreadContext::readLastSuspend()
{
    return thread->lastSuspend;
}


void
InOrderThreadContext::profileClear()
{
    thread->profileClear();
}


void
InOrderThreadContext::profileSample()
{
    thread->profileSample();
}

void
InOrderThreadContext::takeOverFrom(ThreadContext *old_context)
{
    // some things should already be set up
    assert(getSystemPtr() == old_context->getSystemPtr());
    assert(getProcessPtr() == old_context->getProcessPtr());

    // copy over functional state
    setStatus(old_context->status());
    copyArchRegs(old_context);

    thread->funcExeInst = old_context->readFuncExeInst();
 
    old_context->setStatus(ThreadContext::Halted);

    thread->inSyscall = false;
    thread->trapPending = false;
}

void
InOrderThreadContext::activate(int delay)
{
    DPRINTF(InOrderCPU, "Calling activate on Thread Context %d\n",
            getThreadNum());

    if (thread->status() == ThreadContext::Active)
        return;

    thread->setStatus(ThreadContext::Active);

    cpu->activateContext(thread->threadId(), delay);
}


void
InOrderThreadContext::suspend(int delay)
{
    DPRINTF(InOrderCPU, "Calling suspend on Thread Context %d\n",
            getThreadNum());

    if (thread->status() == ThreadContext::Suspended)
        return;

    thread->setStatus(ThreadContext::Suspended);
    cpu->suspendContext(thread->threadId());
}

void
InOrderThreadContext::halt(int delay)
{
    DPRINTF(InOrderCPU, "Calling halt on Thread Context %d\n",
            getThreadNum());

    if (thread->status() == ThreadContext::Halted)
        return;

    thread->setStatus(ThreadContext::Halted);
    cpu->haltContext(thread->threadId());
}


void
InOrderThreadContext::regStats(const std::string &name)
{
    if (FullSystem) {
        thread->kernelStats = new TheISA::Kernel::Statistics(cpu->system);
        thread->kernelStats->regStats(name + ".kern");
    }
}


void
InOrderThreadContext::serialize(std::ostream &os)
{
    panic("serialize unimplemented");
}


void
InOrderThreadContext::unserialize(Checkpoint *cp, const std::string &section)
{
    panic("unserialize unimplemented");    
}


void
InOrderThreadContext::copyArchRegs(ThreadContext *src_tc)
{
    TheISA::copyRegs(src_tc, this);
}


void
InOrderThreadContext::clearArchRegs()
{
    cpu->isa[thread->threadId()].clear();
}


uint64_t
InOrderThreadContext::readIntReg(int reg_idx)
{
    ThreadID tid = thread->threadId();
    reg_idx = cpu->isa[tid].flattenIntIndex(reg_idx);
    return cpu->readIntReg(reg_idx, tid);
}

FloatReg
InOrderThreadContext::readFloatReg(int reg_idx)
{
    ThreadID tid = thread->threadId();
    reg_idx = cpu->isa[tid].flattenFloatIndex(reg_idx);
    return cpu->readFloatReg(reg_idx, tid);
}

FloatRegBits
InOrderThreadContext::readFloatRegBits(int reg_idx)
{
    ThreadID tid = thread->threadId();
    reg_idx = cpu->isa[tid].flattenFloatIndex(reg_idx);
    return cpu->readFloatRegBits(reg_idx, tid);
}

uint64_t
InOrderThreadContext::readRegOtherThread(int reg_idx, ThreadID tid)
{
    return cpu->readRegOtherThread(reg_idx, tid);
}

void
InOrderThreadContext::setIntReg(int reg_idx, uint64_t val)
{
    ThreadID tid = thread->threadId();
    reg_idx = cpu->isa[tid].flattenIntIndex(reg_idx);
    cpu->setIntReg(reg_idx, val, tid);
}

void
InOrderThreadContext::setFloatReg(int reg_idx, FloatReg val)
{
    ThreadID tid = thread->threadId();
    reg_idx = cpu->isa[tid].flattenFloatIndex(reg_idx);
    cpu->setFloatReg(reg_idx, val, tid);
}

void
InOrderThreadContext::setFloatRegBits(int reg_idx, FloatRegBits val)
{
    ThreadID tid = thread->threadId();
    reg_idx = cpu->isa[tid].flattenFloatIndex(reg_idx);
    cpu->setFloatRegBits(reg_idx, val, tid);
}

void
InOrderThreadContext::setRegOtherThread(int misc_reg, const MiscReg &val,
                                        ThreadID tid)
{
    cpu->setRegOtherThread(misc_reg, val, tid);
}

void
InOrderThreadContext::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    cpu->setMiscRegNoEffect(misc_reg, val, thread->threadId());
}

void
InOrderThreadContext::setMiscReg(int misc_reg, const MiscReg &val)
{
    cpu->setMiscReg(misc_reg, val, thread->threadId());
}
