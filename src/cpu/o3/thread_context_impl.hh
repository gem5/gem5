/*
 * Copyright (c) 2010-2012, 2016-2017 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#ifndef __CPU_O3_THREAD_CONTEXT_IMPL_HH__
#define __CPU_O3_THREAD_CONTEXT_IMPL_HH__

#include "arch/generic/traits.hh"
#include "arch/kernel_stats.hh"
#include "arch/registers.hh"
#include "config/the_isa.hh"
#include "cpu/o3/thread_context.hh"
#include "cpu/quiesce_event.hh"
#include "debug/O3CPU.hh"

template <class Impl>
PortProxy&
O3ThreadContext<Impl>::getVirtProxy()
{
    return thread->getVirtProxy();
}

template <class Impl>
void
O3ThreadContext<Impl>::dumpFuncProfile()
{
    thread->dumpFuncProfile();
}

template <class Impl>
void
O3ThreadContext<Impl>::takeOverFrom(ThreadContext *old_context)
{
    ::takeOverFrom(*this, *old_context);

    this->getIsaPtr()->takeOverFrom(this, old_context);

    TheISA::Decoder *newDecoder = getDecoderPtr();
    TheISA::Decoder *oldDecoder = old_context->getDecoderPtr();
    newDecoder->takeOverFrom(oldDecoder);

    thread->kernelStats = old_context->getKernelStats();
    thread->funcExeInst = old_context->readFuncExeInst();

    thread->noSquashFromTC = false;
    thread->trapPending = false;
}

template <class Impl>
void
O3ThreadContext<Impl>::activate()
{
    DPRINTF(O3CPU, "Calling activate on Thread Context %d\n",
            threadId());

    if (thread->status() == ThreadContext::Active)
        return;

    thread->lastActivate = curTick();
    thread->setStatus(ThreadContext::Active);

    // status() == Suspended
    cpu->activateContext(thread->threadId());
}

template <class Impl>
void
O3ThreadContext<Impl>::suspend()
{
    DPRINTF(O3CPU, "Calling suspend on Thread Context %d\n",
            threadId());

    if (thread->status() == ThreadContext::Suspended)
        return;

    if (cpu->isDraining()) {
        DPRINTF(O3CPU, "Ignoring suspend on TC due to pending drain\n");
        return;
    }

    thread->lastActivate = curTick();
    thread->lastSuspend = curTick();

    thread->setStatus(ThreadContext::Suspended);
    cpu->suspendContext(thread->threadId());
}

template <class Impl>
void
O3ThreadContext<Impl>::halt()
{
    DPRINTF(O3CPU, "Calling halt on Thread Context %d\n", threadId());

    if (thread->status() == ThreadContext::Halting ||
        thread->status() == ThreadContext::Halted)
        return;

    // the thread is not going to halt/terminate immediately in this cycle.
    // The thread will be removed after an exit trap is processed
    // (e.g., after trapLatency cycles). Until then, the thread's status
    // will be Halting.
    thread->setStatus(ThreadContext::Halting);

    // add this thread to the exiting list to mark that it is trying to exit.
    cpu->addThreadToExitingList(thread->threadId());
}

template <class Impl>
void
O3ThreadContext<Impl>::regStats(const std::string &name)
{
    if (FullSystem) {
        thread->kernelStats = new TheISA::Kernel::Statistics();
        thread->kernelStats->regStats(name + ".kern");
    }
}

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
{
    thread->profileClear();
}

template <class Impl>
void
O3ThreadContext<Impl>::profileSample()
{
    thread->profileSample();
}

template <class Impl>
void
O3ThreadContext<Impl>::copyArchRegs(ThreadContext *tc)
{
    // Set vector renaming mode before copying registers
    cpu->vecRenameMode(RenameMode<TheISA::ISA>::mode(tc->pcState()));

    // Prevent squashing
    thread->noSquashFromTC = true;
    TheISA::copyRegs(tc, this);
    thread->noSquashFromTC = false;

    if (!FullSystem)
        this->thread->funcExeInst = tc->readFuncExeInst();
}

template <class Impl>
void
O3ThreadContext<Impl>::clearArchRegs()
{
    cpu->isa[thread->threadId()]->clear(this);
}

template <class Impl>
RegVal
O3ThreadContext<Impl>::readIntRegFlat(RegIndex reg_idx) const
{
    return cpu->readArchIntReg(reg_idx, thread->threadId());
}

template <class Impl>
RegVal
O3ThreadContext<Impl>::readFloatRegFlat(RegIndex reg_idx) const
{
    return cpu->readArchFloatReg(reg_idx, thread->threadId());
}

template <class Impl>
const TheISA::VecRegContainer&
O3ThreadContext<Impl>::readVecRegFlat(RegIndex reg_id) const
{
    return cpu->readArchVecReg(reg_id, thread->threadId());
}

template <class Impl>
TheISA::VecRegContainer&
O3ThreadContext<Impl>::getWritableVecRegFlat(RegIndex reg_id)
{
    return cpu->getWritableArchVecReg(reg_id, thread->threadId());
}

template <class Impl>
const TheISA::VecElem&
O3ThreadContext<Impl>::readVecElemFlat(RegIndex idx,
                                       const ElemIndex& elemIndex) const
{
    return cpu->readArchVecElem(idx, elemIndex, thread->threadId());
}

template <class Impl>
const TheISA::VecPredRegContainer&
O3ThreadContext<Impl>::readVecPredRegFlat(RegIndex reg_id) const
{
    return cpu->readArchVecPredReg(reg_id, thread->threadId());
}

template <class Impl>
TheISA::VecPredRegContainer&
O3ThreadContext<Impl>::getWritableVecPredRegFlat(RegIndex reg_id)
{
    return cpu->getWritableArchVecPredReg(reg_id, thread->threadId());
}

template <class Impl>
RegVal
O3ThreadContext<Impl>::readCCRegFlat(RegIndex reg_idx) const
{
    return cpu->readArchCCReg(reg_idx, thread->threadId());
}

template <class Impl>
void
O3ThreadContext<Impl>::setIntRegFlat(RegIndex reg_idx, RegVal val)
{
    cpu->setArchIntReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

template <class Impl>
void
O3ThreadContext<Impl>::setFloatRegFlat(RegIndex reg_idx, RegVal val)
{
    cpu->setArchFloatReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

template <class Impl>
void
O3ThreadContext<Impl>::setVecRegFlat(
        RegIndex reg_idx, const VecRegContainer& val)
{
    cpu->setArchVecReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

template <class Impl>
void
O3ThreadContext<Impl>::setVecElemFlat(RegIndex idx,
        const ElemIndex& elemIndex, const VecElem& val)
{
    cpu->setArchVecElem(idx, elemIndex, val, thread->threadId());
    conditionalSquash();
}

template <class Impl>
void
O3ThreadContext<Impl>::setVecPredRegFlat(RegIndex reg_idx,
                                         const VecPredRegContainer& val)
{
    cpu->setArchVecPredReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

template <class Impl>
void
O3ThreadContext<Impl>::setCCRegFlat(RegIndex reg_idx, RegVal val)
{
    cpu->setArchCCReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

template <class Impl>
void
O3ThreadContext<Impl>::pcState(const TheISA::PCState &val)
{
    cpu->pcState(val, thread->threadId());

    conditionalSquash();
}

template <class Impl>
void
O3ThreadContext<Impl>::pcStateNoRecord(const TheISA::PCState &val)
{
    cpu->pcState(val, thread->threadId());

    conditionalSquash();
}

template <class Impl>
RegId
O3ThreadContext<Impl>::flattenRegId(const RegId& regId) const
{
    return cpu->isa[thread->threadId()]->flattenRegId(regId);
}

template <class Impl>
void
O3ThreadContext<Impl>::setMiscRegNoEffect(RegIndex misc_reg, RegVal val)
{
    cpu->setMiscRegNoEffect(misc_reg, val, thread->threadId());

    conditionalSquash();
}

template <class Impl>
void
O3ThreadContext<Impl>::setMiscReg(RegIndex misc_reg, RegVal val)
{
    cpu->setMiscReg(misc_reg, val, thread->threadId());

    conditionalSquash();
}

#endif //__CPU_O3_THREAD_CONTEXT_IMPL_HH__
