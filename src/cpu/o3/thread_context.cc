/*
 * Copyright (c) 2010-2012, 2016-2017, 2019 ARM Limited
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

#include "cpu/o3/thread_context.hh"

#include "arch/vecregs.hh"
#include "config/the_isa.hh"
#include "debug/O3CPU.hh"

namespace gem5
{

namespace o3
{

PortProxy&
ThreadContext::getVirtProxy()
{
    return thread->getVirtProxy();
}

void
ThreadContext::takeOverFrom(gem5::ThreadContext *old_context)
{
    gem5::takeOverFrom(*this, *old_context);

    getIsaPtr()->takeOverFrom(this, old_context);

    TheISA::Decoder *newDecoder = getDecoderPtr();
    TheISA::Decoder *oldDecoder = old_context->getDecoderPtr();
    newDecoder->takeOverFrom(oldDecoder);

    thread->noSquashFromTC = false;
    thread->trapPending = false;
}

void
ThreadContext::activate()
{
    DPRINTF(O3CPU, "Calling activate on Thread Context %d\n",
            threadId());

    if (thread->status() == gem5::ThreadContext::Active)
        return;

    thread->lastActivate = curTick();
    thread->setStatus(gem5::ThreadContext::Active);

    // status() == Suspended
    cpu->activateContext(thread->threadId());
}

void
ThreadContext::suspend()
{
    DPRINTF(O3CPU, "Calling suspend on Thread Context %d\n",
            threadId());

    if (thread->status() == gem5::ThreadContext::Suspended)
        return;

    if (cpu->isDraining()) {
        DPRINTF(O3CPU, "Ignoring suspend on TC due to pending drain\n");
        return;
    }

    thread->lastActivate = curTick();
    thread->lastSuspend = curTick();

    thread->setStatus(gem5::ThreadContext::Suspended);
    cpu->suspendContext(thread->threadId());
}

void
ThreadContext::halt()
{
    DPRINTF(O3CPU, "Calling halt on Thread Context %d\n", threadId());

    if (thread->status() == gem5::ThreadContext::Halting ||
        thread->status() == gem5::ThreadContext::Halted)
        return;

    // the thread is not going to halt/terminate immediately in this cycle.
    // The thread will be removed after an exit trap is processed
    // (e.g., after trapLatency cycles). Until then, the thread's status
    // will be Halting.
    thread->setStatus(gem5::ThreadContext::Halting);

    // add this thread to the exiting list to mark that it is trying to exit.
    cpu->addThreadToExitingList(thread->threadId());
}

Tick
ThreadContext::readLastActivate()
{
    return thread->lastActivate;
}

Tick
ThreadContext::readLastSuspend()
{
    return thread->lastSuspend;
}

void
ThreadContext::copyArchRegs(gem5::ThreadContext *tc)
{
    // Set vector renaming mode before copying registers
    cpu->vecRenameMode(tc->getIsaPtr()->vecRegRenameMode(tc));

    // Prevent squashing
    thread->noSquashFromTC = true;
    getIsaPtr()->copyRegsFrom(tc);
    thread->noSquashFromTC = false;
}

void
ThreadContext::clearArchRegs()
{
    cpu->isa[thread->threadId()]->clear();
}

RegVal
ThreadContext::readIntRegFlat(RegIndex reg_idx) const
{
    return cpu->readArchIntReg(reg_idx, thread->threadId());
}

RegVal
ThreadContext::readFloatRegFlat(RegIndex reg_idx) const
{
    return cpu->readArchFloatReg(reg_idx, thread->threadId());
}

const TheISA::VecRegContainer&
ThreadContext::readVecRegFlat(RegIndex reg_id) const
{
    return cpu->readArchVecReg(reg_id, thread->threadId());
}

TheISA::VecRegContainer&
ThreadContext::getWritableVecRegFlat(RegIndex reg_id)
{
    return cpu->getWritableArchVecReg(reg_id, thread->threadId());
}

const TheISA::VecElem&
ThreadContext::readVecElemFlat(RegIndex idx, const ElemIndex& elemIndex) const
{
    return cpu->readArchVecElem(idx, elemIndex, thread->threadId());
}

const TheISA::VecPredRegContainer&
ThreadContext::readVecPredRegFlat(RegIndex reg_id) const
{
    return cpu->readArchVecPredReg(reg_id, thread->threadId());
}

TheISA::VecPredRegContainer&
ThreadContext::getWritableVecPredRegFlat(RegIndex reg_id)
{
    return cpu->getWritableArchVecPredReg(reg_id, thread->threadId());
}

RegVal
ThreadContext::readCCRegFlat(RegIndex reg_idx) const
{
    return cpu->readArchCCReg(reg_idx, thread->threadId());
}

void
ThreadContext::setIntRegFlat(RegIndex reg_idx, RegVal val)
{
    cpu->setArchIntReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

void
ThreadContext::setFloatRegFlat(RegIndex reg_idx, RegVal val)
{
    cpu->setArchFloatReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

void
ThreadContext::setVecRegFlat(
        RegIndex reg_idx, const TheISA::VecRegContainer& val)
{
    cpu->setArchVecReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

void
ThreadContext::setVecElemFlat(RegIndex idx,
        const ElemIndex& elemIndex, const TheISA::VecElem& val)
{
    cpu->setArchVecElem(idx, elemIndex, val, thread->threadId());
    conditionalSquash();
}

void
ThreadContext::setVecPredRegFlat(RegIndex reg_idx,
        const TheISA::VecPredRegContainer& val)
{
    cpu->setArchVecPredReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

void
ThreadContext::setCCRegFlat(RegIndex reg_idx, RegVal val)
{
    cpu->setArchCCReg(reg_idx, val, thread->threadId());

    conditionalSquash();
}

void
ThreadContext::pcState(const TheISA::PCState &val)
{
    cpu->pcState(val, thread->threadId());

    conditionalSquash();
}

void
ThreadContext::pcStateNoRecord(const TheISA::PCState &val)
{
    cpu->pcState(val, thread->threadId());

    conditionalSquash();
}

RegId
ThreadContext::flattenRegId(const RegId& regId) const
{
    return cpu->isa[thread->threadId()]->flattenRegId(regId);
}

void
ThreadContext::setMiscRegNoEffect(RegIndex misc_reg, RegVal val)
{
    cpu->setMiscRegNoEffect(misc_reg, val, thread->threadId());

    conditionalSquash();
}

void
ThreadContext::setMiscReg(RegIndex misc_reg, RegVal val)
{
    cpu->setMiscReg(misc_reg, val, thread->threadId());

    conditionalSquash();
}

// hardware transactional memory
void
ThreadContext::htmAbortTransaction(uint64_t htmUid,
        HtmFailureFaultCause cause)
{
    cpu->htmSendAbortSignal(thread->threadId(), htmUid, cause);

    conditionalSquash();
}

BaseHTMCheckpointPtr&
ThreadContext::getHtmCheckpointPtr()
{
    return thread->htmCheckpoint;
}

void
ThreadContext::setHtmCheckpointPtr(BaseHTMCheckpointPtr new_cpt)
{
    thread->htmCheckpoint = std::move(new_cpt);
}

} // namespace o3
} // namespace gem5
