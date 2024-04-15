/*
 * Copyright (c) 2018, 2020 ARM Limited
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
 * Copyright (c) 2001-2006 The Regents of The University of Michigan
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

#include "cpu/simple_thread.hh"

#include <string>

#include "arch/generic/decoder.hh"
#include "base/callback.hh"
#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/simple/base.hh"
#include "cpu/thread_context.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "params/BaseCPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

// constructor
SimpleThread::SimpleThread(BaseCPU *_cpu, int _thread_num, System *_sys,
                           Process *_process, BaseMMU *_mmu, BaseISA *_isa,
                           InstDecoder *_decoder)
    : ThreadState(_cpu, _thread_num, _process),
      regFiles{ { { *_isa->regClasses().at(IntRegClass) },
                  { *_isa->regClasses().at(FloatRegClass) },
                  { *_isa->regClasses().at(VecRegClass) },
                  { *_isa->regClasses().at(VecElemClass) },
                  { *_isa->regClasses().at(VecPredRegClass) },
                  { *_isa->regClasses().at(MatRegClass) },
                  { *_isa->regClasses().at(CCRegClass) } } },
      isa(_isa),
      predicate(true),
      memAccPredicate(true),
      comInstEventQueue("instruction-based event queue"),
      system(_sys),
      mmu(_mmu),
      decoder(_decoder),
      htmTransactionStarts(0),
      htmTransactionStops(0)
{
    clearArchRegs();
}

SimpleThread::SimpleThread(BaseCPU *_cpu, int _thread_num, System *_sys,
                           BaseMMU *_mmu, BaseISA *_isa, InstDecoder *_decoder)
    : SimpleThread(_cpu, _thread_num, _sys, nullptr, _mmu, _isa, _decoder)
{}

void
SimpleThread::takeOverFrom(ThreadContext *oldContext)
{
    gem5::takeOverFrom(*this, *oldContext);
    decoder->takeOverFrom(oldContext->getDecoderPtr());

    isa->takeOverFrom(this, oldContext);

    storeCondFailures = 0;
}

void
SimpleThread::copyState(ThreadContext *oldContext)
{
    // copy over functional state
    _status = oldContext->status();
    copyArchRegs(oldContext);

    _threadId = oldContext->threadId();
    _contextId = oldContext->contextId();
}

void
SimpleThread::serialize(CheckpointOut &cp) const
{
    ThreadState::serialize(cp);
    gem5::serialize(*this, cp);
}

void
SimpleThread::unserialize(CheckpointIn &cp)
{
    ThreadState::unserialize(cp);
    gem5::unserialize(*this, cp);
}

void
SimpleThread::activate()
{
    if (status() == ThreadContext::Active)
        return;

    lastActivate = curTick();
    _status = ThreadContext::Active;
    baseCpu->activateContext(_threadId);
}

void
SimpleThread::suspend()
{
    if (status() == ThreadContext::Suspended)
        return;

    lastActivate = curTick();
    lastSuspend = curTick();
    _status = ThreadContext::Suspended;
    baseCpu->suspendContext(_threadId);
}

void
SimpleThread::halt()
{
    if (status() == ThreadContext::Halted)
        return;

    _status = ThreadContext::Halted;
    baseCpu->haltContext(_threadId);
}

void
SimpleThread::copyArchRegs(ThreadContext *src_tc)
{
    getIsaPtr()->copyRegsFrom(src_tc);
}

// hardware transactional memory
void
SimpleThread::htmAbortTransaction(uint64_t htm_uid, HtmFailureFaultCause cause)
{
    baseCpu->htmSendAbortSignal(threadId(), htm_uid, cause);

    // these must be reset after the abort signal has been sent
    htmTransactionStarts = 0;
    htmTransactionStops = 0;
}

BaseHTMCheckpointPtr &
SimpleThread::getHtmCheckpointPtr()
{
    return _htmCheckpoint;
}

void
SimpleThread::setHtmCheckpointPtr(BaseHTMCheckpointPtr new_cpt)
{
    _htmCheckpoint = std::move(new_cpt);
}

} // namespace gem5
