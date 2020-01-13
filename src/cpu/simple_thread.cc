/*
 * Copyright (c) 2018 ARM Limited
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

#include "arch/isa_traits.hh"
#include "arch/kernel_stats.hh"
#include "arch/stacktrace.hh"
#include "arch/utility.hh"
#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/profile.hh"
#include "cpu/quiesce_event.hh"
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

using namespace std;

// constructor
SimpleThread::SimpleThread(BaseCPU *_cpu, int _thread_num, System *_sys,
                           Process *_process, BaseTLB *_itb,
                           BaseTLB *_dtb, BaseISA *_isa)
    : ThreadState(_cpu, _thread_num, _process),
      isa(dynamic_cast<TheISA::ISA *>(_isa)),
      predicate(true), memAccPredicate(true),
      comInstEventQueue("instruction-based event queue"),
      system(_sys), itb(_itb), dtb(_dtb), decoder(TheISA::Decoder(isa))
{
    assert(isa);
    clearArchRegs();
    quiesceEvent = new EndQuiesceEvent(this);
}

SimpleThread::SimpleThread(BaseCPU *_cpu, int _thread_num, System *_sys,
                           BaseTLB *_itb, BaseTLB *_dtb,
                           BaseISA *_isa, bool use_kernel_stats)
    : ThreadState(_cpu, _thread_num, NULL),
      isa(dynamic_cast<TheISA::ISA *>(_isa)),
      predicate(true), memAccPredicate(true),
      comInstEventQueue("instruction-based event queue"),
      system(_sys), itb(_itb), dtb(_dtb), decoder(TheISA::Decoder(isa))
{
    assert(isa);

    quiesceEvent = new EndQuiesceEvent(this);

    clearArchRegs();

    if (baseCpu->params()->profile) {
        profile = new FunctionProfile(system->workload->symtab(this));
        Callback *cb =
            new MakeCallback<SimpleThread,
            &SimpleThread::dumpFuncProfile>(this);
        registerExitCallback(cb);
    }

    // let's fill with a dummy node for now so we don't get a segfault
    // on the first cycle when there's no node available.
    static ProfileNode dummyNode;
    profileNode = &dummyNode;
    profilePC = 3;

    if (use_kernel_stats)
        kernelStats = new TheISA::Kernel::Statistics();
}

void
SimpleThread::takeOverFrom(ThreadContext *oldContext)
{
    ::takeOverFrom(*this, *oldContext);
    decoder.takeOverFrom(oldContext->getDecoderPtr());

    isa->takeOverFrom(this, oldContext);

    kernelStats = oldContext->getKernelStats();
    funcExeInst = oldContext->readFuncExeInst();
    storeCondFailures = 0;
}

void
SimpleThread::copyState(ThreadContext *oldContext)
{
    // copy over functional state
    _status = oldContext->status();
    copyArchRegs(oldContext);
    if (FullSystem)
        funcExeInst = oldContext->readFuncExeInst();

    _threadId = oldContext->threadId();
    _contextId = oldContext->contextId();
}

void
SimpleThread::serialize(CheckpointOut &cp) const
{
    ThreadState::serialize(cp);
    ::serialize(*this, cp);
}


void
SimpleThread::unserialize(CheckpointIn &cp)
{
    ThreadState::unserialize(cp);
    ::unserialize(*this, cp);
}

void
SimpleThread::startup()
{
    isa->startup(this);
}

void
SimpleThread::dumpFuncProfile()
{
    OutputStream *os(simout.create(csprintf("profile.%s.dat", baseCpu->name())));
    profile->dump(this, *os->stream());
    simout.close(os);
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
SimpleThread::regStats(const string &name)
{
    if (FullSystem && kernelStats)
        kernelStats->regStats(name + ".kern");
}

void
SimpleThread::copyArchRegs(ThreadContext *src_tc)
{
    TheISA::copyRegs(src_tc, this);
}
