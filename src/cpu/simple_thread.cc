/*
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
 *
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 *          Lisa Hsu
 *          Kevin Lim
 */

#include <string>

#include "arch/isa_traits.hh"
#include "arch/utility.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "mem/vport.hh"
#include "params/BaseCPU.hh"

#if FULL_SYSTEM
#include "arch/kernel_stats.hh"
#include "arch/stacktrace.hh"
#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/profile.hh"
#include "cpu/quiesce_event.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"
#else
#include "mem/translating_port.hh"
#include "sim/process.hh"
#include "sim/system.hh"
#endif

using namespace std;

// constructor
#if !FULL_SYSTEM
SimpleThread::SimpleThread(BaseCPU *_cpu, int _thread_num, Process *_process,
                           TheISA::TLB *_itb, TheISA::TLB *_dtb)
    : ThreadState(_cpu, _thread_num, _process),
      cpu(_cpu), itb(_itb), dtb(_dtb)
{
    clearArchRegs();
    tc = new ProxyThreadContext<SimpleThread>(this);
}
#else
SimpleThread::SimpleThread(BaseCPU *_cpu, int _thread_num, System *_sys,
                           TheISA::TLB *_itb, TheISA::TLB *_dtb,
                           bool use_kernel_stats)
    : ThreadState(_cpu, _thread_num, NULL),
      cpu(_cpu), system(_sys), itb(_itb), dtb(_dtb)

{
    tc = new ProxyThreadContext<SimpleThread>(this);

    quiesceEvent = new EndQuiesceEvent(tc);

    clearArchRegs();

    if (cpu->params()->profile) {
        profile = new FunctionProfile(system->kernelSymtab);
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
        kernelStats = new TheISA::Kernel::Statistics(system);
}
#endif

SimpleThread::SimpleThread()
    : ThreadState(NULL, -1, NULL)
{
    tc = new ProxyThreadContext<SimpleThread>(this);
}

SimpleThread::~SimpleThread()
{
    delete physPort;
    delete virtPort;
    delete tc;
}

void
SimpleThread::takeOverFrom(ThreadContext *oldContext)
{
    // some things should already be set up
#if FULL_SYSTEM
    assert(system == oldContext->getSystemPtr());
#else
    assert(process == oldContext->getProcessPtr());
#endif

    copyState(oldContext);
#if FULL_SYSTEM
    EndQuiesceEvent *quiesce = oldContext->getQuiesceEvent();
    if (quiesce) {
        // Point the quiesce event's TC at this TC so that it wakes up
        // the proper CPU.
        quiesce->tc = tc;
    }
    if (quiesceEvent) {
        quiesceEvent->tc = tc;
    }

    TheISA::Kernel::Statistics *stats = oldContext->getKernelStats();
    if (stats) {
        kernelStats = stats;
    }
#endif

    storeCondFailures = 0;

    oldContext->setStatus(ThreadContext::Halted);
}

void
SimpleThread::copyTC(ThreadContext *context)
{
    copyState(context);

#if FULL_SYSTEM
    EndQuiesceEvent *quiesce = context->getQuiesceEvent();
    if (quiesce) {
        quiesceEvent = quiesce;
    }
    TheISA::Kernel::Statistics *stats = context->getKernelStats();
    if (stats) {
        kernelStats = stats;
    }
#endif
}

void
SimpleThread::copyState(ThreadContext *oldContext)
{
    // copy over functional state
    _status = oldContext->status();
    copyArchRegs(oldContext);
#if !FULL_SYSTEM
    funcExeInst = oldContext->readFuncExeInst();
#endif

    _threadId = oldContext->threadId();
    _contextId = oldContext->contextId();
}

void
SimpleThread::serialize(ostream &os)
{
    ThreadState::serialize(os);
    SERIALIZE_ARRAY(floatRegs.i, TheISA::NumFloatRegs);
    SERIALIZE_ARRAY(intRegs, TheISA::NumIntRegs);
    _pcState.serialize(os);
    // thread_num and cpu_id are deterministic from the config

    // 
    // Now must serialize all the ISA dependent state
    //
    isa.serialize(cpu, os);
}


void
SimpleThread::unserialize(Checkpoint *cp, const std::string &section)
{
    ThreadState::unserialize(cp, section);
    UNSERIALIZE_ARRAY(floatRegs.i, TheISA::NumFloatRegs);
    UNSERIALIZE_ARRAY(intRegs, TheISA::NumIntRegs);
    _pcState.unserialize(cp, section);
    // thread_num and cpu_id are deterministic from the config

    // 
    // Now must unserialize all the ISA dependent state
    //
    isa.unserialize(cpu, cp, section);
}

#if FULL_SYSTEM
void
SimpleThread::dumpFuncProfile()
{
    std::ostream *os = simout.create(csprintf("profile.%s.dat", cpu->name()));
    profile->dump(tc, *os);
}
#endif

void
SimpleThread::activate(int delay)
{
    if (status() == ThreadContext::Active)
        return;

    lastActivate = curTick();

//    if (status() == ThreadContext::Unallocated) {
//      cpu->activateWhenReady(_threadId);
//      return;
//   }

    _status = ThreadContext::Active;

    // status() == Suspended
    cpu->activateContext(_threadId, delay);
}

void
SimpleThread::suspend()
{
    if (status() == ThreadContext::Suspended)
        return;

    lastActivate = curTick();
    lastSuspend = curTick();
/*
#if FULL_SYSTEM
    // Don't change the status from active if there are pending interrupts
    if (cpu->checkInterrupts()) {
        assert(status() == ThreadContext::Active);
        return;
    }
#endif
*/
    _status = ThreadContext::Suspended;
    cpu->suspendContext(_threadId);
}


void
SimpleThread::halt()
{
    if (status() == ThreadContext::Halted)
        return;

    _status = ThreadContext::Halted;
    cpu->haltContext(_threadId);
}


void
SimpleThread::regStats(const string &name)
{
#if FULL_SYSTEM
    if (kernelStats)
        kernelStats->regStats(name + ".kern");
#endif
}

void
SimpleThread::copyArchRegs(ThreadContext *src_tc)
{
    TheISA::copyRegs(src_tc, tc);
}

