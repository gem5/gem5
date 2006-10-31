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
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"

#if FULL_SYSTEM
#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/profile.hh"
#include "cpu/quiesce_event.hh"
#include "kern/kernel_stats.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"
#include "arch/stacktrace.hh"
#else
#include "sim/process.hh"
#include "sim/system.hh"
#include "mem/translating_port.hh"
#endif

using namespace std;

// constructor
#if FULL_SYSTEM
SimpleThread::SimpleThread(BaseCPU *_cpu, int _thread_num, System *_sys,
                           TheISA::ITB *_itb, TheISA::DTB *_dtb,
                           bool use_kernel_stats)
    : ThreadState(-1, _thread_num), cpu(_cpu), system(_sys), itb(_itb),
      dtb(_dtb)

{
    tc = new ProxyThreadContext<SimpleThread>(this);

    quiesceEvent = new EndQuiesceEvent(tc);

    regs.clear();

    if (cpu->params->profile) {
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

    if (use_kernel_stats) {
        kernelStats = new Kernel::Statistics(system);
    } else {
        kernelStats = NULL;
    }
    Port *mem_port;
    physPort = new FunctionalPort(csprintf("%s-%d-funcport",
                                           cpu->name(), tid));
    mem_port = system->physmem->getPort("functional");
    mem_port->setPeer(physPort);
    physPort->setPeer(mem_port);

    virtPort = new VirtualPort(csprintf("%s-%d-vport",
                                        cpu->name(), tid));
    mem_port = system->physmem->getPort("functional");
    mem_port->setPeer(virtPort);
    virtPort->setPeer(mem_port);
}
#else
SimpleThread::SimpleThread(BaseCPU *_cpu, int _thread_num,
                         Process *_process, int _asid, MemObject* memobj)
    : ThreadState(-1, _thread_num, _process, _asid, memobj),
      cpu(_cpu)
{
    /* Use this port to for syscall emulation writes to memory. */
    Port *mem_port;
    port = new TranslatingPort(csprintf("%s-%d-funcport",
                                        cpu->name(), tid),
                               process->pTable, false);
    mem_port = memobj->getPort("functional");
    mem_port->setPeer(port);
    port->setPeer(mem_port);

    regs.clear();
    tc = new ProxyThreadContext<SimpleThread>(this);
}

#endif

SimpleThread::SimpleThread()
#if FULL_SYSTEM
    : ThreadState(-1, -1)
#else
    : ThreadState(-1, -1, NULL, -1, NULL)
#endif
{
    tc = new ProxyThreadContext<SimpleThread>(this);
    regs.clear();
}

SimpleThread::~SimpleThread()
{
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

    Kernel::Statistics *stats = oldContext->getKernelStats();
    if (stats) {
        kernelStats = stats;
    }
#endif

    storeCondFailures = 0;

    oldContext->setStatus(ThreadContext::Unallocated);
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
    Kernel::Statistics *stats = context->getKernelStats();
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
    cpuId = oldContext->readCpuId();
#if !FULL_SYSTEM
    funcExeInst = oldContext->readFuncExeInst();
#endif
    inst = oldContext->getInst();
}

void
SimpleThread::serialize(ostream &os)
{
    ThreadState::serialize(os);
    regs.serialize(os);
    // thread_num and cpu_id are deterministic from the config
}


void
SimpleThread::unserialize(Checkpoint *cp, const std::string &section)
{
    ThreadState::unserialize(cp, section);
    regs.unserialize(cp, section);
    // thread_num and cpu_id are deterministic from the config
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

    lastActivate = curTick;

    if (status() == ThreadContext::Unallocated) {
        cpu->activateWhenReady(tid);
        return;
    }

    _status = ThreadContext::Active;

    // status() == Suspended
    cpu->activateContext(tid, delay);
}

void
SimpleThread::suspend()
{
    if (status() == ThreadContext::Suspended)
        return;

    lastActivate = curTick;
    lastSuspend = curTick;
/*
#if FULL_SYSTEM
    // Don't change the status from active if there are pending interrupts
    if (cpu->check_interrupts()) {
        assert(status() == ThreadContext::Active);
        return;
    }
#endif
*/
    _status = ThreadContext::Suspended;
    cpu->suspendContext(tid);
}

void
SimpleThread::deallocate()
{
    if (status() == ThreadContext::Unallocated)
        return;

    _status = ThreadContext::Unallocated;
    cpu->deallocateContext(tid);
}

void
SimpleThread::halt()
{
    if (status() == ThreadContext::Halted)
        return;

    _status = ThreadContext::Halted;
    cpu->haltContext(tid);
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

#if FULL_SYSTEM
VirtualPort*
SimpleThread::getVirtPort(ThreadContext *src_tc)
{
    if (!src_tc)
        return virtPort;

    VirtualPort *vp;
    Port *mem_port;

    vp = new VirtualPort("tc-vport", src_tc);
    mem_port = system->physmem->getPort("functional");
    mem_port->setPeer(vp);
    vp->setPeer(mem_port);
    return vp;
}

void
SimpleThread::delVirtPort(VirtualPort *vp)
{
    if (vp != virtPort) {
        delete vp->getPeer();
        delete vp;
    }
}


#endif

