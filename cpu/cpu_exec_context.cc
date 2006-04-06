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
 */

#include <string>

#include "arch/isa_traits.hh"
#include "cpu/base.hh"
#include "cpu/cpu_exec_context.hh"
#include "cpu/exec_context.hh"

#if FULL_SYSTEM
#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/profile.hh"
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
CPUExecContext::CPUExecContext(BaseCPU *_cpu, int _thread_num, System *_sys,
                         AlphaITB *_itb, AlphaDTB *_dtb)
    : _status(ExecContext::Unallocated), cpu(_cpu), thread_num(_thread_num),
      cpu_id(-1), lastActivate(0), lastSuspend(0), system(_sys), itb(_itb),
      dtb(_dtb), profile(NULL), quiesceEvent(this), func_exe_inst(0),
      storeCondFailures(0)

{
    proxy = new ProxyExecContext<CPUExecContext>(this);

    memset(&regs, 0, sizeof(RegFile));

    if (cpu->params->profile) {
        profile = new FunctionProfile(system->kernelSymtab);
        Callback *cb =
            new MakeCallback<CPUExecContext,
            &CPUExecContext::dumpFuncProfile>(this);
        registerExitCallback(cb);
    }

    // let's fill with a dummy node for now so we don't get a segfault
    // on the first cycle when there's no node available.
    static ProfileNode dummyNode;
    profileNode = &dummyNode;
    profilePC = 3;

    Port *mem_port;
    physPort = new FunctionalPort();
    mem_port = system->physmem->getPort("functional");
    mem_port->setPeer(physPort);
    physPort->setPeer(mem_port);

    virtPort = new VirtualPort();
    mem_port = system->physmem->getPort("functional");
    mem_port->setPeer(virtPort);
    virtPort->setPeer(mem_port);
}
#else
CPUExecContext::CPUExecContext(BaseCPU *_cpu, int _thread_num,
                         Process *_process, int _asid, MemObject* memobj)
    : _status(ExecContext::Unallocated),
      cpu(_cpu), thread_num(_thread_num), cpu_id(-1), lastActivate(0),
      lastSuspend(0), process(_process), asid(_asid),
      func_exe_inst(0), storeCondFailures(0)
{
    /* Use this port to for syscall emulation writes to memory. */
    Port *mem_port;
    port = new TranslatingPort(process->pTable, false);
    mem_port = memobj->getPort("functional");
    mem_port->setPeer(port);
    port->setPeer(mem_port);

    memset(&regs, 0, sizeof(RegFile));
    proxy = new ProxyExecContext<CPUExecContext>(this);
}

CPUExecContext::CPUExecContext(RegFile *regFile)
    : cpu(NULL), thread_num(-1), process(NULL), asid(-1),
      func_exe_inst(0), storeCondFailures(0)
{
    regs = *regFile;
    proxy = new ProxyExecContext<CPUExecContext>(this);
}

#endif

CPUExecContext::~CPUExecContext()
{
    delete proxy;
}

#if FULL_SYSTEM
void
CPUExecContext::dumpFuncProfile()
{
    std::ostream *os = simout.create(csprintf("profile.%s.dat", cpu->name()));
    profile->dump(proxy, *os);
}

CPUExecContext::EndQuiesceEvent::EndQuiesceEvent(CPUExecContext *_cpuXC)
    : Event(&mainEventQueue), cpuXC(_cpuXC)
{
}

void
CPUExecContext::EndQuiesceEvent::process()
{
    cpuXC->activate();
}

const char*
CPUExecContext::EndQuiesceEvent::description()
{
    return "End Quiesce Event.";
}

void
CPUExecContext::profileClear()
{
    if (profile)
        profile->clear();
}

void
CPUExecContext::profileSample()
{
    if (profile)
        profile->sample(profileNode, profilePC);
}

#endif

void
CPUExecContext::takeOverFrom(ExecContext *oldContext)
{
    // some things should already be set up
#if FULL_SYSTEM
    assert(system == oldContext->getSystemPtr());
#else
    assert(process == oldContext->getProcessPtr());
#endif

    // copy over functional state
    _status = oldContext->status();
    copyArchRegs(oldContext);
    cpu_id = oldContext->readCpuId();
#if !FULL_SYSTEM
    func_exe_inst = oldContext->readFuncExeInst();
#endif

    storeCondFailures = 0;

    oldContext->setStatus(ExecContext::Unallocated);
}

void
CPUExecContext::serialize(ostream &os)
{
    SERIALIZE_ENUM(_status);
    regs.serialize(os);
    // thread_num and cpu_id are deterministic from the config
    SERIALIZE_SCALAR(func_exe_inst);
    SERIALIZE_SCALAR(inst);

#if FULL_SYSTEM
    Tick quiesceEndTick = 0;
    if (quiesceEvent.scheduled())
        quiesceEndTick = quiesceEvent.when();
    SERIALIZE_SCALAR(quiesceEndTick);

#endif
}


void
CPUExecContext::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ENUM(_status);
    regs.unserialize(cp, section);
    // thread_num and cpu_id are deterministic from the config
    UNSERIALIZE_SCALAR(func_exe_inst);
    UNSERIALIZE_SCALAR(inst);

#if FULL_SYSTEM
    Tick quiesceEndTick;
    UNSERIALIZE_SCALAR(quiesceEndTick);
    if (quiesceEndTick)
        quiesceEvent.schedule(quiesceEndTick);
#endif
}


void
CPUExecContext::activate(int delay)
{
    if (status() == ExecContext::Active)
        return;

    lastActivate = curTick;

    _status = ExecContext::Active;
    cpu->activateContext(thread_num, delay);
}

void
CPUExecContext::suspend()
{
    if (status() == ExecContext::Suspended)
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
    _status = ExecContext::Suspended;
    cpu->suspendContext(thread_num);
}

void
CPUExecContext::deallocate()
{
    if (status() == ExecContext::Unallocated)
        return;

    _status = ExecContext::Unallocated;
    cpu->deallocateContext(thread_num);
}

void
CPUExecContext::halt()
{
    if (status() == ExecContext::Halted)
        return;

    _status = ExecContext::Halted;
    cpu->haltContext(thread_num);
}


void
CPUExecContext::regStats(const string &name)
{
}

void
CPUExecContext::copyArchRegs(ExecContext *xc)
{
    TheISA::copyRegs(xc, proxy);
}

#if FULL_SYSTEM
VirtualPort*
CPUExecContext::getVirtPort(ExecContext *xc)
{
    if (!xc)
        return virtPort;

    VirtualPort *vp;
    Port *mem_port;

    vp = new VirtualPort(xc);
    mem_port = system->physmem->getPort("functional");
    mem_port->setPeer(vp);
    vp->setPeer(mem_port);
    return vp;
}

void
CPUExecContext::delVirtPort(VirtualPort *vp)
{
    assert(!vp->nullExecContext());
    delete vp->getPeer();
    delete vp;
}


#endif

