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

#include "cpu/base.hh"
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
#include "sim/system.hh"
#include "targetarch/stacktrace.hh"
#else
#include "sim/process.hh"
#endif

using namespace std;

// constructor
#if FULL_SYSTEM
ExecContext::ExecContext(BaseCPU *_cpu, int _thread_num, System *_sys,
                         AlphaITB *_itb, AlphaDTB *_dtb,
                         FunctionalMemory *_mem)
    : _status(ExecContext::Unallocated), cpu(_cpu), thread_num(_thread_num),
      cpu_id(-1), lastActivate(0), lastSuspend(0), mem(_mem), itb(_itb),
      dtb(_dtb), system(_sys), memctrl(_sys->memctrl), physmem(_sys->physmem),
      kernelBinning(system->kernelBinning), bin(kernelBinning->bin),
      fnbin(kernelBinning->fnbin), profile(NULL), quiesceEvent(this),
      func_exe_inst(0), storeCondFailures(0)
{
    kernelStats = new Kernel::Statistics(this);
    memset(&regs, 0, sizeof(RegFile));

    if (cpu->params->profile) {
        profile = new FunctionProfile(system->kernelSymtab);
        Callback *cb =
            new MakeCallback<ExecContext, &ExecContext::dumpFuncProfile>(this);
        registerExitCallback(cb);
    }

    // let's fill with a dummy node for now so we don't get a segfault
    // on the first cycle when there's no node available.
    static ProfileNode dummyNode;
    profileNode = &dummyNode;
    profilePC = 3;
}
#else
ExecContext::ExecContext(BaseCPU *_cpu, int _thread_num,
                         Process *_process, int _asid)
    : _status(ExecContext::Unallocated),
      cpu(_cpu), thread_num(_thread_num), cpu_id(-1), lastActivate(0),
      lastSuspend(0), process(_process), mem(process->getMemory()), asid(_asid),
      func_exe_inst(0), storeCondFailures(0)
{
    memset(&regs, 0, sizeof(RegFile));
}

ExecContext::ExecContext(BaseCPU *_cpu, int _thread_num,
                         FunctionalMemory *_mem, int _asid)
    : cpu(_cpu), thread_num(_thread_num), process(0), mem(_mem), asid(_asid),
      func_exe_inst(0), storeCondFailures(0)
{
    memset(&regs, 0, sizeof(RegFile));
}
#endif

ExecContext::~ExecContext()
{
#if FULL_SYSTEM
    delete kernelStats;
#endif
}

#if FULL_SYSTEM
void
ExecContext::dumpFuncProfile()
{
    std::ostream *os = simout.create(csprintf("profile.%s.dat", cpu->name()));
    profile->dump(this, *os);
}

ExecContext::EndQuiesceEvent::EndQuiesceEvent(ExecContext *_xc)
    : Event(&mainEventQueue), xc(_xc)
{
}

void
ExecContext::EndQuiesceEvent::process()
{
    xc->activate();
}

const char*
ExecContext::EndQuiesceEvent::description()
{
    return "End Quiesce Event.";
}
#endif

void
ExecContext::takeOverFrom(ExecContext *oldContext)
{
    // some things should already be set up
    assert(mem == oldContext->mem);
#if FULL_SYSTEM
    assert(system == oldContext->system);
#else
    assert(process == oldContext->process);
#endif

    // copy over functional state
    _status = oldContext->_status;
    regs = oldContext->regs;
    cpu_id = oldContext->cpu_id;
    func_exe_inst = oldContext->func_exe_inst;

    storeCondFailures = 0;

    oldContext->_status = ExecContext::Unallocated;
}

void
ExecContext::serialize(ostream &os)
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
    kernelStats->serialize(os);

#endif
}


void
ExecContext::unserialize(Checkpoint *cp, const std::string &section)
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

    kernelStats->unserialize(cp, section);
#endif
}


void
ExecContext::activate(int delay)
{
    if (status() == Active)
        return;

    lastActivate = curTick;

    _status = Active;
    cpu->activateContext(thread_num, delay);
}

void
ExecContext::suspend()
{
    if (status() == Suspended)
        return;

    lastActivate = curTick;
    lastSuspend = curTick;
/*
#if FULL_SYSTEM
    // Don't change the status from active if there are pending interrupts
    if (cpu->check_interrupts()) {
        assert(status() == Active);
        return;
    }
#endif
*/
    _status = Suspended;
    cpu->suspendContext(thread_num);
}

void
ExecContext::deallocate()
{
    if (status() == Unallocated)
        return;

    _status = Unallocated;
    cpu->deallocateContext(thread_num);
}

void
ExecContext::halt()
{
    if (status() == Halted)
        return;

    _status = Halted;
    cpu->haltContext(thread_num);
}


void
ExecContext::regStats(const string &name)
{
#if FULL_SYSTEM
    kernelStats->regStats(name + ".kern");
#endif
}

void
ExecContext::trap(Fault fault)
{
    //TheISA::trap(fault);    //One possible way to do it...

    /** @todo: Going to hack it for now.  Do a true fixup later. */
#if FULL_SYSTEM
    ev5_trap(fault);
#else
    fatal("fault (%d) detected @ PC 0x%08p", fault, readPC());
#endif
}
