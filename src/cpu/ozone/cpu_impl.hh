/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 *          Nathan Binkert
 */

#ifndef __CPU_OZONE_CPU_IMPL_HH__
#define __CPU_OZONE_CPU_IMPL_HH__

#include "arch/alpha/osfpal.hh"
#include "arch/isa_traits.hh" // For MachInst
#include "arch/kernel_stats.hh"
#include "arch/tlb.hh"
#include "arch/types.hh"
#include "arch/vtophys.hh"
#include "base/callback.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/checker/thread_context.hh"
#include "cpu/ozone/cpu.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/profile.hh"
#include "cpu/quiesce_event.hh"
#include "cpu/simple_thread.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace TheISA;

template <class Impl>
OzoneCPU<Impl>::TickEvent::TickEvent(OzoneCPU *c, int w)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c), width(w)
{
}

template <class Impl>
void
OzoneCPU<Impl>::TickEvent::process()
{
    cpu->tick();
}

template <class Impl>
const char *
OzoneCPU<Impl>::TickEvent::description() const
{
    return "OzoneCPU tick";
}

template <class Impl>
OzoneCPU<Impl>::OzoneCPU(Params *p)
    : BaseCPU(p), thread(this, 0, p->workload[0], 0), tickEvent(this,
            p->width),
#ifndef NDEBUG
      instcount(0),
#endif
      comm(5, 5)
{
    frontEnd = new FrontEnd(p);
    backEnd = new BackEnd(p);

    _status = Idle;

    if (p->checker) {
        BaseCPU *temp_checker = p->checker;
        checker = dynamic_cast<Checker<DynInstPtr> *>(temp_checker);
        checker->setSystem(p->system);
        checkerTC = new CheckerThreadContext<OzoneTC>(&ozoneTC, checker);
        thread.tc = checkerTC;
        tc = checkerTC;
    } else {
        // If checker is not being used, then the xcProxy points
        // directly to the CPU's ExecContext.
        checker = NULL;
        thread.tc = &ozoneTC;
        tc = &ozoneTC;
    }

    ozoneTC.cpu = this;
    ozoneTC.thread = &thread;

    thread.noSquashFromTC = false;

    itb = p->itb;
    dtb = p->dtb;

    if (FullSystem) {
        // Setup thread state stuff.
        thread.cpu = this;
        thread.setTid(0);

        thread.quiesceEvent = new EndQuiesceEvent(tc);

        system = p->system;
        physmem = p->system->physmem;

        if (p->profile) {
            thread.profile = new FunctionProfile(p->system->kernelSymtab);
            // @todo: This might be better as an ThreadContext instead of
            // OzoneTC
            Callback *cb =
                new MakeCallback<OzoneTC,
                &OzoneTC::dumpFuncProfile>(&ozoneTC);
            registerExitCallback(cb);
        }

        // let's fill with a dummy node for now so we don't get a segfault
        // on the first cycle when there's no node available.
        static ProfileNode dummyNode;
        thread.profileNode = &dummyNode;
        thread.profilePC = 3;
    } else {
        thread.cpu = this;
    }

    numInst = 0;
    startNumInst = 0;

    threadContexts.push_back(tc);

    frontEnd->setCPU(this);
    backEnd->setCPU(this);

    frontEnd->setTC(tc);
    backEnd->setTC(tc);

    frontEnd->setThreadState(&thread);
    backEnd->setThreadState(&thread);

    frontEnd->setCommBuffer(&comm);
    backEnd->setCommBuffer(&comm);

    frontEnd->setBackEnd(backEnd);
    backEnd->setFrontEnd(frontEnd);

    globalSeqNum = 1;

    lockFlag = 0;

    // Setup rename table, initializing all values to ready.
    for (int i = 0; i < TheISA::TotalNumRegs; ++i) {
        thread.renameTable[i] = new DynInst(this);
        thread.renameTable[i]->setResultReady();
    }

    frontEnd->renameTable.copyFrom(thread.renameTable);
    backEnd->renameTable.copyFrom(thread.renameTable);

    thread.connectMemPorts(tc);

    DPRINTF(OzoneCPU, "OzoneCPU: Created Ozone cpu object.\n");
}

template <class Impl>
OzoneCPU<Impl>::~OzoneCPU()
{
}

template <class Impl>
void
OzoneCPU<Impl>::switchOut()
{
    BaseCPU::switchOut();
    switchCount = 0;
    // Front end needs state from back end, so switch out the back end first.
    backEnd->switchOut();
    frontEnd->switchOut();
}

template <class Impl>
void
OzoneCPU<Impl>::signalSwitched()
{
    // Only complete the switchout when both the front end and back
    // end have signalled they are ready to switch.
    if (++switchCount == 2) {
        backEnd->doSwitchOut();
        frontEnd->doSwitchOut();

        if (checker)
            checker->switchOut();

        _status = SwitchedOut;
#ifndef NDEBUG
        // Loop through all registers
        for (int i = 0; i < AlphaISA::TotalNumRegs; ++i) {
            assert(thread.renameTable[i] == frontEnd->renameTable[i]);

            assert(thread.renameTable[i] == backEnd->renameTable[i]);

            DPRINTF(OzoneCPU, "Checking if register %i matches.\n", i);
        }
#endif

        if (tickEvent.scheduled())
            tickEvent.squash();
    }
    assert(switchCount <= 2);
}

template <class Impl>
void
OzoneCPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    thread.trapPending = false;
    thread.noSquashFromTC = false;

    backEnd->takeOverFrom();
    frontEnd->takeOverFrom();
    frontEnd->renameTable.copyFrom(thread.renameTable);
    backEnd->renameTable.copyFrom(thread.renameTable);
    assert(!tickEvent.scheduled());

#ifndef NDEBUG
    // Check rename table.
    for (int i = 0; i < TheISA::TotalNumRegs; ++i) {
        assert(thread.renameTable[i]->isResultReady());
    }
#endif

    // @todo: Fix hardcoded number
    // Clear out any old information in time buffer.
    for (int i = 0; i < 15; ++i) {
        comm.advance();
    }

    // if any of this CPU's ThreadContexts are active, mark the CPU as
    // running and schedule its tick event.
    for (int i = 0; i < threadContexts.size(); ++i) {
        ThreadContext *tc = threadContexts[i];
        if (tc->status() == ThreadContext::Active &&
            _status != Running) {
            _status = Running;
            tickEvent.schedule(curTick());
        }
    }
    // Nothing running, change status to reflect that we're no longer
    // switched out.
    if (_status == SwitchedOut) {
        _status = Idle;
    }
}

template <class Impl>
void
OzoneCPU<Impl>::activateContext(int thread_num, int delay)
{
    // Eventually change this in SMT.
    assert(thread_num == 0);

    assert(_status == Idle);
    notIdleFraction = 1;
    scheduleTickEvent(delay);
    _status = Running;
    if (thread.quiesceEvent && thread.quiesceEvent->scheduled())
        thread.quiesceEvent->deschedule();
    thread.setStatus(ThreadContext::Active);
    frontEnd->wakeFromQuiesce();
}

template <class Impl>
void
OzoneCPU<Impl>::suspendContext(int thread_num)
{
    // Eventually change this in SMT.
    assert(thread_num == 0);
    // @todo: Figure out how to initially set the status properly so
    // this is running.
//    assert(_status == Running);
    notIdleFraction = 0;
    unscheduleTickEvent();
    _status = Idle;
}

template <class Impl>
void
OzoneCPU<Impl>::deallocateContext(int thread_num, int delay)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}

template <class Impl>
void
OzoneCPU<Impl>::haltContext(int thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}

template <class Impl>
void
OzoneCPU<Impl>::regStats()
{
    using namespace Stats;

    BaseCPU::regStats();

    thread.numInsts
        .name(name() + ".num_insts")
        .desc("Number of instructions executed")
        ;

    thread.numMemRefs
        .name(name() + ".num_refs")
        .desc("Number of memory references")
        ;

    notIdleFraction
        .name(name() + ".not_idle_fraction")
        .desc("Percentage of non-idle cycles")
        ;

    idleFraction
        .name(name() + ".idle_fraction")
        .desc("Percentage of idle cycles")
        ;

    quiesceCycles
        .name(name() + ".quiesce_cycles")
        .desc("Number of cycles spent in quiesce")
        ;

    idleFraction = constant(1.0) - notIdleFraction;

    frontEnd->regStats();
    backEnd->regStats();
}

template <class Impl>
void
OzoneCPU<Impl>::resetStats()
{
//    startNumInst = numInst;
    notIdleFraction = (_status != Idle);
}

template <class Impl>
void
OzoneCPU<Impl>::init()
{
    BaseCPU::init();

    // Mark this as in syscall so it won't need to squash
    thread.noSquashFromTC = true;
    if (FullSystem) {
        for (int i = 0; i < threadContexts.size(); ++i) {
            ThreadContext *tc = threadContexts[i];

            // initialize CPU, including PC
            TheISA::initCPU(tc, tc->contextId());
        }
    }
    frontEnd->renameTable.copyFrom(thread.renameTable);
    backEnd->renameTable.copyFrom(thread.renameTable);

    thread.noSquashFromTC = false;
}

template <class Impl>
void
OzoneCPU<Impl>::serialize(std::ostream &os)
{
    BaseCPU::serialize(os);
    SERIALIZE_ENUM(_status);
    nameOut(os, csprintf("%s.tc", name()));
    ozoneTC.serialize(os);
    nameOut(os, csprintf("%s.tickEvent", name()));
    tickEvent.serialize(os);

    // Use SimpleThread's ability to checkpoint to make it easier to
    // write out the registers.  Also make this static so it doesn't
    // get instantiated multiple times (causes a panic in statistics).
    static SimpleThread temp;

    nameOut(os, csprintf("%s.xc.0", name()));
    temp.copyTC(thread.getTC());
    temp.serialize(os);
}

template <class Impl>
void
OzoneCPU<Impl>::unserialize(Checkpoint *cp, const std::string &section)
{
    BaseCPU::unserialize(cp, section);
    UNSERIALIZE_ENUM(_status);
    ozoneTC.unserialize(cp, csprintf("%s.tc", section));
    tickEvent.unserialize(cp, csprintf("%s.tickEvent", section));

    // Use SimpleThread's ability to checkpoint to make it easier to
    // read in the registers.  Also make this static so it doesn't
    // get instantiated multiple times (causes a panic in statistics).
    static SimpleThread temp;

    temp.copyTC(thread.getTC());
    temp.unserialize(cp, csprintf("%s.xc.0", section));
    thread.getTC()->copyArchRegs(temp.getTC());
}

template <class Impl>
Addr
OzoneCPU<Impl>::dbg_vtophys(Addr addr)
{
    return vtophys(tc, addr);
}

template <class Impl>
void
OzoneCPU<Impl>::wakeup()
{
    if (_status == Idle) {
        DPRINTF(IPI,"Suspended Processor awoke\n");
        // Hack for now.  Otherwise might have to go through the tc, or
        // I need to figure out what's the right thing to call.
        activateContext(thread.threadId(), 1);
    }
}

/* start simulation, program loaded, processor precise state initialized */
template <class Impl>
void
OzoneCPU<Impl>::tick()
{
    DPRINTF(OzoneCPU, "\n\nOzoneCPU: Ticking cpu.\n");

    _status = Running;
    thread.renameTable[ZeroReg]->setIntResult(0);
    thread.renameTable[ZeroReg+TheISA::FP_Reg_Base]->
        setDoubleResult(0.0);

    comm.advance();
    frontEnd->tick();
    backEnd->tick();

    // check for instruction-count-based events
    comInstEventQueue[0]->serviceEvents(numInst);

    if (!tickEvent.scheduled() && _status == Running)
        tickEvent.schedule(curTick() + ticks(1));
}

template <class Impl>
void
OzoneCPU<Impl>::squashFromTC()
{
    thread.noSquashFromTC = true;
    backEnd->generateTCEvent();
}

template <class Impl>
void
OzoneCPU<Impl>::syscall(uint64_t &callnum)
{
    // Not sure this copy is needed, depending on how the TC proxy is made.
    thread.renameTable.copyFrom(backEnd->renameTable);

    thread.noSquashFromTC = true;

    thread.funcExeInst++;

    DPRINTF(OzoneCPU, "FuncExeInst: %i\n", thread.funcExeInst);

    thread.process->syscall(callnum, tc);

    thread.funcExeInst--;

    thread.noSquashFromTC = false;

    frontEnd->renameTable.copyFrom(thread.renameTable);
    backEnd->renameTable.copyFrom(thread.renameTable);
}

template <class Impl>
Fault
OzoneCPU<Impl>::hwrei()
{
    // Need to move this to ISA code
    // May also need to make this per thread

    lockFlag = false;
    lockAddrList.clear();
    thread.kernelStats->hwrei();

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
void
OzoneCPU<Impl>::processInterrupts()
{
    // Check for interrupts here.  For now can copy the code that
    // exists within isa_fullsys_traits.hh.  Also assume that thread 0
    // is the one that handles the interrupts.

    // Check if there are any outstanding interrupts
    //Handle the interrupts
    Fault interrupt = this->interrupts->getInterrupt(thread.getTC());

    if (interrupt != NoFault) {
        this->interrupts->updateIntrInfo(thread.getTC());
        interrupt->invoke(thread.getTC());
    }
}

template <class Impl>
bool
OzoneCPU<Impl>::simPalCheck(int palFunc)
{
    // Need to move this to ISA code
    // May also need to make this per thread
    thread.kernelStats->callpal(palFunc, tc);

    switch (palFunc) {
      case PAL::halt:
        haltContext(thread.threadId());
        if (--System::numSystemsRunning == 0)
            exitSimLoop("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (system->breakpoint())
            return false;
        break;
    }

    return true;
}

template <class Impl>
BaseCPU *
OzoneCPU<Impl>::OzoneTC::getCpuPtr()
{
    return cpu;
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setStatus(Status new_status)
{
    thread->setStatus(new_status);
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::activate(int delay)
{
    cpu->activateContext(thread->threadId(), delay);
}

/// Set the status to Suspended.
template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::suspend()
{
    cpu->suspendContext(thread->threadId());
}

/// Set the status to Halted.
template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::halt()
{
    cpu->haltContext(thread->threadId());
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::dumpFuncProfile()
{
    thread->dumpFuncProfile();
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::takeOverFrom(ThreadContext *old_context)
{
    // some things should already be set up
    assert(getSystemPtr() == old_context->getSystemPtr());
    assert(getProcessPtr() == old_context->getProcessPtr());

    // copy over functional state
    setStatus(old_context->status());
    copyArchRegs(old_context);
    setCpuId(old_context->cpuId());
    setContextId(old_context->contextId());

    setFuncExeInst(old_context->readFuncExeInst());
    EndQuiesceEvent *other_quiesce = old_context->getQuiesceEvent();
    if (other_quiesce) {
        // Point the quiesce event's TC at this TC so that it wakes up
        // the proper CPU.
        other_quiesce->tc = this;
    }
    if (thread->quiesceEvent) {
        thread->quiesceEvent->tc = this;
    }

    // Copy kernel stats pointer from old context.
    thread->kernelStats = old_context->getKernelStats();
//    storeCondFailures = 0;
    cpu->lockFlag = false;
#endif

    old_context->setStatus(ThreadContext::Halted);
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::regStats(const std::string &name)
{
    if (FullSystem) {
        thread->kernelStats = new TheISA::Kernel::Statistics(cpu->system);
        thread->kernelStats->regStats(name + ".kern");
    }
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::serialize(std::ostream &os)
{
    // Once serialization is added, serialize the quiesce event and
    // kernel stats.  Will need to make sure there aren't multiple
    // things that serialize them.
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::unserialize(Checkpoint *cp, const std::string &section)
{ }

template <class Impl>
EndQuiesceEvent *
OzoneCPU<Impl>::OzoneTC::getQuiesceEvent()
{
    return thread->quiesceEvent;
}

template <class Impl>
Tick
OzoneCPU<Impl>::OzoneTC::readLastActivate()
{
    return thread->lastActivate;
}

template <class Impl>
Tick
OzoneCPU<Impl>::OzoneTC::readLastSuspend()
{
    return thread->lastSuspend;
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::profileClear()
{
    thread->profileClear();
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::profileSample()
{
    thread->profileSample();
}

template <class Impl>
int
OzoneCPU<Impl>::OzoneTC::threadId() const
{
    return thread->threadId();
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::copyArchRegs(ThreadContext *tc)
{
    thread->PC = tc->readPC();
    thread->nextPC = tc->readNextPC();

    cpu->frontEnd->setPC(thread->PC);
    cpu->frontEnd->setNextPC(thread->nextPC);

    // First loop through the integer registers.
    for (int i = 0; i < TheISA::NumIntRegs; ++i) {
/*        DPRINTF(OzoneCPU, "Copying over register %i, had data %lli, "
                "now has data %lli.\n",
                i, thread->renameTable[i]->readIntResult(),
                tc->readIntReg(i));
*/
        thread->renameTable[i]->setIntResult(tc->readIntReg(i));
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < TheISA::NumFloatRegs; ++i) {
        int fp_idx = i + TheISA::FP_Reg_Base;
        thread->renameTable[fp_idx]->setIntResult(tc->readFloatRegBits(i));
    }

    thread->funcExeInst = tc->readFuncExeInst();

    // Need to copy the TC values into the current rename table,
    // copy the misc regs.
    copyMiscRegs(tc, this);
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::clearArchRegs()
{
    panic("Unimplemented!");
}

template <class Impl>
uint64_t
OzoneCPU<Impl>::OzoneTC::readIntReg(int reg_idx)
{
    return thread->renameTable[reg_idx]->readIntResult();
}

template <class Impl>
double
OzoneCPU<Impl>::OzoneTC::readFloatReg(int reg_idx)
{
    int idx = reg_idx + TheISA::FP_Reg_Base;
    return thread->renameTable[idx]->readFloatResult();
}

template <class Impl>
uint64_t
OzoneCPU<Impl>::OzoneTC::readFloatRegBits(int reg_idx)
{
    int idx = reg_idx + TheISA::FP_Reg_Base;
    return thread->renameTable[idx]->readIntResult();
}

template <class Impl>
CCReg
OzoneCPU<Impl>::OzoneTC::readCCReg(int reg_idx)
{
    return thread->renameTable[reg_idx]->readCCResult();
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setIntReg(int reg_idx, uint64_t val)
{
    thread->renameTable[reg_idx]->setIntResult(val);

    if (!thread->noSquashFromTC) {
        cpu->squashFromTC();
    }
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setFloatReg(int reg_idx, FloatReg val)
{
    int idx = reg_idx + TheISA::FP_Reg_Base;

    thread->renameTable[idx]->setDoubleResult(val);

    if (!thread->noSquashFromTC) {
        cpu->squashFromTC();
    }
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setFloatRegBits(int reg_idx, FloatRegBits val)
{
    panic("Unimplemented!");
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setCCReg(int reg_idx, CCReg val)
{
    thread->renameTable[reg_idx]->setCCResult(val);

    if (!thread->noSquashFromTC) {
        cpu->squashFromTC();
    }
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setPC(Addr val)
{
    thread->PC = val;
    cpu->frontEnd->setPC(val);

    if (!thread->noSquashFromTC) {
        cpu->squashFromTC();
    }
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setNextPC(Addr val)
{
    thread->nextPC = val;
    cpu->frontEnd->setNextPC(val);

    if (!thread->noSquashFromTC) {
        cpu->squashFromTC();
    }
}

template <class Impl>
TheISA::MiscReg
OzoneCPU<Impl>::OzoneTC::readMiscRegNoEffect(int misc_reg)
{
    return thread->miscRegFile.readRegNoEffect(misc_reg);
}

template <class Impl>
TheISA::MiscReg
OzoneCPU<Impl>::OzoneTC::readMiscReg(int misc_reg)
{
    return thread->miscRegFile.readReg(misc_reg, this);
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    // Needs to setup a squash event unless we're in syscall mode
    thread->miscRegFile.setRegNoEffect(misc_reg, val);

    if (!thread->noSquashFromTC) {
        cpu->squashFromTC();
    }
}

#endif//__CPU_OZONE_CPU_IMPL_HH__

template <class Impl>
void
OzoneCPU<Impl>::OzoneTC::setMiscReg(int misc_reg, const MiscReg &val)
{
    // Needs to setup a squash event unless we're in syscall mode
    thread->miscRegFile.setReg(misc_reg, val, this);

    if (!thread->noSquashFromTC) {
        cpu->squashFromTC();
    }
}
