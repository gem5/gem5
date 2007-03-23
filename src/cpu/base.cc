/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 */

#include <iostream>
#include <string>
#include <sstream>

#include "base/cprintf.hh"
#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "cpu/base.hh"
#include "cpu/cpuevent.hh"
#include "cpu/thread_context.hh"
#include "cpu/profile.hh"
#include "sim/sim_exit.hh"
#include "sim/param.hh"
#include "sim/process.hh"
#include "sim/sim_events.hh"
#include "sim/system.hh"

#include "base/trace.hh"

// Hack
#include "sim/stat_control.hh"

using namespace std;

vector<BaseCPU *> BaseCPU::cpuList;

// This variable reflects the max number of threads in any CPU.  Be
// careful to only use it once all the CPUs that you care about have
// been initialized
int maxThreadsPerCPU = 1;

CPUProgressEvent::CPUProgressEvent(EventQueue *q, Tick ival,
                                   BaseCPU *_cpu)
    : Event(q, Event::Stat_Event_Pri), interval(ival),
      lastNumInst(0), cpu(_cpu)
{
    if (interval)
        schedule(curTick + interval);
}

void
CPUProgressEvent::process()
{
    Counter temp = cpu->totalInstructions();
#ifndef NDEBUG
    double ipc = double(temp - lastNumInst) / (interval / cpu->cycles(1));

    DPRINTFN("%s progress event, instructions committed: %lli, IPC: %0.8d\n",
             cpu->name(), temp - lastNumInst, ipc);
    ipc = 0.0;
#else
    cprintf("%lli: %s progress event, instructions committed: %lli\n",
            curTick, cpu->name(), temp - lastNumInst);
#endif
    lastNumInst = temp;
    schedule(curTick + interval);
}

const char *
CPUProgressEvent::description()
{
    return "CPU Progress event";
}

#if FULL_SYSTEM
BaseCPU::BaseCPU(Params *p)
    : MemObject(p->name), clock(p->clock), instCnt(0),
      params(p), number_of_threads(p->numberOfThreads), system(p->system),
      phase(p->phase)
#else
BaseCPU::BaseCPU(Params *p)
    : MemObject(p->name), clock(p->clock), params(p),
      number_of_threads(p->numberOfThreads), system(p->system),
      phase(p->phase)
#endif
{
//    currentTick = curTick;
    DPRINTF(FullCPU, "BaseCPU: Creating object, mem address %#x.\n", this);

    // add self to global list of CPUs
    cpuList.push_back(this);

    DPRINTF(FullCPU, "BaseCPU: CPU added to cpuList, mem address %#x.\n",
            this);

    if (number_of_threads > maxThreadsPerCPU)
        maxThreadsPerCPU = number_of_threads;

    // allocate per-thread instruction-based event queues
    comInstEventQueue = new EventQueue *[number_of_threads];
    for (int i = 0; i < number_of_threads; ++i)
        comInstEventQueue[i] = new EventQueue("instruction-based event queue");

    //
    // set up instruction-count-based termination events, if any
    //
    if (p->max_insts_any_thread != 0)
        for (int i = 0; i < number_of_threads; ++i)
            schedExitSimLoop("a thread reached the max instruction count",
                             p->max_insts_any_thread, 0,
                             comInstEventQueue[i]);

    if (p->max_insts_all_threads != 0) {
        // allocate & initialize shared downcounter: each event will
        // decrement this when triggered; simulation will terminate
        // when counter reaches 0
        int *counter = new int;
        *counter = number_of_threads;
        for (int i = 0; i < number_of_threads; ++i)
            new CountedExitEvent(comInstEventQueue[i],
                "all threads reached the max instruction count",
                p->max_insts_all_threads, *counter);
    }

    // allocate per-thread load-based event queues
    comLoadEventQueue = new EventQueue *[number_of_threads];
    for (int i = 0; i < number_of_threads; ++i)
        comLoadEventQueue[i] = new EventQueue("load-based event queue");

    //
    // set up instruction-count-based termination events, if any
    //
    if (p->max_loads_any_thread != 0)
        for (int i = 0; i < number_of_threads; ++i)
            schedExitSimLoop("a thread reached the max load count",
                             p->max_loads_any_thread, 0,
                             comLoadEventQueue[i]);

    if (p->max_loads_all_threads != 0) {
        // allocate & initialize shared downcounter: each event will
        // decrement this when triggered; simulation will terminate
        // when counter reaches 0
        int *counter = new int;
        *counter = number_of_threads;
        for (int i = 0; i < number_of_threads; ++i)
            new CountedExitEvent(comLoadEventQueue[i],
                "all threads reached the max load count",
                p->max_loads_all_threads, *counter);
    }

    functionTracingEnabled = false;
    if (p->functionTrace) {
        functionTraceStream = simout.find(csprintf("ftrace.%s", name()));
        currentFunctionStart = currentFunctionEnd = 0;
        functionEntryTick = p->functionTraceStart;

        if (p->functionTraceStart == 0) {
            functionTracingEnabled = true;
        } else {
            Event *e =
                new EventWrapper<BaseCPU, &BaseCPU::enableFunctionTrace>(this,
                                                                         true);
            e->schedule(p->functionTraceStart);
        }
    }
#if FULL_SYSTEM
    profileEvent = NULL;
    if (params->profile)
        profileEvent = new ProfileEvent(this, params->profile);
#endif
}

BaseCPU::Params::Params()
{
#if FULL_SYSTEM
    profile = false;
#endif
    checker = NULL;
}

void
BaseCPU::enableFunctionTrace()
{
    functionTracingEnabled = true;
}

BaseCPU::~BaseCPU()
{
}

void
BaseCPU::init()
{
    if (!params->deferRegistration)
        registerThreadContexts();
}

void
BaseCPU::startup()
{
#if FULL_SYSTEM
    if (!params->deferRegistration && profileEvent)
        profileEvent->schedule(curTick);
#endif

    if (params->progress_interval) {
        new CPUProgressEvent(&mainEventQueue,
                             cycles(params->progress_interval),
                             this);
    }
}


void
BaseCPU::regStats()
{
    using namespace Stats;

    numCycles
        .name(name() + ".numCycles")
        .desc("number of cpu cycles simulated")
        ;

    int size = threadContexts.size();
    if (size > 1) {
        for (int i = 0; i < size; ++i) {
            stringstream namestr;
            ccprintf(namestr, "%s.ctx%d", name(), i);
            threadContexts[i]->regStats(namestr.str());
        }
    } else if (size == 1)
        threadContexts[0]->regStats(name());

#if FULL_SYSTEM
#endif
}

Tick
BaseCPU::nextCycle()
{
    Tick next_tick = curTick - phase + clock - 1;
    next_tick -= (next_tick % clock);
    next_tick += phase;
    return next_tick;
}

Tick
BaseCPU::nextCycle(Tick begin_tick)
{
    Tick next_tick = begin_tick;
    next_tick -= (next_tick % clock);
    next_tick += phase;

    while (next_tick < curTick)
        next_tick += clock;

    assert(next_tick >= curTick);
    return next_tick;
}

void
BaseCPU::registerThreadContexts()
{
    for (int i = 0; i < threadContexts.size(); ++i) {
        ThreadContext *tc = threadContexts[i];

#if FULL_SYSTEM
        int id = params->cpu_id;
        if (id != -1)
            id += i;

        tc->setCpuId(system->registerThreadContext(tc, id));
#else
        tc->setCpuId(tc->getProcessPtr()->registerThreadContext(tc));
#endif
    }
}


int
BaseCPU::findContext(ThreadContext *tc)
{
    for (int i = 0; i < threadContexts.size(); ++i) {
        if (tc == threadContexts[i])
            return i;
    }
    return 0;
}

void
BaseCPU::switchOut()
{
//    panic("This CPU doesn't support sampling!");
#if FULL_SYSTEM
    if (profileEvent && profileEvent->scheduled())
        profileEvent->deschedule();
#endif
}

void
BaseCPU::takeOverFrom(BaseCPU *oldCPU)
{
    assert(threadContexts.size() == oldCPU->threadContexts.size());

    for (int i = 0; i < threadContexts.size(); ++i) {
        ThreadContext *newTC = threadContexts[i];
        ThreadContext *oldTC = oldCPU->threadContexts[i];

        newTC->takeOverFrom(oldTC);

        CpuEvent::replaceThreadContext(oldTC, newTC);

        assert(newTC->readCpuId() == oldTC->readCpuId());
#if FULL_SYSTEM
        system->replaceThreadContext(newTC, newTC->readCpuId());
#else
        assert(newTC->getProcessPtr() == oldTC->getProcessPtr());
        newTC->getProcessPtr()->replaceThreadContext(newTC, newTC->readCpuId());
#endif

//    TheISA::compareXCs(oldXC, newXC);
    }

#if FULL_SYSTEM
    interrupts = oldCPU->interrupts;

    for (int i = 0; i < threadContexts.size(); ++i)
        threadContexts[i]->profileClear();

    // The Sampler must take care of this!
//    if (profileEvent)
//        profileEvent->schedule(curTick);
#endif
}


#if FULL_SYSTEM
BaseCPU::ProfileEvent::ProfileEvent(BaseCPU *_cpu, int _interval)
    : Event(&mainEventQueue), cpu(_cpu), interval(_interval)
{ }

void
BaseCPU::ProfileEvent::process()
{
    for (int i = 0, size = cpu->threadContexts.size(); i < size; ++i) {
        ThreadContext *tc = cpu->threadContexts[i];
        tc->profileSample();
    }

    schedule(curTick + interval);
}

void
BaseCPU::post_interrupt(int int_type)
{
    interrupts.post(int_type);
}

void
BaseCPU::post_interrupt(int int_num, int index)
{
    interrupts.post(int_num, index);
}

void
BaseCPU::clear_interrupt(int int_num, int index)
{
    interrupts.clear(int_num, index);
}

void
BaseCPU::clear_interrupts()
{
    interrupts.clear_all();
}


void
BaseCPU::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(instCnt);
    interrupts.serialize(os);
}

void
BaseCPU::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(instCnt);
    interrupts.unserialize(cp, section);
}

#endif // FULL_SYSTEM

void
BaseCPU::traceFunctionsInternal(Addr pc)
{
    if (!debugSymbolTable)
        return;

    // if pc enters different function, print new function symbol and
    // update saved range.  Otherwise do nothing.
    if (pc < currentFunctionStart || pc >= currentFunctionEnd) {
        string sym_str;
        bool found = debugSymbolTable->findNearestSymbol(pc, sym_str,
                                                         currentFunctionStart,
                                                         currentFunctionEnd);

        if (!found) {
            // no symbol found: use addr as label
            sym_str = csprintf("0x%x", pc);
            currentFunctionStart = pc;
            currentFunctionEnd = pc + 1;
        }

        ccprintf(*functionTraceStream, " (%d)\n%d: %s",
                 curTick - functionEntryTick, curTick, sym_str);
        functionEntryTick = curTick;
    }
}


DEFINE_SIM_OBJECT_CLASS_NAME("BaseCPU", BaseCPU)
