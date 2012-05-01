/*
 * Copyright (c) 2011-2012 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2011 Regents of the University of California
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
 *          Rick Strong
 */

#include <iostream>
#include <sstream>
#include <string>

#include "arch/tlb.hh"
#include "base/loader/symtab.hh"
#include "base/cprintf.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/cpuevent.hh"
#include "cpu/profile.hh"
#include "cpu/thread_context.hh"
#include "debug/SyscallVerbose.hh"
#include "params/BaseCPU.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

// Hack
#include "sim/stat_control.hh"

using namespace std;

vector<BaseCPU *> BaseCPU::cpuList;

// This variable reflects the max number of threads in any CPU.  Be
// careful to only use it once all the CPUs that you care about have
// been initialized
int maxThreadsPerCPU = 1;

CPUProgressEvent::CPUProgressEvent(BaseCPU *_cpu, Tick ival)
    : Event(Event::Progress_Event_Pri), _interval(ival), lastNumInst(0),
      cpu(_cpu), _repeatEvent(true)
{
    if (_interval)
        cpu->schedule(this, curTick() + _interval);
}

void
CPUProgressEvent::process()
{
    Counter temp = cpu->totalOps();
#ifndef NDEBUG
    double ipc = double(temp - lastNumInst) / (_interval / cpu->ticks(1));

    DPRINTFN("%s progress event, total committed:%i, progress insts committed: "
             "%lli, IPC: %0.8d\n", cpu->name(), temp, temp - lastNumInst,
             ipc);
    ipc = 0.0;
#else
    cprintf("%lli: %s progress event, total committed:%i, progress insts "
            "committed: %lli\n", curTick(), cpu->name(), temp,
            temp - lastNumInst);
#endif
    lastNumInst = temp;

    if (_repeatEvent)
        cpu->schedule(this, curTick() + _interval);
}

const char *
CPUProgressEvent::description() const
{
    return "CPU Progress";
}

BaseCPU::BaseCPU(Params *p, bool is_checker)
    : MemObject(p), clock(p->clock), instCnt(0), _cpuId(p->cpu_id),
      _instMasterId(p->system->getMasterId(name() + ".inst")),
      _dataMasterId(p->system->getMasterId(name() + ".data")),
      interrupts(p->interrupts),
      numThreads(p->numThreads), system(p->system),
      phase(p->phase)
{
//    currentTick = curTick();

    // if Python did not provide a valid ID, do it here
    if (_cpuId == -1 ) {
        _cpuId = cpuList.size();
    }

    // add self to global list of CPUs
    cpuList.push_back(this);

    DPRINTF(SyscallVerbose, "Constructing CPU with id %d\n", _cpuId);

    if (numThreads > maxThreadsPerCPU)
        maxThreadsPerCPU = numThreads;

    // allocate per-thread instruction-based event queues
    comInstEventQueue = new EventQueue *[numThreads];
    for (ThreadID tid = 0; tid < numThreads; ++tid)
        comInstEventQueue[tid] =
            new EventQueue("instruction-based event queue");

    //
    // set up instruction-count-based termination events, if any
    //
    if (p->max_insts_any_thread != 0) {
        const char *cause = "a thread reached the max instruction count";
        for (ThreadID tid = 0; tid < numThreads; ++tid) {
            Event *event = new SimLoopExitEvent(cause, 0);
            comInstEventQueue[tid]->schedule(event, p->max_insts_any_thread);
        }
    }

    if (p->max_insts_all_threads != 0) {
        const char *cause = "all threads reached the max instruction count";

        // allocate & initialize shared downcounter: each event will
        // decrement this when triggered; simulation will terminate
        // when counter reaches 0
        int *counter = new int;
        *counter = numThreads;
        for (ThreadID tid = 0; tid < numThreads; ++tid) {
            Event *event = new CountedExitEvent(cause, *counter);
            comInstEventQueue[tid]->schedule(event, p->max_insts_all_threads);
        }
    }

    // allocate per-thread load-based event queues
    comLoadEventQueue = new EventQueue *[numThreads];
    for (ThreadID tid = 0; tid < numThreads; ++tid)
        comLoadEventQueue[tid] = new EventQueue("load-based event queue");

    //
    // set up instruction-count-based termination events, if any
    //
    if (p->max_loads_any_thread != 0) {
        const char *cause = "a thread reached the max load count";
        for (ThreadID tid = 0; tid < numThreads; ++tid) {
            Event *event = new SimLoopExitEvent(cause, 0);
            comLoadEventQueue[tid]->schedule(event, p->max_loads_any_thread);
        }
    }

    if (p->max_loads_all_threads != 0) {
        const char *cause = "all threads reached the max load count";
        // allocate & initialize shared downcounter: each event will
        // decrement this when triggered; simulation will terminate
        // when counter reaches 0
        int *counter = new int;
        *counter = numThreads;
        for (ThreadID tid = 0; tid < numThreads; ++tid) {
            Event *event = new CountedExitEvent(cause, *counter);
            comLoadEventQueue[tid]->schedule(event, p->max_loads_all_threads);
        }
    }

    functionTracingEnabled = false;
    if (p->function_trace) {
        const string fname = csprintf("ftrace.%s", name());
        functionTraceStream = simout.find(fname);
        if (!functionTraceStream)
            functionTraceStream = simout.create(fname);

        currentFunctionStart = currentFunctionEnd = 0;
        functionEntryTick = p->function_trace_start;

        if (p->function_trace_start == 0) {
            functionTracingEnabled = true;
        } else {
            typedef EventWrapper<BaseCPU, &BaseCPU::enableFunctionTrace> wrap;
            Event *event = new wrap(this, true);
            schedule(event, p->function_trace_start);
        }
    }

    // The interrupts should always be present unless this CPU is
    // switched in later or in case it is a checker CPU
    if (!params()->defer_registration && !is_checker) {
        if (interrupts) {
            interrupts->setCPU(this);
        } else {
            fatal("CPU %s has no interrupt controller.\n"
                  "Ensure createInterruptController() is called.\n", name());
        }
    }

    if (FullSystem) {
        profileEvent = NULL;
        if (params()->profile)
            profileEvent = new ProfileEvent(this, params()->profile);
    }
    tracer = params()->tracer;
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
    if (!params()->defer_registration)
        registerThreadContexts();
}

void
BaseCPU::startup()
{
    if (FullSystem) {
        if (!params()->defer_registration && profileEvent)
            schedule(profileEvent, curTick());
    }

    if (params()->progress_interval) {
        Tick num_ticks = ticks(params()->progress_interval);

        new CPUProgressEvent(this, num_ticks);
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

    numWorkItemsStarted
        .name(name() + ".numWorkItemsStarted")
        .desc("number of work items this cpu started")
        ;

    numWorkItemsCompleted
        .name(name() + ".numWorkItemsCompleted")
        .desc("number of work items this cpu completed")
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
}

MasterPort &
BaseCPU::getMasterPort(const string &if_name, int idx)
{
    // Get the right port based on name. This applies to all the
    // subclasses of the base CPU and relies on their implementation
    // of getDataPort and getInstPort. In all cases there methods
    // return a CpuPort pointer.
    if (if_name == "dcache_port")
        return getDataPort();
    else if (if_name == "icache_port")
        return getInstPort();
    else
        return MemObject::getMasterPort(if_name, idx);
}

Tick
BaseCPU::nextCycle()
{
    Tick next_tick = curTick() - phase + clock - 1;
    next_tick -= (next_tick % clock);
    next_tick += phase;
    return next_tick;
}

Tick
BaseCPU::nextCycle(Tick begin_tick)
{
    Tick next_tick = begin_tick;
    if (next_tick % clock != 0)
        next_tick = next_tick - (next_tick % clock) + clock;
    next_tick += phase;

    assert(next_tick >= curTick());
    return next_tick;
}

void
BaseCPU::registerThreadContexts()
{
    ThreadID size = threadContexts.size();
    for (ThreadID tid = 0; tid < size; ++tid) {
        ThreadContext *tc = threadContexts[tid];

        /** This is so that contextId and cpuId match where there is a
         * 1cpu:1context relationship.  Otherwise, the order of registration
         * could affect the assignment and cpu 1 could have context id 3, for
         * example.  We may even want to do something like this for SMT so that
         * cpu 0 has the lowest thread contexts and cpu N has the highest, but
         * I'll just do this for now
         */
        if (numThreads == 1)
            tc->setContextId(system->registerThreadContext(tc, _cpuId));
        else
            tc->setContextId(system->registerThreadContext(tc));

        if (!FullSystem)
            tc->getProcessPtr()->assignThreadContext(tc->contextId());
    }
}


int
BaseCPU::findContext(ThreadContext *tc)
{
    ThreadID size = threadContexts.size();
    for (ThreadID tid = 0; tid < size; ++tid) {
        if (tc == threadContexts[tid])
            return tid;
    }
    return 0;
}

void
BaseCPU::switchOut()
{
    if (profileEvent && profileEvent->scheduled())
        deschedule(profileEvent);
}

void
BaseCPU::takeOverFrom(BaseCPU *oldCPU)
{
    assert(threadContexts.size() == oldCPU->threadContexts.size());

    _cpuId = oldCPU->cpuId();

    ThreadID size = threadContexts.size();
    for (ThreadID i = 0; i < size; ++i) {
        ThreadContext *newTC = threadContexts[i];
        ThreadContext *oldTC = oldCPU->threadContexts[i];

        newTC->takeOverFrom(oldTC);

        CpuEvent::replaceThreadContext(oldTC, newTC);

        assert(newTC->contextId() == oldTC->contextId());
        assert(newTC->threadId() == oldTC->threadId());
        system->replaceThreadContext(newTC, newTC->contextId());

        /* This code no longer works since the zero register (e.g.,
         * r31 on Alpha) doesn't necessarily contain zero at this
         * point.
           if (DTRACE(Context))
            ThreadContext::compare(oldTC, newTC);
        */

        MasterPort *old_itb_port = oldTC->getITBPtr()->getMasterPort();
        MasterPort *old_dtb_port = oldTC->getDTBPtr()->getMasterPort();
        MasterPort *new_itb_port = newTC->getITBPtr()->getMasterPort();
        MasterPort *new_dtb_port = newTC->getDTBPtr()->getMasterPort();

        // Move over any table walker ports if they exist
        if (new_itb_port && !new_itb_port->isConnected()) {
            assert(old_itb_port);
            SlavePort &slavePort = old_itb_port->getSlavePort();
            new_itb_port->bind(slavePort);
        }
        if (new_dtb_port && !new_dtb_port->isConnected()) {
            assert(old_dtb_port);
            SlavePort &slavePort = old_dtb_port->getSlavePort();
            new_dtb_port->bind(slavePort);
        }

        // Checker whether or not we have to transfer CheckerCPU
        // objects over in the switch
        CheckerCPU *oldChecker = oldTC->getCheckerCpuPtr();
        CheckerCPU *newChecker = newTC->getCheckerCpuPtr();
        if (oldChecker && newChecker) {
            MasterPort *old_checker_itb_port =
                oldChecker->getITBPtr()->getMasterPort();
            MasterPort *old_checker_dtb_port =
                oldChecker->getDTBPtr()->getMasterPort();
            MasterPort *new_checker_itb_port =
                newChecker->getITBPtr()->getMasterPort();
            MasterPort *new_checker_dtb_port =
                newChecker->getDTBPtr()->getMasterPort();

            // Move over any table walker ports if they exist for checker
            if (new_checker_itb_port && !new_checker_itb_port->isConnected()) {
                assert(old_checker_itb_port);
                SlavePort &slavePort = old_checker_itb_port->getSlavePort();;
                new_checker_itb_port->bind(slavePort);
            }
            if (new_checker_dtb_port && !new_checker_dtb_port->isConnected()) {
                assert(old_checker_dtb_port);
                SlavePort &slavePort = old_checker_dtb_port->getSlavePort();;
                new_checker_dtb_port->bind(slavePort);
            }
        }
    }

    interrupts = oldCPU->interrupts;
    interrupts->setCPU(this);

    if (FullSystem) {
        for (ThreadID i = 0; i < size; ++i)
            threadContexts[i]->profileClear();

        if (profileEvent)
            schedule(profileEvent, curTick());
    }

    // Connect new CPU to old CPU's memory only if new CPU isn't
    // connected to anything.  Also connect old CPU's memory to new
    // CPU.
    if (!getInstPort().isConnected()) {
        getInstPort().bind(oldCPU->getInstPort().getSlavePort());
    }

    if (!getDataPort().isConnected()) {
        getDataPort().bind(oldCPU->getDataPort().getSlavePort());
    }
}


BaseCPU::ProfileEvent::ProfileEvent(BaseCPU *_cpu, Tick _interval)
    : cpu(_cpu), interval(_interval)
{ }

void
BaseCPU::ProfileEvent::process()
{
    ThreadID size = cpu->threadContexts.size();
    for (ThreadID i = 0; i < size; ++i) {
        ThreadContext *tc = cpu->threadContexts[i];
        tc->profileSample();
    }

    cpu->schedule(this, curTick() + interval);
}

void
BaseCPU::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(instCnt);
    interrupts->serialize(os);
}

void
BaseCPU::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(instCnt);
    interrupts->unserialize(cp, section);
}

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
                 curTick() - functionEntryTick, curTick(), sym_str);
        functionEntryTick = curTick();
    }
}

bool
BaseCPU::CpuPort::recvTimingResp(PacketPtr pkt)
{
    panic("BaseCPU doesn't expect recvTiming!\n");
    return true;
}

void
BaseCPU::CpuPort::recvRetry()
{
    panic("BaseCPU doesn't expect recvRetry!\n");
}

void
BaseCPU::CpuPort::recvFunctionalSnoop(PacketPtr pkt)
{
    // No internal storage to update (in the general case). A CPU with
    // internal storage, e.g. an LSQ that should be part of the
    // coherent memory has to check against stored data.
}
