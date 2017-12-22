/*
 * Copyright (c) 2011-2012,2016-2017 ARM Limited
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
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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

#include "cpu/base.hh"

#include <iostream>
#include <sstream>
#include <string>

#include "arch/generic/tlb.hh"
#include "base/cprintf.hh"
#include "base/loader/symtab.hh"
#include "base/logging.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/cpuevent.hh"
#include "cpu/profile.hh"
#include "cpu/thread_context.hh"
#include "debug/Mwait.hh"
#include "debug/SyscallVerbose.hh"
#include "mem/page_table.hh"
#include "params/BaseCPU.hh"
#include "sim/clocked_object.hh"
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

    if (_repeatEvent)
      cpu->schedule(this, curTick() + _interval);

    if (cpu->switchedOut()) {
      return;
    }

#ifndef NDEBUG
    double ipc = double(temp - lastNumInst) / (_interval / cpu->clockPeriod());

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
}

const char *
CPUProgressEvent::description() const
{
    return "CPU Progress";
}

BaseCPU::BaseCPU(Params *p, bool is_checker)
    : MemObject(p), instCnt(0), _cpuId(p->cpu_id), _socketId(p->socket_id),
      _instMasterId(p->system->getMasterId(name() + ".inst")),
      _dataMasterId(p->system->getMasterId(name() + ".data")),
      _taskId(ContextSwitchTaskId::Unknown), _pid(invldPid),
      _switchedOut(p->switched_out), _cacheLineSize(p->system->cacheLineSize()),
      interrupts(p->interrupts), profileEvent(NULL),
      numThreads(p->numThreads), system(p->system),
      previousCycle(0), previousState(CPU_STATE_SLEEP),
      functionTraceStream(nullptr), currentFunctionStart(0),
      currentFunctionEnd(0), functionEntryTick(0),
      addressMonitor(p->numThreads),
      syscallRetryLatency(p->syscallRetryLatency),
      pwrGatingLatency(p->pwr_gating_latency),
      powerGatingOnIdle(p->power_gating_on_idle),
      enterPwrGatingEvent([this]{ enterPwrGating(); }, name())
{
    // if Python did not provide a valid ID, do it here
    if (_cpuId == -1 ) {
        _cpuId = cpuList.size();
    }

    // add self to global list of CPUs
    cpuList.push_back(this);

    DPRINTF(SyscallVerbose, "Constructing CPU with id %d, socket id %d\n",
                _cpuId, _socketId);

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
        for (ThreadID tid = 0; tid < numThreads; ++tid)
            scheduleInstStop(tid, p->max_insts_any_thread, cause);
    }

    // Set up instruction-count-based termination events for SimPoints
    // Typically, there are more than one action points.
    // Simulation.py is responsible to take the necessary actions upon
    // exitting the simulation loop.
    if (!p->simpoint_start_insts.empty()) {
        const char *cause = "simpoint starting point found";
        for (size_t i = 0; i < p->simpoint_start_insts.size(); ++i)
            scheduleInstStop(0, p->simpoint_start_insts[i], cause);
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
        for (ThreadID tid = 0; tid < numThreads; ++tid)
            scheduleLoadStop(tid, p->max_loads_any_thread, cause);
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
        functionTraceStream = simout.findOrCreate(fname)->stream();

        currentFunctionStart = currentFunctionEnd = 0;
        functionEntryTick = p->function_trace_start;

        if (p->function_trace_start == 0) {
            functionTracingEnabled = true;
        } else {
            Event *event = new EventFunctionWrapper(
                [this]{ enableFunctionTrace(); }, name(), true);
            schedule(event, p->function_trace_start);
        }
    }

    // The interrupts should always be present unless this CPU is
    // switched in later or in case it is a checker CPU
    if (!params()->switched_out && !is_checker) {
        fatal_if(interrupts.size() != numThreads,
                 "CPU %s has %i interrupt controllers, but is expecting one "
                 "per thread (%i)\n",
                 name(), interrupts.size(), numThreads);
        for (ThreadID tid = 0; tid < numThreads; tid++)
            interrupts[tid]->setCPU(this);
    }

    if (FullSystem) {
        if (params()->profile)
            profileEvent = new EventFunctionWrapper(
                [this]{ processProfileEvent(); },
                name());
    }
    tracer = params()->tracer;

    if (params()->isa.size() != numThreads) {
        fatal("Number of ISAs (%i) assigned to the CPU does not equal number "
              "of threads (%i).\n", params()->isa.size(), numThreads);
    }
}

void
BaseCPU::enableFunctionTrace()
{
    functionTracingEnabled = true;
}

BaseCPU::~BaseCPU()
{
    delete profileEvent;
    delete[] comLoadEventQueue;
    delete[] comInstEventQueue;
}

void
BaseCPU::armMonitor(ThreadID tid, Addr address)
{
    assert(tid < numThreads);
    AddressMonitor &monitor = addressMonitor[tid];

    monitor.armed = true;
    monitor.vAddr = address;
    monitor.pAddr = 0x0;
    DPRINTF(Mwait,"[tid:%d] Armed monitor (vAddr=0x%lx)\n", tid, address);
}

bool
BaseCPU::mwait(ThreadID tid, PacketPtr pkt)
{
    assert(tid < numThreads);
    AddressMonitor &monitor = addressMonitor[tid];

    if (!monitor.gotWakeup) {
        int block_size = cacheLineSize();
        uint64_t mask = ~((uint64_t)(block_size - 1));

        assert(pkt->req->hasPaddr());
        monitor.pAddr = pkt->getAddr() & mask;
        monitor.waiting = true;

        DPRINTF(Mwait,"[tid:%d] mwait called (vAddr=0x%lx, "
                "line's paddr=0x%lx)\n", tid, monitor.vAddr, monitor.pAddr);
        return true;
    } else {
        monitor.gotWakeup = false;
        return false;
    }
}

void
BaseCPU::mwaitAtomic(ThreadID tid, ThreadContext *tc, BaseTLB *dtb)
{
    assert(tid < numThreads);
    AddressMonitor &monitor = addressMonitor[tid];

    Request req;
    Addr addr = monitor.vAddr;
    int block_size = cacheLineSize();
    uint64_t mask = ~((uint64_t)(block_size - 1));
    int size = block_size;

    //The address of the next line if it crosses a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, block_size);

    if (secondAddr > addr)
        size = secondAddr - addr;

    req.setVirt(0, addr, size, 0x0, dataMasterId(), tc->instAddr());

    // translate to physical address
    Fault fault = dtb->translateAtomic(&req, tc, BaseTLB::Read);
    assert(fault == NoFault);

    monitor.pAddr = req.getPaddr() & mask;
    monitor.waiting = true;

    DPRINTF(Mwait,"[tid:%d] mwait called (vAddr=0x%lx, line's paddr=0x%lx)\n",
            tid, monitor.vAddr, monitor.pAddr);
}

void
BaseCPU::init()
{
    if (!params()->switched_out) {
        registerThreadContexts();

        verifyMemoryMode();
    }
}

void
BaseCPU::startup()
{
    if (FullSystem) {
        if (!params()->switched_out && profileEvent)
            schedule(profileEvent, curTick());
    }

    if (params()->progress_interval) {
        new CPUProgressEvent(this, params()->progress_interval);
    }

    if (_switchedOut)
        ClockedObject::pwrState(Enums::PwrState::OFF);

    // Assumption CPU start to operate instantaneously without any latency
    if (ClockedObject::pwrState() == Enums::PwrState::UNDEFINED)
        ClockedObject::pwrState(Enums::PwrState::ON);

}

ProbePoints::PMUUPtr
BaseCPU::pmuProbePoint(const char *name)
{
    ProbePoints::PMUUPtr ptr;
    ptr.reset(new ProbePoints::PMU(getProbeManager(), name));

    return ptr;
}

void
BaseCPU::regProbePoints()
{
    ppAllCycles = pmuProbePoint("Cycles");
    ppActiveCycles = pmuProbePoint("ActiveCycles");

    ppRetiredInsts = pmuProbePoint("RetiredInsts");
    ppRetiredLoads = pmuProbePoint("RetiredLoads");
    ppRetiredStores = pmuProbePoint("RetiredStores");
    ppRetiredBranches = pmuProbePoint("RetiredBranches");

    ppSleeping = new ProbePointArg<bool>(this->getProbeManager(),
                                         "Sleeping");
}

void
BaseCPU::probeInstCommit(const StaticInstPtr &inst)
{
    if (!inst->isMicroop() || inst->isLastMicroop())
        ppRetiredInsts->notify(1);


    if (inst->isLoad())
        ppRetiredLoads->notify(1);

    if (inst->isStore())
        ppRetiredStores->notify(1);

    if (inst->isControl())
        ppRetiredBranches->notify(1);
}

void
BaseCPU::regStats()
{
    MemObject::regStats();

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

BaseMasterPort &
BaseCPU::getMasterPort(const string &if_name, PortID idx)
{
    // Get the right port based on name. This applies to all the
    // subclasses of the base CPU and relies on their implementation
    // of getDataPort and getInstPort. In all cases there methods
    // return a MasterPort pointer.
    if (if_name == "dcache_port")
        return getDataPort();
    else if (if_name == "icache_port")
        return getInstPort();
    else
        return MemObject::getMasterPort(if_name, idx);
}

void
BaseCPU::registerThreadContexts()
{
    assert(system->multiThread || numThreads == 1);

    ThreadID size = threadContexts.size();
    for (ThreadID tid = 0; tid < size; ++tid) {
        ThreadContext *tc = threadContexts[tid];

        if (system->multiThread) {
            tc->setContextId(system->registerThreadContext(tc));
        } else {
            tc->setContextId(system->registerThreadContext(tc, _cpuId));
        }

        if (!FullSystem)
            tc->getProcessPtr()->assignThreadContext(tc->contextId());
    }
}

void
BaseCPU::deschedulePowerGatingEvent()
{
    if (enterPwrGatingEvent.scheduled()){
        deschedule(enterPwrGatingEvent);
    }
}

void
BaseCPU::schedulePowerGatingEvent()
{
    for (auto tc : threadContexts) {
        if (tc->status() == ThreadContext::Active)
            return;
    }

    if (ClockedObject::pwrState() == Enums::PwrState::CLK_GATED &&
        powerGatingOnIdle) {
        assert(!enterPwrGatingEvent.scheduled());
        // Schedule a power gating event when clock gated for the specified
        // amount of time
        schedule(enterPwrGatingEvent, clockEdge(pwrGatingLatency));
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
BaseCPU::activateContext(ThreadID thread_num)
{
    // Squash enter power gating event while cpu gets activated
    if (enterPwrGatingEvent.scheduled())
        deschedule(enterPwrGatingEvent);
    // For any active thread running, update CPU power state to active (ON)
    ClockedObject::pwrState(Enums::PwrState::ON);

    updateCycleCounters(CPU_STATE_WAKEUP);
}

void
BaseCPU::suspendContext(ThreadID thread_num)
{
    // Check if all threads are suspended
    for (auto t : threadContexts) {
        if (t->status() != ThreadContext::Suspended) {
            return;
        }
    }

    // All CPU thread are suspended, update cycle count
    updateCycleCounters(CPU_STATE_SLEEP);

    // All CPU threads suspended, enter lower power state for the CPU
    ClockedObject::pwrState(Enums::PwrState::CLK_GATED);

    // If pwrGatingLatency is set to 0 then this mechanism is disabled
    if (powerGatingOnIdle) {
        // Schedule power gating event when clock gated for pwrGatingLatency
        // cycles
        schedule(enterPwrGatingEvent, clockEdge(pwrGatingLatency));
    }
}

void
BaseCPU::haltContext(ThreadID thread_num)
{
    updateCycleCounters(BaseCPU::CPU_STATE_SLEEP);
}

void
BaseCPU::enterPwrGating(void)
{
    ClockedObject::pwrState(Enums::PwrState::OFF);
}

void
BaseCPU::switchOut()
{
    assert(!_switchedOut);
    _switchedOut = true;
    if (profileEvent && profileEvent->scheduled())
        deschedule(profileEvent);

    // Flush all TLBs in the CPU to avoid having stale translations if
    // it gets switched in later.
    flushTLBs();

    // Go to the power gating state
    ClockedObject::pwrState(Enums::PwrState::OFF);
}

void
BaseCPU::takeOverFrom(BaseCPU *oldCPU)
{
    assert(threadContexts.size() == oldCPU->threadContexts.size());
    assert(_cpuId == oldCPU->cpuId());
    assert(_switchedOut);
    assert(oldCPU != this);
    _pid = oldCPU->getPid();
    _taskId = oldCPU->taskId();
    // Take over the power state of the switchedOut CPU
    ClockedObject::pwrState(oldCPU->pwrState());

    previousState = oldCPU->previousState;
    previousCycle = oldCPU->previousCycle;

    _switchedOut = false;

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

        BaseMasterPort *old_itb_port = oldTC->getITBPtr()->getMasterPort();
        BaseMasterPort *old_dtb_port = oldTC->getDTBPtr()->getMasterPort();
        BaseMasterPort *new_itb_port = newTC->getITBPtr()->getMasterPort();
        BaseMasterPort *new_dtb_port = newTC->getDTBPtr()->getMasterPort();

        // Move over any table walker ports if they exist
        if (new_itb_port) {
            assert(!new_itb_port->isConnected());
            assert(old_itb_port);
            assert(old_itb_port->isConnected());
            BaseSlavePort &slavePort = old_itb_port->getSlavePort();
            old_itb_port->unbind();
            new_itb_port->bind(slavePort);
        }
        if (new_dtb_port) {
            assert(!new_dtb_port->isConnected());
            assert(old_dtb_port);
            assert(old_dtb_port->isConnected());
            BaseSlavePort &slavePort = old_dtb_port->getSlavePort();
            old_dtb_port->unbind();
            new_dtb_port->bind(slavePort);
        }
        newTC->getITBPtr()->takeOverFrom(oldTC->getITBPtr());
        newTC->getDTBPtr()->takeOverFrom(oldTC->getDTBPtr());

        // Checker whether or not we have to transfer CheckerCPU
        // objects over in the switch
        CheckerCPU *oldChecker = oldTC->getCheckerCpuPtr();
        CheckerCPU *newChecker = newTC->getCheckerCpuPtr();
        if (oldChecker && newChecker) {
            BaseMasterPort *old_checker_itb_port =
                oldChecker->getITBPtr()->getMasterPort();
            BaseMasterPort *old_checker_dtb_port =
                oldChecker->getDTBPtr()->getMasterPort();
            BaseMasterPort *new_checker_itb_port =
                newChecker->getITBPtr()->getMasterPort();
            BaseMasterPort *new_checker_dtb_port =
                newChecker->getDTBPtr()->getMasterPort();

            newChecker->getITBPtr()->takeOverFrom(oldChecker->getITBPtr());
            newChecker->getDTBPtr()->takeOverFrom(oldChecker->getDTBPtr());

            // Move over any table walker ports if they exist for checker
            if (new_checker_itb_port) {
                assert(!new_checker_itb_port->isConnected());
                assert(old_checker_itb_port);
                assert(old_checker_itb_port->isConnected());
                BaseSlavePort &slavePort =
                    old_checker_itb_port->getSlavePort();
                old_checker_itb_port->unbind();
                new_checker_itb_port->bind(slavePort);
            }
            if (new_checker_dtb_port) {
                assert(!new_checker_dtb_port->isConnected());
                assert(old_checker_dtb_port);
                assert(old_checker_dtb_port->isConnected());
                BaseSlavePort &slavePort =
                    old_checker_dtb_port->getSlavePort();
                old_checker_dtb_port->unbind();
                new_checker_dtb_port->bind(slavePort);
            }
        }
    }

    interrupts = oldCPU->interrupts;
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        interrupts[tid]->setCPU(this);
    }
    oldCPU->interrupts.clear();

    if (FullSystem) {
        for (ThreadID i = 0; i < size; ++i)
            threadContexts[i]->profileClear();

        if (profileEvent)
            schedule(profileEvent, curTick());
    }

    // All CPUs have an instruction and a data port, and the new CPU's
    // ports are dangling while the old CPU has its ports connected
    // already. Unbind the old CPU and then bind the ports of the one
    // we are switching to.
    assert(!getInstPort().isConnected());
    assert(oldCPU->getInstPort().isConnected());
    BaseSlavePort &inst_peer_port = oldCPU->getInstPort().getSlavePort();
    oldCPU->getInstPort().unbind();
    getInstPort().bind(inst_peer_port);

    assert(!getDataPort().isConnected());
    assert(oldCPU->getDataPort().isConnected());
    BaseSlavePort &data_peer_port = oldCPU->getDataPort().getSlavePort();
    oldCPU->getDataPort().unbind();
    getDataPort().bind(data_peer_port);
}

void
BaseCPU::flushTLBs()
{
    for (ThreadID i = 0; i < threadContexts.size(); ++i) {
        ThreadContext &tc(*threadContexts[i]);
        CheckerCPU *checker(tc.getCheckerCpuPtr());

        tc.getITBPtr()->flushAll();
        tc.getDTBPtr()->flushAll();
        if (checker) {
            checker->getITBPtr()->flushAll();
            checker->getDTBPtr()->flushAll();
        }
    }
}

void
BaseCPU::processProfileEvent()
{
    ThreadID size = threadContexts.size();

    for (ThreadID i = 0; i < size; ++i)
        threadContexts[i]->profileSample();

    schedule(profileEvent, curTick() + params()->profile);
}

void
BaseCPU::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(instCnt);

    if (!_switchedOut) {
        /* Unlike _pid, _taskId is not serialized, as they are dynamically
         * assigned unique ids that are only meaningful for the duration of
         * a specific run. We will need to serialize the entire taskMap in
         * system. */
        SERIALIZE_SCALAR(_pid);

        // Serialize the threads, this is done by the CPU implementation.
        for (ThreadID i = 0; i < numThreads; ++i) {
            ScopedCheckpointSection sec(cp, csprintf("xc.%i", i));
            interrupts[i]->serialize(cp);
            serializeThread(cp, i);
        }
    }
}

void
BaseCPU::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(instCnt);

    if (!_switchedOut) {
        UNSERIALIZE_SCALAR(_pid);

        // Unserialize the threads, this is done by the CPU implementation.
        for (ThreadID i = 0; i < numThreads; ++i) {
            ScopedCheckpointSection sec(cp, csprintf("xc.%i", i));
            interrupts[i]->unserialize(cp);
            unserializeThread(cp, i);
        }
    }
}

void
BaseCPU::scheduleInstStop(ThreadID tid, Counter insts, const char *cause)
{
    const Tick now(comInstEventQueue[tid]->getCurTick());
    Event *event(new LocalSimLoopExitEvent(cause, 0));

    comInstEventQueue[tid]->schedule(event, now + insts);
}

uint64_t
BaseCPU::getCurrentInstCount(ThreadID tid)
{
    return Tick(comInstEventQueue[tid]->getCurTick());
}

AddressMonitor::AddressMonitor() {
    armed = false;
    waiting = false;
    gotWakeup = false;
}

bool AddressMonitor::doMonitor(PacketPtr pkt) {
    assert(pkt->req->hasPaddr());
    if (armed && waiting) {
        if (pAddr == pkt->getAddr()) {
            DPRINTF(Mwait,"pAddr=0x%lx invalidated: waking up core\n",
                    pkt->getAddr());
            waiting = false;
            return true;
        }
    }
    return false;
}

void
BaseCPU::scheduleLoadStop(ThreadID tid, Counter loads, const char *cause)
{
    const Tick now(comLoadEventQueue[tid]->getCurTick());
    Event *event(new LocalSimLoopExitEvent(cause, 0));

    comLoadEventQueue[tid]->schedule(event, now + loads);
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
BaseCPU::waitForRemoteGDB() const
{
    return params()->wait_for_remote_gdb;
}
