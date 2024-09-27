/*
 * Copyright (c) 2011-2012,2016-2017, 2019-2020 Arm Limited
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
 */

#include "cpu/base.hh"

#include <iostream>
#include <sstream>
#include <string>

#include "arch/generic/decoder.hh"
#include "arch/generic/isa.hh"
#include "arch/generic/tlb.hh"
#include "base/cprintf.hh"
#include "base/loader/symtab.hh"
#include "base/logging.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/thread_context.hh"
#include "debug/Mwait.hh"
#include "debug/SyscallVerbose.hh"
#include "debug/Thread.hh"
#include "mem/page_table.hh"
#include "params/BaseCPU.hh"
#include "sim/clocked_object.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/root.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

// Hack
#include "sim/stat_control.hh"

namespace gem5
{

std::unique_ptr<BaseCPU::GlobalStats> BaseCPU::globalStats;

std::vector<BaseCPU *> BaseCPU::cpuList;

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

BaseCPU::BaseCPU(const Params &p, bool is_checker)
    : ClockedObject(p), instCnt(0), _cpuId(p.cpu_id), _socketId(p.socket_id),
      _instRequestorId(p.system->getRequestorId(this, "inst")),
      _dataRequestorId(p.system->getRequestorId(this, "data")),
      _taskId(context_switch_task_id::Unknown), _pid(invldPid),
      _switchedOut(p.switched_out), _cacheLineSize(p.system->cacheLineSize()),
      modelResetPort(p.name + ".model_reset"),
      interrupts(p.interrupts), numThreads(p.numThreads), system(p.system),
      previousCycle(0), previousState(CPU_STATE_SLEEP),
      functionTraceStream(nullptr), currentFunctionStart(0),
      currentFunctionEnd(0), functionEntryTick(0),
      baseStats(this),
      addressMonitor(p.numThreads),
      syscallRetryLatency(p.syscallRetryLatency),
      pwrGatingLatency(p.pwr_gating_latency),
      powerGatingOnIdle(p.power_gating_on_idle),
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

    functionTracingEnabled = false;
    if (p.function_trace) {
        const std::string fname = csprintf("ftrace.%s", name());
        functionTraceStream = simout.findOrCreate(fname)->stream();

        currentFunctionStart = currentFunctionEnd = 0;
        functionEntryTick = p.function_trace_start;

        if (p.function_trace_start == 0) {
            functionTracingEnabled = true;
        } else {
            Event *event = new EventFunctionWrapper(
                [this]{ enableFunctionTrace(); }, name(), true);
            schedule(event, p.function_trace_start);
        }
    }

    tracer = params().tracer;

    if (params().isa.size() != numThreads) {
        fatal("Number of ISAs (%i) assigned to the CPU does not equal number "
              "of threads (%i).\n", params().isa.size(), numThreads);
    }

    if (!FullSystem && params().workload.size() != numThreads) {
        fatal("Number of processes (cpu.workload) (%i) assigned to the CPU "
              "does not equal number of threads (%i).\n",
              params().workload.size(), numThreads);
    }

    modelResetPort.onChange([this](const bool &new_val) {
        setReset(new_val);
    });

    for (int i = 0; i < params().port_cpu_idle_pins_connection_count; i++) {
        cpuIdlePins.emplace_back(new IntSourcePin<BaseCPU>(
            csprintf("%s.cpu_idle_pins[%d]", name(), i), i, this));
    }

    // create a stat group object for each thread on this core
    fetchStats.reserve(numThreads);
    executeStats.reserve(numThreads);
    commitStats.reserve(numThreads);
    for (int i = 0; i < numThreads; i++) {
        // create fetchStat object for thread i and set rate formulas
        FetchCPUStats* fetchStatptr = new FetchCPUStats(this, i);
        fetchStatptr->fetchRate = fetchStatptr->numInsts / baseStats.numCycles;
        fetchStatptr->branchRate = fetchStatptr->numBranches /
            baseStats.numCycles;
        fetchStats.emplace_back(fetchStatptr);

        // create executeStat object for thread i and set rate formulas
        ExecuteCPUStats* executeStatptr = new ExecuteCPUStats(this, i);
        executeStatptr->instRate = executeStatptr->numInsts /
            baseStats.numCycles;
        executeStats.emplace_back(executeStatptr);

        // create commitStat object for thread i and set ipc, cpi formulas
        CommitCPUStats* commitStatptr = new CommitCPUStats(this, i);
        commitStatptr->ipc = commitStatptr->numInsts / baseStats.numCycles;
        commitStatptr->cpi = baseStats.numCycles / commitStatptr->numInsts;
        commitStats.emplace_back(commitStatptr);
    }
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
BaseCPU::postInterrupt(ThreadID tid, int int_num, int index)
{
    interrupts[tid]->post(int_num, index);
    // Only wake up syscall emulation if it is not waiting on a futex.
    // This is to model the fact that instructions such as ARM SEV
    // should wake up a WFE sleep, but not a futex syscall WAIT.
    if (FullSystem || !system->futexMap.is_waiting(threadContexts[tid]))
        wakeup(tid);
}

void
BaseCPU::armMonitor(ThreadID tid, Addr address)
{
    assert(tid < numThreads);
    AddressMonitor &monitor = addressMonitor[tid];

    monitor.armed = true;
    monitor.vAddr = address;
    monitor.pAddr = 0x0;
    DPRINTF(Mwait, "[tid:%d] Armed monitor (vAddr=0x%lx)\n", tid, address);
}

bool
BaseCPU::mwait(ThreadID tid, PacketPtr pkt)
{
    assert(tid < numThreads);
    AddressMonitor &monitor = addressMonitor[tid];

    if (!monitor.gotWakeup) {
        Addr block_size = cacheLineSize();
        Addr mask = ~(block_size - 1);

        assert(pkt->req->hasPaddr());
        monitor.pAddr = pkt->getAddr() & mask;
        monitor.waiting = true;

        DPRINTF(Mwait, "[tid:%d] mwait called (vAddr=0x%lx, "
                "line's paddr=0x%lx)\n", tid, monitor.vAddr, monitor.pAddr);
        return true;
    } else {
        monitor.gotWakeup = false;
        return false;
    }
}

void
BaseCPU::mwaitAtomic(ThreadID tid, ThreadContext *tc, BaseMMU *mmu)
{
    assert(tid < numThreads);
    AddressMonitor &monitor = addressMonitor[tid];

    RequestPtr req = std::make_shared<Request>();

    Addr addr = monitor.vAddr;
    Addr block_size = cacheLineSize();
    Addr mask = ~(block_size - 1);
    int size = block_size;

    //The address of the next line if it crosses a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, block_size);

    if (secondAddr > addr)
        size = secondAddr - addr;

    req->setVirt(addr, size, 0x0, dataRequestorId(),
            tc->pcState().instAddr());

    // translate to physical address
    Fault fault = mmu->translateAtomic(req, tc, BaseMMU::Read);
    assert(fault == NoFault);

    monitor.pAddr = req->getPaddr() & mask;
    monitor.waiting = true;

    DPRINTF(Mwait, "[tid:%d] mwait called (vAddr=0x%lx, line's paddr=0x%lx)\n",
            tid, monitor.vAddr, monitor.pAddr);
}

void
BaseCPU::init()
{
    // Set up instruction-count-based termination events, if any. This needs
    // to happen after threadContexts has been constructed.
    if (params().max_insts_any_thread != 0) {
        scheduleInstStopAnyThread(params().max_insts_any_thread);
    }

    // Set up instruction-count-based termination events for SimPoints
    // Typically, there are more than one action points.
    // Simulation.py is responsible to take the necessary actions upon
    // exitting the simulation loop.
    if (!params().simpoint_start_insts.empty()) {
        scheduleSimpointsInstStop(params().simpoint_start_insts);
    }

    if (params().max_insts_all_threads != 0) {
        std::string cause = "all threads reached the max instruction count";

        // allocate & initialize shared downcounter: each event will
        // decrement this when triggered; simulation will terminate
        // when counter reaches 0
        int *counter = new int;
        *counter = numThreads;
        for (ThreadID tid = 0; tid < numThreads; ++tid) {
            Event *event = new CountedExitEvent(cause, *counter);
            threadContexts[tid]->scheduleInstCountEvent(
                    event, params().max_insts_all_threads);
        }
    }

    if (!params().switched_out) {
        registerThreadContexts();

        verifyMemoryMode();
    }
}

void
BaseCPU::startup()
{
    if (params().progress_interval) {
        new CPUProgressEvent(this, params().progress_interval);
    }

    if (_switchedOut)
        powerState->set(enums::PwrState::OFF);

    // Assumption CPU start to operate instantaneously without any latency
    if (powerState->get() == enums::PwrState::UNDEFINED)
        powerState->set(enums::PwrState::ON);

}

probing::PMUUPtr
BaseCPU::pmuProbePoint(const char *name)
{
    probing::PMUUPtr ptr;
    ptr.reset(new probing::PMU(getProbeManager(), name));

    return ptr;
}

void
BaseCPU::regProbePoints()
{
    ppAllCycles = pmuProbePoint("Cycles");
    ppActiveCycles = pmuProbePoint("ActiveCycles");

    ppRetiredInsts = pmuProbePoint("RetiredInsts");
    ppRetiredInstsPC = pmuProbePoint("RetiredInstsPC");
    ppRetiredLoads = pmuProbePoint("RetiredLoads");
    ppRetiredStores = pmuProbePoint("RetiredStores");
    ppRetiredBranches = pmuProbePoint("RetiredBranches");

    ppSleeping = new ProbePointArg<bool>(this->getProbeManager(),
                                         "Sleeping");
}

void
BaseCPU::probeInstCommit(const StaticInstPtr &inst, Addr pc)
{
    if (!inst->isMicroop() || inst->isLastMicroop()) {
        ppRetiredInsts->notify(1);
        ppRetiredInstsPC->notify(pc);
    }

    if (inst->isLoad())
        ppRetiredLoads->notify(1);

    if (inst->isStore() || inst->isAtomic())
        ppRetiredStores->notify(1);

    if (inst->isControl())
        ppRetiredBranches->notify(1);
}

BaseCPU::
BaseCPUStats::BaseCPUStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(numCycles, statistics::units::Cycle::get(),
               "Number of cpu cycles simulated"),
      ADD_STAT(cpi, statistics::units::Rate<
                statistics::units::Cycle, statistics::units::Count>::get(),
               "CPI: cycles per instruction (core level)"),
      ADD_STAT(ipc, statistics::units::Rate<
                statistics::units::Count, statistics::units::Cycle>::get(),
               "IPC: instructions per cycle (core level)"),
      ADD_STAT(numWorkItemsStarted, statistics::units::Count::get(),
               "Number of work items this cpu started"),
      ADD_STAT(numWorkItemsCompleted, statistics::units::Count::get(),
               "Number of work items this cpu completed")
{
    cpi.precision(6);
    cpi = numCycles / numInsts;

    ipc.precision(6);
    ipc = numInsts / numCycles;
}

void
BaseCPU::regStats()
{
    ClockedObject::regStats();

    if (!globalStats) {
        /* We need to construct the global CPU stat structure here
         * since it needs a pointer to the Root object. */
        globalStats.reset(new GlobalStats(Root::root()));
    }

    using namespace statistics;

    int size = threadContexts.size();
    if (size > 1) {
        for (int i = 0; i < size; ++i) {
            std::stringstream namestr;
            ccprintf(namestr, "%s.ctx%d", name(), i);
            threadContexts[i]->regStats(namestr.str());
        }
    } else if (size == 1)
        threadContexts[0]->regStats(name());
}

Port &
BaseCPU::getPort(const std::string &if_name, PortID idx)
{
    // Get the right port based on name. This applies to all the
    // subclasses of the base CPU and relies on their implementation
    // of getDataPort and getInstPort.
    if (if_name == "dcache_port")
        return getDataPort();
    else if (if_name == "icache_port")
        return getInstPort();
    else if (if_name == "model_reset")
        return modelResetPort;
    else if (if_name == "cpu_idle_pins")
        return *cpuIdlePins[idx];
    else
        return ClockedObject::getPort(if_name, idx);
}

void
BaseCPU::registerThreadContexts()
{
    assert(system->multiThread || numThreads == 1);

    fatal_if(interrupts.size() != numThreads,
             "CPU %s has %i interrupt controllers, but is expecting one "
             "per thread (%i)\n",
             name(), interrupts.size(), numThreads);

    for (ThreadID tid = 0; tid < threadContexts.size(); ++tid) {
        ThreadContext *tc = threadContexts[tid];

        system->registerThreadContext(tc);

        if (!FullSystem)
            tc->getProcessPtr()->assignThreadContext(tc->contextId());

        interrupts[tid]->setThreadContext(tc);
        tc->getIsaPtr()->setThreadContext(tc);
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

    if (powerState->get() == enums::PwrState::CLK_GATED &&
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
    if (modelResetPort.state()) {
        DPRINTF(Thread, "CPU in reset, not activating context %d\n",
                threadContexts[thread_num]->contextId());
        return;
    }

    DPRINTF(Thread, "activate contextId %d\n",
            threadContexts[thread_num]->contextId());

    if (thread_num < cpuIdlePins.size()) {
        cpuIdlePins[thread_num]->lower();
    }

    // Squash enter power gating event while cpu gets activated
    if (enterPwrGatingEvent.scheduled())
        deschedule(enterPwrGatingEvent);
    // For any active thread running, update CPU power state to active (ON)
    powerState->set(enums::PwrState::ON);

    updateCycleCounters(CPU_STATE_WAKEUP);
}

void
BaseCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(Thread, "suspend contextId %d\n",
            threadContexts[thread_num]->contextId());

    if (thread_num < cpuIdlePins.size()) {
        cpuIdlePins[thread_num]->raise();
    }

    // Check if all threads are suspended
    for (auto t : threadContexts) {
        if (t->status() != ThreadContext::Suspended) {
            return;
        }
    }

    // All CPU thread are suspended, update cycle count
    updateCycleCounters(CPU_STATE_SLEEP);

    // All CPU threads suspended, enter lower power state for the CPU
    powerState->set(enums::PwrState::CLK_GATED);

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
    powerState->set(enums::PwrState::OFF);
}

void
BaseCPU::switchOut()
{
    assert(!_switchedOut);
    _switchedOut = true;

    // Flush all TLBs in the CPU to avoid having stale translations if
    // it gets switched in later.
    flushTLBs();

    // Go to the power gating state
    powerState->set(enums::PwrState::OFF);
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
    powerState->set(oldCPU->powerState->get());

    previousState = oldCPU->previousState;
    previousCycle = oldCPU->previousCycle;

    _switchedOut = false;

    ThreadID size = threadContexts.size();
    for (ThreadID i = 0; i < size; ++i) {
        ThreadContext *newTC = threadContexts[i];
        ThreadContext *oldTC = oldCPU->threadContexts[i];

        newTC->getIsaPtr()->setThreadContext(newTC);

        newTC->takeOverFrom(oldTC);

        assert(newTC->contextId() == oldTC->contextId());
        assert(newTC->threadId() == oldTC->threadId());
        system->replaceThreadContext(newTC, newTC->contextId());

        /* This code no longer works since the zero register (e.g.,
         * r31 on Alpha) doesn't necessarily contain zero at this
         * point.
           if (debug::Context)
            ThreadContext::compare(oldTC, newTC);
        */

        newTC->getMMUPtr()->takeOverFrom(oldTC->getMMUPtr());

        // Checker whether or not we have to transfer CheckerCPU
        // objects over in the switch
        CheckerCPU *old_checker = oldTC->getCheckerCpuPtr();
        CheckerCPU *new_checker = newTC->getCheckerCpuPtr();
        if (old_checker && new_checker) {
            new_checker->getMMUPtr()->takeOverFrom(old_checker->getMMUPtr());
        }
    }

    interrupts = oldCPU->interrupts;
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        interrupts[tid]->setThreadContext(threadContexts[tid]);
    }
    oldCPU->interrupts.clear();

    // All CPUs have an instruction and a data port, and the new CPU's
    // ports are dangling while the old CPU has its ports connected
    // already. Unbind the old CPU and then bind the ports of the one
    // we are switching to.
    getInstPort().takeOverFrom(&oldCPU->getInstPort());
    getDataPort().takeOverFrom(&oldCPU->getDataPort());

    // Switch over the reset line as well, if necessary.
    if (oldCPU->modelResetPort.isConnected())
        modelResetPort.takeOverFrom(&oldCPU->modelResetPort);
}

void
BaseCPU::setReset(bool state)
{
    for (auto tc: threadContexts) {
        if (state) {
            // As we enter reset, stop execution.
            tc->quiesce();
        } else {
            // As we leave reset, first reset thread state,
            tc->getIsaPtr()->resetThread();
            // reset the decoder in case it had partially decoded something,
            tc->getDecoderPtr()->reset();
            // reset MMU,
            tc->getMMUPtr()->reset();
            // Clear any interrupts,
            interrupts[tc->threadId()]->clearAll();
            // and finally reenable execution.
            tc->activate();
        }
    }
}

void
BaseCPU::flushTLBs()
{
    for (ThreadID i = 0; i < threadContexts.size(); ++i) {
        ThreadContext &tc(*threadContexts[i]);
        CheckerCPU *checker(tc.getCheckerCpuPtr());

        tc.getMMUPtr()->flushAll();
        if (checker) {
            checker->getMMUPtr()->flushAll();
        }
    }
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
BaseCPU::scheduleInstStop(ThreadID tid, Counter insts, std::string cause)
{
    const Tick now(getCurrentInstCount(tid));
    Event *event(new LocalSimLoopExitEvent(cause, 0));

    threadContexts[tid]->scheduleInstCountEvent(event, now + insts);
}

Tick
BaseCPU::getCurrentInstCount(ThreadID tid)
{
    return threadContexts[tid]->getCurrentInstCount();
}

AddressMonitor::AddressMonitor()
{
    armed = false;
    waiting = false;
    gotWakeup = false;
}

bool
AddressMonitor::doMonitor(PacketPtr pkt)
{
    assert(pkt->req->hasPaddr());
    if (armed && waiting) {
        if (pAddr == pkt->getAddr()) {
            DPRINTF(Mwait, "pAddr=0x%lx invalidated: waking up core\n",
                    pkt->getAddr());
            waiting = false;
            return true;
        }
    }
    return false;
}


void
BaseCPU::traceFunctionsInternal(Addr pc)
{
    if (loader::debugSymbolTable.empty())
        return;

    // if pc enters different function, print new function symbol and
    // update saved range.  Otherwise do nothing.
    if (pc < currentFunctionStart || pc >= currentFunctionEnd) {
        auto it = loader::debugSymbolTable.findNearest(
                pc, currentFunctionEnd);

        std::string sym_str;
        if (it == loader::debugSymbolTable.end()) {
            // no symbol found: use addr as label
            sym_str = csprintf("%#x", pc);
            currentFunctionStart = pc;
            currentFunctionEnd = pc + 1;
        } else {
            sym_str = it->name();
            currentFunctionStart = it->address();
        }

        ccprintf(*functionTraceStream, " (%d)\n%d: %s",
                 curTick() - functionEntryTick, curTick(), sym_str);
        functionEntryTick = curTick();
    }
}

void
BaseCPU::scheduleSimpointsInstStop(std::vector<Counter> inst_starts)
{
    std::string cause = "simpoint starting point found";
    for (size_t i = 0; i < inst_starts.size(); ++i) {
        scheduleInstStop(0, inst_starts[i], cause);
    }
}

void
BaseCPU::scheduleInstStopAnyThread(Counter max_insts)
{
    std::string cause = "a thread reached the max instruction count";
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        scheduleInstStop(tid, max_insts, cause);
    }
}

BaseCPU::GlobalStats::GlobalStats(statistics::Group *parent)
    : statistics::Group(parent),
    ADD_STAT(simInsts, statistics::units::Count::get(),
             "Number of instructions simulated"),
    ADD_STAT(simOps, statistics::units::Count::get(),
             "Number of ops (including micro ops) simulated"),
    ADD_STAT(hostInstRate, statistics::units::Rate<
                statistics::units::Count, statistics::units::Second>::get(),
             "Simulator instruction rate (inst/s)"),
    ADD_STAT(hostOpRate, statistics::units::Rate<
                statistics::units::Count, statistics::units::Second>::get(),
             "Simulator op (including micro ops) rate (op/s)")
{
    simInsts
        .functor(BaseCPU::GlobalStats::numSimulatedInsts)
        .precision(0)
        .prereq(simInsts)
        ;

    simOps
        .functor(BaseCPU::GlobalStats::numSimulatedOps)
        .precision(0)
        .prereq(simOps)
        ;

    hostInstRate
        .precision(0)
        .prereq(simInsts)
        ;

    hostOpRate
        .precision(0)
        .prereq(simOps)
        ;

    hostInstRate = simInsts / hostSeconds;
    hostOpRate = simOps / hostSeconds;
}

BaseCPU::
FetchCPUStats::FetchCPUStats(statistics::Group *parent, int thread_id)
    : statistics::Group(parent, csprintf("fetchStats%i", thread_id).c_str()),
    ADD_STAT(numInsts, statistics::units::Count::get(),
             "Number of instructions fetched (thread level)"),
    ADD_STAT(numOps, statistics::units::Count::get(),
             "Number of ops (including micro ops) fetched (thread level)"),
    ADD_STAT(fetchRate, statistics::units::Rate<
             statistics::units::Count, statistics::units::Cycle>::get(),
             "Number of inst fetches per cycle"),
    ADD_STAT(numBranches, statistics::units::Count::get(),
             "Number of branches fetched"),
    ADD_STAT(branchRate, statistics::units::Ratio::get(),
             "Number of branch fetches per cycle"),
    ADD_STAT(icacheStallCycles, statistics::units::Cycle::get(),
             "ICache total stall cycles"),
    ADD_STAT(numFetchSuspends, statistics::units::Count::get(),
             "Number of times Execute suspended instruction fetching")

{
    fetchRate
        .flags(statistics::total);

    numBranches
        .prereq(numBranches);

    branchRate
        .flags(statistics::total);

    icacheStallCycles
        .prereq(icacheStallCycles);

}

// means it is incremented in a vector indexing and not directly
BaseCPU::
ExecuteCPUStats::ExecuteCPUStats(statistics::Group *parent, int thread_id)
    : statistics::Group(parent, csprintf("executeStats%i", thread_id).c_str()),
    ADD_STAT(numInsts, statistics::units::Count::get(),
             "Number of executed instructions"),
    ADD_STAT(numNop, statistics::units::Count::get(),
             "Number of nop insts executed"),
    ADD_STAT(numBranches, statistics::units::Count::get(),
             "Number of branches executed"),
    ADD_STAT(numLoadInsts, statistics::units::Count::get(),
             "Number of load instructions executed"),
    ADD_STAT(numStoreInsts, statistics::units::Count::get(),
             "Number of stores executed"),
    ADD_STAT(instRate, statistics::units::Rate<
                statistics::units::Count, statistics::units::Cycle>::get(),
             "Inst execution rate"),
    ADD_STAT(dcacheStallCycles, statistics::units::Cycle::get(),
             "DCache total stall cycles"),
    ADD_STAT(numCCRegReads, statistics::units::Count::get(),
             "Number of times the CC registers were read"),
    ADD_STAT(numCCRegWrites, statistics::units::Count::get(),
             "Number of times the CC registers were written"),
    ADD_STAT(numFpAluAccesses, statistics::units::Count::get(),
             "Number of float alu accesses"),
    ADD_STAT(numFpRegReads, statistics::units::Count::get(),
             "Number of times the floating registers were read"),
    ADD_STAT(numFpRegWrites, statistics::units::Count::get(),
             "Number of times the floating registers were written"),
    ADD_STAT(numIntAluAccesses, statistics::units::Count::get(),
             "Number of integer alu accesses"),
    ADD_STAT(numIntRegReads, statistics::units::Count::get(),
             "Number of times the integer registers were read"),
    ADD_STAT(numIntRegWrites, statistics::units::Count::get(),
             "Number of times the integer registers were written"),
    ADD_STAT(numMemRefs, statistics::units::Count::get(),
             "Number of memory refs"),
    ADD_STAT(numMiscRegReads, statistics::units::Count::get(),
             "Number of times the Misc registers were read"),
    ADD_STAT(numMiscRegWrites, statistics::units::Count::get(),
             "Number of times the Misc registers were written"),
    ADD_STAT(numVecAluAccesses, statistics::units::Count::get(),
             "Number of vector alu accesses"),
    ADD_STAT(numVecPredRegReads, statistics::units::Count::get(),
             "Number of times the predicate registers were read"),
    ADD_STAT(numVecPredRegWrites, statistics::units::Count::get(),
             "Number of times the predicate registers were written"),
    ADD_STAT(numVecRegReads, statistics::units::Count::get(),
             "Number of times the vector registers were read"),
    ADD_STAT(numVecRegWrites, statistics::units::Count::get(),
             "Number of times the vector registers were written"),
    ADD_STAT(numDiscardedOps, statistics::units::Count::get(),
             "Number of ops (including micro ops) which were discarded before "
             "commit")
{
    numStoreInsts = numMemRefs - numLoadInsts;

    dcacheStallCycles
        .prereq(dcacheStallCycles);
    numCCRegReads
        .prereq(numCCRegReads)
        .flags(statistics::nozero);
    numCCRegWrites
        .prereq(numCCRegWrites)
        .flags(statistics::nozero);
    numFpAluAccesses
        .prereq(numFpAluAccesses);
    numFpRegReads
        .prereq(numFpRegReads);
    numIntAluAccesses
        .prereq(numIntAluAccesses);
    numIntRegReads
        .prereq(numIntRegReads);
    numIntRegWrites
        .prereq(numIntRegWrites);
    numMiscRegReads
        .prereq(numMiscRegReads);
    numMiscRegWrites
        .prereq(numMiscRegWrites);
    numVecPredRegReads
        .prereq(numVecPredRegReads);
    numVecPredRegWrites
        .prereq(numVecPredRegWrites);
    numVecRegReads
        .prereq(numVecRegReads);
    numVecRegWrites
        .prereq(numVecRegWrites);
}

BaseCPU::
CommitCPUStats::CommitCPUStats(statistics::Group *parent, int thread_id)
    : statistics::Group(parent, csprintf("commitStats%i", thread_id).c_str()),
    ADD_STAT(numInsts, statistics::units::Count::get(),
             "Number of instructions committed (thread level)"),
    ADD_STAT(numOps, statistics::units::Count::get(),
             "Number of ops (including micro ops) committed (thread level)"),
    ADD_STAT(numInstsNotNOP, statistics::units::Count::get(),
             "Number of instructions committed excluding NOPs or prefetches"),
    ADD_STAT(numOpsNotNOP, statistics::units::Count::get(),
             "Number of Ops (including micro ops) Simulated"),
    ADD_STAT(cpi, statistics::units::Rate<
                statistics::units::Cycle, statistics::units::Count>::get(),
             "CPI: cycles per instruction (thread level)"),
    ADD_STAT(ipc, statistics::units::Rate<
                statistics::units::Count, statistics::units::Cycle>::get(),
             "IPC: instructions per cycle (thread level)"),
    ADD_STAT(numMemRefs, statistics::units::Count::get(),
            "Number of memory references committed"),
    ADD_STAT(numFpInsts, statistics::units::Count::get(),
            "Number of float instructions"),
    ADD_STAT(numIntInsts, statistics::units::Count::get(),
            "Number of integer instructions"),
    ADD_STAT(numLoadInsts, statistics::units::Count::get(),
            "Number of load instructions"),
    ADD_STAT(numStoreInsts, statistics::units::Count::get(),
            "Number of store instructions"),
    ADD_STAT(numVecInsts, statistics::units::Count::get(),
            "Number of vector instructions"),
    ADD_STAT(committedInstType, statistics::units::Count::get(),
            "Class of committed instruction."),
    ADD_STAT(committedControl, statistics::units::Count::get(),
             "Class of control type instructions committed")
{
    numInsts
        .prereq(numInsts);

    cpi.precision(6);
    ipc.precision(6);

    committedInstType
        .init(enums::Num_OpClass)
        .flags(statistics::total | statistics::pdf | statistics::dist);

    for (unsigned i = 0; i < Num_OpClasses; ++i) {
        committedInstType.subname(i, enums::OpClassStrings[i]);
    }

    committedControl
        .init(StaticInstFlags::Flags::Num_Flags)
        .flags(statistics::nozero);

    for (unsigned i = 0; i < StaticInstFlags::Flags::Num_Flags; i++) {
        committedControl.subname(i, StaticInstFlags::FlagsStrings[i]);
    }
}


void
BaseCPU::
CommitCPUStats::updateComCtrlStats(const StaticInstPtr staticInst)
{
    /* Add a count for every control instruction type */
    if (staticInst->isControl()) {
        if (staticInst->isReturn()) {
            committedControl[gem5::StaticInstFlags::Flags::IsReturn]++;
        }
        if (staticInst->isCall()) {
            committedControl[gem5::StaticInstFlags::Flags::IsCall]++;
        }
        if (staticInst->isDirectCtrl()) {
            committedControl[gem5::StaticInstFlags::Flags::IsDirectControl]++;
        }
        if (staticInst->isIndirectCtrl()) {
            committedControl
                [gem5::StaticInstFlags::Flags::IsIndirectControl]++;
        }
        if (staticInst->isCondCtrl()) {
            committedControl[gem5::StaticInstFlags::Flags::IsCondControl]++;
        }
        if (staticInst->isUncondCtrl()) {
            committedControl[gem5::StaticInstFlags::Flags::IsUncondControl]++;
        }
        committedControl[gem5::StaticInstFlags::Flags::IsControl]++;
    }
}

} // namespace gem5
