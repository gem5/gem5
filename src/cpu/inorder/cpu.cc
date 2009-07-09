/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 *
 */

#include <algorithm>

#include "arch/utility.hh"
#include "config/full_system.hh"
#include "cpu/exetrace.hh"
#include "cpu/activity.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "cpu/base.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/thread_context.hh"
#include "cpu/inorder/thread_state.hh"
#include "cpu/inorder/cpu.hh"
#include "params/InOrderCPU.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/first_stage.hh"
#include "cpu/inorder/resources/resource_list.hh"
#include "cpu/inorder/resource_pool.hh"
#include "mem/translating_port.hh"
#include "sim/process.hh"
#include "sim/stat_control.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

InOrderCPU::TickEvent::TickEvent(InOrderCPU *c)
  : Event(CPU_Tick_Pri), cpu(c)
{ }


void
InOrderCPU::TickEvent::process()
{
    cpu->tick();
}


const char *
InOrderCPU::TickEvent::description()
{
    return "InOrderCPU tick event";
}

InOrderCPU::CPUEvent::CPUEvent(InOrderCPU *_cpu, CPUEventType e_type,
                             Fault fault, ThreadID _tid, unsigned _vpe)
    : Event(CPU_Tick_Pri), cpu(_cpu)
{
    setEvent(e_type, fault, _tid, _vpe);
}


std::string InOrderCPU::eventNames[NumCPUEvents] =
{
    "ActivateThread",
    "DeallocateThread",
    "SuspendThread",
    "DisableThreads",
    "EnableThreads",
    "DisableVPEs",
    "EnableVPEs",
    "Trap",
    "InstGraduated",
    "SquashAll",
    "UpdatePCs"
};

void
InOrderCPU::CPUEvent::process()
{
    switch (cpuEventType)
    {
      case ActivateThread:
        cpu->activateThread(tid);
        break;

      //@TODO: Consider Implementing "Suspend Thread" as Separate from Deallocate
      case SuspendThread: // Suspend & Deallocate are same for now.
        //cpu->suspendThread(tid);
        //break;
      case DeallocateThread:
        cpu->deallocateThread(tid);
        break;

      case EnableVPEs:
        cpu->enableVPEs(vpe);
        break;

      case DisableVPEs:
        cpu->disableVPEs(tid, vpe);
        break;

      case EnableThreads:
        cpu->enableThreads(vpe);
        break;

      case DisableThreads:
        cpu->disableThreads(tid, vpe);
        break;

      case Trap:
        cpu->trapCPU(fault, tid);
        break;

      default:
        fatal("Unrecognized Event Type %d", cpuEventType);
    }

    cpu->cpuEventRemoveList.push(this);
}

const char *
InOrderCPU::CPUEvent::description()
{
    return "InOrderCPU event";
}

void
InOrderCPU::CPUEvent::scheduleEvent(int delay)
{
    if (squashed())
      mainEventQueue.reschedule(this,curTick + cpu->ticks(delay));
    else if (!scheduled())
      mainEventQueue.schedule(this,curTick + cpu->ticks(delay));
}

void
InOrderCPU::CPUEvent::unscheduleEvent()
{
    if (scheduled())
        squash();
}

InOrderCPU::InOrderCPU(Params *params)
    : BaseCPU(params),
      cpu_id(params->cpu_id),
      coreType("default"),
      _status(Idle),
      tickEvent(this),
      timeBuffer(2 , 2),
      removeInstsThisCycle(false),
      activityRec(params->name, NumStages, 10, params->activity),
      switchCount(0),
      deferRegistration(false/*params->deferRegistration*/),
      stageTracing(params->stageTracing),
      numVirtProcs(1)
{
    cpu_params = params;

    resPool = new ResourcePool(this, params);

    // Resize for Multithreading CPUs
    thread.resize(numThreads);

    ThreadID active_threads = params->workload.size();

    if (active_threads > MaxThreads) {
        panic("Workload Size too large. Increase the 'MaxThreads'"
              "in your InOrder implementation or "
              "edit your workload size.");
    }

    // Bind the fetch & data ports from the resource pool.
    fetchPortIdx = resPool->getPortIdx(params->fetchMemPort);
    if (fetchPortIdx == 0) {
        fatal("Unable to find port to fetch instructions from.\n");
    }

    dataPortIdx = resPool->getPortIdx(params->dataMemPort);
    if (dataPortIdx == 0) {
        fatal("Unable to find port for data.\n");
    }

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        if (tid < (ThreadID)params->workload.size()) {
            DPRINTF(InOrderCPU, "Workload[%i] process is %#x\n",
                    tid, this->thread[tid]);
            this->thread[tid] =
                new Thread(this, tid, params->workload[tid], tid);
        } else {
            //Allocate Empty thread so M5 can use later
            //when scheduling threads to CPU
            Process* dummy_proc = params->workload[0];
            this->thread[tid] = new Thread(this, tid, dummy_proc, tid);
        }

        // Setup the TC that will serve as the interface to the threads/CPU.
        InOrderThreadContext *tc = new InOrderThreadContext;
        tc->cpu = this;
        tc->thread = thread[tid];

        // Give the thread the TC.
        thread[tid]->tc = tc;
        thread[tid]->setFuncExeInst(0);
        globalSeqNum[tid] = 1;

        // Add the TC to the CPU's list of TC's.
        this->threadContexts.push_back(tc);
    }

    // Initialize TimeBuffer Stage Queues
    for (int stNum=0; stNum < NumStages - 1; stNum++) {
        stageQueue[stNum] = new StageQueue(NumStages, NumStages);
        stageQueue[stNum]->id(stNum);
    }


    // Set Up Pipeline Stages
    for (int stNum=0; stNum < NumStages; stNum++) {
        if (stNum == 0)
            pipelineStage[stNum] = new FirstStage(params, stNum);
        else
            pipelineStage[stNum] = new PipelineStage(params, stNum);

        pipelineStage[stNum]->setCPU(this);
        pipelineStage[stNum]->setActiveThreads(&activeThreads);
        pipelineStage[stNum]->setTimeBuffer(&timeBuffer);

        // Take Care of 1st/Nth stages
        if (stNum > 0)
            pipelineStage[stNum]->setPrevStageQueue(stageQueue[stNum - 1]);
        if (stNum < NumStages - 1)
            pipelineStage[stNum]->setNextStageQueue(stageQueue[stNum]);
    }

    // Initialize thread specific variables
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        archRegDepMap[tid].setCPU(this);

        nonSpecInstActive[tid] = false;
        nonSpecSeqNum[tid] = 0;

        squashSeqNum[tid] = MaxAddr;
        lastSquashCycle[tid] = 0;

        intRegFile[tid].clear();
        floatRegFile[tid].clear();
        isa[tid].clear();

        isa[tid].expandForMultithreading(numThreads, numVirtProcs);
    }

    lastRunningCycle = curTick;
    contextSwitch = false;

    // Define dummy instructions and resource requests to be used.
    DynInstPtr dummyBufferInst = new InOrderDynInst(this, NULL, 0, 0);
    dummyReq = new ResourceRequest(NULL, NULL, 0, 0, 0, 0);

    // Reset CPU to reset state.
#if FULL_SYSTEM
    Fault resetFault = new ResetFault();
    resetFault->invoke(tcBase());
#else
    reset();
#endif

    // Schedule First Tick Event, CPU will reschedule itself from here on out.
    scheduleTickEvent(0);
}


void
InOrderCPU::regStats()
{
    /* Register the Resource Pool's stats here.*/
    resPool->regStats();

    /* Register any of the InOrderCPU's stats here.*/
    timesIdled
        .name(name() + ".timesIdled")
        .desc("Number of times that the entire CPU went into an idle state and"
              " unscheduled itself")
        .prereq(timesIdled);

    idleCycles
        .name(name() + ".idleCycles")
        .desc("Total number of cycles that the CPU has spent unscheduled due "
              "to idling")
        .prereq(idleCycles);

    threadCycles
        .init(numThreads)
        .name(name() + ".threadCycles")
        .desc("Total Number of Cycles A Thread Was Active in CPU (Per-Thread)");

    smtCycles
        .name(name() + ".smtCycles")
        .desc("Total number of cycles that the CPU was simultaneous multithreading.(SMT)");

    committedInsts
        .init(numThreads)
        .name(name() + ".committedInsts")
        .desc("Number of Instructions Simulated (Per-Thread)");

    smtCommittedInsts
        .init(numThreads)
        .name(name() + ".smtCommittedInsts")
        .desc("Number of SMT Instructions Simulated (Per-Thread)");

    totalCommittedInsts
        .name(name() + ".committedInsts_total")
        .desc("Number of Instructions Simulated (Total)");

    cpi
        .name(name() + ".cpi")
        .desc("CPI: Cycles Per Instruction (Per-Thread)")
        .precision(6);
    cpi = threadCycles / committedInsts;

    smtCpi
        .name(name() + ".smt_cpi")
        .desc("CPI: Total SMT-CPI")
        .precision(6);
    smtCpi = smtCycles / smtCommittedInsts;

    totalCpi
        .name(name() + ".cpi_total")
        .desc("CPI: Total CPI of All Threads")
        .precision(6);
    totalCpi = numCycles / totalCommittedInsts;

    ipc
        .name(name() + ".ipc")
        .desc("IPC: Instructions Per Cycle (Per-Thread)")
        .precision(6);
    ipc =  committedInsts / threadCycles;

    smtIpc
        .name(name() + ".smt_ipc")
        .desc("IPC: Total SMT-IPC")
        .precision(6);
    smtIpc = smtCommittedInsts / smtCycles;

    totalIpc
        .name(name() + ".ipc_total")
        .desc("IPC: Total IPC of All Threads")
        .precision(6);
        totalIpc =  totalCommittedInsts / numCycles;

    BaseCPU::regStats();
}


void
InOrderCPU::tick()
{
    DPRINTF(InOrderCPU, "\n\nInOrderCPU: Ticking main, InOrderCPU.\n");

    ++numCycles;

    //Tick each of the stages
    for (int stNum=NumStages - 1; stNum >= 0 ; stNum--) {
        pipelineStage[stNum]->tick();
    }

    // Now advance the time buffers one tick
    timeBuffer.advance();
    for (int sqNum=0; sqNum < NumStages - 1; sqNum++) {
        stageQueue[sqNum]->advance();
    }
    activityRec.advance();

    // Any squashed requests, events, or insts then remove them now
    cleanUpRemovedReqs();
    cleanUpRemovedEvents();
    cleanUpRemovedInsts();

    // Re-schedule CPU for this cycle
    if (!tickEvent.scheduled()) {
        if (_status == SwitchedOut) {
            // increment stat
            lastRunningCycle = curTick;
        } else if (!activityRec.active()) {
            DPRINTF(InOrderCPU, "sleeping CPU.\n");
            lastRunningCycle = curTick;
            timesIdled++;
        } else {
            //Tick next_tick = curTick + cycles(1);
            //tickEvent.schedule(next_tick);
            mainEventQueue.schedule(&tickEvent, nextCycle(curTick + 1));
            DPRINTF(InOrderCPU, "Scheduled CPU for next tick @ %i.\n", nextCycle() + curTick);
        }
    }

    tickThreadStats();
    updateThreadPriority();
}


void
InOrderCPU::init()
{
    if (!deferRegistration) {
        registerThreadContexts();
    }

    // Set inSyscall so that the CPU doesn't squash when initially
    // setting up registers.
    for (ThreadID tid = 0; tid < numThreads; ++tid)
        thread[tid]->inSyscall = true;

#if FULL_SYSTEM
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        ThreadContext *src_tc = threadContexts[tid];
        TheISA::initCPU(src_tc, src_tc->contextId());
    }
#endif

    // Clear inSyscall.
    for (ThreadID tid = 0; tid < numThreads; ++tid)
        thread[tid]->inSyscall = false;

    // Call Initializiation Routine for Resource Pool
    resPool->init();
}

void
InOrderCPU::readFunctional(Addr addr, uint32_t &buffer)
{
    tcBase()->getMemPort()->readBlob(addr, (uint8_t*)&buffer, sizeof(uint32_t));
    buffer = gtoh(buffer);
}

void
InOrderCPU::reset()
{
    for (int i = 0; i < numThreads; i++) {
        isa[i].reset(coreType, numThreads,
                numVirtProcs, dynamic_cast<BaseCPU*>(this));
    }
}

Port*
InOrderCPU::getPort(const std::string &if_name, int idx)
{
    return resPool->getPort(if_name, idx);
}

void
InOrderCPU::trap(Fault fault, ThreadID tid, int delay)
{
    //@ Squash Pipeline during TRAP
    scheduleCpuEvent(Trap, fault, tid, 0/*vpe*/, delay);
}

void
InOrderCPU::trapCPU(Fault fault, ThreadID tid)
{
    fault->invoke(tcBase(tid));
}

void
InOrderCPU::scheduleCpuEvent(CPUEventType c_event, Fault fault,
                           ThreadID tid, unsigned vpe, unsigned delay)
{
    CPUEvent *cpu_event = new CPUEvent(this, c_event, fault, tid, vpe);

    if (delay >= 0) {
        DPRINTF(InOrderCPU, "Scheduling CPU Event (%s) for cycle %i.\n",
                eventNames[c_event], curTick + delay);
        mainEventQueue.schedule(cpu_event,curTick + delay);
    } else {
        cpu_event->process();
        cpuEventRemoveList.push(cpu_event);
    }

    // Broadcast event to the Resource Pool
    DynInstPtr dummy_inst =
        new InOrderDynInst(this, NULL, getNextEventNum(), tid);
    resPool->scheduleEvent(c_event, dummy_inst, 0, 0, tid);
}

inline bool
InOrderCPU::isThreadActive(ThreadID tid)
{
  list<ThreadID>::iterator isActive =
      std::find(activeThreads.begin(), activeThreads.end(), tid);

    return (isActive != activeThreads.end());
}


void
InOrderCPU::activateThread(ThreadID tid)
{
    if (!isThreadActive(tid)) {
        DPRINTF(InOrderCPU,
                "Adding Thread %i to active threads list in CPU.\n", tid);
        activeThreads.push_back(tid);

        wakeCPU();
    }
}

void
InOrderCPU::deactivateThread(ThreadID tid)
{
    DPRINTF(InOrderCPU, "[tid:%i]: Calling deactivate thread.\n", tid);

    if (isThreadActive(tid)) {
        DPRINTF(InOrderCPU,"[tid:%i]: Removing from active threads list\n",
                tid);
        list<ThreadID>::iterator thread_it =
            std::find(activeThreads.begin(), activeThreads.end(), tid);

        removePipelineStalls(*thread_it);

        //@TODO: change stage status' to Idle?

        activeThreads.erase(thread_it);
    }
}

void
InOrderCPU::removePipelineStalls(ThreadID tid)
{
    DPRINTF(InOrderCPU,"[tid:%i]: Removing all pipeline stalls\n",
            tid);

    for (int stNum = 0; stNum < NumStages ; stNum++) {
        pipelineStage[stNum]->removeStalls(tid);
    }

}
bool
InOrderCPU::isThreadInCPU(ThreadID tid)
{
  list<ThreadID>::iterator isCurrent =
      std::find(currentThreads.begin(), currentThreads.end(), tid);

    return (isCurrent != currentThreads.end());
}

void
InOrderCPU::addToCurrentThreads(ThreadID tid)
{
    if (!isThreadInCPU(tid)) {
        DPRINTF(InOrderCPU, "Adding Thread %i to current threads list in CPU.\n",
                tid);
        currentThreads.push_back(tid);
    }
}

void
InOrderCPU::removeFromCurrentThreads(ThreadID tid)
{
    if (isThreadInCPU(tid)) {
        DPRINTF(InOrderCPU,
                "Adding Thread %i to current threads list in CPU.\n", tid);
        list<ThreadID>::iterator isCurrent =
            std::find(currentThreads.begin(), currentThreads.end(), tid);
        currentThreads.erase(isCurrent);
    }
}

bool
InOrderCPU::isThreadSuspended(ThreadID tid)
{
  list<ThreadID>::iterator isSuspended =
      std::find(suspendedThreads.begin(), suspendedThreads.end(), tid);

    return (isSuspended!= suspendedThreads.end());
}

void
InOrderCPU::enableVirtProcElement(unsigned vpe)
{
    DPRINTF(InOrderCPU, "[vpe:%i]: Scheduling  "
            "Enabling of concurrent virtual processor execution",
            vpe);

    scheduleCpuEvent(EnableVPEs, NoFault, 0/*tid*/, vpe);
}

void
InOrderCPU::enableVPEs(unsigned vpe)
{
    DPRINTF(InOrderCPU, "[vpe:%i]: Enabling Concurrent Execution "
            "virtual processors %i", vpe);

    list<ThreadID>::iterator thread_it = currentThreads.begin();

    while (thread_it != currentThreads.end()) {
        if (!isThreadSuspended(*thread_it)) {
            activateThread(*thread_it);
        }
        thread_it++;
    }
}

void
InOrderCPU::disableVirtProcElement(ThreadID tid, unsigned vpe)
{
    DPRINTF(InOrderCPU, "[vpe:%i]: Scheduling  "
            "Disabling of concurrent virtual processor execution",
            vpe);

    scheduleCpuEvent(DisableVPEs, NoFault, 0/*tid*/, vpe);
}

void
InOrderCPU::disableVPEs(ThreadID tid, unsigned vpe)
{
    DPRINTF(InOrderCPU, "[vpe:%i]: Disabling Concurrent Execution of "
            "virtual processors %i", vpe);

    unsigned base_vpe = TheISA::getVirtProcNum(tcBase(tid));

    list<ThreadID>::iterator thread_it = activeThreads.begin();

    vector<list<ThreadID>::iterator> removeList;

    while (thread_it != activeThreads.end()) {
        if (base_vpe != vpe) {
            removeList.push_back(thread_it);
        }
        thread_it++;
    }

    for (int i = 0; i < removeList.size(); i++) {
        activeThreads.erase(removeList[i]);
    }
}

void
InOrderCPU::enableMultiThreading(unsigned vpe)
{
    // Schedule event to take place at end of cycle
    DPRINTF(InOrderCPU, "[vpe:%i]: Scheduling Enable Multithreading on "
            "virtual processor %i", vpe);

    scheduleCpuEvent(EnableThreads, NoFault, 0/*tid*/, vpe);
}

void
InOrderCPU::enableThreads(unsigned vpe)
{
    DPRINTF(InOrderCPU, "[vpe:%i]: Enabling Multithreading on "
            "virtual processor %i", vpe);

    list<ThreadID>::iterator thread_it = currentThreads.begin();

    while (thread_it != currentThreads.end()) {
        if (TheISA::getVirtProcNum(tcBase(*thread_it)) == vpe) {
            if (!isThreadSuspended(*thread_it)) {
                activateThread(*thread_it);
            }
        }
        thread_it++;
    }
}
void
InOrderCPU::disableMultiThreading(ThreadID tid, unsigned vpe)
{
    // Schedule event to take place at end of cycle
   DPRINTF(InOrderCPU, "[tid:%i]: Scheduling Disable Multithreading on "
            "virtual processor %i", tid, vpe);

    scheduleCpuEvent(DisableThreads, NoFault, tid, vpe);
}

void
InOrderCPU::disableThreads(ThreadID tid, unsigned vpe)
{
    DPRINTF(InOrderCPU, "[tid:%i]: Disabling Multithreading on "
            "virtual processor %i", tid, vpe);

    list<ThreadID>::iterator thread_it = activeThreads.begin();

    vector<list<ThreadID>::iterator> removeList;

    while (thread_it != activeThreads.end()) {
        if (TheISA::getVirtProcNum(tcBase(*thread_it)) == vpe) {
            removeList.push_back(thread_it);
        }
        thread_it++;
    }

    for (int i = 0; i < removeList.size(); i++) {
        activeThreads.erase(removeList[i]);
    }
}

void
InOrderCPU::updateThreadPriority()
{
    if (activeThreads.size() > 1)
    {
        //DEFAULT TO ROUND ROBIN SCHEME
        //e.g. Move highest priority to end of thread list
        list<ThreadID>::iterator list_begin = activeThreads.begin();
        list<ThreadID>::iterator list_end   = activeThreads.end();

        unsigned high_thread = *list_begin;

        activeThreads.erase(list_begin);

        activeThreads.push_back(high_thread);
    }
}

inline void
InOrderCPU::tickThreadStats()
{
    /** Keep track of cycles that each thread is active */
    list<ThreadID>::iterator thread_it = activeThreads.begin();
    while (thread_it != activeThreads.end()) {
        threadCycles[*thread_it]++;
        thread_it++;
    }

    // Keep track of cycles where SMT is active
    if (activeThreads.size() > 1) {
        smtCycles++;
    }
}

void
InOrderCPU::activateContext(ThreadID tid, int delay)
{
    DPRINTF(InOrderCPU,"[tid:%i]: Activating ...\n", tid);

    scheduleCpuEvent(ActivateThread, NoFault, tid, 0/*vpe*/, delay);

    // Be sure to signal that there's some activity so the CPU doesn't
    // deschedule itself.
    activityRec.activity();

    _status = Running;
}


void
InOrderCPU::suspendContext(ThreadID tid, int delay)
{
    scheduleCpuEvent(SuspendThread, NoFault, tid, 0/*vpe*/, delay);
    //_status = Idle;
}

void
InOrderCPU::suspendThread(ThreadID tid)
{
    DPRINTF(InOrderCPU,"[tid: %i]: Suspended ...\n", tid);
    deactivateThread(tid);
}

void
InOrderCPU::deallocateContext(ThreadID tid, int delay)
{
    scheduleCpuEvent(DeallocateThread, NoFault, tid, 0/*vpe*/, delay);
}

void
InOrderCPU::deallocateThread(ThreadID tid)
{
    DPRINTF(InOrderCPU,"[tid:%i]: Deallocating ...", tid);

    removeFromCurrentThreads(tid);

    deactivateThread(tid);

    squashThreadInPipeline(tid);
}

void
InOrderCPU::squashThreadInPipeline(ThreadID tid)
{
    //Squash all instructions in each stage
    for (int stNum=NumStages - 1; stNum >= 0 ; stNum--) {
        pipelineStage[stNum]->squash(0 /*seq_num*/, tid);
    }
}

void
InOrderCPU::haltContext(ThreadID tid, int delay)
{
    DPRINTF(InOrderCPU, "[tid:%i]: Halt context called.\n", tid);

    // Halt is same thing as deallocate for now
    // @TODO: Differentiate between halt & deallocate in the CPU
    // model
    deallocateContext(tid, delay);
}

void
InOrderCPU::insertThread(ThreadID tid)
{
    panic("Unimplemented Function\n.");
}

void
InOrderCPU::removeThread(ThreadID tid)
{
    DPRINTF(InOrderCPU, "Removing Thread %i from CPU.\n", tid);

    /** Broadcast to CPU resources*/
}

PipelineStage*
InOrderCPU::getPipeStage(int stage_num)
{
    return pipelineStage[stage_num];
}


void
InOrderCPU::activateWhenReady(ThreadID tid)
{
    panic("Unimplemented Function\n.");
}


uint64_t
InOrderCPU::readPC(ThreadID tid)
{
    return PC[tid];
}


void
InOrderCPU::setPC(Addr new_PC, ThreadID tid)
{
    PC[tid] = new_PC;
}


uint64_t
InOrderCPU::readNextPC(ThreadID tid)
{
    return nextPC[tid];
}


void
InOrderCPU::setNextPC(uint64_t new_NPC, ThreadID tid)
{
    nextPC[tid] = new_NPC;
}


uint64_t
InOrderCPU::readNextNPC(ThreadID tid)
{
    return nextNPC[tid];
}


void
InOrderCPU::setNextNPC(uint64_t new_NNPC, ThreadID tid)
{
    nextNPC[tid] = new_NNPC;
}

uint64_t
InOrderCPU::readIntReg(int reg_idx, ThreadID tid)
{
    return intRegFile[tid].readReg(reg_idx);
}

FloatReg
InOrderCPU::readFloatReg(int reg_idx, ThreadID tid, int width)
{

    return floatRegFile[tid].readReg(reg_idx, width);
}

FloatRegBits
InOrderCPU::readFloatRegBits(int reg_idx, ThreadID tid, int width)
{;
    return floatRegFile[tid].readRegBits(reg_idx, width);
}

void
InOrderCPU::setIntReg(int reg_idx, uint64_t val, ThreadID tid)
{
    intRegFile[tid].setReg(reg_idx, val);
}


void
InOrderCPU::setFloatReg(int reg_idx, FloatReg val, ThreadID tid, int width)
{
    floatRegFile[tid].setReg(reg_idx, val, width);
}


void
InOrderCPU::setFloatRegBits(int reg_idx, FloatRegBits val, ThreadID tid,
                            int width)
{
    floatRegFile[tid].setRegBits(reg_idx, val, width);
}

uint64_t
InOrderCPU::readRegOtherThread(unsigned reg_idx, ThreadID tid)
{
    // If Default value is set, then retrieve target thread
    if (tid == InvalidThreadID) {
        tid = TheISA::getTargetThread(tcBase(tid));
    }

    if (reg_idx < FP_Base_DepTag) {                   // Integer Register File
        return readIntReg(reg_idx, tid);
    } else if (reg_idx < Ctrl_Base_DepTag) {          // Float Register File
        reg_idx -= FP_Base_DepTag;
        return readFloatRegBits(reg_idx, tid);
    } else {
        reg_idx -= Ctrl_Base_DepTag;
        return readMiscReg(reg_idx, tid);  // Misc. Register File
    }
}
void
InOrderCPU::setRegOtherThread(unsigned reg_idx, const MiscReg &val,
                              ThreadID tid)
{
    // If Default value is set, then retrieve target thread
    if (tid == InvalidThreadID) {
        tid = TheISA::getTargetThread(tcBase(tid));
    }

    if (reg_idx < FP_Base_DepTag) {            // Integer Register File
        setIntReg(reg_idx, val, tid);
    } else if (reg_idx < Ctrl_Base_DepTag) {   // Float Register File
        reg_idx -= FP_Base_DepTag;
        setFloatRegBits(reg_idx, val, tid);
    } else {
        reg_idx -= Ctrl_Base_DepTag;
        setMiscReg(reg_idx, val, tid); // Misc. Register File
    }
}

MiscReg
InOrderCPU::readMiscRegNoEffect(int misc_reg, ThreadID tid)
{
    return isa[tid].readMiscRegNoEffect(misc_reg);
}

MiscReg
InOrderCPU::readMiscReg(int misc_reg, ThreadID tid)
{
    return isa[tid].readMiscReg(misc_reg, tcBase(tid));
}

void
InOrderCPU::setMiscRegNoEffect(int misc_reg, const MiscReg &val, ThreadID tid)
{
    isa[tid].setMiscRegNoEffect(misc_reg, val);
}

void
InOrderCPU::setMiscReg(int misc_reg, const MiscReg &val, ThreadID tid)
{
    isa[tid].setMiscReg(misc_reg, val, tcBase(tid));
}


InOrderCPU::ListIt
InOrderCPU::addInst(DynInstPtr &inst)
{
    ThreadID tid = inst->readTid();

    instList[tid].push_back(inst);

    return --(instList[tid].end());
}

void
InOrderCPU::instDone(DynInstPtr inst, ThreadID tid)
{
    // Set the CPU's PCs - This contributes to the precise state of the CPU which can be used
    // when restoring a thread to the CPU after a fork or after an exception
    // @TODO: Set-Up Grad-Info/Committed-Info to let ThreadState know if it's a branch or not
    setPC(inst->readPC(), tid);
    setNextPC(inst->readNextPC(), tid);
    setNextNPC(inst->readNextNPC(), tid);

    // Finalize Trace Data For Instruction
    if (inst->traceData) {
        //inst->traceData->setCycle(curTick);
        inst->traceData->setFetchSeq(inst->seqNum);
        //inst->traceData->setCPSeq(cpu->tcBase(tid)->numInst);
        inst->traceData->dump();
        delete inst->traceData;
        inst->traceData = NULL;
    }

    // Set Last Graduated Instruction In Thread State
    //thread[tid]->lastGradInst = inst;

    // Increment thread-state's instruction count
    thread[tid]->numInst++;

    // Increment thread-state's instruction stats
    thread[tid]->numInsts++;

    // Count committed insts per thread stats
    committedInsts[tid]++;

    // Count total insts committed stat
    totalCommittedInsts++;

    // Count SMT-committed insts per thread stat
    if (numActiveThreads() > 1) {
        smtCommittedInsts[tid]++;
    }

    // Check for instruction-count-based events.
    comInstEventQueue[tid]->serviceEvents(thread[tid]->numInst);

    // Broadcast to other resources an instruction
    // has been completed
    resPool->scheduleEvent((CPUEventType)ResourcePool::InstGraduated, inst, tid);

    // Finally, remove instruction from CPU
    removeInst(inst);
}

void
InOrderCPU::addToRemoveList(DynInstPtr &inst)
{
    removeInstsThisCycle = true;

    removeList.push(inst->getInstListIt());
}

void
InOrderCPU::removeInst(DynInstPtr &inst)
{
    DPRINTF(InOrderCPU, "Removing graduated instruction [tid:%i] PC %#x "
            "[sn:%lli]\n",
            inst->threadNumber, inst->readPC(), inst->seqNum);

    removeInstsThisCycle = true;

    // Remove the instruction.
    removeList.push(inst->getInstListIt());
}

void
InOrderCPU::removeInstsUntil(const InstSeqNum &seq_num, ThreadID tid)
{
    //assert(!instList[tid].empty());

    removeInstsThisCycle = true;

    ListIt inst_iter = instList[tid].end();

    inst_iter--;

    DPRINTF(InOrderCPU, "Deleting instructions from CPU instruction "
            "list that are from [tid:%i] and above [sn:%lli] (end=%lli).\n",
            tid, seq_num, (*inst_iter)->seqNum);

    while ((*inst_iter)->seqNum > seq_num) {

        bool break_loop = (inst_iter == instList[tid].begin());

        squashInstIt(inst_iter, tid);

        inst_iter--;

        if (break_loop)
            break;
    }
}


inline void
InOrderCPU::squashInstIt(const ListIt &instIt, ThreadID tid)
{
    if ((*instIt)->threadNumber == tid) {
        DPRINTF(InOrderCPU, "Squashing instruction, "
                "[tid:%i] [sn:%lli] PC %#x\n",
                (*instIt)->threadNumber,
                (*instIt)->seqNum,
                (*instIt)->readPC());

        (*instIt)->setSquashed();

        removeList.push(instIt);
    }
}


void
InOrderCPU::cleanUpRemovedInsts()
{
    while (!removeList.empty()) {
        DPRINTF(InOrderCPU, "Removing instruction, "
                "[tid:%i] [sn:%lli] PC %#x\n",
                (*removeList.front())->threadNumber,
                (*removeList.front())->seqNum,
                (*removeList.front())->readPC());

        DynInstPtr inst = *removeList.front();
        ThreadID tid = inst->threadNumber;

        // Make Sure Resource Schedule Is Emptied Out
        ThePipeline::ResSchedule *inst_sched = &inst->resSched;
        while (!inst_sched->empty()) {
            ThePipeline::ScheduleEntry* sch_entry = inst_sched->top();
            inst_sched->pop();
            delete sch_entry;
        }

        // Remove From Register Dependency Map, If Necessary
        archRegDepMap[(*removeList.front())->threadNumber].
            remove((*removeList.front()));


        // Clear if Non-Speculative
        if (inst->staticInst &&
              inst->seqNum == nonSpecSeqNum[tid] &&
                nonSpecInstActive[tid] == true) {
            nonSpecInstActive[tid] = false;
        }

        instList[tid].erase(removeList.front());

        removeList.pop();

        DPRINTF(RefCount, "pop from remove list: [sn:%i]: Refcount = %i.\n",
                inst->seqNum,
                0/*inst->curCount()*/);

    }

    removeInstsThisCycle = false;
}

void
InOrderCPU::cleanUpRemovedReqs()
{
    while (!reqRemoveList.empty()) {
        ResourceRequest *res_req = reqRemoveList.front();

        DPRINTF(RefCount, "[tid:%i]: Removing Request, "
                "[sn:%lli] [slot:%i] [stage_num:%i] [res:%s] [refcount:%i].\n",
                res_req->inst->threadNumber,
                res_req->inst->seqNum,
                res_req->getSlot(),
                res_req->getStageNum(),
                res_req->res->name(),
                0/*res_req->inst->curCount()*/);

        reqRemoveList.pop();

        delete res_req;

        DPRINTF(RefCount, "after remove request: [sn:%i]: Refcount = %i.\n",
                res_req->inst->seqNum,
                0/*res_req->inst->curCount()*/);
    }
}

void
InOrderCPU::cleanUpRemovedEvents()
{
    while (!cpuEventRemoveList.empty()) {
        Event *cpu_event = cpuEventRemoveList.front();
        cpuEventRemoveList.pop();
        delete cpu_event;
    }
}


void
InOrderCPU::dumpInsts()
{
    int num = 0;

    ListIt inst_list_it = instList[0].begin();

    cprintf("Dumping Instruction List\n");

    while (inst_list_it != instList[0].end()) {
        cprintf("Instruction:%i\nPC:%#x\n[tid:%i]\n[sn:%lli]\nIssued:%i\n"
                "Squashed:%i\n\n",
                num, (*inst_list_it)->readPC(), (*inst_list_it)->threadNumber,
                (*inst_list_it)->seqNum, (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());
        inst_list_it++;
        ++num;
    }
}

void
InOrderCPU::wakeCPU()
{
    if (/*activityRec.active() || */tickEvent.scheduled()) {
        DPRINTF(Activity, "CPU already running.\n");
        return;
    }

    DPRINTF(Activity, "Waking up CPU\n");

    //@todo: figure out how to count idleCycles correctly
    //idleCycles += (curTick - 1) - lastRunningCycle;

    mainEventQueue.schedule(&tickEvent, curTick);
}

void
InOrderCPU::syscall(int64_t callnum, ThreadID tid)
{
    DPRINTF(InOrderCPU, "[tid:%i] Executing syscall().\n\n", tid);

    DPRINTF(Activity,"Activity: syscall() called.\n");

    // Temporarily increase this by one to account for the syscall
    // instruction.
    ++(this->thread[tid]->funcExeInst);

    // Execute the actual syscall.
    this->thread[tid]->syscall(callnum);

    // Decrease funcExeInst by one as the normal commit will handle
    // incrementing it.
    --(this->thread[tid]->funcExeInst);

    // Clear Non-Speculative Block Variable
    nonSpecInstActive[tid] = false;
}

void
InOrderCPU::prefetch(DynInstPtr inst)
{
    Resource *mem_res = resPool->getResource(dataPortIdx);
    return mem_res->prefetch(inst);
}

void
InOrderCPU::writeHint(DynInstPtr inst)
{
    Resource *mem_res = resPool->getResource(dataPortIdx);
    return mem_res->writeHint(inst);
}


TheISA::TLB*
InOrderCPU::getITBPtr()
{
    CacheUnit *itb_res =
        dynamic_cast<CacheUnit*>(resPool->getResource(fetchPortIdx));
    return itb_res->tlb();
}


TheISA::TLB*
InOrderCPU::getDTBPtr()
{
    CacheUnit *dtb_res =
        dynamic_cast<CacheUnit*>(resPool->getResource(dataPortIdx));
    return dtb_res->tlb();
}

template <class T>
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, T &data, unsigned flags)
{
    //@TODO: Generalize name "CacheUnit" to "MemUnit" just in case
    //       you want to run w/out caches?
    CacheUnit *cache_res = dynamic_cast<CacheUnit*>(resPool->getResource(dataPortIdx));

    return cache_res->read(inst, addr, data, flags);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, Twin32_t &data, unsigned flags);

template
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, Twin64_t &data, unsigned flags);

template
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, uint64_t &data, unsigned flags);

template
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, uint32_t &data, unsigned flags);

template
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, uint16_t &data, unsigned flags);

template
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, uint8_t &data, unsigned flags);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, double &data, unsigned flags)
{
    return read(inst, addr, *(uint64_t*)&data, flags);
}

template<>
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, float &data, unsigned flags)
{
    return read(inst, addr, *(uint32_t*)&data, flags);
}


template<>
Fault
InOrderCPU::read(DynInstPtr inst, Addr addr, int32_t &data, unsigned flags)
{
    return read(inst, addr, (uint32_t&)data, flags);
}

template <class T>
Fault
InOrderCPU::write(DynInstPtr inst, T data, Addr addr, unsigned flags,
                  uint64_t *write_res)
{
    //@TODO: Generalize name "CacheUnit" to "MemUnit" just in case
    //       you want to run w/out caches?
    CacheUnit *cache_res =
        dynamic_cast<CacheUnit*>(resPool->getResource(dataPortIdx));
    return cache_res->write(inst, data, addr, flags, write_res);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template
Fault
InOrderCPU::write(DynInstPtr inst, Twin32_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
InOrderCPU::write(DynInstPtr inst, Twin64_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
InOrderCPU::write(DynInstPtr inst, uint64_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
InOrderCPU::write(DynInstPtr inst, uint32_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
InOrderCPU::write(DynInstPtr inst, uint16_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
InOrderCPU::write(DynInstPtr inst, uint8_t data, Addr addr,
                       unsigned flags, uint64_t *res);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
InOrderCPU::write(DynInstPtr inst, double data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(inst, *(uint64_t*)&data, addr, flags, res);
}

template<>
Fault
InOrderCPU::write(DynInstPtr inst, float data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(inst, *(uint32_t*)&data, addr, flags, res);
}


template<>
Fault
InOrderCPU::write(DynInstPtr inst, int32_t data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(inst, (uint32_t)data, addr, flags, res);
}
