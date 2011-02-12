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
#include "config/the_isa.hh"
#include "cpu/activity.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/first_stage.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource_pool.hh"
#include "cpu/inorder/resources/resource_list.hh"
#include "cpu/inorder/thread_context.hh"
#include "cpu/inorder/thread_state.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "mem/translating_port.hh"
#include "params/InOrderCPU.hh"
#include "sim/process.hh"
#include "sim/stat_control.hh"

#if FULL_SYSTEM
#include "cpu/quiesce_event.hh"
#include "sim/system.hh"
#endif

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/osfpal.hh"
#endif

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
                               Fault fault, ThreadID _tid, DynInstPtr inst,
                               unsigned event_pri_offset)
    : Event(Event::Priority((unsigned int)CPU_Tick_Pri + event_pri_offset)),
      cpu(_cpu)
{
    setEvent(e_type, fault, _tid, inst);
}


std::string InOrderCPU::eventNames[NumCPUEvents] =
{
    "ActivateThread",
    "ActivateNextReadyThread",
    "DeactivateThread",
    "HaltThread",
    "SuspendThread",
    "Trap",
    "InstGraduated",
    "SquashFromMemStall",
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

      case ActivateNextReadyThread:
        cpu->activateNextReadyThread();
        break;

      case DeactivateThread:
        cpu->deactivateThread(tid);
        break;

      case HaltThread:
        cpu->haltThread(tid);
        break;

      case SuspendThread: 
        cpu->suspendThread(tid);
        break;

      case SquashFromMemStall:
        cpu->squashDueToMemStall(inst->squashingStage, inst->seqNum, tid);
        break;

      case Trap:
        cpu->trapCPU(fault, tid, inst);
        break;

      default:
        fatal("Unrecognized Event Type %s", eventNames[cpuEventType]);    
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
    assert(!scheduled() || squashed());
    cpu->reschedule(this, cpu->nextCycle(curTick() + cpu->ticks(delay)), true);
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
      stageWidth(params->stageWidth),
      timeBuffer(2 , 2),
      removeInstsThisCycle(false),
      activityRec(params->name, NumStages, 10, params->activity),
#if FULL_SYSTEM
      system(params->system),
      physmem(system->physmem),
#endif // FULL_SYSTEM
#ifdef DEBUG
      cpuEventNum(0),
      resReqCount(0),
#endif // DEBUG
      switchCount(0),
      deferRegistration(false/*params->deferRegistration*/),
      stageTracing(params->stageTracing),
      instsPerSwitch(0)
{    
    ThreadID active_threads;
    cpu_params = params;

    resPool = new ResourcePool(this, params);

    // Resize for Multithreading CPUs
    thread.resize(numThreads);

#if FULL_SYSTEM
    active_threads = 1;
#else
    active_threads = params->workload.size();

    if (active_threads > MaxThreads) {
        panic("Workload Size too large. Increase the 'MaxThreads'"
              "in your InOrder implementation or "
              "edit your workload size.");
    }

    
    if (active_threads > 1) {
        threadModel = (InOrderCPU::ThreadModel) params->threadModel;

        if (threadModel == SMT) {
            DPRINTF(InOrderCPU, "Setting Thread Model to SMT.\n");            
        } else if (threadModel == SwitchOnCacheMiss) {
            DPRINTF(InOrderCPU, "Setting Thread Model to "
                    "Switch On Cache Miss\n");
        }
        
    } else {
        threadModel = Single;
    }
     
        
    
#endif

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
#if FULL_SYSTEM
        // SMT is not supported in FS mode yet.
        assert(numThreads == 1);
        thread[tid] = new Thread(this, 0);
#else
        if (tid < (ThreadID)params->workload.size()) {
            DPRINTF(InOrderCPU, "Workload[%i] process is %#x\n",
                    tid, params->workload[tid]->prog_fname);
            thread[tid] =
                new Thread(this, tid, params->workload[tid]);
        } else {
            //Allocate Empty thread so M5 can use later
            //when scheduling threads to CPU
            Process* dummy_proc = params->workload[0];
            thread[tid] = new Thread(this, tid, dummy_proc);
        }
        
        // Eventually set this with parameters...
        asid[tid] = tid;
#endif

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

        memset(intRegs[tid], 0, sizeof(intRegs[tid]));
        memset(floatRegs.i[tid], 0, sizeof(floatRegs.i[tid]));
        isa[tid].clear();

        isa[tid].expandForMultithreading(numThreads, 1/*numVirtProcs*/);

        // Define dummy instructions and resource requests to be used.
        dummyInst[tid] = new InOrderDynInst(this, 
                                            thread[tid], 
                                            0, 
                                            tid, 
                                            asid[tid]);

        dummyReq[tid] = new ResourceRequest(resPool->getResource(0), 
                                            dummyInst[tid], 
                                            0, 
                                            0, 
                                            0, 
                                            0);        
    }

    dummyReqInst = new InOrderDynInst(this, NULL, 0, 0, 0);
    dummyReqInst->setSquashed();
    dummyReqInst->resetInstCount();

    dummyBufferInst = new InOrderDynInst(this, NULL, 0, 0, 0);
    dummyBufferInst->setSquashed();
    dummyBufferInst->resetInstCount();

    endOfSkedIt = skedCache.end();
    
    lastRunningCycle = curTick();

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

InOrderCPU::~InOrderCPU()
{
    delete resPool;
}

std::map<InOrderCPU::SkedID, ThePipeline::RSkedPtr> InOrderCPU::skedCache;

void
InOrderCPU::regStats()
{
    /* Register the Resource Pool's stats here.*/
    resPool->regStats();

    /* Register for each Pipeline Stage */
    for (int stage_num=0; stage_num < ThePipeline::NumStages; stage_num++) {
        pipelineStage[stage_num]->regStats();
    }

    /* Register any of the InOrderCPU's stats here.*/
    instsPerCtxtSwitch
        .name(name() + ".instsPerContextSwitch")
        .desc("Instructions Committed Per Context Switch")
        .prereq(instsPerCtxtSwitch);
    
    numCtxtSwitches
        .name(name() + ".contextSwitches")
        .desc("Number of context switches");

    comLoads
        .name(name() + ".comLoads")
        .desc("Number of Load instructions committed");

    comStores
        .name(name() + ".comStores")
        .desc("Number of Store instructions committed");

    comBranches
        .name(name() + ".comBranches")
        .desc("Number of Branches instructions committed");

    comNops
        .name(name() + ".comNops")
        .desc("Number of Nop instructions committed");

    comNonSpec
        .name(name() + ".comNonSpec")
        .desc("Number of Non-Speculative instructions committed");

    comInts
        .name(name() + ".comInts")
        .desc("Number of Integer instructions committed");

    comFloats
        .name(name() + ".comFloats")
        .desc("Number of Floating Point instructions committed");
            
    timesIdled
        .name(name() + ".timesIdled")
        .desc("Number of times that the entire CPU went into an idle state and"
              " unscheduled itself")
        .prereq(timesIdled);

    idleCycles
        .name(name() + ".idleCycles")
        .desc("Number of cycles cpu's stages were not processed");

    runCycles
        .name(name() + ".runCycles")
        .desc("Number of cycles cpu stages are processed.");

    activity
        .name(name() + ".activity")
        .desc("Percentage of cycles cpu is active")
        .precision(6);
    activity = (runCycles / numCycles) * 100;

    threadCycles
        .init(numThreads)
        .name(name() + ".threadCycles")
        .desc("Total Number of Cycles A Thread Was Active in CPU (Per-Thread)");

    smtCycles
        .name(name() + ".smtCycles")
        .desc("Total number of cycles that the CPU was in SMT-mode");

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
    cpi = numCycles / committedInsts;

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
    ipc =  committedInsts / numCycles;

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

    bool pipes_idle = true;
    
    //Tick each of the stages
    for (int stNum=NumStages - 1; stNum >= 0 ; stNum--) {
        pipelineStage[stNum]->tick();

        pipes_idle = pipes_idle && pipelineStage[stNum]->idle;
    }

    if (pipes_idle)
        idleCycles++;
    else
        runCycles++;
    
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
            lastRunningCycle = curTick();
        } else if (!activityRec.active()) {
            DPRINTF(InOrderCPU, "sleeping CPU.\n");
            lastRunningCycle = curTick();
            timesIdled++;
        } else {
            //Tick next_tick = curTick() + cycles(1);
            //tickEvent.schedule(next_tick);
            schedule(&tickEvent, nextCycle(curTick() + 1));
            DPRINTF(InOrderCPU, "Scheduled CPU for next tick @ %i.\n", 
                    nextCycle(curTick() + 1));
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
InOrderCPU::reset()
{
    for (int i = 0; i < numThreads; i++) {
        isa[i].reset(coreType, numThreads,
                     1/*numVirtProcs*/, dynamic_cast<BaseCPU*>(this));
    }
}

Port*
InOrderCPU::getPort(const std::string &if_name, int idx)
{
    return resPool->getPort(if_name, idx);
}

#if FULL_SYSTEM
Fault
InOrderCPU::hwrei(ThreadID tid)
{
    panic("hwrei: Unimplemented");
    
    return NoFault;
}


bool
InOrderCPU::simPalCheck(int palFunc, ThreadID tid)
{
    panic("simPalCheck: Unimplemented");

    return true;
}


Fault
InOrderCPU::getInterrupts()
{
    // Check if there are any outstanding interrupts
    return interrupts->getInterrupt(threadContexts[0]);
}


void
InOrderCPU::processInterrupts(Fault interrupt)
{
    // Check for interrupts here.  For now can copy the code that
    // exists within isa_fullsys_traits.hh.  Also assume that thread 0
    // is the one that handles the interrupts.
    // @todo: Possibly consolidate the interrupt checking code.
    // @todo: Allow other threads to handle interrupts.

    assert(interrupt != NoFault);
    interrupts->updateIntrInfo(threadContexts[0]);

    DPRINTF(InOrderCPU, "Interrupt %s being handled\n", interrupt->name());

    // Note: Context ID ok here? Impl. of FS mode needs to revisit this
    trap(interrupt, threadContexts[0]->contextId(), dummyBufferInst);
}


void
InOrderCPU::updateMemPorts()
{
    // Update all ThreadContext's memory ports (Functional/Virtual
    // Ports)
    ThreadID size = thread.size();
    for (ThreadID i = 0; i < size; ++i)
        thread[i]->connectMemPorts(thread[i]->getTC());
}
#endif

void
InOrderCPU::trap(Fault fault, ThreadID tid, DynInstPtr inst, int delay)
{
    //@ Squash Pipeline during TRAP
    scheduleCpuEvent(Trap, fault, tid, inst, delay);
}

void
InOrderCPU::trapCPU(Fault fault, ThreadID tid, DynInstPtr inst)
{
    fault->invoke(tcBase(tid), inst->staticInst);
}

void 
InOrderCPU::squashFromMemStall(DynInstPtr inst, ThreadID tid, int delay)
{
    scheduleCpuEvent(SquashFromMemStall, NoFault, tid, inst, delay);
}


void
InOrderCPU::squashDueToMemStall(int stage_num, InstSeqNum seq_num,
                                ThreadID tid)
{
    DPRINTF(InOrderCPU, "Squashing Pipeline Stages Due to Memory Stall...\n");
        
    // Squash all instructions in each stage including 
    // instruction that caused the squash (seq_num - 1)
    // NOTE: The stage bandwidth needs to be cleared so thats why
    //       the stalling instruction is squashed as well. The stalled
    //       instruction is previously placed in another intermediate buffer
    //       while it's stall is being handled.
    InstSeqNum squash_seq_num = seq_num - 1;
    
    for (int stNum=stage_num; stNum >= 0 ; stNum--) {
        pipelineStage[stNum]->squashDueToMemStall(squash_seq_num, tid);
    }
}

void
InOrderCPU::scheduleCpuEvent(CPUEventType c_event, Fault fault,
                             ThreadID tid, DynInstPtr inst, 
                             unsigned delay, unsigned event_pri_offset)
{
    CPUEvent *cpu_event = new CPUEvent(this, c_event, fault, tid, inst,
                                       event_pri_offset);

    Tick sked_tick = nextCycle(curTick() + ticks(delay));
    if (delay >= 0) {
        DPRINTF(InOrderCPU, "Scheduling CPU Event (%s) for cycle %i, [tid:%i].\n",
                eventNames[c_event], curTick() + delay, tid);
        schedule(cpu_event, sked_tick);
    } else {
        cpu_event->process();
        cpuEventRemoveList.push(cpu_event);
    }

    // Broadcast event to the Resource Pool
    // Need to reset tid just in case this is a dummy instruction
    inst->setTid(tid);        
    resPool->scheduleEvent(c_event, inst, 0, 0, tid);
}

bool
InOrderCPU::isThreadActive(ThreadID tid)
{
  list<ThreadID>::iterator isActive =
      std::find(activeThreads.begin(), activeThreads.end(), tid);

    return (isActive != activeThreads.end());
}

bool
InOrderCPU::isThreadReady(ThreadID tid)
{
  list<ThreadID>::iterator isReady =
      std::find(readyThreads.begin(), readyThreads.end(), tid);

    return (isReady != readyThreads.end());
}

bool
InOrderCPU::isThreadSuspended(ThreadID tid)
{
  list<ThreadID>::iterator isSuspended =
      std::find(suspendedThreads.begin(), suspendedThreads.end(), tid);

    return (isSuspended != suspendedThreads.end());
}

void
InOrderCPU::activateNextReadyThread()
{
    if (readyThreads.size() >= 1) {          
        ThreadID ready_tid = readyThreads.front();
        
        // Activate in Pipeline
        activateThread(ready_tid);                        
        
        // Activate in Resource Pool
        resPool->activateAll(ready_tid);
        
        list<ThreadID>::iterator ready_it =
            std::find(readyThreads.begin(), readyThreads.end(), ready_tid);
        readyThreads.erase(ready_it);                        
    } else {
        DPRINTF(InOrderCPU,
                "Attempting to activate new thread, but No Ready Threads to"
                "activate.\n");
        DPRINTF(InOrderCPU,
                "Unable to switch to next active thread.\n");
    }        
}

void
InOrderCPU::activateThread(ThreadID tid)
{
    if (isThreadSuspended(tid)) {
        DPRINTF(InOrderCPU,
                "Removing [tid:%i] from suspended threads list.\n", tid);

        list<ThreadID>::iterator susp_it =
            std::find(suspendedThreads.begin(), suspendedThreads.end(), 
                      tid);
        suspendedThreads.erase(susp_it);                        
    }

    if (threadModel == SwitchOnCacheMiss &&
        numActiveThreads() == 1) {
        DPRINTF(InOrderCPU,
                "Ignoring activation of [tid:%i], since [tid:%i] is "
                "already running.\n", tid, activeThreadId());
        
        DPRINTF(InOrderCPU,"Placing [tid:%i] on ready threads list\n", 
                tid);        

        readyThreads.push_back(tid);
        
    } else if (!isThreadActive(tid)) {                
        DPRINTF(InOrderCPU,
                "Adding [tid:%i] to active threads list.\n", tid);
        activeThreads.push_back(tid);
        
        activateThreadInPipeline(tid);

        thread[tid]->lastActivate = curTick();            

        tcBase(tid)->setStatus(ThreadContext::Active);    

        wakeCPU();

        numCtxtSwitches++;        
    }
}

void
InOrderCPU::activateThreadInPipeline(ThreadID tid)
{
    for (int stNum=0; stNum < NumStages; stNum++) {
        pipelineStage[stNum]->activateThread(tid);
    }    
}

void
InOrderCPU::deactivateContext(ThreadID tid, int delay)
{
    DPRINTF(InOrderCPU,"[tid:%i]: Deactivating ...\n", tid);

    scheduleCpuEvent(DeactivateThread, NoFault, tid, dummyInst[tid], delay);

    // Be sure to signal that there's some activity so the CPU doesn't
    // deschedule itself.
    activityRec.activity();

    _status = Running;
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

        activeThreads.erase(thread_it);

        // Ideally, this should be triggered from the
        // suspendContext/Thread functions
        tcBase(tid)->setStatus(ThreadContext::Suspended);    
    }

    assert(!isThreadActive(tid));    
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

    
    scheduleCpuEvent(ActivateThread, NoFault, tid, dummyInst[tid], delay);

    // Be sure to signal that there's some activity so the CPU doesn't
    // deschedule itself.
    activityRec.activity();

    _status = Running;
}

void
InOrderCPU::activateNextReadyContext(int delay)
{
    DPRINTF(InOrderCPU,"Activating next ready thread\n");

    // NOTE: Add 5 to the event priority so that we always activate
    // threads after we've finished deactivating, squashing,etc.
    // other threads
    scheduleCpuEvent(ActivateNextReadyThread, NoFault, 0/*tid*/, dummyInst[0], 
                     delay, 5);

    // Be sure to signal that there's some activity so the CPU doesn't
    // deschedule itself.
    activityRec.activity();

    _status = Running;
}

void
InOrderCPU::haltContext(ThreadID tid, int delay)
{
    DPRINTF(InOrderCPU, "[tid:%i]: Calling Halt Context...\n", tid);

    scheduleCpuEvent(HaltThread, NoFault, tid, dummyInst[tid], delay);

    activityRec.activity();
}

void
InOrderCPU::haltThread(ThreadID tid)
{
    DPRINTF(InOrderCPU, "[tid:%i]: Placing on Halted Threads List...\n", tid);
    deactivateThread(tid);
    squashThreadInPipeline(tid);   
    haltedThreads.push_back(tid);    

    tcBase(tid)->setStatus(ThreadContext::Halted);    

    if (threadModel == SwitchOnCacheMiss) {        
        activateNextReadyContext();    
    }
}

void
InOrderCPU::suspendContext(ThreadID tid, int delay)
{
    scheduleCpuEvent(SuspendThread, NoFault, tid, dummyInst[tid], delay);
}

void
InOrderCPU::suspendThread(ThreadID tid)
{
    DPRINTF(InOrderCPU, "[tid:%i]: Placing on Suspended Threads List...\n",
            tid);
    deactivateThread(tid);
    suspendedThreads.push_back(tid);    
    thread[tid]->lastSuspend = curTick();    

    tcBase(tid)->setStatus(ThreadContext::Suspended);    
}

void
InOrderCPU::squashThreadInPipeline(ThreadID tid)
{
    //Squash all instructions in each stage
    for (int stNum=NumStages - 1; stNum >= 0 ; stNum--) {
        pipelineStage[stNum]->squash(0 /*seq_num*/, tid);
    }
}

PipelineStage*
InOrderCPU::getPipeStage(int stage_num)
{
    return pipelineStage[stage_num];
}

uint64_t
InOrderCPU::readIntReg(int reg_idx, ThreadID tid)
{
    return intRegs[tid][reg_idx];
}

FloatReg
InOrderCPU::readFloatReg(int reg_idx, ThreadID tid)
{
    return floatRegs.f[tid][reg_idx];
}

FloatRegBits
InOrderCPU::readFloatRegBits(int reg_idx, ThreadID tid)
{;
    return floatRegs.i[tid][reg_idx];
}

void
InOrderCPU::setIntReg(int reg_idx, uint64_t val, ThreadID tid)
{
    intRegs[tid][reg_idx] = val;
}


void
InOrderCPU::setFloatReg(int reg_idx, FloatReg val, ThreadID tid)
{
    floatRegs.f[tid][reg_idx] = val;
}


void
InOrderCPU::setFloatRegBits(int reg_idx, FloatRegBits val, ThreadID tid)
{
    floatRegs.i[tid][reg_idx] = val;
}

uint64_t
InOrderCPU::readRegOtherThread(unsigned reg_idx, ThreadID tid)
{
    // If Default value is set, then retrieve target thread
    if (tid == InvalidThreadID) {
        tid = TheISA::getTargetThread(tcBase(tid));
    }

    if (reg_idx < FP_Base_DepTag) {                   
        // Integer Register File
        return readIntReg(reg_idx, tid);
    } else if (reg_idx < Ctrl_Base_DepTag) {          
        // Float Register File
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
InOrderCPU::updateContextSwitchStats()
{
    // Set Average Stat Here, then reset to 0    
    instsPerCtxtSwitch = instsPerSwitch;
    instsPerSwitch = 0;
}

    
void
InOrderCPU::instDone(DynInstPtr inst, ThreadID tid)
{
    // Set the CPU's PCs - This contributes to the precise state of the CPU 
    // which can be used when restoring a thread to the CPU after after any
    // type of context switching activity (fork, exception, etc.)
    pcState(inst->pcState(), tid);

    if (inst->isControl()) {
        thread[tid]->lastGradIsBranch = true;
        thread[tid]->lastBranchPC = inst->pcState();
        TheISA::advancePC(thread[tid]->lastBranchPC, inst->staticInst);
    } else {
        thread[tid]->lastGradIsBranch = false;
    }
        

    // Finalize Trace Data For Instruction
    if (inst->traceData) {
        //inst->traceData->setCycle(curTick());
        inst->traceData->setFetchSeq(inst->seqNum);
        //inst->traceData->setCPSeq(cpu->tcBase(tid)->numInst);
        inst->traceData->dump();
        delete inst->traceData;
        inst->traceData = NULL;
    }

    // Increment active thread's instruction count
    instsPerSwitch++;
    
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

    // Instruction-Mix Stats
    if (inst->isLoad()) {
        comLoads++;
    } else if (inst->isStore()) {
        comStores++;
    } else if (inst->isControl()) {
        comBranches++;
    } else if (inst->isNop()) {
        comNops++;
    } else if (inst->isNonSpeculative()) {
        comNonSpec++;
    } else if (inst->isInteger()) {
        comInts++;
    } else if (inst->isFloating()) {
        comFloats++;
    }

    // Check for instruction-count-based events.
    comInstEventQueue[tid]->serviceEvents(thread[tid]->numInst);

    // Broadcast to other resources an instruction
    // has been completed
    resPool->scheduleEvent((CPUEventType)ResourcePool::InstGraduated, inst, 
                           0, 0, tid);

    // Finally, remove instruction from CPU
    removeInst(inst);
}

// currently unused function, but substitute repetitive code w/this function
// call
void
InOrderCPU::addToRemoveList(DynInstPtr &inst)
{
    removeInstsThisCycle = true;
    if (!inst->isRemoveList()) {            
        DPRINTF(InOrderCPU, "Pushing instruction [tid:%i] PC %s "
                "[sn:%lli] to remove list\n",
                inst->threadNumber, inst->pcState(), inst->seqNum);
        inst->setRemoveList();        
        removeList.push(inst->getInstListIt());
    }  else {
        DPRINTF(InOrderCPU, "Ignoring instruction removal for [tid:%i] PC %s "
                "[sn:%lli], already remove list\n",
                inst->threadNumber, inst->pcState(), inst->seqNum);
    }
    
}

void
InOrderCPU::removeInst(DynInstPtr &inst)
{
    DPRINTF(InOrderCPU, "Removing graduated instruction [tid:%i] PC %s "
            "[sn:%lli]\n",
            inst->threadNumber, inst->pcState(), inst->seqNum);

    removeInstsThisCycle = true;

    // Remove the instruction.
    if (!inst->isRemoveList()) {            
        DPRINTF(InOrderCPU, "Pushing instruction [tid:%i] PC %s "
                "[sn:%lli] to remove list\n",
                inst->threadNumber, inst->pcState(), inst->seqNum);
        inst->setRemoveList();        
        removeList.push(inst->getInstListIt());
    } else {
        DPRINTF(InOrderCPU, "Ignoring instruction removal for [tid:%i] PC %s "
                "[sn:%lli], already on remove list\n",
                inst->threadNumber, inst->pcState(), inst->seqNum);
    }

}

void
InOrderCPU::removeInstsUntil(const InstSeqNum &seq_num, ThreadID tid)
{
    //assert(!instList[tid].empty());

    removeInstsThisCycle = true;

    ListIt inst_iter = instList[tid].end();

    inst_iter--;

    DPRINTF(InOrderCPU, "Squashing instructions from CPU instruction "
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
                "[tid:%i] [sn:%lli] PC %s\n",
                (*instIt)->threadNumber,
                (*instIt)->seqNum,
                (*instIt)->pcState());

        (*instIt)->setSquashed();

        if (!(*instIt)->isRemoveList()) {            
            DPRINTF(InOrderCPU, "Pushing instruction [tid:%i] PC %s "
                    "[sn:%lli] to remove list\n",
                    (*instIt)->threadNumber, (*instIt)->pcState(),
                    (*instIt)->seqNum);
            (*instIt)->setRemoveList();        
            removeList.push(instIt);
        } else {
            DPRINTF(InOrderCPU, "Ignoring instruction removal for [tid:%i]"
                    " PC %s [sn:%lli], already on remove list\n",
                    (*instIt)->threadNumber, (*instIt)->pcState(),
                    (*instIt)->seqNum);
        }
    
    }
    
}


void
InOrderCPU::cleanUpRemovedInsts()
{
    while (!removeList.empty()) {
        DPRINTF(InOrderCPU, "Removing instruction, "
                "[tid:%i] [sn:%lli] PC %s\n",
                (*removeList.front())->threadNumber,
                (*removeList.front())->seqNum,
               (*removeList.front())->pcState());

        DynInstPtr inst = *removeList.front();
        ThreadID tid = inst->threadNumber;

        // Make Sure Resource Schedule Is Emptied Out
        ThePipeline::ResSchedule *inst_sched = &inst->resSched;
        while (!inst_sched->empty()) {
            ScheduleEntry* sch_entry = inst_sched->top();
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
    }

    removeInstsThisCycle = false;
}

void
InOrderCPU::cleanUpRemovedReqs()
{
    while (!reqRemoveList.empty()) {
        ResourceRequest *res_req = reqRemoveList.front();

        DPRINTF(RefCount, "[tid:%i] [sn:%lli]: Removing Request "
                "[stage_num:%i] [res:%s] [slot:%i] [completed:%i].\n",
                res_req->inst->threadNumber,
                res_req->inst->seqNum,
                res_req->getStageNum(),
                res_req->res->name(),
                (res_req->isCompleted()) ?
                res_req->getComplSlot() : res_req->getSlot(),
                res_req->isCompleted());

        reqRemoveList.pop();

        delete res_req;
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
        cprintf("Instruction:%i\nPC:%s\n[tid:%i]\n[sn:%lli]\nIssued:%i\n"
                "Squashed:%i\n\n",
                num, (*inst_list_it)->pcState(),
                (*inst_list_it)->threadNumber,
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

    Tick extra_cycles = tickToCycles((curTick() - 1) - lastRunningCycle);

    idleCycles += extra_cycles;    
    for (int stage_num = 0; stage_num < NumStages; stage_num++) {
        pipelineStage[stage_num]->idleCycles += extra_cycles;
    }    

    numCycles += extra_cycles;

    schedule(&tickEvent, nextCycle(curTick()));
}

#if FULL_SYSTEM

void
InOrderCPU::wakeup()
{
    if (thread[0]->status() != ThreadContext::Suspended)
        return;

    wakeCPU();

    DPRINTF(Quiesce, "Suspended Processor woken\n");
    threadContexts[0]->activate();
}
#endif

#if !FULL_SYSTEM
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
#endif

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

Fault
InOrderCPU::read(DynInstPtr inst, Addr addr,
                 uint8_t *data, unsigned size, unsigned flags)
{
    //@TODO: Generalize name "CacheUnit" to "MemUnit" just in case
    //       you want to run w/out caches?
    CacheUnit *cache_res = 
        dynamic_cast<CacheUnit*>(resPool->getResource(dataPortIdx));

    return cache_res->read(inst, addr, data, size, flags);
}

Fault
InOrderCPU::write(DynInstPtr inst, uint8_t *data, unsigned size,
                  Addr addr, unsigned flags, uint64_t *write_res)
{
    //@TODO: Generalize name "CacheUnit" to "MemUnit" just in case
    //       you want to run w/out caches?
    CacheUnit *cache_res =
        dynamic_cast<CacheUnit*>(resPool->getResource(dataPortIdx));
    return cache_res->write(inst, data, size, addr, flags, write_res);
}
