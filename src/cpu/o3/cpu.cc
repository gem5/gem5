/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 *          Korey Sewell
 */

#include "config/full_system.hh"
#include "config/use_checker.hh"

#if FULL_SYSTEM
#include "cpu/quiesce_event.hh"
#include "sim/system.hh"
#else
#include "sim/process.hh"
#endif

#include "cpu/activity.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "cpu/o3/isa_specific.hh"
#include "cpu/o3/cpu.hh"

#include "sim/core.hh"
#include "sim/stat_control.hh"

#if USE_CHECKER
#include "cpu/checker/cpu.hh"
#endif

using namespace std;
using namespace TheISA;

BaseO3CPU::BaseO3CPU(Params *params)
    : BaseCPU(params), cpu_id(0)
{
}

void
BaseO3CPU::regStats()
{
    BaseCPU::regStats();
}

template <class Impl>
FullO3CPU<Impl>::TickEvent::TickEvent(FullO3CPU<Impl> *c)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c)
{
}

template <class Impl>
void
FullO3CPU<Impl>::TickEvent::process()
{
    cpu->tick();
}

template <class Impl>
const char *
FullO3CPU<Impl>::TickEvent::description()
{
    return "FullO3CPU tick event";
}

template <class Impl>
FullO3CPU<Impl>::ActivateThreadEvent::ActivateThreadEvent()
    : Event(&mainEventQueue, CPU_Switch_Pri)
{
}

template <class Impl>
void
FullO3CPU<Impl>::ActivateThreadEvent::init(int thread_num,
                                           FullO3CPU<Impl> *thread_cpu)
{
    tid = thread_num;
    cpu = thread_cpu;
}

template <class Impl>
void
FullO3CPU<Impl>::ActivateThreadEvent::process()
{
    cpu->activateThread(tid);
}

template <class Impl>
const char *
FullO3CPU<Impl>::ActivateThreadEvent::description()
{
    return "FullO3CPU \"Activate Thread\" event";
}

template <class Impl>
FullO3CPU<Impl>::DeallocateContextEvent::DeallocateContextEvent()
    : Event(&mainEventQueue, CPU_Tick_Pri), tid(0), remove(false), cpu(NULL)
{
}

template <class Impl>
void
FullO3CPU<Impl>::DeallocateContextEvent::init(int thread_num,
                                              FullO3CPU<Impl> *thread_cpu)
{
    tid = thread_num;
    cpu = thread_cpu;
    remove = false;
}

template <class Impl>
void
FullO3CPU<Impl>::DeallocateContextEvent::process()
{
    cpu->deactivateThread(tid);
    if (remove)
        cpu->removeThread(tid);
}

template <class Impl>
const char *
FullO3CPU<Impl>::DeallocateContextEvent::description()
{
    return "FullO3CPU \"Deallocate Context\" event";
}

template <class Impl>
FullO3CPU<Impl>::FullO3CPU(O3CPU *o3_cpu, Params *params)
    : BaseO3CPU(params),
#if FULL_SYSTEM
      itb(params->itb),
      dtb(params->dtb),
#endif
      tickEvent(this),
      removeInstsThisCycle(false),
      fetch(o3_cpu, params),
      decode(o3_cpu, params),
      rename(o3_cpu, params),
      iew(o3_cpu, params),
      commit(o3_cpu, params),

      regFile(o3_cpu, params->numPhysIntRegs,
              params->numPhysFloatRegs),

      freeList(params->numberOfThreads,
               TheISA::NumIntRegs, params->numPhysIntRegs,
               TheISA::NumFloatRegs, params->numPhysFloatRegs),

      rob(o3_cpu,
          params->numROBEntries, params->squashWidth,
          params->smtROBPolicy, params->smtROBThreshold,
          params->numberOfThreads),

      scoreboard(params->numberOfThreads,
                 TheISA::NumIntRegs, params->numPhysIntRegs,
                 TheISA::NumFloatRegs, params->numPhysFloatRegs,
                 TheISA::NumMiscRegs * number_of_threads,
                 TheISA::ZeroReg),

      timeBuffer(params->backComSize, params->forwardComSize),
      fetchQueue(params->backComSize, params->forwardComSize),
      decodeQueue(params->backComSize, params->forwardComSize),
      renameQueue(params->backComSize, params->forwardComSize),
      iewQueue(params->backComSize, params->forwardComSize),
      activityRec(NumStages,
                  params->backComSize + params->forwardComSize,
                  params->activity),

      globalSeqNum(1),
#if FULL_SYSTEM
      system(params->system),
      physmem(system->physmem),
#endif // FULL_SYSTEM
      drainCount(0),
      deferRegistration(params->deferRegistration),
      numThreads(number_of_threads)
{
    if (!deferRegistration) {
        _status = Running;
    } else {
        _status = Idle;
    }

    checker = NULL;

    if (params->checker) {
#if USE_CHECKER
        BaseCPU *temp_checker = params->checker;
        checker = dynamic_cast<Checker<DynInstPtr> *>(temp_checker);
#if FULL_SYSTEM
        checker->setSystem(params->system);
#endif
#else
        panic("Checker enabled but not compiled in!");
#endif // USE_CHECKER
    }

#if !FULL_SYSTEM
    thread.resize(number_of_threads);
    tids.resize(number_of_threads);
#endif

    // The stages also need their CPU pointer setup.  However this
    // must be done at the upper level CPU because they have pointers
    // to the upper level CPU, and not this FullO3CPU.

    // Set up Pointers to the activeThreads list for each stage
    fetch.setActiveThreads(&activeThreads);
    decode.setActiveThreads(&activeThreads);
    rename.setActiveThreads(&activeThreads);
    iew.setActiveThreads(&activeThreads);
    commit.setActiveThreads(&activeThreads);

    // Give each of the stages the time buffer they will use.
    fetch.setTimeBuffer(&timeBuffer);
    decode.setTimeBuffer(&timeBuffer);
    rename.setTimeBuffer(&timeBuffer);
    iew.setTimeBuffer(&timeBuffer);
    commit.setTimeBuffer(&timeBuffer);

    // Also setup each of the stages' queues.
    fetch.setFetchQueue(&fetchQueue);
    decode.setFetchQueue(&fetchQueue);
    commit.setFetchQueue(&fetchQueue);
    decode.setDecodeQueue(&decodeQueue);
    rename.setDecodeQueue(&decodeQueue);
    rename.setRenameQueue(&renameQueue);
    iew.setRenameQueue(&renameQueue);
    iew.setIEWQueue(&iewQueue);
    commit.setIEWQueue(&iewQueue);
    commit.setRenameQueue(&renameQueue);

    commit.setIEWStage(&iew);
    rename.setIEWStage(&iew);
    rename.setCommitStage(&commit);

#if !FULL_SYSTEM
    int active_threads = params->workload.size();

    if (active_threads > Impl::MaxThreads) {
        panic("Workload Size too large. Increase the 'MaxThreads'"
              "constant in your O3CPU impl. file (e.g. o3/alpha/impl.hh) or "
              "edit your workload size.");
    }
#else
    int active_threads = 1;
#endif

    //Make Sure That this a Valid Architeture
    assert(params->numPhysIntRegs   >= numThreads * TheISA::NumIntRegs);
    assert(params->numPhysFloatRegs >= numThreads * TheISA::NumFloatRegs);

    rename.setScoreboard(&scoreboard);
    iew.setScoreboard(&scoreboard);

    // Setup the rename map for whichever stages need it.
    PhysRegIndex lreg_idx = 0;
    PhysRegIndex freg_idx = params->numPhysIntRegs; //Index to 1 after int regs

    for (int tid=0; tid < numThreads; tid++) {
        bool bindRegs = (tid <= active_threads - 1);

        commitRenameMap[tid].init(TheISA::NumIntRegs,
                                  params->numPhysIntRegs,
                                  lreg_idx,            //Index for Logical. Regs

                                  TheISA::NumFloatRegs,
                                  params->numPhysFloatRegs,
                                  freg_idx,            //Index for Float Regs

                                  TheISA::NumMiscRegs,

                                  TheISA::ZeroReg,
                                  TheISA::ZeroReg,

                                  tid,
                                  false);

        renameMap[tid].init(TheISA::NumIntRegs,
                            params->numPhysIntRegs,
                            lreg_idx,                  //Index for Logical. Regs

                            TheISA::NumFloatRegs,
                            params->numPhysFloatRegs,
                            freg_idx,                  //Index for Float Regs

                            TheISA::NumMiscRegs,

                            TheISA::ZeroReg,
                            TheISA::ZeroReg,

                            tid,
                            bindRegs);

        activateThreadEvent[tid].init(tid, this);
        deallocateContextEvent[tid].init(tid, this);
    }

    rename.setRenameMap(renameMap);
    commit.setRenameMap(commitRenameMap);

    // Give renameMap & rename stage access to the freeList;
    for (int i=0; i < numThreads; i++) {
        renameMap[i].setFreeList(&freeList);
    }
    rename.setFreeList(&freeList);

    // Setup the ROB for whichever stages need it.
    commit.setROB(&rob);

    lastRunningCycle = curTick;

    lastActivatedCycle = -1;

    // Give renameMap & rename stage access to the freeList;
    //for (int i=0; i < numThreads; i++) {
        //globalSeqNum[i] = 1;
        //}

    contextSwitch = false;
}

template <class Impl>
FullO3CPU<Impl>::~FullO3CPU()
{
}

template <class Impl>
void
FullO3CPU<Impl>::fullCPURegStats()
{
    BaseO3CPU::regStats();

    // Register any of the O3CPU's stats here.
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

    // Number of Instructions simulated
    // --------------------------------
    // Should probably be in Base CPU but need templated
    // MaxThreads so put in here instead
    committedInsts
        .init(numThreads)
        .name(name() + ".committedInsts")
        .desc("Number of Instructions Simulated");

    totalCommittedInsts
        .name(name() + ".committedInsts_total")
        .desc("Number of Instructions Simulated");

    cpi
        .name(name() + ".cpi")
        .desc("CPI: Cycles Per Instruction")
        .precision(6);
    cpi = numCycles / committedInsts;

    totalCpi
        .name(name() + ".cpi_total")
        .desc("CPI: Total CPI of All Threads")
        .precision(6);
    totalCpi = numCycles / totalCommittedInsts;

    ipc
        .name(name() + ".ipc")
        .desc("IPC: Instructions Per Cycle")
        .precision(6);
    ipc =  committedInsts / numCycles;

    totalIpc
        .name(name() + ".ipc_total")
        .desc("IPC: Total IPC of All Threads")
        .precision(6);
    totalIpc =  totalCommittedInsts / numCycles;

}

template <class Impl>
Port *
FullO3CPU<Impl>::getPort(const std::string &if_name, int idx)
{
    if (if_name == "dcache_port")
        return iew.getDcachePort();
    else if (if_name == "icache_port")
        return fetch.getIcachePort();
    else
        panic("No Such Port\n");
}

template <class Impl>
void
FullO3CPU<Impl>::tick()
{
    DPRINTF(O3CPU, "\n\nFullO3CPU: Ticking main, FullO3CPU.\n");

    ++numCycles;

//    activity = false;

    //Tick each of the stages
    fetch.tick();

    decode.tick();

    rename.tick();

    iew.tick();

    commit.tick();

#if !FULL_SYSTEM
    doContextSwitch();
#endif

    // Now advance the time buffers
    timeBuffer.advance();

    fetchQueue.advance();
    decodeQueue.advance();
    renameQueue.advance();
    iewQueue.advance();

    activityRec.advance();

    if (removeInstsThisCycle) {
        cleanUpRemovedInsts();
    }

    if (!tickEvent.scheduled()) {
        if (_status == SwitchedOut ||
            getState() == SimObject::Drained) {
            DPRINTF(O3CPU, "Switched out!\n");
            // increment stat
            lastRunningCycle = curTick;
        } else if (!activityRec.active() || _status == Idle) {
            DPRINTF(O3CPU, "Idle!\n");
            lastRunningCycle = curTick;
            timesIdled++;
        } else {
            tickEvent.schedule(nextCycle(curTick + cycles(1)));
            DPRINTF(O3CPU, "Scheduling next tick!\n");
        }
    }

#if !FULL_SYSTEM
    updateThreadPriority();
#endif

}

template <class Impl>
void
FullO3CPU<Impl>::init()
{
    if (!deferRegistration) {
        registerThreadContexts();
    }

    // Set inSyscall so that the CPU doesn't squash when initially
    // setting up registers.
    for (int i = 0; i < number_of_threads; ++i)
        thread[i]->inSyscall = true;

    for (int tid=0; tid < number_of_threads; tid++) {
#if FULL_SYSTEM
        ThreadContext *src_tc = threadContexts[tid];
#else
        ThreadContext *src_tc = thread[tid]->getTC();
#endif
        // Threads start in the Suspended State
        if (src_tc->status() != ThreadContext::Suspended) {
            continue;
        }

#if FULL_SYSTEM
        TheISA::initCPU(src_tc, src_tc->readCpuId());
#endif
    }

    // Clear inSyscall.
    for (int i = 0; i < number_of_threads; ++i)
        thread[i]->inSyscall = false;

    // Initialize stages.
    fetch.initStage();
    iew.initStage();
    rename.initStage();
    commit.initStage();

    commit.setThreads(thread);
}

template <class Impl>
void
FullO3CPU<Impl>::activateThread(unsigned tid)
{
    list<unsigned>::iterator isActive = find(
        activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i]: Calling activate thread.\n", tid);

    if (isActive == activeThreads.end()) {
        DPRINTF(O3CPU, "[tid:%i]: Adding to active threads list\n",
                tid);

        activeThreads.push_back(tid);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::deactivateThread(unsigned tid)
{
    //Remove From Active List, if Active
    list<unsigned>::iterator thread_it =
        find(activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i]: Calling deactivate thread.\n", tid);

    if (thread_it != activeThreads.end()) {
        DPRINTF(O3CPU,"[tid:%i]: Removing from active threads list\n",
                tid);
        activeThreads.erase(thread_it);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::activateContext(int tid, int delay)
{
    // Needs to set each stage to running as well.
    if (delay){
        DPRINTF(O3CPU, "[tid:%i]: Scheduling thread context to activate "
                "on cycle %d\n", tid, curTick + cycles(delay));
        scheduleActivateThreadEvent(tid, delay);
    } else {
        activateThread(tid);
    }

    if (lastActivatedCycle < curTick) {
        scheduleTickEvent(delay);

        // Be sure to signal that there's some activity so the CPU doesn't
        // deschedule itself.
        activityRec.activity();
        fetch.wakeFromQuiesce();

        lastActivatedCycle = curTick;

        _status = Running;
    }
}

template <class Impl>
bool
FullO3CPU<Impl>::deallocateContext(int tid, bool remove, int delay)
{
    // Schedule removal of thread data from CPU
    if (delay){
        DPRINTF(O3CPU, "[tid:%i]: Scheduling thread context to deallocate "
                "on cycle %d\n", tid, curTick + cycles(delay));
        scheduleDeallocateContextEvent(tid, remove, delay);
        return false;
    } else {
        deactivateThread(tid);
        if (remove)
            removeThread(tid);
        return true;
    }
}

template <class Impl>
void
FullO3CPU<Impl>::suspendContext(int tid)
{
    DPRINTF(O3CPU,"[tid: %i]: Suspending Thread Context.\n", tid);
    bool deallocated = deallocateContext(tid, false, 1);
    // If this was the last thread then unschedule the tick event.
    if (activeThreads.size() == 1 && !deallocated ||
        activeThreads.size() == 0)
        unscheduleTickEvent();
    _status = Idle;
}

template <class Impl>
void
FullO3CPU<Impl>::haltContext(int tid)
{
    //For now, this is the same as deallocate
    DPRINTF(O3CPU,"[tid:%i]: Halt Context called. Deallocating", tid);
    deallocateContext(tid, true, 1);
}

template <class Impl>
void
FullO3CPU<Impl>::insertThread(unsigned tid)
{
    DPRINTF(O3CPU,"[tid:%i] Initializing thread into CPU");
    // Will change now that the PC and thread state is internal to the CPU
    // and not in the ThreadContext.
#if FULL_SYSTEM
    ThreadContext *src_tc = system->threadContexts[tid];
#else
    ThreadContext *src_tc = tcBase(tid);
#endif

    //Bind Int Regs to Rename Map
    for (int ireg = 0; ireg < TheISA::NumIntRegs; ireg++) {
        PhysRegIndex phys_reg = freeList.getIntReg();

        renameMap[tid].setEntry(ireg,phys_reg);
        scoreboard.setReg(phys_reg);
    }

    //Bind Float Regs to Rename Map
    for (int freg = 0; freg < TheISA::NumFloatRegs; freg++) {
        PhysRegIndex phys_reg = freeList.getFloatReg();

        renameMap[tid].setEntry(freg,phys_reg);
        scoreboard.setReg(phys_reg);
    }

    //Copy Thread Data Into RegFile
    //this->copyFromTC(tid);

    //Set PC/NPC/NNPC
    setPC(src_tc->readPC(), tid);
    setNextPC(src_tc->readNextPC(), tid);
    setNextNPC(src_tc->readNextNPC(), tid);

    src_tc->setStatus(ThreadContext::Active);

    activateContext(tid,1);

    //Reset ROB/IQ/LSQ Entries
    commit.rob->resetEntries();
    iew.resetEntries();
}

template <class Impl>
void
FullO3CPU<Impl>::removeThread(unsigned tid)
{
    DPRINTF(O3CPU,"[tid:%i] Removing thread context from CPU.\n", tid);

    // Copy Thread Data From RegFile
    // If thread is suspended, it might be re-allocated
    //this->copyToTC(tid);

    // Unbind Int Regs from Rename Map
    for (int ireg = 0; ireg < TheISA::NumIntRegs; ireg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(ireg);

        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Unbind Float Regs from Rename Map
    for (int freg = 0; freg < TheISA::NumFloatRegs; freg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(freg);

        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Squash Throughout Pipeline
    InstSeqNum squash_seq_num = commit.rob->readHeadInst(tid)->seqNum;
    fetch.squash(0, sizeof(TheISA::MachInst), squash_seq_num, true, tid);
    decode.squash(tid);
    rename.squash(squash_seq_num, tid);
    iew.squash(tid);
    commit.rob->squash(squash_seq_num, tid);

    assert(iew.ldstQueue.getCount(tid) == 0);

    // Reset ROB/IQ/LSQ Entries

    // Commented out for now.  This should be possible to do by
    // telling all the pipeline stages to drain first, and then
    // checking until the drain completes.  Once the pipeline is
    // drained, call resetEntries(). - 10-09-06 ktlim
/*
    if (activeThreads.size() >= 1) {
        commit.rob->resetEntries();
        iew.resetEntries();
    }
*/
}


template <class Impl>
void
FullO3CPU<Impl>::activateWhenReady(int tid)
{
    DPRINTF(O3CPU,"[tid:%i]: Checking if resources are available for incoming"
            "(e.g. PhysRegs/ROB/IQ/LSQ) \n",
            tid);

    bool ready = true;

    if (freeList.numFreeIntRegs() >= TheISA::NumIntRegs) {
        DPRINTF(O3CPU,"[tid:%i] Suspending thread due to not enough "
                "Phys. Int. Regs.\n",
                tid);
        ready = false;
    } else if (freeList.numFreeFloatRegs() >= TheISA::NumFloatRegs) {
        DPRINTF(O3CPU,"[tid:%i] Suspending thread due to not enough "
                "Phys. Float. Regs.\n",
                tid);
        ready = false;
    } else if (commit.rob->numFreeEntries() >=
               commit.rob->entryAmount(activeThreads.size() + 1)) {
        DPRINTF(O3CPU,"[tid:%i] Suspending thread due to not enough "
                "ROB entries.\n",
                tid);
        ready = false;
    } else if (iew.instQueue.numFreeEntries() >=
               iew.instQueue.entryAmount(activeThreads.size() + 1)) {
        DPRINTF(O3CPU,"[tid:%i] Suspending thread due to not enough "
                "IQ entries.\n",
                tid);
        ready = false;
    } else if (iew.ldstQueue.numFreeEntries() >=
               iew.ldstQueue.entryAmount(activeThreads.size() + 1)) {
        DPRINTF(O3CPU,"[tid:%i] Suspending thread due to not enough "
                "LSQ entries.\n",
                tid);
        ready = false;
    }

    if (ready) {
        insertThread(tid);

        contextSwitch = false;

        cpuWaitList.remove(tid);
    } else {
        suspendContext(tid);

        //blocks fetch
        contextSwitch = true;

        //@todo: dont always add to waitlist
        //do waitlist
        cpuWaitList.push_back(tid);
    }
}

#if FULL_SYSTEM
template <class Impl>
void
FullO3CPU<Impl>::updateMemPorts()
{
    // Update all ThreadContext's memory ports (Functional/Virtual
    // Ports)
    for (int i = 0; i < thread.size(); ++i)
        thread[i]->connectMemPorts();
}
#endif

template <class Impl>
void
FullO3CPU<Impl>::serialize(std::ostream &os)
{
    SimObject::State so_state = SimObject::getState();
    SERIALIZE_ENUM(so_state);
    BaseCPU::serialize(os);
    nameOut(os, csprintf("%s.tickEvent", name()));
    tickEvent.serialize(os);

    // Use SimpleThread's ability to checkpoint to make it easier to
    // write out the registers.  Also make this static so it doesn't
    // get instantiated multiple times (causes a panic in statistics).
    static SimpleThread temp;

    for (int i = 0; i < thread.size(); i++) {
        nameOut(os, csprintf("%s.xc.%i", name(), i));
        temp.copyTC(thread[i]->getTC());
        temp.serialize(os);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::unserialize(Checkpoint *cp, const std::string &section)
{
    SimObject::State so_state;
    UNSERIALIZE_ENUM(so_state);
    BaseCPU::unserialize(cp, section);
    tickEvent.unserialize(cp, csprintf("%s.tickEvent", section));

    // Use SimpleThread's ability to checkpoint to make it easier to
    // read in the registers.  Also make this static so it doesn't
    // get instantiated multiple times (causes a panic in statistics).
    static SimpleThread temp;

    for (int i = 0; i < thread.size(); i++) {
        temp.copyTC(thread[i]->getTC());
        temp.unserialize(cp, csprintf("%s.xc.%i", section, i));
        thread[i]->getTC()->copyArchRegs(temp.getTC());
    }
}

template <class Impl>
unsigned int
FullO3CPU<Impl>::drain(Event *drain_event)
{
    DPRINTF(O3CPU, "Switching out\n");

    // If the CPU isn't doing anything, then return immediately.
    if (_status == Idle || _status == SwitchedOut) {
        return 0;
    }

    drainCount = 0;
    fetch.drain();
    decode.drain();
    rename.drain();
    iew.drain();
    commit.drain();

    // Wake the CPU and record activity so everything can drain out if
    // the CPU was not able to immediately drain.
    if (getState() != SimObject::Drained) {
        // A bit of a hack...set the drainEvent after all the drain()
        // calls have been made, that way if all of the stages drain
        // immediately, the signalDrained() function knows not to call
        // process on the drain event.
        drainEvent = drain_event;

        wakeCPU();
        activityRec.activity();

        return 1;
    } else {
        return 0;
    }
}

template <class Impl>
void
FullO3CPU<Impl>::resume()
{
    fetch.resume();
    decode.resume();
    rename.resume();
    iew.resume();
    commit.resume();

    changeState(SimObject::Running);

    if (_status == SwitchedOut || _status == Idle)
        return;

#if FULL_SYSTEM
    assert(system->getMemoryMode() == System::Timing);
#endif

    if (!tickEvent.scheduled())
        tickEvent.schedule(nextCycle());
    _status = Running;
}

template <class Impl>
void
FullO3CPU<Impl>::signalDrained()
{
    if (++drainCount == NumStages) {
        if (tickEvent.scheduled())
            tickEvent.squash();

        changeState(SimObject::Drained);

        BaseCPU::switchOut();

        if (drainEvent) {
            drainEvent->process();
            drainEvent = NULL;
        }
    }
    assert(drainCount <= 5);
}

template <class Impl>
void
FullO3CPU<Impl>::switchOut()
{
    fetch.switchOut();
    rename.switchOut();
    iew.switchOut();
    commit.switchOut();
    instList.clear();
    while (!removeList.empty()) {
        removeList.pop();
    }

    _status = SwitchedOut;
#if USE_CHECKER
    if (checker)
        checker->switchOut();
#endif
    if (tickEvent.scheduled())
        tickEvent.squash();
}

template <class Impl>
void
FullO3CPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    // Flush out any old data from the time buffers.
    for (int i = 0; i < timeBuffer.getSize(); ++i) {
        timeBuffer.advance();
        fetchQueue.advance();
        decodeQueue.advance();
        renameQueue.advance();
        iewQueue.advance();
    }

    activityRec.reset();

    BaseCPU::takeOverFrom(oldCPU, fetch.getIcachePort(), iew.getDcachePort());

    fetch.takeOverFrom();
    decode.takeOverFrom();
    rename.takeOverFrom();
    iew.takeOverFrom();
    commit.takeOverFrom();

    assert(!tickEvent.scheduled());

    // @todo: Figure out how to properly select the tid to put onto
    // the active threads list.
    int tid = 0;

    list<unsigned>::iterator isActive = find(
        activeThreads.begin(), activeThreads.end(), tid);

    if (isActive == activeThreads.end()) {
        //May Need to Re-code this if the delay variable is the delay
        //needed for thread to activate
        DPRINTF(O3CPU, "Adding Thread %i to active threads list\n",
                tid);

        activeThreads.push_back(tid);
    }

    // Set all statuses to active, schedule the CPU's tick event.
    // @todo: Fix up statuses so this is handled properly
    for (int i = 0; i < threadContexts.size(); ++i) {
        ThreadContext *tc = threadContexts[i];
        if (tc->status() == ThreadContext::Active && _status != Running) {
            _status = Running;
            tickEvent.schedule(nextCycle());
        }
    }
    if (!tickEvent.scheduled())
        tickEvent.schedule(nextCycle());
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readIntReg(int reg_idx)
{
    return regFile.readIntReg(reg_idx);
}

template <class Impl>
FloatReg
FullO3CPU<Impl>::readFloatReg(int reg_idx, int width)
{
    return regFile.readFloatReg(reg_idx, width);
}

template <class Impl>
FloatReg
FullO3CPU<Impl>::readFloatReg(int reg_idx)
{
    return regFile.readFloatReg(reg_idx);
}

template <class Impl>
FloatRegBits
FullO3CPU<Impl>::readFloatRegBits(int reg_idx, int width)
{
    return regFile.readFloatRegBits(reg_idx, width);
}

template <class Impl>
FloatRegBits
FullO3CPU<Impl>::readFloatRegBits(int reg_idx)
{
    return regFile.readFloatRegBits(reg_idx);
}

template <class Impl>
void
FullO3CPU<Impl>::setIntReg(int reg_idx, uint64_t val)
{
    regFile.setIntReg(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatReg(int reg_idx, FloatReg val, int width)
{
    regFile.setFloatReg(reg_idx, val, width);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatReg(int reg_idx, FloatReg val)
{
    regFile.setFloatReg(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegBits(int reg_idx, FloatRegBits val, int width)
{
    regFile.setFloatRegBits(reg_idx, val, width);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegBits(int reg_idx, FloatRegBits val)
{
    regFile.setFloatRegBits(reg_idx, val);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchIntReg(int reg_idx, unsigned tid)
{
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(reg_idx);

    return regFile.readIntReg(phys_reg);
}

template <class Impl>
float
FullO3CPU<Impl>::readArchFloatRegSingle(int reg_idx, unsigned tid)
{
    int idx = reg_idx + TheISA::FP_Base_DepTag;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(idx);

    return regFile.readFloatReg(phys_reg);
}

template <class Impl>
double
FullO3CPU<Impl>::readArchFloatRegDouble(int reg_idx, unsigned tid)
{
    int idx = reg_idx + TheISA::FP_Base_DepTag;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(idx);

    return regFile.readFloatReg(phys_reg, 64);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchFloatRegInt(int reg_idx, unsigned tid)
{
    int idx = reg_idx + TheISA::FP_Base_DepTag;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(idx);

    return regFile.readFloatRegBits(phys_reg);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchIntReg(int reg_idx, uint64_t val, unsigned tid)
{
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(reg_idx);

    regFile.setIntReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatRegSingle(int reg_idx, float val, unsigned tid)
{
    int idx = reg_idx + TheISA::FP_Base_DepTag;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(idx);

    regFile.setFloatReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatRegDouble(int reg_idx, double val, unsigned tid)
{
    int idx = reg_idx + TheISA::FP_Base_DepTag;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(idx);

    regFile.setFloatReg(phys_reg, val, 64);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatRegInt(int reg_idx, uint64_t val, unsigned tid)
{
    int idx = reg_idx + TheISA::FP_Base_DepTag;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(idx);

    regFile.setFloatRegBits(phys_reg, val);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readPC(unsigned tid)
{
    return commit.readPC(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::setPC(Addr new_PC,unsigned tid)
{
    commit.setPC(new_PC, tid);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readNextPC(unsigned tid)
{
    return commit.readNextPC(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::setNextPC(uint64_t val,unsigned tid)
{
    commit.setNextPC(val, tid);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readNextNPC(unsigned tid)
{
    return commit.readNextNPC(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::setNextNPC(uint64_t val,unsigned tid)
{
    commit.setNextNPC(val, tid);
}

template <class Impl>
typename FullO3CPU<Impl>::ListIt
FullO3CPU<Impl>::addInst(DynInstPtr &inst)
{
    instList.push_back(inst);

    return --(instList.end());
}

template <class Impl>
void
FullO3CPU<Impl>::instDone(unsigned tid)
{
    // Keep an instruction count.
    thread[tid]->numInst++;
    thread[tid]->numInsts++;
    committedInsts[tid]++;
    totalCommittedInsts++;

    // Check for instruction-count-based events.
    comInstEventQueue[tid]->serviceEvents(thread[tid]->numInst);
}

template <class Impl>
void
FullO3CPU<Impl>::addToRemoveList(DynInstPtr &inst)
{
    removeInstsThisCycle = true;

    removeList.push(inst->getInstListIt());
}

template <class Impl>
void
FullO3CPU<Impl>::removeFrontInst(DynInstPtr &inst)
{
    DPRINTF(O3CPU, "Removing committed instruction [tid:%i] PC %#x "
            "[sn:%lli]\n",
            inst->threadNumber, inst->readPC(), inst->seqNum);

    removeInstsThisCycle = true;

    // Remove the front instruction.
    removeList.push(inst->getInstListIt());
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsNotInROB(unsigned tid,
                                     bool squash_delay_slot,
                                     const InstSeqNum &delay_slot_seq_num)
{
    DPRINTF(O3CPU, "Thread %i: Deleting instructions from instruction"
            " list.\n", tid);

    ListIt end_it;

    bool rob_empty = false;

    if (instList.empty()) {
        return;
    } else if (rob.isEmpty(/*tid*/)) {
        DPRINTF(O3CPU, "ROB is empty, squashing all insts.\n");
        end_it = instList.begin();
        rob_empty = true;
    } else {
        end_it = (rob.readTailInst(tid))->getInstListIt();
        DPRINTF(O3CPU, "ROB is not empty, squashing insts not in ROB.\n");
    }

    removeInstsThisCycle = true;

    ListIt inst_it = instList.end();

    inst_it--;

    // Walk through the instruction list, removing any instructions
    // that were inserted after the given instruction iterator, end_it.
    while (inst_it != end_it) {
        assert(!instList.empty());

#if ISA_HAS_DELAY_SLOT
        if(!squash_delay_slot &&
           delay_slot_seq_num >= (*inst_it)->seqNum) {
            break;
        }
#endif
        squashInstIt(inst_it, tid);

        inst_it--;
    }

    // If the ROB was empty, then we actually need to remove the first
    // instruction as well.
    if (rob_empty) {
        squashInstIt(inst_it, tid);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsUntil(const InstSeqNum &seq_num,
                                  unsigned tid)
{
    assert(!instList.empty());

    removeInstsThisCycle = true;

    ListIt inst_iter = instList.end();

    inst_iter--;

    DPRINTF(O3CPU, "Deleting instructions from instruction "
            "list that are from [tid:%i] and above [sn:%lli] (end=%lli).\n",
            tid, seq_num, (*inst_iter)->seqNum);

    while ((*inst_iter)->seqNum > seq_num) {

        bool break_loop = (inst_iter == instList.begin());

        squashInstIt(inst_iter, tid);

        inst_iter--;

        if (break_loop)
            break;
    }
}

template <class Impl>
inline void
FullO3CPU<Impl>::squashInstIt(const ListIt &instIt, const unsigned &tid)
{
    if ((*instIt)->threadNumber == tid) {
        DPRINTF(O3CPU, "Squashing instruction, "
                "[tid:%i] [sn:%lli] PC %#x\n",
                (*instIt)->threadNumber,
                (*instIt)->seqNum,
                (*instIt)->readPC());

        // Mark it as squashed.
        (*instIt)->setSquashed();

        // @todo: Formulate a consistent method for deleting
        // instructions from the instruction list
        // Remove the instruction from the list.
        removeList.push(instIt);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::cleanUpRemovedInsts()
{
    while (!removeList.empty()) {
        DPRINTF(O3CPU, "Removing instruction, "
                "[tid:%i] [sn:%lli] PC %#x\n",
                (*removeList.front())->threadNumber,
                (*removeList.front())->seqNum,
                (*removeList.front())->readPC());

        instList.erase(removeList.front());

        removeList.pop();
    }

    removeInstsThisCycle = false;
}
/*
template <class Impl>
void
FullO3CPU<Impl>::removeAllInsts()
{
    instList.clear();
}
*/
template <class Impl>
void
FullO3CPU<Impl>::dumpInsts()
{
    int num = 0;

    ListIt inst_list_it = instList.begin();

    cprintf("Dumping Instruction List\n");

    while (inst_list_it != instList.end()) {
        cprintf("Instruction:%i\nPC:%#x\n[tid:%i]\n[sn:%lli]\nIssued:%i\n"
                "Squashed:%i\n\n",
                num, (*inst_list_it)->readPC(), (*inst_list_it)->threadNumber,
                (*inst_list_it)->seqNum, (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());
        inst_list_it++;
        ++num;
    }
}
/*
template <class Impl>
void
FullO3CPU<Impl>::wakeDependents(DynInstPtr &inst)
{
    iew.wakeDependents(inst);
}
*/
template <class Impl>
void
FullO3CPU<Impl>::wakeCPU()
{
    if (activityRec.active() || tickEvent.scheduled()) {
        DPRINTF(Activity, "CPU already running.\n");
        return;
    }

    DPRINTF(Activity, "Waking up CPU\n");

    idleCycles += (curTick - 1) - lastRunningCycle;

    tickEvent.schedule(nextCycle());
}

template <class Impl>
int
FullO3CPU<Impl>::getFreeTid()
{
    for (int i=0; i < numThreads; i++) {
        if (!tids[i]) {
            tids[i] = true;
            return i;
        }
    }

    return -1;
}

template <class Impl>
void
FullO3CPU<Impl>::doContextSwitch()
{
    if (contextSwitch) {

        //ADD CODE TO DEACTIVE THREAD HERE (???)

        for (int tid=0; tid < cpuWaitList.size(); tid++) {
            activateWhenReady(tid);
        }

        if (cpuWaitList.size() == 0)
            contextSwitch = true;
    }
}

template <class Impl>
void
FullO3CPU<Impl>::updateThreadPriority()
{
    if (activeThreads.size() > 1)
    {
        //DEFAULT TO ROUND ROBIN SCHEME
        //e.g. Move highest priority to end of thread list
        list<unsigned>::iterator list_begin = activeThreads.begin();
        list<unsigned>::iterator list_end   = activeThreads.end();

        unsigned high_thread = *list_begin;

        activeThreads.erase(list_begin);

        activeThreads.push_back(high_thread);
    }
}

// Forward declaration of FullO3CPU.
template class FullO3CPU<O3CPUImpl>;
