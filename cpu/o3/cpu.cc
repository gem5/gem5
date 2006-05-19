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
 */

#include "config/full_system.hh"

#if FULL_SYSTEM
#include "sim/system.hh"
#else
#include "sim/process.hh"
#endif

#include "cpu/activity.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/cpu_exec_context.hh"
#include "cpu/exec_context.hh"
#include "cpu/o3/alpha_dyn_inst.hh"
#include "cpu/o3/alpha_impl.hh"
#include "cpu/o3/cpu.hh"

#include "sim/root.hh"
#include "sim/stat_control.hh"

using namespace std;

BaseFullCPU::BaseFullCPU(Params *params)
    : BaseCPU(params), cpu_id(0)
{
}

void
BaseFullCPU::regStats()
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
FullO3CPU<Impl>::FullO3CPU(Params *params)
    : BaseFullCPU(params),
      tickEvent(this),
      removeInstsThisCycle(false),
      fetch(params),
      decode(params),
      rename(params),
      iew(params),
      commit(params),

      regFile(params->numPhysIntRegs, params->numPhysFloatRegs),

      freeList(params->numberOfThreads,//number of activeThreads
               TheISA::NumIntRegs, params->numPhysIntRegs,
               TheISA::NumFloatRegs, params->numPhysFloatRegs),

      rob(params->numROBEntries, params->squashWidth,
          params->smtROBPolicy, params->smtROBThreshold,
          params->numberOfThreads),

      scoreboard(params->numberOfThreads,//number of activeThreads
                 TheISA::NumIntRegs, params->numPhysIntRegs,
                 TheISA::NumFloatRegs, params->numPhysFloatRegs,
                 TheISA::NumMiscRegs * number_of_threads,
                 TheISA::ZeroReg),

      // For now just have these time buffers be pretty big.
      // @todo: Make these time buffer sizes parameters or derived
      // from latencies
      timeBuffer(5, 5),
      fetchQueue(5, 5),
      decodeQueue(5, 5),
      renameQueue(5, 5),
      iewQueue(5, 5),
      activityRec(NumStages, 10, params->activity),

      globalSeqNum(1),

#if FULL_SYSTEM
      system(params->system),
      memCtrl(system->memctrl),
      physmem(system->physmem),
      mem(params->mem),
#else
//      pTable(params->pTable),
      mem(params->workload[0]->getMemory()),
#endif // FULL_SYSTEM
      switchCount(0),
      icacheInterface(params->icacheInterface),
      dcacheInterface(params->dcacheInterface),
      deferRegistration(params->deferRegistration),
      numThreads(number_of_threads)
{
    _status = Idle;

    if (params->checker) {
        BaseCPU *temp_checker = params->checker;
        checker = dynamic_cast<Checker<DynInstPtr> *>(temp_checker);
        checker->setMemory(mem);
#if FULL_SYSTEM
        checker->setSystem(params->system);
#endif
    } else {
        checker = NULL;
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

    commit.setFetchStage(&fetch);
    commit.setIEWStage(&iew);
    rename.setIEWStage(&iew);
    rename.setCommitStage(&commit);

#if !FULL_SYSTEM
    int active_threads = params->workload.size();
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
    }

    rename.setRenameMap(renameMap);
    commit.setRenameMap(commitRenameMap);

    // Give renameMap & rename stage access to the freeList;
    for (int i=0; i < numThreads; i++) {
        renameMap[i].setFreeList(&freeList);
    }
    rename.setFreeList(&freeList);

    // Setup the page table for whichever stages need it.
#if !FULL_SYSTEM
//    fetch.setPageTable(pTable);
//    iew.setPageTable(pTable);
#endif

    // Setup the ROB for whichever stages need it.
    commit.setROB(&rob);

    lastRunningCycle = curTick;

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
    BaseFullCPU::regStats();

    // Register any of the FullCPU's stats here.
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
    cpi = simTicks / committedInsts;

    totalCpi
        .name(name() + ".cpi_total")
        .desc("CPI: Total CPI of All Threads")
        .precision(6);
    totalCpi = simTicks / totalCommittedInsts;

    ipc
        .name(name() + ".ipc")
        .desc("IPC: Instructions Per Cycle")
        .precision(6);
    ipc =  committedInsts / simTicks;

    totalIpc
        .name(name() + ".ipc_total")
        .desc("IPC: Total IPC of All Threads")
        .precision(6);
    totalIpc =  totalCommittedInsts / simTicks;

}

template <class Impl>
void
FullO3CPU<Impl>::tick()
{
    DPRINTF(FullCPU, "\n\nFullCPU: Ticking main, FullO3CPU.\n");

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
        if (_status == SwitchedOut) {
            // increment stat
            lastRunningCycle = curTick;
        } else if (!activityRec.active()) {
            lastRunningCycle = curTick;
            timesIdled++;
        } else {
            tickEvent.schedule(curTick + cycles(1));
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
        registerExecContexts();
    }

    // Set inSyscall so that the CPU doesn't squash when initially
    // setting up registers.
    for (int i = 0; i < number_of_threads; ++i)
        thread[i]->inSyscall = true;

    for (int tid=0; tid < number_of_threads; tid++) {
#if FULL_SYSTEM
        ExecContext *src_xc = execContexts[tid];
#else
        ExecContext *src_xc = thread[tid]->getXCProxy();
#endif
        // Threads start in the Suspended State
        if (src_xc->status() != ExecContext::Suspended) {
            continue;
        }

#if FULL_SYSTEM
        TheISA::initCPU(src_xc, src_xc->readCpuId());
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
FullO3CPU<Impl>::insertThread(unsigned tid)
{
    DPRINTF(FullCPU,"[tid:%i] Initializing thread data");
    // Will change now that the PC and thread state is internal to the CPU
    // and not in the CPUExecContext.
#if 0
#if FULL_SYSTEM
    ExecContext *src_xc = system->execContexts[tid];
#else
    CPUExecContext *src_xc = thread[tid];
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
    this->copyFromXC(tid);

    //Set PC/NPC
    regFile.pc[tid]  = src_xc->readPC();
    regFile.npc[tid] = src_xc->readNextPC();

    src_xc->setStatus(ExecContext::Active);

    activateContext(tid,1);

    //Reset ROB/IQ/LSQ Entries
    commit.rob->resetEntries();
    iew.resetEntries();
#endif
}

template <class Impl>
void
FullO3CPU<Impl>::removeThread(unsigned tid)
{
    DPRINTF(FullCPU,"[tid:%i] Removing thread data");
#if 0
    //Unbind Int Regs from Rename Map
    for (int ireg = 0; ireg < TheISA::NumIntRegs; ireg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(ireg);

        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    //Unbind Float Regs from Rename Map
    for (int freg = 0; freg < TheISA::NumFloatRegs; freg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(freg);

        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    //Copy Thread Data From RegFile
    /* Fix Me:
     * Do we really need to do this if we are removing a thread
     * in the sense that it's finished (exiting)? If the thread is just
     * being suspended we might...
     */
//    this->copyToXC(tid);

    //Squash Throughout Pipeline
    fetch.squash(0,tid);
    decode.squash(tid);
    rename.squash(tid);

    assert(iew.ldstQueue.getCount(tid) == 0);

    //Reset ROB/IQ/LSQ Entries
    if (activeThreads.size() >= 1) {
        commit.rob->resetEntries();
        iew.resetEntries();
    }
#endif
}


template <class Impl>
void
FullO3CPU<Impl>::activateWhenReady(int tid)
{
    DPRINTF(FullCPU,"[tid:%i]: Checking if resources are available for incoming"
            "(e.g. PhysRegs/ROB/IQ/LSQ) \n",
            tid);

    bool ready = true;

    if (freeList.numFreeIntRegs() >= TheISA::NumIntRegs) {
        DPRINTF(FullCPU,"[tid:%i] Suspending thread due to not enough "
                "Phys. Int. Regs.\n",
                tid);
        ready = false;
    } else if (freeList.numFreeFloatRegs() >= TheISA::NumFloatRegs) {
        DPRINTF(FullCPU,"[tid:%i] Suspending thread due to not enough "
                "Phys. Float. Regs.\n",
                tid);
        ready = false;
    } else if (commit.rob->numFreeEntries() >=
               commit.rob->entryAmount(activeThreads.size() + 1)) {
        DPRINTF(FullCPU,"[tid:%i] Suspending thread due to not enough "
                "ROB entries.\n",
                tid);
        ready = false;
    } else if (iew.instQueue.numFreeEntries() >=
               iew.instQueue.entryAmount(activeThreads.size() + 1)) {
        DPRINTF(FullCPU,"[tid:%i] Suspending thread due to not enough "
                "IQ entries.\n",
                tid);
        ready = false;
    } else if (iew.ldstQueue.numFreeEntries() >=
               iew.ldstQueue.entryAmount(activeThreads.size() + 1)) {
        DPRINTF(FullCPU,"[tid:%i] Suspending thread due to not enough "
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

        //do waitlist
        cpuWaitList.push_back(tid);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::activateContext(int tid, int delay)
{
    // Needs to set each stage to running as well.
    list<unsigned>::iterator isActive = find(
        activeThreads.begin(), activeThreads.end(), tid);

    if (isActive == activeThreads.end()) {
        //May Need to Re-code this if the delay variable is the
        //delay needed for thread to activate
        DPRINTF(FullCPU, "Adding Thread %i to active threads list\n",
                tid);

        activeThreads.push_back(tid);
    }

    assert(_status == Idle || _status == SwitchedOut);

    scheduleTickEvent(delay);

    // Be sure to signal that there's some activity so the CPU doesn't
    // deschedule itself.
    activityRec.activity();
    fetch.wakeFromQuiesce();

    _status = Running;
}

template <class Impl>
void
FullO3CPU<Impl>::suspendContext(int tid)
{
    DPRINTF(FullCPU,"[tid: %i]: Suspended ...\n", tid);
    unscheduleTickEvent();
    _status = Idle;
/*
    //Remove From Active List, if Active
    list<unsigned>::iterator isActive = find(
        activeThreads.begin(), activeThreads.end(), tid);

    if (isActive != activeThreads.end()) {
        DPRINTF(FullCPU,"[tid:%i]: Removing from active threads list\n",
                tid);
        activeThreads.erase(isActive);
    }
*/
}

template <class Impl>
void
FullO3CPU<Impl>::deallocateContext(int tid)
{
    DPRINTF(FullCPU,"[tid:%i]: Deallocating ...", tid);
/*
    //Remove From Active List, if Active
    list<unsigned>::iterator isActive = find(
        activeThreads.begin(), activeThreads.end(), tid);

    if (isActive != activeThreads.end()) {
        DPRINTF(FullCPU,"[tid:%i]: Removing from active threads list\n",
                tid);
        activeThreads.erase(isActive);

        removeThread(tid);
    }
*/
}

template <class Impl>
void
FullO3CPU<Impl>::haltContext(int tid)
{
    DPRINTF(FullCPU,"[tid:%i]: Halted ...", tid);
/*
    //Remove From Active List, if Active
    list<unsigned>::iterator isActive = find(
        activeThreads.begin(), activeThreads.end(), tid);

    if (isActive != activeThreads.end()) {
        DPRINTF(FullCPU,"[tid:%i]: Removing from active threads list\n",
                tid);
        activeThreads.erase(isActive);

        removeThread(tid);
    }
*/
}

template <class Impl>
void
FullO3CPU<Impl>::switchOut(Sampler *_sampler)
{
    sampler = _sampler;
    switchCount = 0;
    fetch.switchOut();
    decode.switchOut();
    rename.switchOut();
    iew.switchOut();
    commit.switchOut();

    // Wake the CPU and record activity so everything can drain out if
    // the CPU is currently idle.
    wakeCPU();
    activityRec.activity();
}

template <class Impl>
void
FullO3CPU<Impl>::signalSwitched()
{
    if (++switchCount == NumStages) {
        fetch.doSwitchOut();
        rename.doSwitchOut();
        commit.doSwitchOut();
        instList.clear();
        while (!removeList.empty()) {
            removeList.pop();
        }

        if (checker)
            checker->switchOut(sampler);

        if (tickEvent.scheduled())
            tickEvent.squash();
        sampler->signalSwitched();
        _status = SwitchedOut;
    }
    assert(switchCount <= 5);
}

template <class Impl>
void
FullO3CPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    // Flush out any old data from the time buffers.
    for (int i = 0; i < 10; ++i) {
        timeBuffer.advance();
        fetchQueue.advance();
        decodeQueue.advance();
        renameQueue.advance();
        iewQueue.advance();
    }

    activityRec.reset();

    BaseCPU::takeOverFrom(oldCPU);

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
        DPRINTF(FullCPU, "Adding Thread %i to active threads list\n",
                tid);

        activeThreads.push_back(tid);
    }

    // Set all statuses to active, schedule the CPU's tick event.
    // @todo: Fix up statuses so this is handled properly
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];
        if (xc->status() == ExecContext::Active && _status != Running) {
            _status = Running;
            tickEvent.schedule(curTick);
        }
    }
    if (!tickEvent.scheduled())
        tickEvent.schedule(curTick);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readIntReg(int reg_idx)
{
    return regFile.readIntReg(reg_idx);
}

template <class Impl>
float
FullO3CPU<Impl>::readFloatRegSingle(int reg_idx)
{
    return regFile.readFloatRegSingle(reg_idx);
}

template <class Impl>
double
FullO3CPU<Impl>::readFloatRegDouble(int reg_idx)
{
    return regFile.readFloatRegDouble(reg_idx);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readFloatRegInt(int reg_idx)
{
    return regFile.readFloatRegInt(reg_idx);
}

template <class Impl>
void
FullO3CPU<Impl>::setIntReg(int reg_idx, uint64_t val)
{
    regFile.setIntReg(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegSingle(int reg_idx, float val)
{
    regFile.setFloatRegSingle(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegDouble(int reg_idx, double val)
{
    regFile.setFloatRegDouble(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegInt(int reg_idx, uint64_t val)
{
    regFile.setFloatRegInt(reg_idx, val);
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

    return regFile.readFloatRegSingle(phys_reg);
}

template <class Impl>
double
FullO3CPU<Impl>::readArchFloatRegDouble(int reg_idx, unsigned tid)
{
    int idx = reg_idx + TheISA::FP_Base_DepTag;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(idx);

    return regFile.readFloatRegDouble(phys_reg);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchFloatRegInt(int reg_idx, unsigned tid)
{
    int idx = reg_idx + TheISA::FP_Base_DepTag;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(idx);

    return regFile.readFloatRegInt(phys_reg);
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
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(reg_idx);

    regFile.setFloatRegSingle(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatRegDouble(int reg_idx, double val, unsigned tid)
{
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(reg_idx);

    regFile.setFloatRegDouble(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatRegInt(int reg_idx, uint64_t val, unsigned tid)
{
    PhysRegIndex phys_reg = commitRenameMap[tid].lookup(reg_idx);

    regFile.setFloatRegInt(phys_reg, val);
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
    DPRINTF(FullCPU, "FullCPU: Removing committed instruction [tid:%i] PC %#x "
            "[sn:%lli]\n",
            inst->threadNumber, inst->readPC(), inst->seqNum);

    removeInstsThisCycle = true;

    // Remove the front instruction.
    removeList.push(inst->getInstListIt());
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsNotInROB(unsigned tid)
{
    DPRINTF(FullCPU, "FullCPU: Thread %i: Deleting instructions from instruction"
            " list.\n", tid);

    ListIt end_it;

    bool rob_empty = false;

    if (instList.empty()) {
        return;
    } else if (rob.isEmpty(/*tid*/)) {
        DPRINTF(FullCPU, "FullCPU: ROB is empty, squashing all insts.\n");
        end_it = instList.begin();
        rob_empty = true;
    } else {
        end_it = (rob.readTailInst(tid))->getInstListIt();
        DPRINTF(FullCPU, "FullCPU: ROB is not empty, squashing insts not in ROB.\n");
    }

    removeInstsThisCycle = true;

    ListIt inst_it = instList.end();

    inst_it--;

    // Walk through the instruction list, removing any instructions
    // that were inserted after the given instruction iterator, end_it.
    while (inst_it != end_it) {
        assert(!instList.empty());

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

    DPRINTF(FullCPU, "FullCPU: Deleting instructions from instruction "
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
        DPRINTF(FullCPU, "FullCPU: Squashing instruction, "
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
        DPRINTF(FullCPU, "FullCPU: Removing instruction, "
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

    tickEvent.schedule(curTick);
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
template class FullO3CPU<AlphaSimpleImpl>;
