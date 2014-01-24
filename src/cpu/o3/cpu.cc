/*
 * Copyright (c) 2011-2012 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 *          Korey Sewell
 *          Rick Strong
 */

#include "arch/kernel_stats.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/checker/thread_context.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/isa_specific.hh"
#include "cpu/o3/thread_context.hh"
#include "cpu/activity.hh"
#include "cpu/quiesce_event.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/O3CPU.hh"
#include "debug/Quiesce.hh"
#include "enums/MemoryMode.hh"
#include "sim/core.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/osfpal.hh"
#include "debug/Activity.hh"
#endif

struct BaseCPUParams;

using namespace TheISA;
using namespace std;

BaseO3CPU::BaseO3CPU(BaseCPUParams *params)
    : BaseCPU(params)
{
}

void
BaseO3CPU::regStats()
{
    BaseCPU::regStats();
}

template<class Impl>
bool
FullO3CPU<Impl>::IcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(O3CPU, "Fetch unit received timing\n");
    // We shouldn't ever get a block in ownership state
    assert(!(pkt->memInhibitAsserted() && !pkt->sharedAsserted()));
    fetch->processCacheCompletion(pkt);

    return true;
}

template<class Impl>
void
FullO3CPU<Impl>::IcachePort::recvRetry()
{
    fetch->recvRetry();
}

template <class Impl>
bool
FullO3CPU<Impl>::DcachePort::recvTimingResp(PacketPtr pkt)
{
    return lsq->recvTimingResp(pkt);
}

template <class Impl>
void
FullO3CPU<Impl>::DcachePort::recvTimingSnoopReq(PacketPtr pkt)
{
    lsq->recvTimingSnoopReq(pkt);
}

template <class Impl>
void
FullO3CPU<Impl>::DcachePort::recvRetry()
{
    lsq->recvRetry();
}

template <class Impl>
FullO3CPU<Impl>::TickEvent::TickEvent(FullO3CPU<Impl> *c)
    : Event(CPU_Tick_Pri), cpu(c)
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
FullO3CPU<Impl>::TickEvent::description() const
{
    return "FullO3CPU tick";
}

template <class Impl>
FullO3CPU<Impl>::ActivateThreadEvent::ActivateThreadEvent()
    : Event(CPU_Switch_Pri)
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
FullO3CPU<Impl>::ActivateThreadEvent::description() const
{
    return "FullO3CPU \"Activate Thread\"";
}

template <class Impl>
FullO3CPU<Impl>::DeallocateContextEvent::DeallocateContextEvent()
    : Event(CPU_Tick_Pri), tid(0), remove(false), cpu(NULL)
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
FullO3CPU<Impl>::DeallocateContextEvent::description() const
{
    return "FullO3CPU \"Deallocate Context\"";
}

template <class Impl>
FullO3CPU<Impl>::FullO3CPU(DerivO3CPUParams *params)
    : BaseO3CPU(params),
      itb(params->itb),
      dtb(params->dtb),
      tickEvent(this),
#ifndef NDEBUG
      instcount(0),
#endif
      removeInstsThisCycle(false),
      fetch(this, params),
      decode(this, params),
      rename(this, params),
      iew(this, params),
      commit(this, params),

      regFile(params->numPhysIntRegs,
              params->numPhysFloatRegs,
              params->numPhysCCRegs),

      freeList(name() + ".freelist", &regFile),

      rob(this, params),

      scoreboard(name() + ".scoreboard",
                 regFile.totalNumPhysRegs(), TheISA::NumMiscRegs,
                 TheISA::ZeroReg, TheISA::ZeroReg),

      isa(numThreads, NULL),

      icachePort(&fetch, this),
      dcachePort(&iew.ldstQueue, this),

      timeBuffer(params->backComSize, params->forwardComSize),
      fetchQueue(params->backComSize, params->forwardComSize),
      decodeQueue(params->backComSize, params->forwardComSize),
      renameQueue(params->backComSize, params->forwardComSize),
      iewQueue(params->backComSize, params->forwardComSize),
      activityRec(name(), NumStages,
                  params->backComSize + params->forwardComSize,
                  params->activity),

      globalSeqNum(1),
      system(params->system),
      drainManager(NULL),
      lastRunningCycle(curCycle())
{
    if (!params->switched_out) {
        _status = Running;
    } else {
        _status = SwitchedOut;
    }

    if (params->checker) {
        BaseCPU *temp_checker = params->checker;
        checker = dynamic_cast<Checker<Impl> *>(temp_checker);
        checker->setIcachePort(&icachePort);
        checker->setSystem(params->system);
    } else {
        checker = NULL;
    }

    if (!FullSystem) {
        thread.resize(numThreads);
        tids.resize(numThreads);
    }

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

    ThreadID active_threads;
    if (FullSystem) {
        active_threads = 1;
    } else {
        active_threads = params->workload.size();

        if (active_threads > Impl::MaxThreads) {
            panic("Workload Size too large. Increase the 'MaxThreads' "
                  "constant in your O3CPU impl. file (e.g. o3/alpha/impl.hh) "
                  "or edit your workload size.");
        }
    }

    //Make Sure That this a Valid Architeture
    assert(params->numPhysIntRegs   >= numThreads * TheISA::NumIntRegs);
    assert(params->numPhysFloatRegs >= numThreads * TheISA::NumFloatRegs);
    assert(params->numPhysCCRegs >= numThreads * TheISA::NumCCRegs);

    rename.setScoreboard(&scoreboard);
    iew.setScoreboard(&scoreboard);

    // Setup the rename map for whichever stages need it.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        isa[tid] = params->isa[tid];

        // Only Alpha has an FP zero register, so for other ISAs we
        // use an invalid FP register index to avoid special treatment
        // of any valid FP reg.
        RegIndex invalidFPReg = TheISA::NumFloatRegs + 1;
        RegIndex fpZeroReg =
            (THE_ISA == ALPHA_ISA) ? TheISA::ZeroReg : invalidFPReg;

        commitRenameMap[tid].init(&regFile, TheISA::ZeroReg, fpZeroReg,
                                  &freeList);

        renameMap[tid].init(&regFile, TheISA::ZeroReg, fpZeroReg,
                            &freeList);

        activateThreadEvent[tid].init(tid, this);
        deallocateContextEvent[tid].init(tid, this);
    }

    // Initialize rename map to assign physical registers to the
    // architectural registers for active threads only.
    for (ThreadID tid = 0; tid < active_threads; tid++) {
        for (RegIndex ridx = 0; ridx < TheISA::NumIntRegs; ++ridx) {
            // Note that we can't use the rename() method because we don't
            // want special treatment for the zero register at this point
            PhysRegIndex phys_reg = freeList.getIntReg();
            renameMap[tid].setIntEntry(ridx, phys_reg);
            commitRenameMap[tid].setIntEntry(ridx, phys_reg);
        }

        for (RegIndex ridx = 0; ridx < TheISA::NumFloatRegs; ++ridx) {
            PhysRegIndex phys_reg = freeList.getFloatReg();
            renameMap[tid].setFloatEntry(ridx, phys_reg);
            commitRenameMap[tid].setFloatEntry(ridx, phys_reg);
        }

        for (RegIndex ridx = 0; ridx < TheISA::NumCCRegs; ++ridx) {
            PhysRegIndex phys_reg = freeList.getCCReg();
            renameMap[tid].setCCEntry(ridx, phys_reg);
            commitRenameMap[tid].setCCEntry(ridx, phys_reg);
        }
    }

    rename.setRenameMap(renameMap);
    commit.setRenameMap(commitRenameMap);
    rename.setFreeList(&freeList);

    // Setup the ROB for whichever stages need it.
    commit.setROB(&rob);

    lastActivatedCycle = 0;
#if 0
    // Give renameMap & rename stage access to the freeList;
    for (ThreadID tid = 0; tid < numThreads; tid++)
        globalSeqNum[tid] = 1;
#endif

    contextSwitch = false;
    DPRINTF(O3CPU, "Creating O3CPU object.\n");

    // Setup any thread state.
    this->thread.resize(this->numThreads);

    for (ThreadID tid = 0; tid < this->numThreads; ++tid) {
        if (FullSystem) {
            // SMT is not supported in FS mode yet.
            assert(this->numThreads == 1);
            this->thread[tid] = new Thread(this, 0, NULL);
        } else {
            if (tid < params->workload.size()) {
                DPRINTF(O3CPU, "Workload[%i] process is %#x",
                        tid, this->thread[tid]);
                this->thread[tid] = new typename FullO3CPU<Impl>::Thread(
                        (typename Impl::O3CPU *)(this),
                        tid, params->workload[tid]);

                //usedTids[tid] = true;
                //threadMap[tid] = tid;
            } else {
                //Allocate Empty thread so M5 can use later
                //when scheduling threads to CPU
                Process* dummy_proc = NULL;

                this->thread[tid] = new typename FullO3CPU<Impl>::Thread(
                        (typename Impl::O3CPU *)(this),
                        tid, dummy_proc);
                //usedTids[tid] = false;
            }
        }

        ThreadContext *tc;

        // Setup the TC that will serve as the interface to the threads/CPU.
        O3ThreadContext<Impl> *o3_tc = new O3ThreadContext<Impl>;

        tc = o3_tc;

        // If we're using a checker, then the TC should be the
        // CheckerThreadContext.
        if (params->checker) {
            tc = new CheckerThreadContext<O3ThreadContext<Impl> >(
                o3_tc, this->checker);
        }

        o3_tc->cpu = (typename Impl::O3CPU *)(this);
        assert(o3_tc->cpu);
        o3_tc->thread = this->thread[tid];

        if (FullSystem) {
            // Setup quiesce event.
            this->thread[tid]->quiesceEvent = new EndQuiesceEvent(tc);
        }
        // Give the thread the TC.
        this->thread[tid]->tc = tc;

        // Add the TC to the CPU's list of TC's.
        this->threadContexts.push_back(tc);
    }

    // FullO3CPU always requires an interrupt controller.
    if (!params->switched_out && !interrupts) {
        fatal("FullO3CPU %s has no interrupt controller.\n"
              "Ensure createInterruptController() is called.\n", name());
    }

    for (ThreadID tid = 0; tid < this->numThreads; tid++)
        this->thread[tid]->setFuncExeInst(0);
}

template <class Impl>
FullO3CPU<Impl>::~FullO3CPU()
{
}

template <class Impl>
void
FullO3CPU<Impl>::regProbePoints()
{
    ppInstAccessComplete = new ProbePointArg<PacketPtr>(getProbeManager(), "InstAccessComplete");
    ppDataAccessComplete = new ProbePointArg<std::pair<DynInstPtr, PacketPtr> >(getProbeManager(), "DataAccessComplete");
    fetch.regProbePoints();
    iew.regProbePoints();
    commit.regProbePoints();
}

template <class Impl>
void
FullO3CPU<Impl>::regStats()
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

    quiesceCycles
        .name(name() + ".quiesceCycles")
        .desc("Total number of cycles that CPU has spent quiesced or waiting "
              "for an interrupt")
        .prereq(quiesceCycles);

    // Number of Instructions simulated
    // --------------------------------
    // Should probably be in Base CPU but need templated
    // MaxThreads so put in here instead
    committedInsts
        .init(numThreads)
        .name(name() + ".committedInsts")
        .desc("Number of Instructions Simulated");

    committedOps
        .init(numThreads)
        .name(name() + ".committedOps")
        .desc("Number of Ops (including micro ops) Simulated");

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

    this->fetch.regStats();
    this->decode.regStats();
    this->rename.regStats();
    this->iew.regStats();
    this->commit.regStats();
    this->rob.regStats();

    intRegfileReads
        .name(name() + ".int_regfile_reads")
        .desc("number of integer regfile reads")
        .prereq(intRegfileReads);

    intRegfileWrites
        .name(name() + ".int_regfile_writes")
        .desc("number of integer regfile writes")
        .prereq(intRegfileWrites);

    fpRegfileReads
        .name(name() + ".fp_regfile_reads")
        .desc("number of floating regfile reads")
        .prereq(fpRegfileReads);

    fpRegfileWrites
        .name(name() + ".fp_regfile_writes")
        .desc("number of floating regfile writes")
        .prereq(fpRegfileWrites);

    ccRegfileReads
        .name(name() + ".cc_regfile_reads")
        .desc("number of cc regfile reads")
        .prereq(ccRegfileReads);

    ccRegfileWrites
        .name(name() + ".cc_regfile_writes")
        .desc("number of cc regfile writes")
        .prereq(ccRegfileWrites);

    miscRegfileReads
        .name(name() + ".misc_regfile_reads")
        .desc("number of misc regfile reads")
        .prereq(miscRegfileReads);

    miscRegfileWrites
        .name(name() + ".misc_regfile_writes")
        .desc("number of misc regfile writes")
        .prereq(miscRegfileWrites);
}

template <class Impl>
void
FullO3CPU<Impl>::tick()
{
    DPRINTF(O3CPU, "\n\nFullO3CPU: Ticking main, FullO3CPU.\n");
    assert(!switchedOut());
    assert(getDrainState() != Drainable::Drained);

    ++numCycles;

//    activity = false;

    //Tick each of the stages
    fetch.tick();

    decode.tick();

    rename.tick();

    iew.tick();

    commit.tick();

    if (!FullSystem)
        doContextSwitch();

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
            DPRINTF(O3CPU, "Switched out!\n");
            // increment stat
            lastRunningCycle = curCycle();
        } else if (!activityRec.active() || _status == Idle) {
            DPRINTF(O3CPU, "Idle!\n");
            lastRunningCycle = curCycle();
            timesIdled++;
        } else {
            schedule(tickEvent, clockEdge(Cycles(1)));
            DPRINTF(O3CPU, "Scheduling next tick!\n");
        }
    }

    if (!FullSystem)
        updateThreadPriority();

    tryDrain();
}

template <class Impl>
void
FullO3CPU<Impl>::init()
{
    BaseCPU::init();

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        // Set noSquashFromTC so that the CPU doesn't squash when initially
        // setting up registers.
        thread[tid]->noSquashFromTC = true;
        // Initialise the ThreadContext's memory proxies
        thread[tid]->initMemProxies(thread[tid]->getTC());
    }

    if (FullSystem && !params()->switched_out) {
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            ThreadContext *src_tc = threadContexts[tid];
            TheISA::initCPU(src_tc, src_tc->contextId());
        }
    }

    // Clear noSquashFromTC.
    for (int tid = 0; tid < numThreads; ++tid)
        thread[tid]->noSquashFromTC = false;

    commit.setThreads(thread);
}

template <class Impl>
void
FullO3CPU<Impl>::startup()
{
    BaseCPU::startup();
    for (int tid = 0; tid < numThreads; ++tid)
        isa[tid]->startup(threadContexts[tid]);

    fetch.startupStage();
    decode.startupStage();
    iew.startupStage();
    rename.startupStage();
    commit.startupStage();
}

template <class Impl>
void
FullO3CPU<Impl>::activateThread(ThreadID tid)
{
    list<ThreadID>::iterator isActive =
        std::find(activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i]: Calling activate thread.\n", tid);
    assert(!switchedOut());

    if (isActive == activeThreads.end()) {
        DPRINTF(O3CPU, "[tid:%i]: Adding to active threads list\n",
                tid);

        activeThreads.push_back(tid);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::deactivateThread(ThreadID tid)
{
    //Remove From Active List, if Active
    list<ThreadID>::iterator thread_it =
        std::find(activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i]: Calling deactivate thread.\n", tid);
    assert(!switchedOut());

    if (thread_it != activeThreads.end()) {
        DPRINTF(O3CPU,"[tid:%i]: Removing from active threads list\n",
                tid);
        activeThreads.erase(thread_it);
    }
}

template <class Impl>
Counter
FullO3CPU<Impl>::totalInsts() const
{
    Counter total(0);

    ThreadID size = thread.size();
    for (ThreadID i = 0; i < size; i++)
        total += thread[i]->numInst;

    return total;
}

template <class Impl>
Counter
FullO3CPU<Impl>::totalOps() const
{
    Counter total(0);

    ThreadID size = thread.size();
    for (ThreadID i = 0; i < size; i++)
        total += thread[i]->numOp;

    return total;
}

template <class Impl>
void
FullO3CPU<Impl>::activateContext(ThreadID tid, Cycles delay)
{
    assert(!switchedOut());

    // Needs to set each stage to running as well.
    if (delay){
        DPRINTF(O3CPU, "[tid:%i]: Scheduling thread context to activate "
                "on cycle %d\n", tid, clockEdge(delay));
        scheduleActivateThreadEvent(tid, delay);
    } else {
        activateThread(tid);
    }

    // We don't want to wake the CPU if it is drained. In that case,
    // we just want to flag the thread as active and schedule the tick
    // event from drainResume() instead.
    if (getDrainState() == Drainable::Drained)
        return;

    // If we are time 0 or if the last activation time is in the past,
    // schedule the next tick and wake up the fetch unit
    if (lastActivatedCycle == 0 || lastActivatedCycle < curTick()) {
        scheduleTickEvent(delay);

        // Be sure to signal that there's some activity so the CPU doesn't
        // deschedule itself.
        activityRec.activity();
        fetch.wakeFromQuiesce();

        Cycles cycles(curCycle() - lastRunningCycle);
        // @todo: This is an oddity that is only here to match the stats
        if (cycles != 0)
            --cycles;
        quiesceCycles += cycles;

        lastActivatedCycle = curTick();

        _status = Running;
    }
}

template <class Impl>
bool
FullO3CPU<Impl>::scheduleDeallocateContext(ThreadID tid, bool remove,
                                           Cycles delay)
{
    // Schedule removal of thread data from CPU
    if (delay){
        DPRINTF(O3CPU, "[tid:%i]: Scheduling thread context to deallocate "
                "on tick %d\n", tid, clockEdge(delay));
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
FullO3CPU<Impl>::suspendContext(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid: %i]: Suspending Thread Context.\n", tid);
    assert(!switchedOut());
    bool deallocated = scheduleDeallocateContext(tid, false, Cycles(1));
    // If this was the last thread then unschedule the tick event.
    if ((activeThreads.size() == 1 && !deallocated) ||
        activeThreads.size() == 0)
        unscheduleTickEvent();

    DPRINTF(Quiesce, "Suspending Context\n");
    lastRunningCycle = curCycle();
    _status = Idle;
}

template <class Impl>
void
FullO3CPU<Impl>::haltContext(ThreadID tid)
{
    //For now, this is the same as deallocate
    DPRINTF(O3CPU,"[tid:%i]: Halt Context called. Deallocating", tid);
    assert(!switchedOut());
    scheduleDeallocateContext(tid, true, Cycles(1));
}

template <class Impl>
void
FullO3CPU<Impl>::insertThread(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Initializing thread into CPU");
    // Will change now that the PC and thread state is internal to the CPU
    // and not in the ThreadContext.
    ThreadContext *src_tc;
    if (FullSystem)
        src_tc = system->threadContexts[tid];
    else
        src_tc = tcBase(tid);

    //Bind Int Regs to Rename Map
    for (int ireg = 0; ireg < TheISA::NumIntRegs; ireg++) {
        PhysRegIndex phys_reg = freeList.getIntReg();

        renameMap[tid].setEntry(ireg,phys_reg);
        scoreboard.setReg(phys_reg);
    }

    //Bind Float Regs to Rename Map
    int max_reg = TheISA::NumIntRegs + TheISA::NumFloatRegs;
    for (int freg = TheISA::NumIntRegs; freg < max_reg; freg++) {
        PhysRegIndex phys_reg = freeList.getFloatReg();

        renameMap[tid].setEntry(freg,phys_reg);
        scoreboard.setReg(phys_reg);
    }

    //Bind condition-code Regs to Rename Map
    max_reg = TheISA::NumIntRegs + TheISA::NumFloatRegs + TheISA::NumCCRegs;
    for (int creg = TheISA::NumIntRegs + TheISA::NumFloatRegs;
         creg < max_reg; creg++) {
        PhysRegIndex phys_reg = freeList.getCCReg();

        renameMap[tid].setEntry(creg,phys_reg);
        scoreboard.setReg(phys_reg);
    }

    //Copy Thread Data Into RegFile
    //this->copyFromTC(tid);

    //Set PC/NPC/NNPC
    pcState(src_tc->pcState(), tid);

    src_tc->setStatus(ThreadContext::Active);

    activateContext(tid, Cycles(1));

    //Reset ROB/IQ/LSQ Entries
    commit.rob->resetEntries();
    iew.resetEntries();
}

template <class Impl>
void
FullO3CPU<Impl>::removeThread(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Removing thread context from CPU.\n", tid);

    // Copy Thread Data From RegFile
    // If thread is suspended, it might be re-allocated
    // this->copyToTC(tid);


    // @todo: 2-27-2008: Fix how we free up rename mappings
    // here to alleviate the case for double-freeing registers
    // in SMT workloads.

    // Unbind Int Regs from Rename Map
    for (int ireg = 0; ireg < TheISA::NumIntRegs; ireg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(ireg);

        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Unbind Float Regs from Rename Map
    int max_reg = TheISA::NumIntRegs + TheISA::NumFloatRegs;
    for (int freg = TheISA::NumIntRegs; freg < max_reg; freg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(freg);

        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Unbind condition-code Regs from Rename Map
    max_reg = TheISA::NumIntRegs + TheISA::NumFloatRegs + TheISA::NumCCRegs;
    for (int creg = TheISA::NumIntRegs + TheISA::NumFloatRegs;
         creg < max_reg; creg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(creg);

        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Squash Throughout Pipeline
    DynInstPtr inst = commit.rob->readHeadInst(tid);
    InstSeqNum squash_seq_num = inst->seqNum;
    fetch.squash(0, squash_seq_num, inst, tid);
    decode.squash(tid);
    rename.squash(squash_seq_num, tid);
    iew.squash(tid);
    iew.ldstQueue.squash(squash_seq_num, tid);
    commit.rob->squash(squash_seq_num, tid);


    assert(iew.instQueue.getCount(tid) == 0);
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
FullO3CPU<Impl>::activateWhenReady(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i]: Checking if resources are available for incoming"
            "(e.g. PhysRegs/ROB/IQ/LSQ) \n",
            tid);

    bool ready = true;

    // Should these all be '<' not '>='?  This seems backwards...
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
    } else if (freeList.numFreeCCRegs() >= TheISA::NumCCRegs) {
        DPRINTF(O3CPU,"[tid:%i] Suspending thread due to not enough "
                "Phys. CC. Regs.\n",
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

template <class Impl>
Fault
FullO3CPU<Impl>::hwrei(ThreadID tid)
{
#if THE_ISA == ALPHA_ISA
    // Need to clear the lock flag upon returning from an interrupt.
    this->setMiscRegNoEffect(AlphaISA::MISCREG_LOCKFLAG, false, tid);

    this->thread[tid]->kernelStats->hwrei();

    // FIXME: XXX check for interrupts? XXX
#endif
    return NoFault;
}

template <class Impl>
bool
FullO3CPU<Impl>::simPalCheck(int palFunc, ThreadID tid)
{
#if THE_ISA == ALPHA_ISA
    if (this->thread[tid]->kernelStats)
        this->thread[tid]->kernelStats->callpal(palFunc,
                                                this->threadContexts[tid]);

    switch (palFunc) {
      case PAL::halt:
        halt();
        if (--System::numSystemsRunning == 0)
            exitSimLoop("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (this->system->breakpoint())
            return false;
        break;
    }
#endif
    return true;
}

template <class Impl>
Fault
FullO3CPU<Impl>::getInterrupts()
{
    // Check if there are any outstanding interrupts
    return this->interrupts->getInterrupt(this->threadContexts[0]);
}

template <class Impl>
void
FullO3CPU<Impl>::processInterrupts(Fault interrupt)
{
    // Check for interrupts here.  For now can copy the code that
    // exists within isa_fullsys_traits.hh.  Also assume that thread 0
    // is the one that handles the interrupts.
    // @todo: Possibly consolidate the interrupt checking code.
    // @todo: Allow other threads to handle interrupts.

    assert(interrupt != NoFault);
    this->interrupts->updateIntrInfo(this->threadContexts[0]);

    DPRINTF(O3CPU, "Interrupt %s being handled\n", interrupt->name());
    this->trap(interrupt, 0, NULL);
}

template <class Impl>
void
FullO3CPU<Impl>::trap(Fault fault, ThreadID tid, StaticInstPtr inst)
{
    // Pass the thread's TC into the invoke method.
    fault->invoke(this->threadContexts[tid], inst);
}

template <class Impl>
void
FullO3CPU<Impl>::syscall(int64_t callnum, ThreadID tid)
{
    DPRINTF(O3CPU, "[tid:%i] Executing syscall().\n\n", tid);

    DPRINTF(Activity,"Activity: syscall() called.\n");

    // Temporarily increase this by one to account for the syscall
    // instruction.
    ++(this->thread[tid]->funcExeInst);

    // Execute the actual syscall.
    this->thread[tid]->syscall(callnum);

    // Decrease funcExeInst by one as the normal commit will handle
    // incrementing it.
    --(this->thread[tid]->funcExeInst);
}

template <class Impl>
void
FullO3CPU<Impl>::serializeThread(std::ostream &os, ThreadID tid)
{
    thread[tid]->serialize(os);
}

template <class Impl>
void
FullO3CPU<Impl>::unserializeThread(Checkpoint *cp, const std::string &section,
                                   ThreadID tid)
{
    thread[tid]->unserialize(cp, section);
}

template <class Impl>
unsigned int
FullO3CPU<Impl>::drain(DrainManager *drain_manager)
{
    // If the CPU isn't doing anything, then return immediately.
    if (switchedOut()) {
        setDrainState(Drainable::Drained);
        return 0;
    }

    DPRINTF(Drain, "Draining...\n");
    setDrainState(Drainable::Draining);

    // We only need to signal a drain to the commit stage as this
    // initiates squashing controls the draining. Once the commit
    // stage commits an instruction where it is safe to stop, it'll
    // squash the rest of the instructions in the pipeline and force
    // the fetch stage to stall. The pipeline will be drained once all
    // in-flight instructions have retired.
    commit.drain();

    // Wake the CPU and record activity so everything can drain out if
    // the CPU was not able to immediately drain.
    if (!isDrained())  {
        drainManager = drain_manager;

        wakeCPU();
        activityRec.activity();

        DPRINTF(Drain, "CPU not drained\n");

        return 1;
    } else {
        setDrainState(Drainable::Drained);
        DPRINTF(Drain, "CPU is already drained\n");
        if (tickEvent.scheduled())
            deschedule(tickEvent);

        // Flush out any old data from the time buffers.  In
        // particular, there might be some data in flight from the
        // fetch stage that isn't visible in any of the CPU buffers we
        // test in isDrained().
        for (int i = 0; i < timeBuffer.getSize(); ++i) {
            timeBuffer.advance();
            fetchQueue.advance();
            decodeQueue.advance();
            renameQueue.advance();
            iewQueue.advance();
        }

        drainSanityCheck();
        return 0;
    }
}

template <class Impl>
bool
FullO3CPU<Impl>::tryDrain()
{
    if (!drainManager || !isDrained())
        return false;

    if (tickEvent.scheduled())
        deschedule(tickEvent);

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    drainManager->signalDrainDone();
    drainManager = NULL;

    return true;
}

template <class Impl>
void
FullO3CPU<Impl>::drainSanityCheck() const
{
    assert(isDrained());
    fetch.drainSanityCheck();
    decode.drainSanityCheck();
    rename.drainSanityCheck();
    iew.drainSanityCheck();
    commit.drainSanityCheck();
}

template <class Impl>
bool
FullO3CPU<Impl>::isDrained() const
{
    bool drained(true);

    for (ThreadID i = 0; i < thread.size(); ++i) {
        if (activateThreadEvent[i].scheduled()) {
            DPRINTF(Drain, "CPU not drained, tread %i has a "
                    "pending activate event\n", i);
            drained = false;
        }
        if (deallocateContextEvent[i].scheduled()) {
            DPRINTF(Drain, "CPU not drained, tread %i has a "
                    "pending deallocate context event\n", i);
            drained = false;
        }
    }

    if (!instList.empty() || !removeList.empty()) {
        DPRINTF(Drain, "Main CPU structures not drained.\n");
        drained = false;
    }

    if (!fetch.isDrained()) {
        DPRINTF(Drain, "Fetch not drained.\n");
        drained = false;
    }

    if (!decode.isDrained()) {
        DPRINTF(Drain, "Decode not drained.\n");
        drained = false;
    }

    if (!rename.isDrained()) {
        DPRINTF(Drain, "Rename not drained.\n");
        drained = false;
    }

    if (!iew.isDrained()) {
        DPRINTF(Drain, "IEW not drained.\n");
        drained = false;
    }

    if (!commit.isDrained()) {
        DPRINTF(Drain, "Commit not drained.\n");
        drained = false;
    }

    return drained;
}

template <class Impl>
void
FullO3CPU<Impl>::commitDrained(ThreadID tid)
{
    fetch.drainStall(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::drainResume()
{
    setDrainState(Drainable::Running);
    if (switchedOut())
        return;

    DPRINTF(Drain, "Resuming...\n");
    verifyMemoryMode();

    fetch.drainResume();
    commit.drainResume();

    _status = Idle;
    for (ThreadID i = 0; i < thread.size(); i++) {
        if (thread[i]->status() == ThreadContext::Active) {
            DPRINTF(Drain, "Activating thread: %i\n", i);
            activateThread(i);
            _status = Running;
        }
    }

    assert(!tickEvent.scheduled());
    if (_status == Running)
        schedule(tickEvent, nextCycle());
}

template <class Impl>
void
FullO3CPU<Impl>::switchOut()
{
    DPRINTF(O3CPU, "Switching out\n");
    BaseCPU::switchOut();

    activityRec.reset();

    _status = SwitchedOut;

    if (checker)
        checker->switchOut();
}

template <class Impl>
void
FullO3CPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    fetch.takeOverFrom();
    decode.takeOverFrom();
    rename.takeOverFrom();
    iew.takeOverFrom();
    commit.takeOverFrom();

    assert(!tickEvent.scheduled());

    FullO3CPU<Impl> *oldO3CPU = dynamic_cast<FullO3CPU<Impl>*>(oldCPU);
    if (oldO3CPU)
        globalSeqNum = oldO3CPU->globalSeqNum;

    lastRunningCycle = curCycle();
    _status = Idle;
}

template <class Impl>
void
FullO3CPU<Impl>::verifyMemoryMode() const
{
    if (!system->isTimingMode()) {
        fatal("The O3 CPU requires the memory system to be in "
              "'timing' mode.\n");
    }
}

template <class Impl>
TheISA::MiscReg
FullO3CPU<Impl>::readMiscRegNoEffect(int misc_reg, ThreadID tid)
{
    return this->isa[tid]->readMiscRegNoEffect(misc_reg);
}

template <class Impl>
TheISA::MiscReg
FullO3CPU<Impl>::readMiscReg(int misc_reg, ThreadID tid)
{
    miscRegfileReads++;
    return this->isa[tid]->readMiscReg(misc_reg, tcBase(tid));
}

template <class Impl>
void
FullO3CPU<Impl>::setMiscRegNoEffect(int misc_reg,
        const TheISA::MiscReg &val, ThreadID tid)
{
    this->isa[tid]->setMiscRegNoEffect(misc_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setMiscReg(int misc_reg,
        const TheISA::MiscReg &val, ThreadID tid)
{
    miscRegfileWrites++;
    this->isa[tid]->setMiscReg(misc_reg, val, tcBase(tid));
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readIntReg(int reg_idx)
{
    intRegfileReads++;
    return regFile.readIntReg(reg_idx);
}

template <class Impl>
FloatReg
FullO3CPU<Impl>::readFloatReg(int reg_idx)
{
    fpRegfileReads++;
    return regFile.readFloatReg(reg_idx);
}

template <class Impl>
FloatRegBits
FullO3CPU<Impl>::readFloatRegBits(int reg_idx)
{
    fpRegfileReads++;
    return regFile.readFloatRegBits(reg_idx);
}

template <class Impl>
CCReg
FullO3CPU<Impl>::readCCReg(int reg_idx)
{
    ccRegfileReads++;
    return regFile.readCCReg(reg_idx);
}

template <class Impl>
void
FullO3CPU<Impl>::setIntReg(int reg_idx, uint64_t val)
{
    intRegfileWrites++;
    regFile.setIntReg(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatReg(int reg_idx, FloatReg val)
{
    fpRegfileWrites++;
    regFile.setFloatReg(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegBits(int reg_idx, FloatRegBits val)
{
    fpRegfileWrites++;
    regFile.setFloatRegBits(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setCCReg(int reg_idx, CCReg val)
{
    ccRegfileWrites++;
    regFile.setCCReg(reg_idx, val);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchIntReg(int reg_idx, ThreadID tid)
{
    intRegfileReads++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupInt(reg_idx);

    return regFile.readIntReg(phys_reg);
}

template <class Impl>
float
FullO3CPU<Impl>::readArchFloatReg(int reg_idx, ThreadID tid)
{
    fpRegfileReads++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupFloat(reg_idx);

    return regFile.readFloatReg(phys_reg);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchFloatRegInt(int reg_idx, ThreadID tid)
{
    fpRegfileReads++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupFloat(reg_idx);

    return regFile.readFloatRegBits(phys_reg);
}

template <class Impl>
CCReg
FullO3CPU<Impl>::readArchCCReg(int reg_idx, ThreadID tid)
{
    ccRegfileReads++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupCC(reg_idx);

    return regFile.readCCReg(phys_reg);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchIntReg(int reg_idx, uint64_t val, ThreadID tid)
{
    intRegfileWrites++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupInt(reg_idx);

    regFile.setIntReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatReg(int reg_idx, float val, ThreadID tid)
{
    fpRegfileWrites++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupFloat(reg_idx);

    regFile.setFloatReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatRegInt(int reg_idx, uint64_t val, ThreadID tid)
{
    fpRegfileWrites++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupFloat(reg_idx);

    regFile.setFloatRegBits(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchCCReg(int reg_idx, CCReg val, ThreadID tid)
{
    ccRegfileWrites++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupCC(reg_idx);

    regFile.setCCReg(phys_reg, val);
}

template <class Impl>
TheISA::PCState
FullO3CPU<Impl>::pcState(ThreadID tid)
{
    return commit.pcState(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::pcState(const TheISA::PCState &val, ThreadID tid)
{
    commit.pcState(val, tid);
}

template <class Impl>
Addr
FullO3CPU<Impl>::instAddr(ThreadID tid)
{
    return commit.instAddr(tid);
}

template <class Impl>
Addr
FullO3CPU<Impl>::nextInstAddr(ThreadID tid)
{
    return commit.nextInstAddr(tid);
}

template <class Impl>
MicroPC
FullO3CPU<Impl>::microPC(ThreadID tid)
{
    return commit.microPC(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::squashFromTC(ThreadID tid)
{
    this->thread[tid]->noSquashFromTC = true;
    this->commit.generateTCEvent(tid);
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
FullO3CPU<Impl>::instDone(ThreadID tid, DynInstPtr &inst)
{
    // Keep an instruction count.
    if (!inst->isMicroop() || inst->isLastMicroop()) {
        thread[tid]->numInst++;
        thread[tid]->numInsts++;
        committedInsts[tid]++;
        totalCommittedInsts++;
    }
    thread[tid]->numOp++;
    thread[tid]->numOps++;
    committedOps[tid]++;

    system->totalNumInsts++;
    // Check for instruction-count-based events.
    comInstEventQueue[tid]->serviceEvents(thread[tid]->numInst);
    system->instEventQueue.serviceEvents(system->totalNumInsts);
}

template <class Impl>
void
FullO3CPU<Impl>::removeFrontInst(DynInstPtr &inst)
{
    DPRINTF(O3CPU, "Removing committed instruction [tid:%i] PC %s "
            "[sn:%lli]\n",
            inst->threadNumber, inst->pcState(), inst->seqNum);

    removeInstsThisCycle = true;

    // Remove the front instruction.
    removeList.push(inst->getInstListIt());
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsNotInROB(ThreadID tid)
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
FullO3CPU<Impl>::removeInstsUntil(const InstSeqNum &seq_num, ThreadID tid)
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
FullO3CPU<Impl>::squashInstIt(const ListIt &instIt, ThreadID tid)
{
    if ((*instIt)->threadNumber == tid) {
        DPRINTF(O3CPU, "Squashing instruction, "
                "[tid:%i] [sn:%lli] PC %s\n",
                (*instIt)->threadNumber,
                (*instIt)->seqNum,
                (*instIt)->pcState());

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
                "[tid:%i] [sn:%lli] PC %s\n",
                (*removeList.front())->threadNumber,
                (*removeList.front())->seqNum,
                (*removeList.front())->pcState());

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
                num, (*inst_list_it)->instAddr(), (*inst_list_it)->threadNumber,
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

    Cycles cycles(curCycle() - lastRunningCycle);
    // @todo: This is an oddity that is only here to match the stats
    if (cycles != 0)
        --cycles;
    idleCycles += cycles;
    numCycles += cycles;

    schedule(tickEvent, clockEdge());
}

template <class Impl>
void
FullO3CPU<Impl>::wakeup()
{
    if (this->thread[0]->status() != ThreadContext::Suspended)
        return;

    this->wakeCPU();

    DPRINTF(Quiesce, "Suspended Processor woken\n");
    this->threadContexts[0]->activate();
}

template <class Impl>
ThreadID
FullO3CPU<Impl>::getFreeTid()
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (!tids[tid]) {
            tids[tid] = true;
            return tid;
        }
    }

    return InvalidThreadID;
}

template <class Impl>
void
FullO3CPU<Impl>::doContextSwitch()
{
    if (contextSwitch) {

        //ADD CODE TO DEACTIVE THREAD HERE (???)

        ThreadID size = cpuWaitList.size();
        for (ThreadID tid = 0; tid < size; tid++) {
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
    if (activeThreads.size() > 1) {
        //DEFAULT TO ROUND ROBIN SCHEME
        //e.g. Move highest priority to end of thread list
        list<ThreadID>::iterator list_begin = activeThreads.begin();

        unsigned high_thread = *list_begin;

        activeThreads.erase(list_begin);

        activeThreads.push_back(high_thread);
    }
}

// Forward declaration of FullO3CPU.
template class FullO3CPU<O3CPUImpl>;
