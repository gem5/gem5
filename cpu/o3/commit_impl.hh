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

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <stdio.h>
#include <string.h>

#include "base/loader/symtab.hh"
#include "base/timebuf.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/exetrace.hh"
#include "cpu/o3/commit.hh"
#include "cpu/o3/thread_state.hh"

using namespace std;

template <class Impl>
DefaultCommit<Impl>::TrapEvent::TrapEvent(DefaultCommit<Impl> *_commit,
                                          unsigned _tid)
    : Event(&mainEventQueue, CPU_Tick_Pri), commit(_commit), tid(_tid)
{
    this->setFlags(Event::AutoDelete);
}

template <class Impl>
void
DefaultCommit<Impl>::TrapEvent::process()
{
    // This will get reset by commit if it was switched out at the
    // time of this event processing.
    commit->trapSquash[tid] = true;
}

template <class Impl>
const char *
DefaultCommit<Impl>::TrapEvent::description()
{
    return "Trap event";
}

template <class Impl>
DefaultCommit<Impl>::DefaultCommit(Params *params)
    : dcacheInterface(params->dcacheInterface),
      squashCounter(0),
      iewToCommitDelay(params->iewToCommitDelay),
      commitToIEWDelay(params->commitToIEWDelay),
      renameToROBDelay(params->renameToROBDelay),
      fetchToCommitDelay(params->commitToFetchDelay),
      renameWidth(params->renameWidth),
      iewWidth(params->executeWidth),
      commitWidth(params->commitWidth),
      numThreads(params->numberOfThreads),
      switchedOut(false),
      trapLatency(params->trapLatency),
      fetchTrapLatency(params->fetchTrapLatency)
{
    _status = Active;
    _nextStatus = Inactive;
    string policy = params->smtCommitPolicy;

    //Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    //Assign commit policy
    if (policy == "aggressive"){
        commitPolicy = Aggressive;

        DPRINTF(Commit,"Commit Policy set to Aggressive.");
    } else if (policy == "roundrobin"){
        commitPolicy = RoundRobin;

        //Set-Up Priority List
        for (int tid=0; tid < numThreads; tid++) {
            priority_list.push_back(tid);
        }

        DPRINTF(Commit,"Commit Policy set to Round Robin.");
    } else if (policy == "oldestready"){
        commitPolicy = OldestReady;

        DPRINTF(Commit,"Commit Policy set to Oldest Ready.");
    } else {
        assert(0 && "Invalid SMT Commit Policy. Options Are: {Aggressive,"
               "RoundRobin,OldestReady}");
    }

    for (int i=0; i < numThreads; i++) {
        commitStatus[i] = Idle;
        changedROBNumEntries[i] = false;
        trapSquash[i] = false;
        xcSquash[i] = false;
    }

    fetchFaultTick = 0;
    fetchTrapWait = 0;
}

template <class Impl>
std::string
DefaultCommit<Impl>::name() const
{
    return cpu->name() + ".commit";
}

template <class Impl>
void
DefaultCommit<Impl>::regStats()
{
    using namespace Stats;
    commitCommittedInsts
        .name(name() + ".commitCommittedInsts")
        .desc("The number of committed instructions")
        .prereq(commitCommittedInsts);
    commitSquashedInsts
        .name(name() + ".commitSquashedInsts")
        .desc("The number of squashed insts skipped by commit")
        .prereq(commitSquashedInsts);
    commitSquashEvents
        .name(name() + ".commitSquashEvents")
        .desc("The number of times commit is told to squash")
        .prereq(commitSquashEvents);
    commitNonSpecStalls
        .name(name() + ".commitNonSpecStalls")
        .desc("The number of times commit has been forced to stall to "
              "communicate backwards")
        .prereq(commitNonSpecStalls);
    branchMispredicts
        .name(name() + ".branchMispredicts")
        .desc("The number of times a branch was mispredicted")
        .prereq(branchMispredicts);
    numCommittedDist
        .init(0,commitWidth,1)
        .name(name() + ".COM:committed_per_cycle")
        .desc("Number of insts commited each cycle")
        .flags(Stats::pdf)
        ;

    statComInst
        .init(cpu->number_of_threads)
        .name(name() + ".COM:count")
        .desc("Number of instructions committed")
        .flags(total)
        ;

    statComSwp
        .init(cpu->number_of_threads)
        .name(name() + ".COM:swp_count")
        .desc("Number of s/w prefetches committed")
        .flags(total)
        ;

    statComRefs
        .init(cpu->number_of_threads)
        .name(name() +  ".COM:refs")
        .desc("Number of memory references committed")
        .flags(total)
        ;

    statComLoads
        .init(cpu->number_of_threads)
        .name(name() +  ".COM:loads")
        .desc("Number of loads committed")
        .flags(total)
        ;

    statComMembars
        .init(cpu->number_of_threads)
        .name(name() +  ".COM:membars")
        .desc("Number of memory barriers committed")
        .flags(total)
        ;

    statComBranches
        .init(cpu->number_of_threads)
        .name(name() + ".COM:branches")
        .desc("Number of branches committed")
        .flags(total)
        ;

    //
    //  Commit-Eligible instructions...
    //
    //  -> The number of instructions eligible to commit in those
    //  cycles where we reached our commit BW limit (less the number
    //  actually committed)
    //
    //  -> The average value is computed over ALL CYCLES... not just
    //  the BW limited cycles
    //
    //  -> The standard deviation is computed only over cycles where
    //  we reached the BW limit
    //
    commitEligible
        .init(cpu->number_of_threads)
        .name(name() + ".COM:bw_limited")
        .desc("number of insts not committed due to BW limits")
        .flags(total)
        ;

    commitEligibleSamples
        .name(name() + ".COM:bw_lim_events")
        .desc("number cycles where commit BW limit reached")
        ;
}

template <class Impl>
void
DefaultCommit<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(Commit, "Commit: Setting CPU pointer.\n");
    cpu = cpu_ptr;

    // Commit must broadcast the number of free entries it has at the start of
    // the simulation, so it starts as active.
    cpu->activateStage(FullCPU::CommitIdx);

    trapLatency = cpu->cycles(trapLatency);
    fetchTrapLatency = cpu->cycles(fetchTrapLatency);
}

template <class Impl>
void
DefaultCommit<Impl>::setThreads(vector<Thread *> &threads)
{
    thread = threads;
}

template <class Impl>
void
DefaultCommit<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(Commit, "Commit: Setting time buffer pointer.\n");
    timeBuffer = tb_ptr;

    // Setup wire to send information back to IEW.
    toIEW = timeBuffer->getWire(0);

    // Setup wire to read data from IEW (for the ROB).
    robInfoFromIEW = timeBuffer->getWire(-iewToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    DPRINTF(Commit, "Commit: Setting fetch queue pointer.\n");
    fetchQueue = fq_ptr;

    // Setup wire to get instructions from rename (for the ROB).
    fromFetch = fetchQueue->getWire(-fetchToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    DPRINTF(Commit, "Commit: Setting rename queue pointer.\n");
    renameQueue = rq_ptr;

    // Setup wire to get instructions from rename (for the ROB).
    fromRename = renameQueue->getWire(-renameToROBDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr)
{
    DPRINTF(Commit, "Commit: Setting IEW queue pointer.\n");
    iewQueue = iq_ptr;

    // Setup wire to get instructions from IEW.
    fromIEW = iewQueue->getWire(-iewToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setFetchStage(Fetch *fetch_stage)
{
    fetchStage = fetch_stage;
}

template <class Impl>
void
DefaultCommit<Impl>::setIEWStage(IEW *iew_stage)
{
    iewStage = iew_stage;
}

template<class Impl>
void
DefaultCommit<Impl>::setActiveThreads(list<unsigned> *at_ptr)
{
    DPRINTF(Commit, "Commit: Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}

template <class Impl>
void
DefaultCommit<Impl>::setRenameMap(RenameMap rm_ptr[])
{
    DPRINTF(Commit, "Setting rename map pointers.\n");

    for (int i=0; i < numThreads; i++) {
        renameMap[i] = &rm_ptr[i];
    }
}

template <class Impl>
void
DefaultCommit<Impl>::setROB(ROB *rob_ptr)
{
    DPRINTF(Commit, "Commit: Setting ROB pointer.\n");
    rob = rob_ptr;
}

template <class Impl>
void
DefaultCommit<Impl>::initStage()
{
    rob->setActiveThreads(activeThreads);
    rob->resetEntries();

    // Broadcast the number of free entries.
    for (int i=0; i < numThreads; i++) {
        toIEW->commitInfo[i].usedROB = true;
        toIEW->commitInfo[i].freeROBEntries = rob->numFreeEntries(i);
    }

    cpu->activityThisCycle();
}

template <class Impl>
void
DefaultCommit<Impl>::switchOut()
{
    switchPending = true;
}

template <class Impl>
void
DefaultCommit<Impl>::doSwitchOut()
{
    switchedOut = true;
    switchPending = false;
    rob->switchOut();
}

template <class Impl>
void
DefaultCommit<Impl>::takeOverFrom()
{
    switchedOut = false;
    _status = Active;
    _nextStatus = Inactive;
    for (int i=0; i < numThreads; i++) {
        commitStatus[i] = Idle;
        changedROBNumEntries[i] = false;
        trapSquash[i] = false;
        xcSquash[i] = false;
    }
    squashCounter = 0;
    rob->takeOverFrom();
}

template <class Impl>
void
DefaultCommit<Impl>::updateStatus()
{
    // reset ROB changed variable
    list<unsigned>::iterator threads = (*activeThreads).begin();
    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;
        changedROBNumEntries[tid] = false;

        // Also check if any of the threads has a trap pending
        if (commitStatus[tid] == TrapPending ||
            commitStatus[tid] == FetchTrapPending) {
            _nextStatus = Active;
        }
    }

    if (_nextStatus == Inactive && _status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");
        cpu->deactivateStage(FullCPU::CommitIdx);
    } else if (_nextStatus == Active && _status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");
        cpu->activateStage(FullCPU::CommitIdx);
    }

    _status = _nextStatus;
}

template <class Impl>
void
DefaultCommit<Impl>::setNextStatus()
{
    int squashes = 0;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (commitStatus[tid] == ROBSquashing) {
            squashes++;
        }
    }

    assert(squashes == squashCounter);

    // If commit is currently squashing, then it will have activity for the
    // next cycle. Set its next status as active.
    if (squashCounter) {
        _nextStatus = Active;
    }
}

template <class Impl>
bool
DefaultCommit<Impl>::changedROBEntries()
{
    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (changedROBNumEntries[tid]) {
            return true;
        }
    }

    return false;
}

template <class Impl>
unsigned
DefaultCommit<Impl>::numROBFreeEntries(unsigned tid)
{
    return rob->numFreeEntries(tid);
}

template <class Impl>
void
DefaultCommit<Impl>::generateTrapEvent(unsigned tid)
{
    DPRINTF(Commit, "Generating trap event for [tid:%i]\n", tid);

    TrapEvent *trap = new TrapEvent(this, tid);

    trap->schedule(curTick + trapLatency);

    thread[tid]->trapPending = true;
}

template <class Impl>
void
DefaultCommit<Impl>::generateXCEvent(unsigned tid)
{
    DPRINTF(Commit, "Generating XC squash event for [tid:%i]\n", tid);

    xcSquash[tid] = true;
}

template <class Impl>
void
DefaultCommit<Impl>::squashAll(unsigned tid)
{
    // If we want to include the squashing instruction in the squash,
    // then use one older sequence number.
    // Hopefully this doesn't mess things up.  Basically I want to squash
    // all instructions of this thread.
    InstSeqNum squashed_inst = rob->isEmpty() ?
        0 : rob->readHeadInst(tid)->seqNum - 1;;

    // All younger instructions will be squashed. Set the sequence
    // number as the youngest instruction in the ROB (0 in this case.
    // Hopefully nothing breaks.)
    youngestSeqNum[tid] = 0;

    rob->squash(squashed_inst, tid);
    changedROBNumEntries[tid] = true;

    // Send back the sequence number of the squashed instruction.
    toIEW->commitInfo[tid].doneSeqNum = squashed_inst;

    // Send back the squash signal to tell stages that they should
    // squash.
    toIEW->commitInfo[tid].squash = true;

    // Send back the rob squashing signal so other stages know that
    // the ROB is in the process of squashing.
    toIEW->commitInfo[tid].robSquashing = true;

    toIEW->commitInfo[tid].branchMispredict = false;

    toIEW->commitInfo[tid].nextPC = PC[tid];
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromTrap(unsigned tid)
{
    squashAll(tid);

    DPRINTF(Commit, "Squashing from trap, restarting at PC %#x\n", PC[tid]);

    thread[tid]->trapPending = false;
    thread[tid]->inSyscall = false;

    trapSquash[tid] = false;

    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();

    ++squashCounter;
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromXC(unsigned tid)
{
    squashAll(tid);

    DPRINTF(Commit, "Squashing from XC, restarting at PC %#x\n", PC[tid]);

    thread[tid]->inSyscall = false;
    assert(!thread[tid]->trapPending);

    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();

    xcSquash[tid] = false;

    ++squashCounter;
}

template <class Impl>
void
DefaultCommit<Impl>::tick()
{
    wroteToTimeBuffer = false;
    _nextStatus = Inactive;

    if (switchPending && rob->isEmpty() && !iewStage->hasStoresToWB()) {
        cpu->signalSwitched();
        return;
    }

    list<unsigned>::iterator threads = (*activeThreads).begin();

    // Check if any of the threads are done squashing.  Change the
    // status if they are done.
    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (commitStatus[tid] == ROBSquashing) {

            if (rob->isDoneSquashing(tid)) {
                commitStatus[tid] = Running;
                --squashCounter;
            } else {
                DPRINTF(Commit,"[tid:%u]: Still Squashing, cannot commit any"
                        "insts this cycle.\n", tid);
            }
        }
    }

    commit();

    markCompletedInsts();

    threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (!rob->isEmpty(tid) && rob->readHeadInst(tid)->readyToCommit()) {
            // The ROB has more instructions it can commit. Its next status
            // will be active.
            _nextStatus = Active;

            DynInstPtr inst = rob->readHeadInst(tid);

            DPRINTF(Commit,"[tid:%i]: Instruction [sn:%lli] PC %#x is head of"
                    " ROB and ready to commit\n",
                    tid, inst->seqNum, inst->readPC());

        } else if (!rob->isEmpty(tid)) {
            DynInstPtr inst = rob->readHeadInst(tid);

            DPRINTF(Commit,"[tid:%i]: Can't commit, Instruction [sn:%lli] PC "
                    "%#x is head of ROB and not ready\n",
                    tid, inst->seqNum, inst->readPC());
        }

        DPRINTF(Commit, "[tid:%i]: ROB has %d insts & %d free entries.\n",
                tid, rob->countInsts(tid), rob->numFreeEntries(tid));
    }


    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity This Cycle.\n");
        cpu->activityThisCycle();
    }

    updateStatus();
}

template <class Impl>
void
DefaultCommit<Impl>::commit()
{

    //////////////////////////////////////
    // Check for interrupts
    //////////////////////////////////////

#if FULL_SYSTEM
    // Process interrupts if interrupts are enabled, not in PAL mode,
    // and no other traps or external squashes are currently pending.
    // @todo: Allow other threads to handle interrupts.
    if (cpu->checkInterrupts &&
        cpu->check_interrupts() &&
        !cpu->inPalMode(readPC()) &&
        !trapSquash[0] &&
        !xcSquash[0]) {
        // Tell fetch that there is an interrupt pending.  This will
        // make fetch wait until it sees a non PAL-mode PC, at which
        // point it stops fetching instructions.
        toIEW->commitInfo[0].interruptPending = true;

        // Wait until the ROB is empty and all stores have drained in
        // order to enter the interrupt.
        if (rob->isEmpty() && !iewStage->hasStoresToWB()) {
            // Not sure which thread should be the one to interrupt.  For now
            // always do thread 0.
            assert(!thread[0]->inSyscall);
            thread[0]->inSyscall = true;

            // CPU will handle implementation of the interrupt.
            cpu->processInterrupts();

            // Now squash or record that I need to squash this cycle.
            commitStatus[0] = TrapPending;

            // Exit state update mode to avoid accidental updating.
            thread[0]->inSyscall = false;

            // Generate trap squash event.
            generateTrapEvent(0);

            toIEW->commitInfo[0].clearInterrupt = true;

            DPRINTF(Commit, "Interrupt detected.\n");
        } else {
            DPRINTF(Commit, "Interrupt pending, waiting for ROB to empty.\n");
        }
    }
#endif // FULL_SYSTEM

    ////////////////////////////////////
    // Check for any possible squashes, handle them first
    ////////////////////////////////////

    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (fromFetch->fetchFault && commitStatus[0] != TrapPending) {
            // Record the fault.  Wait until it's empty in the ROB.
            // Then handle the trap.  Ignore it if there's already a
            // trap pending as fetch will be redirected.
            fetchFault = fromFetch->fetchFault;
            fetchFaultTick = curTick + fetchTrapLatency;
            commitStatus[0] = FetchTrapPending;
            DPRINTF(Commit, "Fault from fetch recorded.  Will trap if the "
                    "ROB empties without squashing the fault.\n");
            fetchTrapWait = 0;
        }

        // Fetch may tell commit to clear the trap if it's been squashed.
        if (fromFetch->clearFetchFault) {
            DPRINTF(Commit, "Received clear fetch fault signal\n");
            fetchTrapWait = 0;
            if (commitStatus[0] == FetchTrapPending) {
                DPRINTF(Commit, "Clearing fault from fetch\n");
                commitStatus[0] = Running;
            }
        }

        // Not sure which one takes priority.  I think if we have
        // both, that's a bad sign.
        if (trapSquash[tid] == true) {
            assert(!xcSquash[tid]);
            squashFromTrap(tid);
        } else if (xcSquash[tid] == true) {
            squashFromXC(tid);
        }

        // Squashed sequence number must be older than youngest valid
        // instruction in the ROB. This prevents squashes from younger
        // instructions overriding squashes from older instructions.
        if (fromIEW->squash[tid] &&
            commitStatus[tid] != TrapPending &&
            fromIEW->squashedSeqNum[tid] <= youngestSeqNum[tid]) {

            DPRINTF(Commit, "[tid:%i]: Squashing due to PC %#x [sn:%i]\n",
                    tid,
                    fromIEW->mispredPC[tid],
                    fromIEW->squashedSeqNum[tid]);

            DPRINTF(Commit, "[tid:%i]: Redirecting to PC %#x\n",
                    tid,
                    fromIEW->nextPC[tid]);

            commitStatus[tid] = ROBSquashing;

            ++squashCounter;

            // If we want to include the squashing instruction in the squash,
            // then use one older sequence number.
            InstSeqNum squashed_inst = fromIEW->squashedSeqNum[tid];

            if (fromIEW->includeSquashInst[tid] == true)
                squashed_inst--;

            // All younger instructions will be squashed. Set the sequence
            // number as the youngest instruction in the ROB.
            youngestSeqNum[tid] = squashed_inst;

            rob->squash(squashed_inst, tid);
            changedROBNumEntries[tid] = true;

            toIEW->commitInfo[tid].doneSeqNum = squashed_inst;

            toIEW->commitInfo[tid].squash = true;

            // Send back the rob squashing signal so other stages know that
            // the ROB is in the process of squashing.
            toIEW->commitInfo[tid].robSquashing = true;

            toIEW->commitInfo[tid].branchMispredict =
                fromIEW->branchMispredict[tid];

            toIEW->commitInfo[tid].branchTaken =
                fromIEW->branchTaken[tid];

            toIEW->commitInfo[tid].nextPC = fromIEW->nextPC[tid];

            toIEW->commitInfo[tid].mispredPC = fromIEW->mispredPC[tid];

            if (toIEW->commitInfo[tid].branchMispredict) {
                ++branchMispredicts;
            }
        }

    }

    setNextStatus();

    if (squashCounter != numThreads) {
        // If we're not currently squashing, then get instructions.
        getInsts();

        // Try to commit any instructions.
        commitInsts();
    }

    //Check for any activity
    threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (changedROBNumEntries[tid]) {
            toIEW->commitInfo[tid].usedROB = true;
            toIEW->commitInfo[tid].freeROBEntries = rob->numFreeEntries(tid);

            if (rob->isEmpty(tid)) {
                toIEW->commitInfo[tid].emptyROB = true;
            }

            wroteToTimeBuffer = true;
            changedROBNumEntries[tid] = false;
        }
    }
}

template <class Impl>
void
DefaultCommit<Impl>::commitInsts()
{
    ////////////////////////////////////
    // Handle commit
    // Note that commit will be handled prior to putting new
    // instructions in the ROB so that the ROB only tries to commit
    // instructions it has in this current cycle, and not instructions
    // it is writing in during this cycle.  Can't commit and squash
    // things at the same time...
    ////////////////////////////////////

    DPRINTF(Commit, "Trying to commit instructions in the ROB.\n");

    unsigned num_committed = 0;

    DynInstPtr head_inst;
#if FULL_SYSTEM
    // Not the best way to check if the front end is empty, but it should
    // work.
    // @todo: Try to avoid directly accessing fetch.
    if (commitStatus[0] == FetchTrapPending && rob->isEmpty()) {
        DPRINTF(Commit, "Fault from fetch is pending.\n");

        fetchTrapWait++;
        if (fetchTrapWait > 10000000) {
            panic("Fetch trap has been pending for a long time!");
        }
        if (fetchFaultTick > curTick) {
            DPRINTF(Commit, "Not enough cycles since fault, fault will "
                    "happen on %lli\n",
                    fetchFaultTick);
            cpu->activityThisCycle();
            return;
        } else if (iewStage->hasStoresToWB()) {
            DPRINTF(Commit, "IEW still has stores to WB.  Waiting until "
                    "they are completed. fetchTrapWait:%i\n",
                    fetchTrapWait);
            cpu->activityThisCycle();
            return;
        } else if (cpu->inPalMode(readPC())) {
            DPRINTF(Commit, "In pal mode right now. fetchTrapWait:%i\n",
                    fetchTrapWait);
            return;
        } else if (fetchStage->getYoungestSN() > youngestSeqNum[0]) {
            DPRINTF(Commit, "Waiting for front end to drain. fetchTrapWait:%i\n",
                    fetchTrapWait);
            return;
        }
        fetchTrapWait = 0;
        DPRINTF(Commit, "ROB is empty, handling fetch trap.\n");

        assert(!thread[0]->inSyscall);

        thread[0]->inSyscall = true;

        // Consider holding onto the trap and waiting until the trap event
        // happens for this to be executed.
        cpu->trap(fetchFault, 0);

        // Exit state update mode to avoid accidental updating.
        thread[0]->inSyscall = false;

        commitStatus[0] = TrapPending;
        // Set it up so that we squash next cycle
        trapSquash[0] = true;
        return;
    }
#endif

    // Commit as many instructions as possible until the commit bandwidth
    // limit is reached, or it becomes impossible to commit any more.
    while (num_committed < commitWidth) {
        int commit_thread = getCommittingThread();

        if (commit_thread == -1 || !rob->isHeadReady(commit_thread))
            break;

        head_inst = rob->readHeadInst(commit_thread);

        int tid = head_inst->threadNumber;

        assert(tid == commit_thread);

        DPRINTF(Commit, "Trying to commit head instruction, [sn:%i] [tid:%i]\n",
                head_inst->seqNum, tid);

        // If the head instruction is squashed, it is ready to retire
        // (be removed from the ROB) at any time.
        if (head_inst->isSquashed()) {

            DPRINTF(Commit, "Retiring squashed instruction from "
                    "ROB.\n");

            rob->retireHead(commit_thread);

            ++commitSquashedInsts;

            // Record that the number of ROB entries has changed.
            changedROBNumEntries[tid] = true;
        } else {
            PC[tid] = head_inst->readPC();
            nextPC[tid] = head_inst->readNextPC();

            // Increment the total number of non-speculative instructions
            // executed.
            // Hack for now: it really shouldn't happen until after the
            // commit is deemed to be successful, but this count is needed
            // for syscalls.
            thread[tid]->funcExeInst++;

            // Try to commit the head instruction.
            bool commit_success = commitHead(head_inst, num_committed);

            if (commit_success) {
                ++num_committed;

                changedROBNumEntries[tid] = true;

                // Set the doneSeqNum to the youngest committed instruction.
                toIEW->commitInfo[tid].doneSeqNum = head_inst->seqNum;

                ++commitCommittedInsts;

                // To match the old model, don't count nops and instruction
                // prefetches towards the total commit count.
                if (!head_inst->isNop() && !head_inst->isInstPrefetch()) {
                    cpu->instDone(tid);
                }

                PC[tid] = nextPC[tid];
                nextPC[tid] = nextPC[tid] + sizeof(TheISA::MachInst);
#if FULL_SYSTEM
                int count = 0;
                Addr oldpc;
                do {
                    // Debug statement.  Checks to make sure we're not
                    // currently updating state while handling PC events.
                    if (count == 0)
                        assert(!thread[tid]->inSyscall &&
                               !thread[tid]->trapPending);
                    oldpc = PC[tid];
                    cpu->system->pcEventQueue.service(
                        thread[tid]->getXCProxy());
                    count++;
                } while (oldpc != PC[tid]);
                if (count > 1) {
                    DPRINTF(Commit, "PC skip function event, stopping commit\n");
                    break;
                }
#endif
            } else {
                DPRINTF(Commit, "Unable to commit head instruction PC:%#x "
                        "[tid:%i] [sn:%i].\n",
                        head_inst->readPC(), tid ,head_inst->seqNum);
                break;
            }
        }
    }

    DPRINTF(CommitRate, "%i\n", num_committed);
    numCommittedDist.sample(num_committed);

    if (num_committed == commitWidth) {
        commitEligible[0]++;
    }
}

template <class Impl>
bool
DefaultCommit<Impl>::commitHead(DynInstPtr &head_inst, unsigned inst_num)
{
    assert(head_inst);

    int tid = head_inst->threadNumber;

    // If the instruction is not executed yet, then it will need extra
    // handling.  Signal backwards that it should be executed.
    if (!head_inst->isExecuted()) {
        // Keep this number correct.  We have not yet actually executed
        // and committed this instruction.
        thread[tid]->funcExeInst--;

        head_inst->reachedCommit = true;

        if (head_inst->isNonSpeculative() ||
            head_inst->isMemBarrier() ||
            head_inst->isWriteBarrier()) {

            DPRINTF(Commit, "Encountered a barrier or non-speculative "
                    "instruction [sn:%lli] at the head of the ROB, PC %#x.\n",
                    head_inst->seqNum, head_inst->readPC());

#if !FULL_SYSTEM
            // Hack to make sure syscalls/memory barriers/quiesces
            // aren't executed until all stores write back their data.
            // This direct communication shouldn't be used for
            // anything other than this.
            if (inst_num > 0 || iewStage->hasStoresToWB())
#else
            if ((head_inst->isMemBarrier() || head_inst->isWriteBarrier() ||
                    head_inst->isQuiesce()) &&
                iewStage->hasStoresToWB())
#endif
            {
                DPRINTF(Commit, "Waiting for all stores to writeback.\n");
                return false;
            }

            toIEW->commitInfo[tid].nonSpecSeqNum = head_inst->seqNum;

            // Change the instruction so it won't try to commit again until
            // it is executed.
            head_inst->clearCanCommit();

            ++commitNonSpecStalls;

            return false;
        } else if (head_inst->isLoad()) {
            DPRINTF(Commit, "[sn:%lli]: Uncached load, PC %#x.\n",
                    head_inst->seqNum, head_inst->readPC());

            // Send back the non-speculative instruction's sequence
            // number.  Tell the lsq to re-execute the load.
            toIEW->commitInfo[tid].nonSpecSeqNum = head_inst->seqNum;
            toIEW->commitInfo[tid].uncached = true;
            toIEW->commitInfo[tid].uncachedLoad = head_inst;

            head_inst->clearCanCommit();

            return false;
        } else {
            panic("Trying to commit un-executed instruction "
                  "of unknown type!\n");
        }
    }

    if (head_inst->isThreadSync()) {
        // Not handled for now.
        panic("Thread sync instructions are not handled yet.\n");
    }

    // Stores mark themselves as completed.
    if (!head_inst->isStore()) {
        head_inst->setCompleted();
    }

    // Use checker prior to updating anything due to traps or PC
    // based events.
    if (cpu->checker) {
        cpu->checker->tick(head_inst);
    }

    // Check if the instruction caused a fault.  If so, trap.
    Fault inst_fault = head_inst->getFault();

    if (inst_fault != NoFault) {
        head_inst->setCompleted();
#if FULL_SYSTEM
        DPRINTF(Commit, "Inst [sn:%lli] PC %#x has a fault\n",
                head_inst->seqNum, head_inst->readPC());

        if (iewStage->hasStoresToWB() || inst_num > 0) {
            DPRINTF(Commit, "Stores outstanding, fault must wait.\n");
            return false;
        }

        if (cpu->checker && head_inst->isStore()) {
            cpu->checker->tick(head_inst);
        }

        assert(!thread[tid]->inSyscall);

        // Mark that we're in state update mode so that the trap's
        // execution doesn't generate extra squashes.
        thread[tid]->inSyscall = true;

        // DTB will sometimes need the machine instruction for when
        // faults happen.  So we will set it here, prior to the DTB
        // possibly needing it for its fault.
        thread[tid]->setInst(
            static_cast<TheISA::MachInst>(head_inst->staticInst->machInst));

        // Execute the trap.  Although it's slightly unrealistic in
        // terms of timing (as it doesn't wait for the full timing of
        // the trap event to complete before updating state), it's
        // needed to update the state as soon as possible.  This
        // prevents external agents from changing any specific state
        // that the trap need.
        cpu->trap(inst_fault, tid);

        // Exit state update mode to avoid accidental updating.
        thread[tid]->inSyscall = false;

        commitStatus[tid] = TrapPending;

        // Generate trap squash event.
        generateTrapEvent(tid);

        return false;
#else // !FULL_SYSTEM
        panic("fault (%d) detected @ PC %08p", inst_fault,
              head_inst->PC);
#endif // FULL_SYSTEM
    }

    updateComInstStats(head_inst);

    if (head_inst->traceData) {
        head_inst->traceData->setFetchSeq(head_inst->seqNum);
        head_inst->traceData->setCPSeq(thread[tid]->numInst);
        head_inst->traceData->finalize();
        head_inst->traceData = NULL;
    }

    // Update the commit rename map
    for (int i = 0; i < head_inst->numDestRegs(); i++) {
        renameMap[tid]->setEntry(head_inst->destRegIdx(i),
                                 head_inst->renamedDestRegIdx(i));
    }

    // Finally clear the head ROB entry.
    rob->retireHead(tid);

    // Return true to indicate that we have committed an instruction.
    return true;
}

template <class Impl>
void
DefaultCommit<Impl>::getInsts()
{
    // Read any renamed instructions and place them into the ROB.
    int insts_to_process = min((int)renameWidth, fromRename->size);

    for (int inst_num = 0; inst_num < insts_to_process; ++inst_num)
    {
        DynInstPtr inst = fromRename->insts[inst_num];
        int tid = inst->threadNumber;

        if (!inst->isSquashed() &&
            commitStatus[tid] != ROBSquashing) {
            changedROBNumEntries[tid] = true;

            DPRINTF(Commit, "Inserting PC %#x [sn:%i] [tid:%i] into ROB.\n",
                    inst->readPC(), inst->seqNum, tid);

            rob->insertInst(inst);

            assert(rob->getThreadEntries(tid) <= rob->getMaxEntries(tid));

            youngestSeqNum[tid] = inst->seqNum;
        } else {
            DPRINTF(Commit, "Instruction PC %#x [sn:%i] [tid:%i] was "
                    "squashed, skipping.\n",
                    inst->readPC(), inst->seqNum, tid);
        }
    }
}

template <class Impl>
void
DefaultCommit<Impl>::markCompletedInsts()
{
    // Grab completed insts out of the IEW instruction queue, and mark
    // instructions completed within the ROB.
    for (int inst_num = 0;
         inst_num < fromIEW->size && fromIEW->insts[inst_num];
         ++inst_num)
    {
        if (!fromIEW->insts[inst_num]->isSquashed()) {
            DPRINTF(Commit, "[tid:%i]: Marking PC %#x, [sn:%lli] ready "
                    "within ROB.\n",
                    fromIEW->insts[inst_num]->threadNumber,
                    fromIEW->insts[inst_num]->readPC(),
                    fromIEW->insts[inst_num]->seqNum);

            // Mark the instruction as ready to commit.
            fromIEW->insts[inst_num]->setCanCommit();
        }
    }
}

template <class Impl>
bool
DefaultCommit<Impl>::robDoneSquashing()
{
    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (!rob->isDoneSquashing(tid))
            return false;
    }

    return true;
}

template <class Impl>
void
DefaultCommit<Impl>::updateComInstStats(DynInstPtr &inst)
{
    unsigned thread = inst->threadNumber;

    //
    //  Pick off the software prefetches
    //
#ifdef TARGET_ALPHA
    if (inst->isDataPrefetch()) {
        statComSwp[thread]++;
    } else {
        statComInst[thread]++;
    }
#else
    statComInst[thread]++;
#endif

    //
    //  Control Instructions
    //
    if (inst->isControl())
        statComBranches[thread]++;

    //
    //  Memory references
    //
    if (inst->isMemRef()) {
        statComRefs[thread]++;

        if (inst->isLoad()) {
            statComLoads[thread]++;
        }
    }

    if (inst->isMemBarrier()) {
        statComMembars[thread]++;
    }
}

////////////////////////////////////////
//                                    //
//  SMT COMMIT POLICY MAINTAINED HERE //
//                                    //
////////////////////////////////////////
template <class Impl>
int
DefaultCommit<Impl>::getCommittingThread()
{
    if (numThreads > 1) {
        switch (commitPolicy) {

          case Aggressive:
            //If Policy is Aggressive, commit will call
            //this function multiple times per
            //cycle
            return oldestReady();

          case RoundRobin:
            return roundRobin();

          case OldestReady:
            return oldestReady();

          default:
            return -1;
        }
    } else {
        int tid = (*activeThreads).front();

        if (commitStatus[tid] == Running ||
            commitStatus[tid] == Idle ||
            commitStatus[tid] == FetchTrapPending) {
            return tid;
        } else {
            return -1;
        }
    }
}

template<class Impl>
int
DefaultCommit<Impl>::roundRobin()
{
    list<unsigned>::iterator pri_iter = priority_list.begin();
    list<unsigned>::iterator end      = priority_list.end();

    while (pri_iter != end) {
        unsigned tid = *pri_iter;

        if (commitStatus[tid] == Running ||
            commitStatus[tid] == Idle) {

            if (rob->isHeadReady(tid)) {
                priority_list.erase(pri_iter);
                priority_list.push_back(tid);

                return tid;
            }
        }

        pri_iter++;
    }

    return -1;
}

template<class Impl>
int
DefaultCommit<Impl>::oldestReady()
{
    unsigned oldest = 0;
    bool first = true;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (!rob->isEmpty(tid) &&
            (commitStatus[tid] == Running ||
             commitStatus[tid] == Idle ||
             commitStatus[tid] == FetchTrapPending)) {

            if (rob->isHeadReady(tid)) {

                DynInstPtr head_inst = rob->readHeadInst(tid);

                if (first) {
                    oldest = tid;
                    first = false;
                } else if (head_inst->seqNum < oldest) {
                    oldest = tid;
                }
            }
        }
    }

    if (!first) {
        return oldest;
    } else {
        return -1;
    }
}
