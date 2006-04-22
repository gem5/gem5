/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
      numThreads(params->numberOfThreads)
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

    // Hardcoded trap latency.
    trapLatency = 6;
    fetchTrapLatency = 12;
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
    commitCommittedBranches
        .name(name() + ".commitCommittedBranches")
        .desc("The number of committed branches")
        .prereq(commitCommittedBranches);
    commitCommittedLoads
        .name(name() + ".commitCommittedLoads")
        .desc("The number of committed loads")
        .prereq(commitCommittedLoads);
    commitCommittedMemRefs
        .name(name() + ".commitCommittedMemRefs")
        .desc("The number of committed memory references")
        .prereq(commitCommittedMemRefs);
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
DefaultCommit<Impl>::updateStatus()
{
    if (commitStatus[0] == TrapPending ||
        commitStatus[0] == FetchTrapPending) {
        _nextStatus = Active;
    }

    if (_nextStatus == Inactive && _status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");
        cpu->deactivateStage(FullCPU::CommitIdx);
    } else if (_nextStatus == Active && _status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");
        cpu->activateStage(FullCPU::CommitIdx);
    }

    _status = _nextStatus;

    // reset ROB changed variable
    list<unsigned>::iterator threads = (*activeThreads).begin();
    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;
        changedROBNumEntries[tid] = false;
    }
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
DefaultCommit<Impl>::squashFromTrap(unsigned tid)
{
    // If we want to include the squashing instruction in the squash,
    // then use one older sequence number.
    // Hopefully this doesn't mess things up.  Basically I want to squash
    // all instructions of this thread.
    InstSeqNum squashed_inst = rob->isEmpty() ?
        0 : rob->readHeadInst(tid)->seqNum - 1;

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

//    toIEW->commitInfo[tid].branchTaken = fromIEW->branchTaken[tid];

    toIEW->commitInfo[tid].nextPC = PC[tid];

    DPRINTF(Commit, "Squashing from trap, restarting at PC %#x\n", PC[tid]);
    // Hopefully nobody tries to use the mispredPC becuase I said there
    // wasn't a branch mispredict.
//    toIEW->commitInfo[tid].mispredPC = fromIEW->mispredPC[tid];

    thread[tid]->trapPending = false;
    thread[tid]->inSyscall = false;

    trapSquash[tid] = false;

    // Not sure what to set this to...
    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();

    ++squashCounter;
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromXC(unsigned tid)
{
    // For now these are identical.  In the future, the squash from trap
    // might execute the trap prior to the squash.

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

//    toIEW->commitInfo[tid].branchTaken = fromIEW->branchTaken[tid];

    toIEW->commitInfo[tid].nextPC = PC[tid];

    DPRINTF(Commit, "Squashing from XC, restarting at PC %#x\n", PC[tid]);
    // Hopefully nobody tries to use the mispredPC becuase I said there
    // wasn't a branch mispredict.
//    toIEW->commitInfo[tid].mispredPC = fromIEW->mispredPC[tid];

    thread[tid]->inSyscall = false;
    assert(!thread[tid]->trapPending);
    // Not sure what to set this to...
    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();

    xcSquash[tid] = false;

    ++squashCounter;
}

template <class Impl>
void
DefaultCommit<Impl>::squashInFlightInsts(unsigned tid)
{
    // @todo: Fix this hardcoded number.
    for (int i = 0; i < -5; ++i) {
        for (int j = 0; j < (*iewQueue)[i].size; ++j) {
            DynInstPtr inst = (*iewQueue)[i].insts[j];
            if (inst->threadNumber == tid &&
                !inst->isSquashed()) {
                inst->setSquashed();
            }
        }
    }
}

template <class Impl>
void
DefaultCommit<Impl>::tick()
{
    wroteToTimeBuffer = false;
    _nextStatus = Inactive;

    // If the ROB is currently in its squash sequence, then continue
    // to squash.  In this case, commit does not do anything.  Otherwise
    // run commit.
    list<unsigned>::iterator threads = (*activeThreads).begin();

    // Maybe this should be dependent upon any of the commits actually
    // squashing.
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
        DPRINTF(Activity,"Activity This Cycle.\n");
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

    // Process interrupts if interrupts are enabled and not in PAL mode.
    // Take the PC from commit and write it to the IPR, then squash.  The
    // interrupt completing will take care of restoring the PC from that value
    // in the IPR.  Look at IPR[EXC_ADDR];
    // hwrei() is what resets the PC to the place where instruction execution
    // beings again.
#if FULL_SYSTEM
//#if 0
    if (cpu->checkInterrupts &&
        cpu->check_interrupts() &&
        !cpu->inPalMode(readPC()) &&
        !trapSquash[0] &&
        !xcSquash[0]) {
//        commitStatus[0] = TrapPending;
        toIEW->commitInfo[0].interruptPending = true;
        if (rob->isEmpty() && !iewStage->hasStoresToWB()) {
            // Will need to squash all instructions currently in flight and have
            // the interrupt handler restart at the last non-committed inst.
            // Most of that can be handled through the trap() function.  The
            // processInterrupts() function really just checks for interrupts
            // and then calls trap() if there is an interrupt present.

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
    // Check for squash signal, handle that first
    ////////////////////////////////////

    // Check if the IEW stage is telling the ROB to squash.
    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (fromFetch->fetchFault) {
            // Record the fault.  Wait until it's empty in the ROB.  Then handle the trap.
            fetchFault = fromFetch->fetchFault;
            fetchFaultSN = fromFetch->fetchFaultSN;
            fetchFaultTick = curTick + fetchTrapLatency;
            commitStatus[0] = FetchTrapPending;
            DPRINTF(Commit, "Fault from fetch recorded.  Will trap if the "
                    "ROB empties without squashing the fault.\n");
            fetchTrapWait = 0;
        }
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

            DPRINTF(Commit, "[tid:%u]: Squashing instructions in the "
                    "ROB.\n",
                    tid);

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

            // Send back the sequence number of the squashed instruction.
            toIEW->commitInfo[tid].doneSeqNum = squashed_inst;

            // Send back the squash signal to tell stages that they should
            // squash.
            toIEW->commitInfo[tid].squash = true;

            // Send back the rob squashing signal so other stages know that
            // the ROB is in the process of squashing.
            toIEW->commitInfo[tid].robSquashing = true;

            toIEW->commitInfo[tid].branchMispredict =
                fromIEW->branchMispredict[tid];

            toIEW->commitInfo[tid].branchTaken =
                fromIEW->branchTaken[tid];

            toIEW->commitInfo[tid].nextPC = fromIEW->nextPC[tid];

            DPRINTF(Commit, "Squashing from IEW, restarting at PC %#x\n",
                    fromIEW->nextPC[tid]);

            toIEW->commitInfo[tid].mispredPC =
                fromIEW->mispredPC[tid];

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
    // Note that commit will be handled prior to the ROB so that the ROB
    // only tries to commit instructions it has in this current cycle, and
    // not instructions it is writing in during this cycle.
    // Can't commit and squash things at the same time...
    ////////////////////////////////////

    DPRINTF(Commit, "Trying to commit instructions in the ROB.\n");

    unsigned num_committed = 0;

    DynInstPtr head_inst;
#if FULL_SYSTEM
    if (commitStatus[0] == FetchTrapPending) {
        DPRINTF(Commit, "Fault from fetch is pending.\n");
        if (rob->isEmpty()) {
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

        // If the head instruction is squashed, it is ready to retire at any
        // time.  However, we need to avoid updating any other state
        // incorrectly if it's already been squashed.
        if (head_inst->isSquashed()) {

            DPRINTF(Commit, "Retiring squashed instruction from "
                    "ROB.\n");

            // Tell ROB to retire head instruction.  This retires the head
            // inst in the ROB without affecting any other stages.
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

                // Record that the number of ROB entries has changed.
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
#if FULL_SYSTEM
                int count = 0;
                Addr oldpc;
                do {
                    if (count == 0)
                        assert(!thread[tid]->inSyscall && !thread[tid]->trapPending);
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
}

template <class Impl>
bool
DefaultCommit<Impl>::commitHead(DynInstPtr &head_inst, unsigned inst_num)
{
    // Make sure instruction is valid
    assert(head_inst);

    int tid = head_inst->threadNumber;

    // If the instruction is not executed yet, then it is a non-speculative
    // or store inst.  Signal backwards that it should be executed.
    if (!head_inst->isExecuted()) {
        // Keep this number correct.  We have not yet actually executed
        // and committed this instruction.
        thread[tid]->funcExeInst--;

        head_inst->reachedCommit = true;

        if (head_inst->isNonSpeculative() ||
            head_inst->isMemBarrier() ||
            head_inst->isWriteBarrier()) {
#if !FULL_SYSTEM
            // Hack to make sure syscalls aren't executed until all stores
            // write back their data.  This direct communication shouldn't
            // be used for anything other than this.
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

            DPRINTF(Commit, "Encountered a barrier or non-speculative "
                    "instruction [sn:%lli] at the head of the ROB, PC %#x.\n",
                    head_inst->seqNum, head_inst->readPC());

            // Send back the non-speculative instruction's sequence number.
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
            // number.  Maybe just tell the lsq to re-execute the load.
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

    // Now check if it's one of the special trap or barrier or
    // serializing instructions.
    if (head_inst->isThreadSync())/*  ||
//        head_inst->isMemBarrier()  ||
head_inst->isWriteBarrier())*/
    {
        // Not handled for now.
        panic("Barrier instructions are not handled yet.\n");
    }

    // Check if the instruction caused a fault.  If so, trap.
    Fault inst_fault = head_inst->getFault();

    if (inst_fault != NoFault) {
        if (!head_inst->isNop()) {
#if FULL_SYSTEM
            DPRINTF(Commit, "Inst [sn:%lli] PC %#x has a fault\n",
                    head_inst->seqNum, head_inst->readPC());

            assert(!thread[tid]->inSyscall);

            thread[tid]->inSyscall = true;

            // Hack for now; DTB will sometimes need the machine instruction
            // for when faults happen.  So we will set it here, prior to the
            // DTB possibly needing it for this translation.
            thread[tid]->setInst(
                static_cast<TheISA::MachInst>(head_inst->staticInst->machInst));

            // Consider holding onto the trap and waiting until the trap event
            // happens for this to be executed.
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
    }

    // Check if we're really ready to commit.  If not then return false.
    // I'm pretty sure all instructions should be able to commit if they've
    // reached this far.  For now leave this in as a check.
    if (!rob->isHeadReady(tid)) {
        panic("Unable to commit head instruction!\n");
        return false;
    }

    if (head_inst->isControl()) {
        ++commitCommittedBranches;
    }

    // Now that the instruction is going to be committed, finalize its
    // trace data.
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
    //////////////////////////////////////
    // Handle ROB functions
    //////////////////////////////////////

    // Read any renamed instructions and place them into the ROB.  Do this
    // prior to squashing to avoid having instructions in the ROB that
    // don't get squashed properly.
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
            DPRINTF(Commit, "[tid:%i]: Marking PC %#x, SN %i ready within ROB.\n",
                    fromIEW->insts[inst_num]->threadNumber,
                    fromIEW->insts[inst_num]->readPC(),
                    fromIEW->insts[inst_num]->seqNum);

            // Mark the instruction as ready to commit.
            fromIEW->insts[inst_num]->setCanCommit();
        }
    }
}

template <class Impl>
uint64_t
DefaultCommit<Impl>::readPC()
{
    // @todo: Fix this single thread hack.
    return PC[0];
}

template <class Impl>
void
DefaultCommit<Impl>::setSquashing(unsigned tid)
{
    if (_status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");
        _status = Active;
        cpu->activateStage(FullCPU::CommitIdx);
    }

    if (commitStatus[tid] != ROBSquashing) {
        commitStatus[tid] = ROBSquashing;
        ++squashCounter;
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

////////////////////////////////////////
//                                    //
//   SMT COMMIT POLICY MAITAINED HERE //
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
