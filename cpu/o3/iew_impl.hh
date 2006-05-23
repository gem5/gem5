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

// @todo: Fix the instantaneous communication among all the stages within
// iew.  There's a clear delay between issue and execute, yet backwards
// communication happens simultaneously.

#include <queue>

#include "base/timebuf.hh"
#include "cpu/o3/fu_pool.hh"
#include "cpu/o3/iew.hh"

using namespace std;

template<class Impl>
DefaultIEW<Impl>::LdWritebackEvent::LdWritebackEvent(DynInstPtr &_inst,
                                                     DefaultIEW<Impl> *_iew)
    : Event(&mainEventQueue), inst(_inst), iewStage(_iew)
{
    this->setFlags(Event::AutoDelete);
}

template<class Impl>
void
DefaultIEW<Impl>::LdWritebackEvent::process()
{
    DPRINTF(IEW, "Load writeback event [sn:%lli]\n", inst->seqNum);
    DPRINTF(Activity, "Activity: Ld Writeback event [sn:%lli]\n", inst->seqNum);

    //iewStage->ldstQueue.removeMSHR(inst->threadNumber,inst->seqNum);

    if (iewStage->isSwitchedOut()) {
        inst = NULL;
        return;
    } else if (inst->isSquashed()) {
        iewStage->wakeCPU();
        inst = NULL;
        return;
    }

    iewStage->wakeCPU();

    if (!inst->isExecuted()) {
        inst->setExecuted();

        // Complete access to copy data to proper place.
        if (inst->isStore()) {
            inst->completeAcc();
        }
    }

    // Need to insert instruction into queue to commit
    iewStage->instToCommit(inst);

    iewStage->activityThisCycle();

    inst = NULL;
}

template<class Impl>
const char *
DefaultIEW<Impl>::LdWritebackEvent::description()
{
    return "Load writeback event";
}

template<class Impl>
DefaultIEW<Impl>::DefaultIEW(Params *params)
    : // @todo: Make this into a parameter.
      issueToExecQueue(5, 5),
      instQueue(params),
      ldstQueue(params),
      fuPool(params->fuPool),
      commitToIEWDelay(params->commitToIEWDelay),
      renameToIEWDelay(params->renameToIEWDelay),
      issueToExecuteDelay(params->issueToExecuteDelay),
      issueReadWidth(params->issueWidth),
      issueWidth(params->issueWidth),
      executeWidth(params->executeWidth),
      numThreads(params->numberOfThreads),
      switchedOut(false)
{
    _status = Active;
    exeStatus = Running;
    wbStatus = Idle;

    // Setup wire to read instructions coming from issue.
    fromIssue = issueToExecQueue.getWire(-issueToExecuteDelay);

    // Instruction queue needs the queue between issue and execute.
    instQueue.setIssueToExecuteQueue(&issueToExecQueue);

    instQueue.setIEW(this);
    ldstQueue.setIEW(this);

    for (int i=0; i < numThreads; i++) {
        dispatchStatus[i] = Running;
        stalls[i].commit = false;
        fetchRedirect[i] = false;
    }

    updateLSQNextCycle = false;

    skidBufferMax = (3 * (renameToIEWDelay * params->renameWidth)) + issueWidth;
}

template <class Impl>
std::string
DefaultIEW<Impl>::name() const
{
    return cpu->name() + ".iew";
}

template <class Impl>
void
DefaultIEW<Impl>::regStats()
{
    using namespace Stats;

    instQueue.regStats();

    iewIdleCycles
        .name(name() + ".iewIdleCycles")
        .desc("Number of cycles IEW is idle");

    iewSquashCycles
        .name(name() + ".iewSquashCycles")
        .desc("Number of cycles IEW is squashing");

    iewBlockCycles
        .name(name() + ".iewBlockCycles")
        .desc("Number of cycles IEW is blocking");

    iewUnblockCycles
        .name(name() + ".iewUnblockCycles")
        .desc("Number of cycles IEW is unblocking");

    iewDispatchedInsts
        .name(name() + ".iewDispatchedInsts")
        .desc("Number of instructions dispatched to IQ");

    iewDispSquashedInsts
        .name(name() + ".iewDispSquashedInsts")
        .desc("Number of squashed instructions skipped by dispatch");

    iewDispLoadInsts
        .name(name() + ".iewDispLoadInsts")
        .desc("Number of dispatched load instructions");

    iewDispStoreInsts
        .name(name() + ".iewDispStoreInsts")
        .desc("Number of dispatched store instructions");

    iewDispNonSpecInsts
        .name(name() + ".iewDispNonSpecInsts")
        .desc("Number of dispatched non-speculative instructions");

    iewIQFullEvents
        .name(name() + ".iewIQFullEvents")
        .desc("Number of times the IQ has become full, causing a stall");

    iewLSQFullEvents
        .name(name() + ".iewLSQFullEvents")
        .desc("Number of times the LSQ has become full, causing a stall");

    iewExecutedInsts
        .name(name() + ".iewExecutedInsts")
        .desc("Number of executed instructions");

    iewExecLoadInsts
        .init(cpu->number_of_threads)
        .name(name() + ".iewExecLoadInsts")
        .desc("Number of load instructions executed")
        .flags(total);

    iewExecSquashedInsts
        .name(name() + ".iewExecSquashedInsts")
        .desc("Number of squashed instructions skipped in execute");

    memOrderViolationEvents
        .name(name() + ".memOrderViolationEvents")
        .desc("Number of memory order violations");

    predictedTakenIncorrect
        .name(name() + ".predictedTakenIncorrect")
        .desc("Number of branches that were predicted taken incorrectly");

    predictedNotTakenIncorrect
        .name(name() + ".predictedNotTakenIncorrect")
        .desc("Number of branches that were predicted not taken incorrectly");

    branchMispredicts
        .name(name() + ".branchMispredicts")
        .desc("Number of branch mispredicts detected at execute");

    branchMispredicts = predictedTakenIncorrect + predictedNotTakenIncorrect;

    exeSwp
        .init(cpu->number_of_threads)
        .name(name() + ".EXEC:swp")
        .desc("number of swp insts executed")
        .flags(total)
        ;

    exeNop
        .init(cpu->number_of_threads)
        .name(name() + ".EXEC:nop")
        .desc("number of nop insts executed")
        .flags(total)
        ;

    exeRefs
        .init(cpu->number_of_threads)
        .name(name() + ".EXEC:refs")
        .desc("number of memory reference insts executed")
        .flags(total)
        ;

    exeBranches
        .init(cpu->number_of_threads)
        .name(name() + ".EXEC:branches")
        .desc("Number of branches executed")
        .flags(total)
        ;

    issueRate
        .name(name() + ".EXEC:rate")
        .desc("Inst execution rate")
        .flags(total)
        ;
    issueRate = iewExecutedInsts / cpu->numCycles;

    iewExecStoreInsts
        .name(name() + ".EXEC:stores")
        .desc("Number of stores executed")
        .flags(total)
        ;
    iewExecStoreInsts = exeRefs - iewExecLoadInsts;
/*
    for (int i=0; i<Num_OpClasses; ++i) {
        stringstream subname;
        subname << opClassStrings[i] << "_delay";
        issue_delay_dist.subname(i, subname.str());
    }
*/
    //
    //  Other stats
    //

    iewInstsToCommit
        .init(cpu->number_of_threads)
        .name(name() + ".WB:sent")
        .desc("cumulative count of insts sent to commit")
        .flags(total)
        ;

    writebackCount
        .init(cpu->number_of_threads)
        .name(name() + ".WB:count")
        .desc("cumulative count of insts written-back")
        .flags(total)
        ;

    producerInst
        .init(cpu->number_of_threads)
        .name(name() + ".WB:producers")
        .desc("num instructions producing a value")
        .flags(total)
        ;

    consumerInst
        .init(cpu->number_of_threads)
        .name(name() + ".WB:consumers")
        .desc("num instructions consuming a value")
        .flags(total)
        ;

    wbPenalized
        .init(cpu->number_of_threads)
        .name(name() + ".WB:penalized")
        .desc("number of instrctions required to write to 'other' IQ")
        .flags(total)
        ;

    wbPenalizedRate
        .name(name() + ".WB:penalized_rate")
        .desc ("fraction of instructions written-back that wrote to 'other' IQ")
        .flags(total)
        ;

    wbPenalizedRate = wbPenalized / writebackCount;

    wbFanout
        .name(name() + ".WB:fanout")
        .desc("average fanout of values written-back")
        .flags(total)
        ;

    wbFanout = producerInst / consumerInst;

    wbRate
        .name(name() + ".WB:rate")
        .desc("insts written-back per cycle")
        .flags(total)
        ;
    wbRate = writebackCount / cpu->numCycles;
}

template<class Impl>
void
DefaultIEW<Impl>::initStage()
{
    for (int tid=0; tid < numThreads; tid++) {
        toRename->iewInfo[tid].usedIQ = true;
        toRename->iewInfo[tid].freeIQEntries =
            instQueue.numFreeEntries(tid);

        toRename->iewInfo[tid].usedLSQ = true;
        toRename->iewInfo[tid].freeLSQEntries =
            ldstQueue.numFreeEntries(tid);
    }
}

template<class Impl>
void
DefaultIEW<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(IEW, "Setting CPU pointer.\n");
    cpu = cpu_ptr;

    instQueue.setCPU(cpu_ptr);
    ldstQueue.setCPU(cpu_ptr);

    cpu->activateStage(FullCPU::IEWIdx);
}

template<class Impl>
void
DefaultIEW<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(IEW, "Setting time buffer pointer.\n");
    timeBuffer = tb_ptr;

    // Setup wire to read information from time buffer, from commit.
    fromCommit = timeBuffer->getWire(-commitToIEWDelay);

    // Setup wire to write information back to previous stages.
    toRename = timeBuffer->getWire(0);

    toFetch = timeBuffer->getWire(0);

    // Instruction queue also needs main time buffer.
    instQueue.setTimeBuffer(tb_ptr);
}

template<class Impl>
void
DefaultIEW<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    DPRINTF(IEW, "Setting rename queue pointer.\n");
    renameQueue = rq_ptr;

    // Setup wire to read information from rename queue.
    fromRename = renameQueue->getWire(-renameToIEWDelay);
}

template<class Impl>
void
DefaultIEW<Impl>::setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr)
{
    DPRINTF(IEW, "Setting IEW queue pointer.\n");
    iewQueue = iq_ptr;

    // Setup wire to write instructions to commit.
    toCommit = iewQueue->getWire(0);
}

template<class Impl>
void
DefaultIEW<Impl>::setActiveThreads(list<unsigned> *at_ptr)
{
    DPRINTF(IEW, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;

    ldstQueue.setActiveThreads(at_ptr);
    instQueue.setActiveThreads(at_ptr);
}

template<class Impl>
void
DefaultIEW<Impl>::setScoreboard(Scoreboard *sb_ptr)
{
    DPRINTF(IEW, "Setting scoreboard pointer.\n");
    scoreboard = sb_ptr;
}

#if 0
template<class Impl>
void
DefaultIEW<Impl>::setPageTable(PageTable *pt_ptr)
{
    ldstQueue.setPageTable(pt_ptr);
}
#endif

template <class Impl>
void
DefaultIEW<Impl>::switchOut()
{
    cpu->signalSwitched();
}

template <class Impl>
void
DefaultIEW<Impl>::doSwitchOut()
{
    switchedOut = true;

    instQueue.switchOut();
    ldstQueue.switchOut();
    fuPool->switchOut();

    for (int i = 0; i < numThreads; i++) {
        while (!insts[i].empty())
            insts[i].pop();
        while (!skidBuffer[i].empty())
            skidBuffer[i].pop();
    }
}

template <class Impl>
void
DefaultIEW<Impl>::takeOverFrom()
{
    _status = Active;
    exeStatus = Running;
    wbStatus = Idle;
    switchedOut = false;

    instQueue.takeOverFrom();
    ldstQueue.takeOverFrom();
    fuPool->takeOverFrom();

    initStage();
    cpu->activityThisCycle();

    for (int i=0; i < numThreads; i++) {
        dispatchStatus[i] = Running;
        stalls[i].commit = false;
        fetchRedirect[i] = false;
    }

    updateLSQNextCycle = false;

    // @todo: Fix hardcoded number
    for (int i = 0; i < 6; ++i) {
        issueToExecQueue.advance();
    }
}

template<class Impl>
void
DefaultIEW<Impl>::squash(unsigned tid)
{
    DPRINTF(IEW, "[tid:%i]: Squashing all instructions.\n",
            tid);

    // Tell the IQ to start squashing.
    instQueue.squash(tid);

    // Tell the LDSTQ to start squashing.
    ldstQueue.squash(fromCommit->commitInfo[tid].doneSeqNum, tid);

    updatedQueues = true;

    // Clear the skid buffer in case it has any data in it.
    while (!skidBuffer[tid].empty()) {

        if (skidBuffer[tid].front()->isLoad() ||
            skidBuffer[tid].front()->isStore() ) {
            toRename->iewInfo[tid].dispatchedToLSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        skidBuffer[tid].pop();
    }

    while (!insts[tid].empty()) {
        if (insts[tid].front()->isLoad() ||
            insts[tid].front()->isStore() ) {
            toRename->iewInfo[tid].dispatchedToLSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        insts[tid].pop();
    }
}

template<class Impl>
void
DefaultIEW<Impl>::squashDueToBranch(DynInstPtr &inst, unsigned tid)
{
    DPRINTF(IEW, "[tid:%i]: Squashing from a specific instruction, PC: %#x "
            "[sn:%i].\n", tid, inst->readPC(), inst->seqNum);

    toCommit->squash[tid] = true;
    toCommit->squashedSeqNum[tid] = inst->seqNum;
    toCommit->mispredPC[tid] = inst->readPC();
    toCommit->nextPC[tid] = inst->readNextPC();
    toCommit->branchMispredict[tid] = true;
    toCommit->branchTaken[tid] = inst->readNextPC() !=
        (inst->readPC() + sizeof(TheISA::MachInst));

    toCommit->includeSquashInst[tid] = false;

    wroteToTimeBuffer = true;
}

template<class Impl>
void
DefaultIEW<Impl>::squashDueToMemOrder(DynInstPtr &inst, unsigned tid)
{
    DPRINTF(IEW, "[tid:%i]: Squashing from a specific instruction, "
            "PC: %#x [sn:%i].\n", tid, inst->readPC(), inst->seqNum);

    toCommit->squash[tid] = true;
    toCommit->squashedSeqNum[tid] = inst->seqNum;
    toCommit->nextPC[tid] = inst->readNextPC();

    toCommit->includeSquashInst[tid] = false;

    wroteToTimeBuffer = true;
}

template<class Impl>
void
DefaultIEW<Impl>::squashDueToMemBlocked(DynInstPtr &inst, unsigned tid)
{
    DPRINTF(IEW, "[tid:%i]: Memory blocked, squashing load and younger insts, "
            "PC: %#x [sn:%i].\n", tid, inst->readPC(), inst->seqNum);

    toCommit->squash[tid] = true;
    toCommit->squashedSeqNum[tid] = inst->seqNum;
    toCommit->nextPC[tid] = inst->readPC();

    toCommit->includeSquashInst[tid] = true;

    ldstQueue.setLoadBlockedHandled(tid);

    wroteToTimeBuffer = true;
}

template<class Impl>
void
DefaultIEW<Impl>::block(unsigned tid)
{
    DPRINTF(IEW, "[tid:%u]: Blocking.\n", tid);

    if (dispatchStatus[tid] != Blocked &&
        dispatchStatus[tid] != Unblocking) {
        toRename->iewBlock[tid] = true;
        wroteToTimeBuffer = true;
    }

    // Add the current inputs to the skid buffer so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    dispatchStatus[tid] = Blocked;
}

template<class Impl>
void
DefaultIEW<Impl>::unblock(unsigned tid)
{
    DPRINTF(IEW, "[tid:%i]: Reading instructions out of the skid "
            "buffer %u.\n",tid, tid);

    // If the skid bufffer is empty, signal back to previous stages to unblock.
    // Also switch status to running.
    if (skidBuffer[tid].empty()) {
        toRename->iewUnblock[tid] = true;
        wroteToTimeBuffer = true;
        DPRINTF(IEW, "[tid:%i]: Done unblocking.\n",tid);
        dispatchStatus[tid] = Running;
    }
}

template<class Impl>
void
DefaultIEW<Impl>::wakeDependents(DynInstPtr &inst)
{
    instQueue.wakeDependents(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::rescheduleMemInst(DynInstPtr &inst)
{
    instQueue.rescheduleMemInst(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::replayMemInst(DynInstPtr &inst)
{
    instQueue.replayMemInst(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::instToCommit(DynInstPtr &inst)
{
    // First check the time slot that this instruction will write
    // to.  If there are free write ports at the time, then go ahead
    // and write the instruction to that time.  If there are not,
    // keep looking back to see where's the first time there's a
    // free slot.
    while ((*iewQueue)[wbCycle].insts[wbNumInst]) {
        ++wbNumInst;
        if (wbNumInst == issueWidth) {
            ++wbCycle;
            wbNumInst = 0;
        }

        assert(wbCycle < 5);
    }

    // Add finished instruction to queue to commit.
    (*iewQueue)[wbCycle].insts[wbNumInst] = inst;
    (*iewQueue)[wbCycle].size++;
}

template <class Impl>
unsigned
DefaultIEW<Impl>::validInstsFromRename()
{
    unsigned inst_count = 0;

    for (int i=0; i<fromRename->size; i++) {
        if (!fromRename->insts[i]->squashed)
            inst_count++;
    }

    return inst_count;
}

template<class Impl>
void
DefaultIEW<Impl>::skidInsert(unsigned tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop();

        DPRINTF(Decode,"[tid:%i]: Inserting [sn:%lli] PC:%#x into "
                "dispatch skidBuffer %i\n",tid, inst->seqNum,
                inst->readPC(),tid);

        skidBuffer[tid].push(inst);
    }

    assert(skidBuffer[tid].size() <= skidBufferMax &&
           "Skidbuffer Exceeded Max Size");
}

template<class Impl>
int
DefaultIEW<Impl>::skidCount()
{
    int max=0;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned thread_count = skidBuffer[*threads++].size();
        if (max < thread_count)
            max = thread_count;
    }

    return max;
}

template<class Impl>
bool
DefaultIEW<Impl>::skidsEmpty()
{
    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        if (!skidBuffer[*threads++].empty())
            return false;
    }

    return true;
}

template <class Impl>
void
DefaultIEW<Impl>::updateStatus()
{
    bool any_unblocking = false;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (dispatchStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // If there are no ready instructions waiting to be scheduled by the IQ,
    // and there's no stores waiting to write back, and dispatch is not
    // unblocking, then there is no internal activity for the IEW stage.
    if (_status == Active && !instQueue.hasReadyInsts() &&
        !ldstQueue.willWB() && !any_unblocking) {
        DPRINTF(IEW, "IEW switching to idle\n");

        deactivateStage();

        _status = Inactive;
    } else if (_status == Inactive && (instQueue.hasReadyInsts() ||
                                       ldstQueue.willWB() ||
                                       any_unblocking)) {
        // Otherwise there is internal activity.  Set to active.
        DPRINTF(IEW, "IEW switching to active\n");

        activateStage();

        _status = Active;
    }
}

template <class Impl>
void
DefaultIEW<Impl>::resetEntries()
{
    instQueue.resetEntries();
    ldstQueue.resetEntries();
}

template <class Impl>
void
DefaultIEW<Impl>::readStallSignals(unsigned tid)
{
    if (fromCommit->commitBlock[tid]) {
        stalls[tid].commit = true;
    }

    if (fromCommit->commitUnblock[tid]) {
        assert(stalls[tid].commit);
        stalls[tid].commit = false;
    }
}

template <class Impl>
bool
DefaultIEW<Impl>::checkStall(unsigned tid)
{
    bool ret_val(false);

    if (stalls[tid].commit) {
        DPRINTF(IEW,"[tid:%i]: Stall from Commit stage detected.\n",tid);
        ret_val = true;
    } else if (instQueue.isFull(tid)) {
        DPRINTF(IEW,"[tid:%i]: Stall: IQ  is full.\n",tid);
        ret_val = true;
    } else if (ldstQueue.isFull(tid)) {
        DPRINTF(IEW,"[tid:%i]: Stall: LSQ is full\n",tid);

        if (ldstQueue.numLoads(tid) > 0 ) {

            DPRINTF(IEW,"[tid:%i]: LSQ oldest load: [sn:%i] \n",
                    tid,ldstQueue.getLoadHeadSeqNum(tid));
        }

        if (ldstQueue.numStores(tid) > 0) {

            DPRINTF(IEW,"[tid:%i]: LSQ oldest store: [sn:%i] \n",
                    tid,ldstQueue.getStoreHeadSeqNum(tid));
        }

        ret_val = true;
    } else if (ldstQueue.isStalled(tid)) {
        DPRINTF(IEW,"[tid:%i]: Stall: LSQ stall detected.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

template <class Impl>
void
DefaultIEW<Impl>::checkSignalsAndUpdate(unsigned tid)
{
    // Check if there's a squash signal, squash if there is
    // Check stall signals, block if there is.
    // If status was Blocked
    //     if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.

    readStallSignals(tid);

    if (fromCommit->commitInfo[tid].squash) {
        squash(tid);

        if (dispatchStatus[tid] == Blocked ||
            dispatchStatus[tid] == Unblocking) {
            toRename->iewUnblock[tid] = true;
            wroteToTimeBuffer = true;
        }

        dispatchStatus[tid] = Squashing;

        fetchRedirect[tid] = false;
        return;
    }

    if (fromCommit->commitInfo[tid].robSquashing) {
        DPRINTF(IEW, "[tid:%i]: ROB is still squashing.\n");

        dispatchStatus[tid] = Squashing;

        return;
    }

    if (checkStall(tid)) {
        block(tid);
        dispatchStatus[tid] = Blocked;
        return;
    }

    if (dispatchStatus[tid] == Blocked) {
        // Status from previous cycle was blocked, but there are no more stall
        // conditions.  Switch over to unblocking.
        DPRINTF(IEW, "[tid:%i]: Done blocking, switching to unblocking.\n",
                tid);

        dispatchStatus[tid] = Unblocking;

        unblock(tid);

        return;
    }

    if (dispatchStatus[tid] == Squashing) {
        // Switch status to running if rename isn't being told to block or
        // squash this cycle.
        DPRINTF(IEW, "[tid:%i]: Done squashing, switching to running.\n",
                tid);

        dispatchStatus[tid] = Running;

        return;
    }
}

template <class Impl>
void
DefaultIEW<Impl>::sortInsts()
{
    int insts_from_rename = fromRename->size;
#ifdef DEBUG
    for (int i = 0; i < numThreads; i++)
        assert(insts[i].empty());
#endif
    for (int i = 0; i < insts_from_rename; ++i) {
        insts[fromRename->insts[i]->threadNumber].push(fromRename->insts[i]);
    }
}

template <class Impl>
void
DefaultIEW<Impl>::wakeCPU()
{
    cpu->wakeCPU();
}

template <class Impl>
void
DefaultIEW<Impl>::activityThisCycle()
{
    DPRINTF(Activity, "Activity this cycle.\n");
    cpu->activityThisCycle();
}

template <class Impl>
inline void
DefaultIEW<Impl>::activateStage()
{
    DPRINTF(Activity, "Activating stage.\n");
    cpu->activateStage(FullCPU::IEWIdx);
}

template <class Impl>
inline void
DefaultIEW<Impl>::deactivateStage()
{
    DPRINTF(Activity, "Deactivating stage.\n");
    cpu->deactivateStage(FullCPU::IEWIdx);
}

template<class Impl>
void
DefaultIEW<Impl>::dispatch(unsigned tid)
{
    // If status is Running or idle,
    //     call dispatchInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from rename
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (dispatchStatus[tid] == Blocked) {
        ++iewBlockCycles;

    } else if (dispatchStatus[tid] == Squashing) {
        ++iewSquashCycles;
    }

    // Dispatch should try to dispatch as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (dispatchStatus[tid] == Running ||
        dispatchStatus[tid] == Idle) {
        DPRINTF(IEW, "[tid:%i] Not blocked, so attempting to run "
                "dispatch.\n", tid);

        dispatchInsts(tid);
    } else if (dispatchStatus[tid] == Unblocking) {
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(!skidsEmpty());

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        dispatchInsts(tid);

        ++iewUnblockCycles;

        if (validInstsFromRename() && dispatchedAllInsts) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        unblock(tid);
    }
}

template <class Impl>
void
DefaultIEW<Impl>::dispatchInsts(unsigned tid)
{
    dispatchedAllInsts = true;

    // Obtain instructions from skid buffer if unblocking, or queue from rename
    // otherwise.
    std::queue<DynInstPtr> &insts_to_dispatch =
        dispatchStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];

    int insts_to_add = insts_to_dispatch.size();

    DynInstPtr inst;
    bool add_to_iq = false;
    int dis_num_inst = 0;

    // Loop through the instructions, putting them in the instruction
    // queue.
    for ( ; dis_num_inst < insts_to_add &&
              dis_num_inst < issueReadWidth;
          ++dis_num_inst)
    {
        inst = insts_to_dispatch.front();

        if (dispatchStatus[tid] == Unblocking) {
            DPRINTF(IEW, "[tid:%i]: Issue: Examining instruction from skid "
                    "buffer\n", tid);
        }

        // Make sure there's a valid instruction there.
        assert(inst);

        DPRINTF(IEW, "[tid:%i]: Issue: Adding PC %#x [sn:%lli] [tid:%i] to "
                "IQ.\n",
                tid, inst->readPC(), inst->seqNum, inst->threadNumber);

        // Be sure to mark these instructions as ready so that the
        // commit stage can go ahead and execute them, and mark
        // them as issued so the IQ doesn't reprocess them.

        // Check for squashed instructions.
        if (inst->isSquashed()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Squashed instruction encountered, "
                    "not adding to IQ.\n", tid);

            ++iewDispSquashedInsts;

            insts_to_dispatch.pop();

            //Tell Rename That An Instruction has been processed
            if (inst->isLoad() || inst->isStore()) {
                toRename->iewInfo[tid].dispatchedToLSQ++;
            }
            toRename->iewInfo[tid].dispatched++;

            continue;
        }

        // Check for full conditions.
        if (instQueue.isFull(tid)) {
            DPRINTF(IEW, "[tid:%i]: Issue: IQ has become full.\n", tid);

            // Call function to start blocking.
            block(tid);

            // Set unblock to false. Special case where we are using
            // skidbuffer (unblocking) instructions but then we still
            // get full in the IQ.
            toRename->iewUnblock[tid] = false;

            dispatchedAllInsts = false;

            ++iewIQFullEvents;
            break;
        } else if (ldstQueue.isFull(tid)) {
            DPRINTF(IEW, "[tid:%i]: Issue: LSQ has become full.\n",tid);

            // Call function to start blocking.
            block(tid);

            // Set unblock to false. Special case where we are using
            // skidbuffer (unblocking) instructions but then we still
            // get full in the IQ.
            toRename->iewUnblock[tid] = false;

            dispatchedAllInsts = false;

            ++iewLSQFullEvents;
            break;
        }

        // Otherwise issue the instruction just fine.
        if (inst->isLoad()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            // Reserve a spot in the load store queue for this
            // memory access.
            ldstQueue.insertLoad(inst);

            ++iewDispLoadInsts;

            add_to_iq = true;

            toRename->iewInfo[tid].dispatchedToLSQ++;
        } else if (inst->isStore()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            ldstQueue.insertStore(inst);

            ++iewDispStoreInsts;

            if (inst->isNonSpeculative()) {
                // Non-speculative stores (namely store conditionals)
                // need to be set as "canCommit()" so that commit can
                // process them when they reach the head of commit.
                inst->setCanCommit();
                instQueue.insertNonSpec(inst);
                add_to_iq = false;

                ++iewDispNonSpecInsts;
            } else {
                add_to_iq = true;
            }

            toRename->iewInfo[tid].dispatchedToLSQ++;
#if FULL_SYSTEM
        } else if (inst->isMemBarrier() || inst->isWriteBarrier()) {
            // Same as non-speculative stores.
            inst->setCanCommit();
            instQueue.insertBarrier(inst);
            add_to_iq = false;
#endif
        } else if (inst->isNonSpeculative()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Nonspeculative instruction "
                    "encountered, skipping.\n", tid);

            // Same as non-speculative stores.
            inst->setCanCommit();

            // Specifically insert it as nonspeculative.
            instQueue.insertNonSpec(inst);

            ++iewDispNonSpecInsts;

            add_to_iq = false;
        } else if (inst->isNop()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Nop instruction encountered, "
                    "skipping.\n", tid);

            inst->setIssued();
            inst->setExecuted();
            inst->setCanCommit();

            instQueue.recordProducer(inst);

            exeNop[tid]++;

            add_to_iq = false;
        } else if (inst->isExecuted()) {
            assert(0 && "Instruction shouldn't be executed.\n");
            DPRINTF(IEW, "Issue: Executed branch encountered, "
                    "skipping.\n");

            inst->setIssued();
            inst->setCanCommit();

            instQueue.recordProducer(inst);

            add_to_iq = false;
        } else {
            add_to_iq = true;
        }

        // If the instruction queue is not full, then add the
        // instruction.
        if (add_to_iq) {
            instQueue.insert(inst);
        }

        insts_to_dispatch.pop();

        toRename->iewInfo[tid].dispatched++;

        ++iewDispatchedInsts;
    }

    if (!insts_to_dispatch.empty()) {
        DPRINTF(IEW,"[tid:%i]: Issue: Bandwidth Full. Blocking.\n");
        block(tid);
        toRename->iewUnblock[tid] = false;
    }

    if (dispatchStatus[tid] == Idle && dis_num_inst) {
        dispatchStatus[tid] = Running;

        updatedQueues = true;
    }

    dis_num_inst = 0;
}

template <class Impl>
void
DefaultIEW<Impl>::printAvailableInsts()
{
    int inst = 0;

    cout << "Available Instructions: ";

    while (fromIssue->insts[inst]) {

        if (inst%3==0) cout << "\n\t";

        cout << "PC: " << fromIssue->insts[inst]->readPC()
             << " TN: " << fromIssue->insts[inst]->threadNumber
             << " SN: " << fromIssue->insts[inst]->seqNum << " | ";

        inst++;

    }

    cout << "\n";
}

template <class Impl>
void
DefaultIEW<Impl>::executeInsts()
{
    wbNumInst = 0;
    wbCycle = 0;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;
        fetchRedirect[tid] = false;
    }

#if 0
    printAvailableInsts();
#endif

    // Execute/writeback any instructions that are available.
    int insts_to_execute = fromIssue->size;
    int inst_num = 0;
    for (; inst_num < insts_to_execute;
          ++inst_num) {

        DPRINTF(IEW, "Execute: Executing instructions from IQ.\n");

        DynInstPtr inst = instQueue.getInstToExecute();

        DPRINTF(IEW, "Execute: Processing PC %#x, [tid:%i] [sn:%i].\n",
                inst->readPC(), inst->threadNumber,inst->seqNum);

        // Check if the instruction is squashed; if so then skip it
        if (inst->isSquashed()) {
            DPRINTF(IEW, "Execute: Instruction was squashed.\n");

            // Consider this instruction executed so that commit can go
            // ahead and retire the instruction.
            inst->setExecuted();

            // Not sure if I should set this here or just let commit try to
            // commit any squashed instructions.  I like the latter a bit more.
            inst->setCanCommit();

            ++iewExecSquashedInsts;

            continue;
        }

        Fault fault = NoFault;

        // Execute instruction.
        // Note that if the instruction faults, it will be handled
        // at the commit stage.
        if (inst->isMemRef() &&
            (!inst->isDataPrefetch() && !inst->isInstPrefetch())) {
            DPRINTF(IEW, "Execute: Calculating address for memory "
                    "reference.\n");

            // Tell the LDSTQ to execute this instruction (if it is a load).
            if (inst->isLoad()) {
                // Loads will mark themselves as executed, and their writeback
                // event adds the instruction to the queue to commit
                fault = ldstQueue.executeLoad(inst);
            } else if (inst->isStore()) {
                ldstQueue.executeStore(inst);

                // If the store had a fault then it may not have a mem req
                if (inst->req && !(inst->req->flags & LOCKED)) {
                    inst->setExecuted();

                    instToCommit(inst);
                }

                // Store conditionals will mark themselves as
                // executed, and their writeback event will add the
                // instruction to the queue to commit.
            } else {
                panic("Unexpected memory type!\n");
            }

        } else {
            inst->execute();

            inst->setExecuted();

            instToCommit(inst);
        }

        updateExeInstStats(inst);

        // Check if branch prediction was correct, if not then we need
        // to tell commit to squash in flight instructions.  Only
        // handle this if there hasn't already been something that
        // redirects fetch in this group of instructions.

        // This probably needs to prioritize the redirects if a different
        // scheduler is used.  Currently the scheduler schedules the oldest
        // instruction first, so the branch resolution order will be correct.
        unsigned tid = inst->threadNumber;

        if (!fetchRedirect[tid]) {

            if (inst->mispredicted()) {
                fetchRedirect[tid] = true;

                DPRINTF(IEW, "Execute: Branch mispredict detected.\n");
                DPRINTF(IEW, "Execute: Redirecting fetch to PC: %#x.\n",
                        inst->nextPC);

                // If incorrect, then signal the ROB that it must be squashed.
                squashDueToBranch(inst, tid);

                if (inst->predTaken()) {
                    predictedTakenIncorrect++;
                } else {
                    predictedNotTakenIncorrect++;
                }
            } else if (ldstQueue.violation(tid)) {
                fetchRedirect[tid] = true;

                // If there was an ordering violation, then get the
                // DynInst that caused the violation.  Note that this
                // clears the violation signal.
                DynInstPtr violator;
                violator = ldstQueue.getMemDepViolator(tid);

                DPRINTF(IEW, "LDSTQ detected a violation.  Violator PC: "
                        "%#x, inst PC: %#x.  Addr is: %#x.\n",
                        violator->readPC(), inst->readPC(), inst->physEffAddr);

                // Tell the instruction queue that a violation has occured.
                instQueue.violation(inst, violator);

                // Squash.
                squashDueToMemOrder(inst,tid);

                ++memOrderViolationEvents;
            } else if (ldstQueue.loadBlocked(tid) &&
                       !ldstQueue.isLoadBlockedHandled(tid)) {
                fetchRedirect[tid] = true;

                DPRINTF(IEW, "Load operation couldn't execute because the "
                        "memory system is blocked.  PC: %#x [sn:%lli]\n",
                        inst->readPC(), inst->seqNum);

                squashDueToMemBlocked(inst, tid);
            }
        }
    }

    if (inst_num) {
        if (exeStatus == Idle) {
            exeStatus = Running;
        }

        updatedQueues = true;

        cpu->activityThisCycle();
    }

    // Need to reset this in case a writeback event needs to write into the
    // iew queue.  That way the writeback event will write into the correct
    // spot in the queue.
    wbNumInst = 0;
}

template <class Impl>
void
DefaultIEW<Impl>::writebackInsts()
{
    // Loop through the head of the time buffer and wake any
    // dependents.  These instructions are about to write back.  Also
    // mark scoreboard that this instruction is finally complete.
    // Either have IEW have direct access to scoreboard, or have this
    // as part of backwards communication.
    for (int inst_num = 0; inst_num < issueWidth &&
             toCommit->insts[inst_num]; inst_num++) {
        DynInstPtr inst = toCommit->insts[inst_num];
        int tid = inst->threadNumber;

        DPRINTF(IEW, "Sending instructions to commit, PC %#x.\n",
                inst->readPC());

        iewInstsToCommit[tid]++;

        // Some instructions will be sent to commit without having
        // executed because they need commit to handle them.
        // E.g. Uncached loads have not actually executed when they
        // are first sent to commit.  Instead commit must tell the LSQ
        // when it's ready to execute the uncached load.
        if (!inst->isSquashed() && inst->isExecuted()) {
            int dependents = instQueue.wakeDependents(inst);

            for (int i = 0; i < inst->numDestRegs(); i++) {
                //mark as Ready
                DPRINTF(IEW,"Setting Destination Register %i\n",
                        inst->renamedDestRegIdx(i));
                scoreboard->setReg(inst->renamedDestRegIdx(i));
            }

            producerInst[tid]++;
            consumerInst[tid]+= dependents;
            writebackCount[tid]++;
        }
    }
}

template<class Impl>
void
DefaultIEW<Impl>::tick()
{
    wbNumInst = 0;
    wbCycle = 0;

    wroteToTimeBuffer = false;
    updatedQueues = false;

    sortInsts();

    // Free function units marked as being freed this cycle.
    fuPool->processFreeUnits();

    list<unsigned>::iterator threads = (*activeThreads).begin();

    // Check stall and squash signals, dispatch any instructions.
    while (threads != (*activeThreads).end()) {
           unsigned tid = *threads++;

        DPRINTF(IEW,"Issue: Processing [tid:%i]\n",tid);

        checkSignalsAndUpdate(tid);
        dispatch(tid);
    }

    if (exeStatus != Squashing) {
        executeInsts();

        writebackInsts();

        // Have the instruction queue try to schedule any ready instructions.
        // (In actuality, this scheduling is for instructions that will
        // be executed next cycle.)
        instQueue.scheduleReadyInsts();

        // Also should advance its own time buffers if the stage ran.
        // Not the best place for it, but this works (hopefully).
        issueToExecQueue.advance();
    }

    bool broadcast_free_entries = false;

    if (updatedQueues || exeStatus == Running || updateLSQNextCycle) {
        exeStatus = Idle;
        updateLSQNextCycle = false;

        broadcast_free_entries = true;
    }

    // Writeback any stores using any leftover bandwidth.
    ldstQueue.writebackStores();

    // Check the committed load/store signals to see if there's a load
    // or store to commit.  Also check if it's being told to execute a
    // nonspeculative instruction.
    // This is pretty inefficient...

    threads = (*activeThreads).begin();
    while (threads != (*activeThreads).end()) {
        unsigned tid = (*threads++);

        DPRINTF(IEW,"Processing [tid:%i]\n",tid);

        if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
            !fromCommit->commitInfo[tid].squash &&
            !fromCommit->commitInfo[tid].robSquashing) {

            ldstQueue.commitStores(fromCommit->commitInfo[tid].doneSeqNum,tid);

            ldstQueue.commitLoads(fromCommit->commitInfo[tid].doneSeqNum,tid);

            updateLSQNextCycle = true;
            instQueue.commit(fromCommit->commitInfo[tid].doneSeqNum,tid);
        }

        if (fromCommit->commitInfo[tid].nonSpecSeqNum != 0) {

            //DPRINTF(IEW,"NonspecInst from thread %i",tid);
            if (fromCommit->commitInfo[tid].uncached) {
                instQueue.replayMemInst(fromCommit->commitInfo[tid].uncachedLoad);
            } else {
                instQueue.scheduleNonSpec(
                    fromCommit->commitInfo[tid].nonSpecSeqNum);
            }
        }

        if (broadcast_free_entries) {
            toFetch->iewInfo[tid].iqCount =
                instQueue.getCount(tid);
            toFetch->iewInfo[tid].ldstqCount =
                ldstQueue.getCount(tid);

            toRename->iewInfo[tid].usedIQ = true;
            toRename->iewInfo[tid].freeIQEntries =
                instQueue.numFreeEntries();
            toRename->iewInfo[tid].usedLSQ = true;
            toRename->iewInfo[tid].freeLSQEntries =
                ldstQueue.numFreeEntries(tid);

            wroteToTimeBuffer = true;
        }

        DPRINTF(IEW, "[tid:%i], Dispatch dispatched %i instructions.\n",
                tid, toRename->iewInfo[tid].dispatched);
    }

    DPRINTF(IEW, "IQ has %i free entries (Can schedule: %i).  "
            "LSQ has %i free entries.\n",
            instQueue.numFreeEntries(), instQueue.hasReadyInsts(),
            ldstQueue.numFreeEntries());

    updateStatus();

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }
}

template <class Impl>
void
DefaultIEW<Impl>::updateExeInstStats(DynInstPtr &inst)
{
    int thread_number = inst->threadNumber;

    //
    //  Pick off the software prefetches
    //
#ifdef TARGET_ALPHA
    if (inst->isDataPrefetch())
        exeSwp[thread_number]++;
    else
        iewExecutedInsts++;
#else
    iewExecutedInsts[thread_number]++;
#endif

    //
    //  Control operations
    //
    if (inst->isControl())
        exeBranches[thread_number]++;

    //
    //  Memory operations
    //
    if (inst->isMemRef()) {
        exeRefs[thread_number]++;

        if (inst->isLoad()) {
            iewExecLoadInsts[thread_number]++;
        }
    }
}
