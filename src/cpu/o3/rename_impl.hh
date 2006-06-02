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
 */

#include <list>

#include "config/full_system.hh"
#include "cpu/o3/rename.hh"

using namespace std;

template <class Impl>
DefaultRename<Impl>::DefaultRename(Params *params)
    : iewToRenameDelay(params->iewToRenameDelay),
      decodeToRenameDelay(params->decodeToRenameDelay),
      commitToRenameDelay(params->commitToRenameDelay),
      renameWidth(params->renameWidth),
      commitWidth(params->commitWidth),
      numThreads(params->numberOfThreads)
{
    _status = Inactive;

    for (int i=0; i< numThreads; i++) {
        renameStatus[i] = Idle;

        freeEntries[i].iqEntries = 0;
        freeEntries[i].lsqEntries = 0;
        freeEntries[i].robEntries = 0;

        stalls[i].iew = false;
        stalls[i].commit = false;
        serializeInst[i] = NULL;

        instsInProgress[i] = 0;

        emptyROB[i] = true;

        serializeOnNextInst[i] = false;
    }

    // @todo: Make into a parameter.
    skidBufferMax = (2 * (iewToRenameDelay * params->decodeWidth)) + renameWidth;
}

template <class Impl>
std::string
DefaultRename<Impl>::name() const
{
    return cpu->name() + ".rename";
}

template <class Impl>
void
DefaultRename<Impl>::regStats()
{
    renameSquashCycles
        .name(name() + ".RENAME:SquashCycles")
        .desc("Number of cycles rename is squashing")
        .prereq(renameSquashCycles);
    renameIdleCycles
        .name(name() + ".RENAME:IdleCycles")
        .desc("Number of cycles rename is idle")
        .prereq(renameIdleCycles);
    renameBlockCycles
        .name(name() + ".RENAME:BlockCycles")
        .desc("Number of cycles rename is blocking")
        .prereq(renameBlockCycles);
    renameSerializeStallCycles
        .name(name() + ".RENAME:serializeStallCycles")
        .desc("count of cycles rename stalled for serializing inst")
        .flags(Stats::total);
    renameRunCycles
        .name(name() + ".RENAME:RunCycles")
        .desc("Number of cycles rename is running")
        .prereq(renameIdleCycles);
    renameUnblockCycles
        .name(name() + ".RENAME:UnblockCycles")
        .desc("Number of cycles rename is unblocking")
        .prereq(renameUnblockCycles);
    renameRenamedInsts
        .name(name() + ".RENAME:RenamedInsts")
        .desc("Number of instructions processed by rename")
        .prereq(renameRenamedInsts);
    renameSquashedInsts
        .name(name() + ".RENAME:SquashedInsts")
        .desc("Number of squashed instructions processed by rename")
        .prereq(renameSquashedInsts);
    renameROBFullEvents
        .name(name() + ".RENAME:ROBFullEvents")
        .desc("Number of times rename has blocked due to ROB full")
        .prereq(renameROBFullEvents);
    renameIQFullEvents
        .name(name() + ".RENAME:IQFullEvents")
        .desc("Number of times rename has blocked due to IQ full")
        .prereq(renameIQFullEvents);
    renameLSQFullEvents
        .name(name() + ".RENAME:LSQFullEvents")
        .desc("Number of times rename has blocked due to LSQ full")
        .prereq(renameLSQFullEvents);
    renameFullRegistersEvents
        .name(name() + ".RENAME:FullRegisterEvents")
        .desc("Number of times there has been no free registers")
        .prereq(renameFullRegistersEvents);
    renameRenamedOperands
        .name(name() + ".RENAME:RenamedOperands")
        .desc("Number of destination operands rename has renamed")
        .prereq(renameRenamedOperands);
    renameRenameLookups
        .name(name() + ".RENAME:RenameLookups")
        .desc("Number of register rename lookups that rename has made")
        .prereq(renameRenameLookups);
    renameCommittedMaps
        .name(name() + ".RENAME:CommittedMaps")
        .desc("Number of HB maps that are committed")
        .prereq(renameCommittedMaps);
    renameUndoneMaps
        .name(name() + ".RENAME:UndoneMaps")
        .desc("Number of HB maps that are undone due to squashing")
        .prereq(renameUndoneMaps);
    renamedSerializing
        .name(name() + ".RENAME:serializingInsts")
        .desc("count of serializing insts renamed")
        .flags(Stats::total)
        ;
    renamedTempSerializing
        .name(name() + ".RENAME:tempSerializingInsts")
        .desc("count of temporary serializing insts renamed")
        .flags(Stats::total)
        ;
    renameSkidInsts
        .name(name() + ".RENAME:skidInsts")
        .desc("count of insts added to the skid buffer")
        .flags(Stats::total)
        ;
}

template <class Impl>
void
DefaultRename<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(Rename, "Setting CPU pointer.\n");
    cpu = cpu_ptr;
}

template <class Impl>
void
DefaultRename<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(Rename, "Setting time buffer pointer.\n");
    timeBuffer = tb_ptr;

    // Setup wire to read information from time buffer, from IEW stage.
    fromIEW = timeBuffer->getWire(-iewToRenameDelay);

    // Setup wire to read infromation from time buffer, from commit stage.
    fromCommit = timeBuffer->getWire(-commitToRenameDelay);

    // Setup wire to write information to previous stages.
    toDecode = timeBuffer->getWire(0);
}

template <class Impl>
void
DefaultRename<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    DPRINTF(Rename, "Setting rename queue pointer.\n");
    renameQueue = rq_ptr;

    // Setup wire to write information to future stages.
    toIEW = renameQueue->getWire(0);
}

template <class Impl>
void
DefaultRename<Impl>::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    DPRINTF(Rename, "Setting decode queue pointer.\n");
    decodeQueue = dq_ptr;

    // Setup wire to get information from decode.
    fromDecode = decodeQueue->getWire(-decodeToRenameDelay);
}

template <class Impl>
void
DefaultRename<Impl>::initStage()
{
    // Grab the number of free entries directly from the stages.
    for (int tid=0; tid < numThreads; tid++) {
        freeEntries[tid].iqEntries = iew_ptr->instQueue.numFreeEntries(tid);
        freeEntries[tid].lsqEntries = iew_ptr->ldstQueue.numFreeEntries(tid);
        freeEntries[tid].robEntries = commit_ptr->numROBFreeEntries(tid);
        emptyROB[tid] = true;
    }
}

template<class Impl>
void
DefaultRename<Impl>::setActiveThreads(list<unsigned> *at_ptr)
{
    DPRINTF(Rename, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}


template <class Impl>
void
DefaultRename<Impl>::setRenameMap(RenameMap rm_ptr[])
{
    DPRINTF(Rename, "Setting rename map pointers.\n");

    for (int i=0; i<numThreads; i++) {
        renameMap[i] = &rm_ptr[i];
    }
}

template <class Impl>
void
DefaultRename<Impl>::setFreeList(FreeList *fl_ptr)
{
    DPRINTF(Rename, "Setting free list pointer.\n");
    freeList = fl_ptr;
}

template<class Impl>
void
DefaultRename<Impl>::setScoreboard(Scoreboard *_scoreboard)
{
    DPRINTF(Rename, "Setting scoreboard pointer.\n");
    scoreboard = _scoreboard;
}

template <class Impl>
void
DefaultRename<Impl>::switchOut()
{
    cpu->signalSwitched();
}

template <class Impl>
void
DefaultRename<Impl>::doSwitchOut()
{
    for (int i = 0; i < numThreads; i++) {
        typename list<RenameHistory>::iterator hb_it = historyBuffer[i].begin();

        while (!historyBuffer[i].empty()) {
            assert(hb_it != historyBuffer[i].end());

            DPRINTF(Rename, "[tid:%u]: Removing history entry with sequence "
                    "number %i.\n", i, (*hb_it).instSeqNum);

            // Tell the rename map to set the architected register to the
            // previous physical register that it was renamed to.
            renameMap[i]->setEntry(hb_it->archReg, hb_it->prevPhysReg);

            // Put the renamed physical register back on the free list.
            freeList->addReg(hb_it->newPhysReg);

            historyBuffer[i].erase(hb_it++);
        }
        insts[i].clear();
        skidBuffer[i].clear();
    }
}

template <class Impl>
void
DefaultRename<Impl>::takeOverFrom()
{
    _status = Inactive;
    initStage();

    // Reset all state prior to taking over from the other CPU.
    for (int i=0; i< numThreads; i++) {
        renameStatus[i] = Idle;

        stalls[i].iew = false;
        stalls[i].commit = false;
        serializeInst[i] = NULL;

        instsInProgress[i] = 0;

        emptyROB[i] = true;

        serializeOnNextInst[i] = false;
    }
}

template <class Impl>
void
DefaultRename<Impl>::squash(unsigned tid)
{
    DPRINTF(Rename, "[tid:%u]: Squashing instructions.\n",tid);

    // Clear the stall signal if rename was blocked or unblocking before.
    // If it still needs to block, the blocking should happen the next
    // cycle and there should be space to hold everything due to the squash.
    if (renameStatus[tid] == Blocked ||
        renameStatus[tid] == Unblocking ||
        renameStatus[tid] == SerializeStall) {
#if 0
        // In syscall emulation, we can have both a block and a squash due
        // to a syscall in the same cycle.  This would cause both signals to
        // be high.  This shouldn't happen in full system.
        if (toDecode->renameBlock[tid]) {
            toDecode->renameBlock[tid] = 0;
        } else {
            toDecode->renameUnblock[tid] = 1;
        }
#else
        toDecode->renameUnblock[tid] = 1;
#endif
        serializeInst[tid] = NULL;
    }

    // Set the status to Squashing.
    renameStatus[tid] = Squashing;

    // Squash any instructions from decode.
    unsigned squashCount = 0;

    for (int i=0; i<fromDecode->size; i++) {
        if (fromDecode->insts[i]->threadNumber == tid) {
            fromDecode->insts[i]->squashed = true;
            wroteToTimeBuffer = true;
            squashCount++;
        }
    }

    insts[tid].clear();

    // Clear the skid buffer in case it has any data in it.
    skidBuffer[tid].clear();

    doSquash(tid);
}

template <class Impl>
void
DefaultRename<Impl>::tick()
{
    wroteToTimeBuffer = false;

    blockThisCycle = false;

    bool status_change = false;

    toIEWIndex = 0;

    sortInsts();

    list<unsigned>::iterator threads = (*activeThreads).begin();

    // Check stall and squash signals.
    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        DPRINTF(Rename, "Processing [tid:%i]\n", tid);

        status_change = checkSignalsAndUpdate(tid) || status_change;

        rename(status_change, tid);
    }

    if (status_change) {
        updateStatus();
    }

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }

    threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        // If we committed this cycle then doneSeqNum will be > 0
        if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
            !fromCommit->commitInfo[tid].squash &&
            renameStatus[tid] != Squashing) {

            removeFromHistory(fromCommit->commitInfo[tid].doneSeqNum,
                                  tid);
        }
    }

    // @todo: make into updateProgress function
    for (int tid=0; tid < numThreads; tid++) {
        instsInProgress[tid] -= fromIEW->iewInfo[tid].dispatched;

        assert(instsInProgress[tid] >=0);
    }

}

template<class Impl>
void
DefaultRename<Impl>::rename(bool &status_change, unsigned tid)
{
    // If status is Running or idle,
    //     call renameInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from decode
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (renameStatus[tid] == Blocked) {
        ++renameBlockCycles;
    } else if (renameStatus[tid] == Squashing) {
        ++renameSquashCycles;
    } else if (renameStatus[tid] == SerializeStall) {
        ++renameSerializeStallCycles;
    }

    if (renameStatus[tid] == Running ||
        renameStatus[tid] == Idle) {
        DPRINTF(Rename, "[tid:%u]: Not blocked, so attempting to run "
                "stage.\n", tid);

        renameInsts(tid);
    } else if (renameStatus[tid] == Unblocking) {
        renameInsts(tid);

        if (validInsts()) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        // If we switched over to blocking, then there's a potential for
        // an overall status change.
        status_change = unblock(tid) || status_change || blockThisCycle;
    }
}

template <class Impl>
void
DefaultRename<Impl>::renameInsts(unsigned tid)
{
    // Instructions can be either in the skid buffer or the queue of
    // instructions coming from decode, depending on the status.
    int insts_available = renameStatus[tid] == Unblocking ?
        skidBuffer[tid].size() : insts[tid].size();

    // Check the decode queue to see if instructions are available.
    // If there are no available instructions to rename, then do nothing.
    if (insts_available == 0) {
        DPRINTF(Rename, "[tid:%u]: Nothing to do, breaking out early.\n",
                tid);
        // Should I change status to idle?
        ++renameIdleCycles;
        return;
    } else if (renameStatus[tid] == Unblocking) {
        ++renameUnblockCycles;
    } else if (renameStatus[tid] == Running) {
        ++renameRunCycles;
    }

    DynInstPtr inst;

    // Will have to do a different calculation for the number of free
    // entries.
    int free_rob_entries = calcFreeROBEntries(tid);
    int free_iq_entries  = calcFreeIQEntries(tid);
    int free_lsq_entries = calcFreeLSQEntries(tid);
    int min_free_entries = free_rob_entries;

    FullSource source = ROB;

    if (free_iq_entries < min_free_entries) {
        min_free_entries = free_iq_entries;
        source = IQ;
    }

    if (free_lsq_entries < min_free_entries) {
        min_free_entries = free_lsq_entries;
        source = LSQ;
    }

    // Check if there's any space left.
    if (min_free_entries <= 0) {
        DPRINTF(Rename, "[tid:%u]: Blocking due to no free ROB/IQ/LSQ "
                "entries.\n"
                "ROB has %i free entries.\n"
                "IQ has %i free entries.\n"
                "LSQ has %i free entries.\n",
                tid,
                free_rob_entries,
                free_iq_entries,
                free_lsq_entries);

        blockThisCycle = true;

        block(tid);

        incrFullStat(source);

        return;
    } else if (min_free_entries < insts_available) {
        DPRINTF(Rename, "[tid:%u]: Will have to block this cycle."
                "%i insts available, but only %i insts can be "
                "renamed due to ROB/IQ/LSQ limits.\n",
                tid, insts_available, min_free_entries);

        insts_available = min_free_entries;

        blockThisCycle = true;

        incrFullStat(source);
    }

    InstQueue &insts_to_rename = renameStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];

    DPRINTF(Rename, "[tid:%u]: %i available instructions to "
            "send iew.\n", tid, insts_available);

    DPRINTF(Rename, "[tid:%u]: %i insts pipelining from Rename | %i insts "
            "dispatched to IQ last cycle.\n",
            tid, instsInProgress[tid], fromIEW->iewInfo[tid].dispatched);

    // Handle serializing the next instruction if necessary.
    if (serializeOnNextInst[tid]) {
        if (emptyROB[tid] && instsInProgress[tid] == 0) {
            // ROB already empty; no need to serialize.
            serializeOnNextInst[tid] = false;
        } else if (!insts_to_rename.empty()) {
            insts_to_rename.front()->setSerializeBefore();
        }
    }

    int renamed_insts = 0;

    while (insts_available > 0 &&  toIEWIndex < renameWidth) {
        DPRINTF(Rename, "[tid:%u]: Sending instructions to IEW.\n", tid);

        assert(!insts_to_rename.empty());

        inst = insts_to_rename.front();

        insts_to_rename.pop_front();

        if (renameStatus[tid] == Unblocking) {
            DPRINTF(Rename,"[tid:%u]: Removing [sn:%lli] PC:%#x from rename "
                    "skidBuffer\n",
                    tid, inst->seqNum, inst->readPC());
        }

        if (inst->isSquashed()) {
            DPRINTF(Rename, "[tid:%u]: instruction %i with PC %#x is "
                    "squashed, skipping.\n",
                    tid, inst->seqNum, inst->threadNumber,inst->readPC());

            ++renameSquashedInsts;

            // Decrement how many instructions are available.
            --insts_available;

            continue;
        }

        DPRINTF(Rename, "[tid:%u]: Processing instruction [sn:%lli] with "
                "PC %#x.\n",
                tid, inst->seqNum, inst->readPC());

        // Handle serializeAfter/serializeBefore instructions.
        // serializeAfter marks the next instruction as serializeBefore.
        // serializeBefore makes the instruction wait in rename until the ROB
        // is empty.

        // In this model, IPR accesses are serialize before
        // instructions, and store conditionals are serialize after
        // instructions.  This is mainly due to lack of support for
        // out-of-order operations of either of those classes of
        // instructions.
        if ((inst->isIprAccess() || inst->isSerializeBefore()) &&
            !inst->isSerializeHandled()) {
            DPRINTF(Rename, "Serialize before instruction encountered.\n");

            if (!inst->isTempSerializeBefore()) {
                renamedSerializing++;
                inst->setSerializeHandled();
            } else {
                renamedTempSerializing++;
            }

            // Change status over to SerializeStall so that other stages know
            // what this is blocked on.
            renameStatus[tid] = SerializeStall;

            serializeInst[tid] = inst;

            blockThisCycle = true;

            break;
        } else if ((inst->isStoreConditional() || inst->isSerializeAfter()) &&
                   !inst->isSerializeHandled()) {
            DPRINTF(Rename, "Serialize after instruction encountered.\n");

            renamedSerializing++;

            inst->setSerializeHandled();

            serializeAfter(insts_to_rename, tid);
        }

        // Check here to make sure there are enough destination registers
        // to rename to.  Otherwise block.
        if (renameMap[tid]->numFreeEntries() < inst->numDestRegs()) {
            DPRINTF(Rename, "Blocking due to lack of free "
                    "physical registers to rename to.\n");
            blockThisCycle = true;

            ++renameFullRegistersEvents;

            break;
        }

        renameSrcRegs(inst, inst->threadNumber);

        renameDestRegs(inst, inst->threadNumber);

        ++renamed_insts;

        // Put instruction in rename queue.
        toIEW->insts[toIEWIndex] = inst;
        ++(toIEW->size);

        // Increment which instruction we're on.
        ++toIEWIndex;

        // Decrement how many instructions are available.
        --insts_available;
    }

    instsInProgress[tid] += renamed_insts;
    renameRenamedInsts += renamed_insts;

    // If we wrote to the time buffer, record this.
    if (toIEWIndex) {
        wroteToTimeBuffer = true;
    }

    // Check if there's any instructions left that haven't yet been renamed.
    // If so then block.
    if (insts_available) {
        blockThisCycle = true;
    }

    if (blockThisCycle) {
        block(tid);
        toDecode->renameUnblock[tid] = false;
    }
}

template<class Impl>
void
DefaultRename<Impl>::skidInsert(unsigned tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop_front();

        assert(tid == inst->threadNumber);

        DPRINTF(Rename, "[tid:%u]: Inserting [sn:%lli] PC:%#x into Rename "
                "skidBuffer\n", tid, inst->seqNum, inst->readPC());

        ++renameSkidInsts;

        skidBuffer[tid].push_back(inst);
    }

    if (skidBuffer[tid].size() > skidBufferMax)
        panic("Skidbuffer Exceeded Max Size");
}

template <class Impl>
void
DefaultRename<Impl>::sortInsts()
{
    int insts_from_decode = fromDecode->size;
#ifdef DEBUG
    for (int i=0; i < numThreads; i++)
        assert(insts[i].empty());
#endif
    for (int i = 0; i < insts_from_decode; ++i) {
        DynInstPtr inst = fromDecode->insts[i];
        insts[inst->threadNumber].push_back(inst);
    }
}

template<class Impl>
bool
DefaultRename<Impl>::skidsEmpty()
{
    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        if (!skidBuffer[*threads++].empty())
            return false;
    }

    return true;
}

template<class Impl>
void
DefaultRename<Impl>::updateStatus()
{
    bool any_unblocking = false;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (renameStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // Rename will have activity if it's unblocking.
    if (any_unblocking) {
        if (_status == Inactive) {
            _status = Active;

            DPRINTF(Activity, "Activating stage.\n");

            cpu->activateStage(FullCPU::RenameIdx);
        }
    } else {
        // If it's not unblocking, then rename will not have any internal
        // activity.  Switch it to inactive.
        if (_status == Active) {
            _status = Inactive;
            DPRINTF(Activity, "Deactivating stage.\n");

            cpu->deactivateStage(FullCPU::RenameIdx);
        }
    }
}

template <class Impl>
bool
DefaultRename<Impl>::block(unsigned tid)
{
    DPRINTF(Rename, "[tid:%u]: Blocking.\n", tid);

    // Add the current inputs onto the skid buffer, so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    // Only signal backwards to block if the previous stages do not think
    // rename is already blocked.
    if (renameStatus[tid] != Blocked) {
        if (renameStatus[tid] != Unblocking) {
            toDecode->renameBlock[tid] = true;
            toDecode->renameUnblock[tid] = false;
            wroteToTimeBuffer = true;
        }

        // Rename can not go from SerializeStall to Blocked, otherwise
        // it would not know to complete the serialize stall.
        if (renameStatus[tid] != SerializeStall) {
            // Set status to Blocked.
            renameStatus[tid] = Blocked;
            return true;
        }
    }

    return false;
}

template <class Impl>
bool
DefaultRename<Impl>::unblock(unsigned tid)
{
    DPRINTF(Rename, "[tid:%u]: Trying to unblock.\n", tid);

    // Rename is done unblocking if the skid buffer is empty.
    if (skidBuffer[tid].empty() && renameStatus[tid] != SerializeStall) {

        DPRINTF(Rename, "[tid:%u]: Done unblocking.\n", tid);

        toDecode->renameUnblock[tid] = true;
        wroteToTimeBuffer = true;

        renameStatus[tid] = Running;
        return true;
    }

    return false;
}

template <class Impl>
void
DefaultRename<Impl>::doSquash(unsigned tid)
{
    typename list<RenameHistory>::iterator hb_it = historyBuffer[tid].begin();

    InstSeqNum squashed_seq_num = fromCommit->commitInfo[tid].doneSeqNum;

    // After a syscall squashes everything, the history buffer may be empty
    // but the ROB may still be squashing instructions.
    if (historyBuffer[tid].empty()) {
        return;
    }

    // Go through the most recent instructions, undoing the mappings
    // they did and freeing up the registers.
    while (!historyBuffer[tid].empty() &&
           (*hb_it).instSeqNum > squashed_seq_num) {
        assert(hb_it != historyBuffer[tid].end());

        DPRINTF(Rename, "[tid:%u]: Removing history entry with sequence "
                "number %i.\n", tid, (*hb_it).instSeqNum);

        // Tell the rename map to set the architected register to the
        // previous physical register that it was renamed to.
        renameMap[tid]->setEntry(hb_it->archReg, hb_it->prevPhysReg);

        // Put the renamed physical register back on the free list.
        freeList->addReg(hb_it->newPhysReg);

        historyBuffer[tid].erase(hb_it++);

        ++renameUndoneMaps;
    }
}

template<class Impl>
void
DefaultRename<Impl>::removeFromHistory(InstSeqNum inst_seq_num, unsigned tid)
{
    DPRINTF(Rename, "[tid:%u]: Removing a committed instruction from the "
            "history buffer %u (size=%i), until [sn:%lli].\n",
            tid, tid, historyBuffer[tid].size(), inst_seq_num);

    typename list<RenameHistory>::iterator hb_it = historyBuffer[tid].end();

    --hb_it;

    if (historyBuffer[tid].empty()) {
        DPRINTF(Rename, "[tid:%u]: History buffer is empty.\n", tid);
        return;
    } else if (hb_it->instSeqNum > inst_seq_num) {
        DPRINTF(Rename, "[tid:%u]: Old sequence number encountered.  Ensure "
                "that a syscall happened recently.\n", tid);
        return;
    }

    // Commit all the renames up until (and including) the committed sequence
    // number. Some or even all of the committed instructions may not have
    // rename histories if they did not have destination registers that were
    // renamed.
    while (!historyBuffer[tid].empty() &&
           hb_it != historyBuffer[tid].end() &&
           (*hb_it).instSeqNum <= inst_seq_num) {

        DPRINTF(Rename, "[tid:%u]: Freeing up older rename of reg %i, "
                "[sn:%lli].\n",
                tid, (*hb_it).prevPhysReg, (*hb_it).instSeqNum);

        freeList->addReg((*hb_it).prevPhysReg);
        ++renameCommittedMaps;

        historyBuffer[tid].erase(hb_it--);
    }
}

template <class Impl>
inline void
DefaultRename<Impl>::renameSrcRegs(DynInstPtr &inst,unsigned tid)
{
    assert(renameMap[tid] != 0);

    unsigned num_src_regs = inst->numSrcRegs();

    // Get the architectual register numbers from the source and
    // destination operands, and redirect them to the right register.
    // Will need to mark dependencies though.
    for (int src_idx = 0; src_idx < num_src_regs; src_idx++) {
        RegIndex src_reg = inst->srcRegIdx(src_idx);

        // Look up the source registers to get the phys. register they've
        // been renamed to, and set the sources to those registers.
        PhysRegIndex renamed_reg = renameMap[tid]->lookup(src_reg);

        DPRINTF(Rename, "[tid:%u]: Looking up arch reg %i, got "
                "physical reg %i.\n", tid, (int)src_reg,
                (int)renamed_reg);

        inst->renameSrcReg(src_idx, renamed_reg);

        // See if the register is ready or not.
        if (scoreboard->getReg(renamed_reg) == true) {
            DPRINTF(Rename, "[tid:%u]: Register is ready.\n", tid);

            inst->markSrcRegReady(src_idx);
        }

        ++renameRenameLookups;
    }
}

template <class Impl>
inline void
DefaultRename<Impl>::renameDestRegs(DynInstPtr &inst,unsigned tid)
{
    typename RenameMap::RenameInfo rename_result;

    unsigned num_dest_regs = inst->numDestRegs();

    // Rename the destination registers.
    for (int dest_idx = 0; dest_idx < num_dest_regs; dest_idx++) {
        RegIndex dest_reg = inst->destRegIdx(dest_idx);

        // Get the physical register that the destination will be
        // renamed to.
        rename_result = renameMap[tid]->rename(dest_reg);

        //Mark Scoreboard entry as not ready
        scoreboard->unsetReg(rename_result.first);

        DPRINTF(Rename, "[tid:%u]: Renaming arch reg %i to physical "
                "reg %i.\n", tid, (int)dest_reg,
                (int)rename_result.first);

        // Record the rename information so that a history can be kept.
        RenameHistory hb_entry(inst->seqNum, dest_reg,
                               rename_result.first,
                               rename_result.second);

        historyBuffer[tid].push_front(hb_entry);

        DPRINTF(Rename, "[tid:%u]: Adding instruction to history buffer, "
                "[sn:%lli].\n",tid,
                (*historyBuffer[tid].begin()).instSeqNum);

        // Tell the instruction to rename the appropriate destination
        // register (dest_idx) to the new physical register
        // (rename_result.first), and record the previous physical
        // register that the same logical register was renamed to
        // (rename_result.second).
        inst->renameDestReg(dest_idx,
                            rename_result.first,
                            rename_result.second);

        ++renameRenamedOperands;
    }
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeROBEntries(unsigned tid)
{
    int num_free = freeEntries[tid].robEntries -
                  (instsInProgress[tid] - fromIEW->iewInfo[tid].dispatched);

    //DPRINTF(Rename,"[tid:%i]: %i rob free\n",tid,num_free);

    return num_free;
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeIQEntries(unsigned tid)
{
    int num_free = freeEntries[tid].iqEntries -
                  (instsInProgress[tid] - fromIEW->iewInfo[tid].dispatched);

    //DPRINTF(Rename,"[tid:%i]: %i iq free\n",tid,num_free);

    return num_free;
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeLSQEntries(unsigned tid)
{
    int num_free = freeEntries[tid].lsqEntries -
                  (instsInProgress[tid] - fromIEW->iewInfo[tid].dispatchedToLSQ);

    //DPRINTF(Rename,"[tid:%i]: %i lsq free\n",tid,num_free);

    return num_free;
}

template <class Impl>
unsigned
DefaultRename<Impl>::validInsts()
{
    unsigned inst_count = 0;

    for (int i=0; i<fromDecode->size; i++) {
        if (!fromDecode->insts[i]->squashed)
            inst_count++;
    }

    return inst_count;
}

template <class Impl>
void
DefaultRename<Impl>::readStallSignals(unsigned tid)
{
    if (fromIEW->iewBlock[tid]) {
        stalls[tid].iew = true;
    }

    if (fromIEW->iewUnblock[tid]) {
        assert(stalls[tid].iew);
        stalls[tid].iew = false;
    }

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
DefaultRename<Impl>::checkStall(unsigned tid)
{
    bool ret_val = false;

    if (stalls[tid].iew) {
        DPRINTF(Rename,"[tid:%i]: Stall from IEW stage detected.\n", tid);
        ret_val = true;
    } else if (stalls[tid].commit) {
        DPRINTF(Rename,"[tid:%i]: Stall from Commit stage detected.\n", tid);
        ret_val = true;
    } else if (calcFreeROBEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: ROB has 0 free entries.\n", tid);
        ret_val = true;
    } else if (calcFreeIQEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: IQ has 0 free entries.\n", tid);
        ret_val = true;
    } else if (calcFreeLSQEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: LSQ has 0 free entries.\n", tid);
        ret_val = true;
    } else if (renameMap[tid]->numFreeEntries() <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: RenameMap has 0 free entries.\n", tid);
        ret_val = true;
    } else if (renameStatus[tid] == SerializeStall &&
               (!emptyROB[tid] || instsInProgress[tid])) {
        DPRINTF(Rename,"[tid:%i]: Stall: Serialize stall and ROB is not "
                "empty.\n",
                tid);
        ret_val = true;
    }

    return ret_val;
}

template <class Impl>
void
DefaultRename<Impl>::readFreeEntries(unsigned tid)
{
    bool updated = false;
    if (fromIEW->iewInfo[tid].usedIQ) {
        freeEntries[tid].iqEntries =
            fromIEW->iewInfo[tid].freeIQEntries;
        updated = true;
    }

    if (fromIEW->iewInfo[tid].usedLSQ) {
        freeEntries[tid].lsqEntries =
            fromIEW->iewInfo[tid].freeLSQEntries;
        updated = true;
    }

    if (fromCommit->commitInfo[tid].usedROB) {
        freeEntries[tid].robEntries =
            fromCommit->commitInfo[tid].freeROBEntries;
        emptyROB[tid] = fromCommit->commitInfo[tid].emptyROB;
        updated = true;
    }

    DPRINTF(Rename, "[tid:%i]: Free IQ: %i, Free ROB: %i, Free LSQ: %i\n",
            tid,
            freeEntries[tid].iqEntries,
            freeEntries[tid].robEntries,
            freeEntries[tid].lsqEntries);

    DPRINTF(Rename, "[tid:%i]: %i instructions not yet in ROB\n",
            tid, instsInProgress[tid]);
}

template <class Impl>
bool
DefaultRename<Impl>::checkSignalsAndUpdate(unsigned tid)
{
    // Check if there's a squash signal, squash if there is
    // Check stall signals, block if necessary.
    // If status was blocked
    //     check if stall conditions have passed
    //         if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.
    // If status was serialize stall
    //     check if ROB is empty and no insts are in flight to the ROB

    readFreeEntries(tid);
    readStallSignals(tid);

    if (fromCommit->commitInfo[tid].squash) {
        DPRINTF(Rename, "[tid:%u]: Squashing instructions due to squash from "
                "commit.\n", tid);

        squash(tid);

        return true;
    }

    if (fromCommit->commitInfo[tid].robSquashing) {
        DPRINTF(Rename, "[tid:%u]: ROB is still squashing.\n", tid);

        renameStatus[tid] = Squashing;

        return true;
    }

    if (checkStall(tid)) {
        return block(tid);
    }

    if (renameStatus[tid] == Blocked) {
        DPRINTF(Rename, "[tid:%u]: Done blocking, switching to unblocking.\n",
                tid);

        renameStatus[tid] = Unblocking;

        unblock(tid);

        return true;
    }

    if (renameStatus[tid] == Squashing) {
        // Switch status to running if rename isn't being told to block or
        // squash this cycle.
        DPRINTF(Rename, "[tid:%u]: Done squashing, switching to running.\n",
                tid);

        renameStatus[tid] = Running;

        return false;
    }

    if (renameStatus[tid] == SerializeStall) {
        // Stall ends once the ROB is free.
        DPRINTF(Rename, "[tid:%u]: Done with serialize stall, switching to "
                "unblocking.\n", tid);

        DynInstPtr serial_inst = serializeInst[tid];

        renameStatus[tid] = Unblocking;

        unblock(tid);

        DPRINTF(Rename, "[tid:%u]: Processing instruction [%lli] with "
                "PC %#x.\n",
                tid, serial_inst->seqNum, serial_inst->readPC());

        // Put instruction into queue here.
        serial_inst->clearSerializeBefore();

        if (!skidBuffer[tid].empty()) {
            skidBuffer[tid].push_front(serial_inst);
        } else {
            insts[tid].push_front(serial_inst);
        }

        DPRINTF(Rename, "[tid:%u]: Instruction must be processed by rename."
                " Adding to front of list.", tid);

        serializeInst[tid] = NULL;

        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause rename to change its status.  Rename remains the same as before.
    return false;
}

template<class Impl>
void
DefaultRename<Impl>::serializeAfter(InstQueue &inst_list,
                                   unsigned tid)
{
    if (inst_list.empty()) {
        // Mark a bit to say that I must serialize on the next instruction.
        serializeOnNextInst[tid] = true;
        return;
    }

    // Set the next instruction as serializing.
    inst_list.front()->setSerializeBefore();
}

template <class Impl>
inline void
DefaultRename<Impl>::incrFullStat(const FullSource &source)
{
    switch (source) {
      case ROB:
        ++renameROBFullEvents;
        break;
      case IQ:
        ++renameIQFullEvents;
        break;
      case LSQ:
        ++renameLSQFullEvents;
        break;
      default:
        panic("Rename full stall stat should be incremented for a reason!");
        break;
    }
}

template <class Impl>
void
DefaultRename<Impl>::dumpHistory()
{
    typename list<RenameHistory>::iterator buf_it;

    for (int i = 0; i < numThreads; i++) {

        buf_it = historyBuffer[i].begin();

        while (buf_it != historyBuffer[i].end()) {
            cprintf("Seq num: %i\nArch reg: %i New phys reg: %i Old phys "
                    "reg: %i\n", (*buf_it).instSeqNum, (int)(*buf_it).archReg,
                    (int)(*buf_it).newPhysReg, (int)(*buf_it).prevPhysReg);

            buf_it++;
        }
    }
}
