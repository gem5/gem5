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

#include "cpu/o3/decode.hh"

using namespace std;

template<class Impl>
DefaultDecode<Impl>::DefaultDecode(Params *params)
    : renameToDecodeDelay(params->renameToDecodeDelay),
      iewToDecodeDelay(params->iewToDecodeDelay),
      commitToDecodeDelay(params->commitToDecodeDelay),
      fetchToDecodeDelay(params->fetchToDecodeDelay),
      decodeWidth(params->decodeWidth),
      numThreads(params->numberOfThreads)
{
    _status = Inactive;

    // Setup status, make sure stall signals are clear.
    for (int i = 0; i < numThreads; ++i) {
        decodeStatus[i] = Idle;

        stalls[i].rename = false;
        stalls[i].iew = false;
        stalls[i].commit = false;
    }

    // @todo: Make into a parameter
    skidBufferMax = (fetchToDecodeDelay * params->fetchWidth) + decodeWidth;
}

template <class Impl>
std::string
DefaultDecode<Impl>::name() const
{
    return cpu->name() + ".decode";
}

template <class Impl>
void
DefaultDecode<Impl>::regStats()
{
    decodeIdleCycles
        .name(name() + ".DECODE:IdleCycles")
        .desc("Number of cycles decode is idle")
        .prereq(decodeIdleCycles);
    decodeBlockedCycles
        .name(name() + ".DECODE:BlockedCycles")
        .desc("Number of cycles decode is blocked")
        .prereq(decodeBlockedCycles);
    decodeRunCycles
        .name(name() + ".DECODE:RunCycles")
        .desc("Number of cycles decode is running")
        .prereq(decodeRunCycles);
    decodeUnblockCycles
        .name(name() + ".DECODE:UnblockCycles")
        .desc("Number of cycles decode is unblocking")
        .prereq(decodeUnblockCycles);
    decodeSquashCycles
        .name(name() + ".DECODE:SquashCycles")
        .desc("Number of cycles decode is squashing")
        .prereq(decodeSquashCycles);
    decodeBranchResolved
        .name(name() + ".DECODE:BranchResolved")
        .desc("Number of times decode resolved a branch")
        .prereq(decodeBranchResolved);
    decodeBranchMispred
        .name(name() + ".DECODE:BranchMispred")
        .desc("Number of times decode detected a branch misprediction")
        .prereq(decodeBranchMispred);
    decodeControlMispred
        .name(name() + ".DECODE:ControlMispred")
        .desc("Number of times decode detected an instruction incorrectly"
              " predicted as a control")
        .prereq(decodeControlMispred);
    decodeDecodedInsts
        .name(name() + ".DECODE:DecodedInsts")
        .desc("Number of instructions handled by decode")
        .prereq(decodeDecodedInsts);
    decodeSquashedInsts
        .name(name() + ".DECODE:SquashedInsts")
        .desc("Number of squashed instructions handled by decode")
        .prereq(decodeSquashedInsts);
}

template<class Impl>
void
DefaultDecode<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(Decode, "Setting CPU pointer.\n");
    cpu = cpu_ptr;
}

template<class Impl>
void
DefaultDecode<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(Decode, "Setting time buffer pointer.\n");
    timeBuffer = tb_ptr;

    // Setup wire to write information back to fetch.
    toFetch = timeBuffer->getWire(0);

    // Create wires to get information from proper places in time buffer.
    fromRename = timeBuffer->getWire(-renameToDecodeDelay);
    fromIEW = timeBuffer->getWire(-iewToDecodeDelay);
    fromCommit = timeBuffer->getWire(-commitToDecodeDelay);
}

template<class Impl>
void
DefaultDecode<Impl>::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    DPRINTF(Decode, "Setting decode queue pointer.\n");
    decodeQueue = dq_ptr;

    // Setup wire to write information to proper place in decode queue.
    toRename = decodeQueue->getWire(0);
}

template<class Impl>
void
DefaultDecode<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    DPRINTF(Decode, "Setting fetch queue pointer.\n");
    fetchQueue = fq_ptr;

    // Setup wire to read information from fetch queue.
    fromFetch = fetchQueue->getWire(-fetchToDecodeDelay);
}

template<class Impl>
void
DefaultDecode<Impl>::setActiveThreads(list<unsigned> *at_ptr)
{
    DPRINTF(Decode, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}

template <class Impl>
void
DefaultDecode<Impl>::switchOut()
{
    // Decode can immediately switch out.
    cpu->signalSwitched();
}

template <class Impl>
void
DefaultDecode<Impl>::takeOverFrom()
{
    _status = Inactive;

    // Be sure to reset state and clear out any old instructions.
    for (int i = 0; i < numThreads; ++i) {
        decodeStatus[i] = Idle;

        stalls[i].rename = false;
        stalls[i].iew = false;
        stalls[i].commit = false;
        while (!insts[i].empty())
            insts[i].pop();
        while (!skidBuffer[i].empty())
            skidBuffer[i].pop();
        branchCount[i] = 0;
    }
    wroteToTimeBuffer = false;
}

template<class Impl>
bool
DefaultDecode<Impl>::checkStall(unsigned tid) const
{
    bool ret_val = false;

    if (stalls[tid].rename) {
        DPRINTF(Decode,"[tid:%i]: Stall fom Rename stage detected.\n", tid);
        ret_val = true;
    } else if (stalls[tid].iew) {
        DPRINTF(Decode,"[tid:%i]: Stall fom IEW stage detected.\n", tid);
        ret_val = true;
    } else if (stalls[tid].commit) {
        DPRINTF(Decode,"[tid:%i]: Stall fom Commit stage detected.\n", tid);
        ret_val = true;
    }

    return ret_val;
}

template<class Impl>
inline bool
DefaultDecode<Impl>::fetchInstsValid()
{
    return fromFetch->size > 0;
}

template<class Impl>
bool
DefaultDecode<Impl>::block(unsigned tid)
{
    DPRINTF(Decode, "[tid:%u]: Blocking.\n", tid);

    // Add the current inputs to the skid buffer so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    // If the decode status is blocked or unblocking then decode has not yet
    // signalled fetch to unblock. In that case, there is no need to tell
    // fetch to block.
    if (decodeStatus[tid] != Blocked) {
        // Set the status to Blocked.
        decodeStatus[tid] = Blocked;

        if (decodeStatus[tid] != Unblocking) {
            toFetch->decodeBlock[tid] = true;
            wroteToTimeBuffer = true;
        }

        return true;
    }

    return false;
}

template<class Impl>
bool
DefaultDecode<Impl>::unblock(unsigned tid)
{
    // Decode is done unblocking only if the skid buffer is empty.
    if (skidBuffer[tid].empty()) {
        DPRINTF(Decode, "[tid:%u]: Done unblocking.\n", tid);
        toFetch->decodeUnblock[tid] = true;
        wroteToTimeBuffer = true;

        decodeStatus[tid] = Running;
        return true;
    }

    DPRINTF(Decode, "[tid:%u]: Currently unblocking.\n", tid);

    return false;
}

template<class Impl>
void
DefaultDecode<Impl>::squash(DynInstPtr &inst, unsigned tid)
{
    DPRINTF(Decode, "[tid:%i]: Squashing due to incorrect branch prediction "
            "detected at decode.\n", tid);

    // Send back mispredict information.
    toFetch->decodeInfo[tid].branchMispredict = true;
    toFetch->decodeInfo[tid].doneSeqNum = inst->seqNum;
    toFetch->decodeInfo[tid].predIncorrect = true;
    toFetch->decodeInfo[tid].squash = true;
    toFetch->decodeInfo[tid].nextPC = inst->readNextPC();
    toFetch->decodeInfo[tid].branchTaken =
        inst->readNextPC() != (inst->readPC() + sizeof(TheISA::MachInst));

    // Might have to tell fetch to unblock.
    if (decodeStatus[tid] == Blocked ||
        decodeStatus[tid] == Unblocking) {
        toFetch->decodeUnblock[tid] = 1;
    }

    // Set status to squashing.
    decodeStatus[tid] = Squashing;

    for (int i=0; i<fromFetch->size; i++) {
        if (fromFetch->insts[i]->threadNumber == tid &&
            fromFetch->insts[i]->seqNum > inst->seqNum) {
            fromFetch->insts[i]->squashed = true;
        }
    }

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    while (!insts[tid].empty()) {
        insts[tid].pop();
    }

    while (!skidBuffer[tid].empty()) {
        skidBuffer[tid].pop();
    }

    // Squash instructions up until this one
    cpu->removeInstsUntil(inst->seqNum, tid);
}

template<class Impl>
unsigned
DefaultDecode<Impl>::squash(unsigned tid)
{
    DPRINTF(Decode, "[tid:%i]: Squashing.\n",tid);

    if (decodeStatus[tid] == Blocked ||
        decodeStatus[tid] == Unblocking) {
#if !FULL_SYSTEM
        // In syscall emulation, we can have both a block and a squash due
        // to a syscall in the same cycle.  This would cause both signals to
        // be high.  This shouldn't happen in full system.
        // @todo: Determine if this still happens.
        if (toFetch->decodeBlock[tid]) {
            toFetch->decodeBlock[tid] = 0;
        } else {
            toFetch->decodeUnblock[tid] = 1;
        }
#else
        toFetch->decodeUnblock[tid] = 1;
#endif
    }

    // Set status to squashing.
    decodeStatus[tid] = Squashing;

    // Go through incoming instructions from fetch and squash them.
    unsigned squash_count = 0;

    for (int i=0; i<fromFetch->size; i++) {
        if (fromFetch->insts[i]->threadNumber == tid) {
            fromFetch->insts[i]->squashed = true;
            squash_count++;
        }
    }

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    while (!insts[tid].empty()) {
        insts[tid].pop();
    }

    while (!skidBuffer[tid].empty()) {
        skidBuffer[tid].pop();
    }

    return squash_count;
}

template<class Impl>
void
DefaultDecode<Impl>::skidInsert(unsigned tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop();

        assert(tid == inst->threadNumber);

        DPRINTF(Decode,"Inserting [sn:%lli] PC:%#x into decode skidBuffer %i\n",
                inst->seqNum, inst->readPC(), inst->threadNumber);

        skidBuffer[tid].push(inst);
    }

    // @todo: Eventually need to enforce this by not letting a thread
    // fetch past its skidbuffer
    assert(skidBuffer[tid].size() <= skidBufferMax);
}

template<class Impl>
bool
DefaultDecode<Impl>::skidsEmpty()
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
DefaultDecode<Impl>::updateStatus()
{
    bool any_unblocking = false;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (decodeStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // Decode will have activity if it's unblocking.
    if (any_unblocking) {
        if (_status == Inactive) {
            _status = Active;

            DPRINTF(Activity, "Activating stage.\n");

            cpu->activateStage(FullCPU::DecodeIdx);
        }
    } else {
        // If it's not unblocking, then decode will not have any internal
        // activity.  Switch it to inactive.
        if (_status == Active) {
            _status = Inactive;
            DPRINTF(Activity, "Deactivating stage.\n");

            cpu->deactivateStage(FullCPU::DecodeIdx);
        }
    }
}

template <class Impl>
void
DefaultDecode<Impl>::sortInsts()
{
    int insts_from_fetch = fromFetch->size;
#ifdef DEBUG
    for (int i=0; i < numThreads; i++)
        assert(insts[i].empty());
#endif
    for (int i = 0; i < insts_from_fetch; ++i) {
        insts[fromFetch->insts[i]->threadNumber].push(fromFetch->insts[i]);
    }
}

template<class Impl>
void
DefaultDecode<Impl>::readStallSignals(unsigned tid)
{
    if (fromRename->renameBlock[tid]) {
        stalls[tid].rename = true;
    }

    if (fromRename->renameUnblock[tid]) {
        assert(stalls[tid].rename);
        stalls[tid].rename = false;
    }

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
DefaultDecode<Impl>::checkSignalsAndUpdate(unsigned tid)
{
    // Check if there's a squash signal, squash if there is.
    // Check stall signals, block if necessary.
    // If status was blocked
    //     Check if stall conditions have passed
    //         if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.

    // Update the per thread stall statuses.
    readStallSignals(tid);

    // Check squash signals from commit.
    if (fromCommit->commitInfo[tid].squash) {

        DPRINTF(Decode, "[tid:%u]: Squashing instructions due to squash "
                "from commit.\n", tid);

        squash(tid);

        return true;
    }

    // Check ROB squash signals from commit.
    if (fromCommit->commitInfo[tid].robSquashing) {
        DPRINTF(Decode, "[tid:%]: ROB is still squashing.\n",tid);

        // Continue to squash.
        decodeStatus[tid] = Squashing;

        return true;
    }

    if (checkStall(tid)) {
        return block(tid);
    }

    if (decodeStatus[tid] == Blocked) {
        DPRINTF(Decode, "[tid:%u]: Done blocking, switching to unblocking.\n",
                tid);

        decodeStatus[tid] = Unblocking;

        unblock(tid);

        return true;
    }

    if (decodeStatus[tid] == Squashing) {
        // Switch status to running if decode isn't being told to block or
        // squash this cycle.
        DPRINTF(Decode, "[tid:%u]: Done squashing, switching to running.\n",
                tid);

        decodeStatus[tid] = Running;

        return false;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause decode to change its status.  Decode remains the same as before.
    return false;
}

template<class Impl>
void
DefaultDecode<Impl>::tick()
{
    wroteToTimeBuffer = false;

    bool status_change = false;

    toRenameIndex = 0;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    sortInsts();

    //Check stall and squash signals.
    while (threads != (*activeThreads).end()) {
    unsigned tid = *threads++;

        DPRINTF(Decode,"Processing [tid:%i]\n",tid);
        status_change =  checkSignalsAndUpdate(tid) || status_change;

        decode(status_change, tid);
    }

    if (status_change) {
        updateStatus();
    }

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");

        cpu->activityThisCycle();
    }
}

template<class Impl>
void
DefaultDecode<Impl>::decode(bool &status_change, unsigned tid)
{
    // If status is Running or idle,
    //     call decodeInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from fetch
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (decodeStatus[tid] == Blocked) {
        ++decodeBlockedCycles;
    } else if (decodeStatus[tid] == Squashing) {
        ++decodeSquashCycles;
    }

    // Decode should try to decode as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (decodeStatus[tid] == Running ||
        decodeStatus[tid] == Idle) {
        DPRINTF(Decode, "[tid:%u] Not blocked, so attempting to run "
                "stage.\n",tid);

        decodeInsts(tid);
    } else if (decodeStatus[tid] == Unblocking) {
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(!skidsEmpty());

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        decodeInsts(tid);

        if (fetchInstsValid()) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        status_change = unblock(tid) || status_change;
    }
}

template <class Impl>
void
DefaultDecode<Impl>::decodeInsts(unsigned tid)
{
    // Instructions can come either from the skid buffer or the list of
    // instructions coming from fetch, depending on decode's status.
    int insts_available = decodeStatus[tid] == Unblocking ?
        skidBuffer[tid].size() : insts[tid].size();

    if (insts_available == 0) {
        DPRINTF(Decode, "[tid:%u] Nothing to do, breaking out"
                " early.\n",tid);
        // Should I change the status to idle?
        ++decodeIdleCycles;
        return;
    } else if (decodeStatus[tid] == Unblocking) {
        DPRINTF(Decode, "[tid:%u] Unblocking, removing insts from skid "
                "buffer.\n",tid);
        ++decodeUnblockCycles;
    } else if (decodeStatus[tid] == Running) {
        ++decodeRunCycles;
    }

    DynInstPtr inst;

    std::queue<DynInstPtr>
        &insts_to_decode = decodeStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];

    DPRINTF(Decode, "[tid:%u]: Sending instruction to rename.\n",tid);

    while (insts_available > 0 && toRenameIndex < decodeWidth) {
        assert(!insts_to_decode.empty());

        inst = insts_to_decode.front();

        insts_to_decode.pop();

        DPRINTF(Decode, "[tid:%u]: Processing instruction [sn:%lli] with "
                "PC %#x\n",
                tid, inst->seqNum, inst->readPC());

        if (inst->isSquashed()) {
            DPRINTF(Decode, "[tid:%u]: Instruction %i with PC %#x is "
                    "squashed, skipping.\n",
                    tid, inst->seqNum, inst->readPC());

            ++decodeSquashedInsts;

            --insts_available;

            continue;
        }

        // Also check if instructions have no source registers.  Mark
        // them as ready to issue at any time.  Not sure if this check
        // should exist here or at a later stage; however it doesn't matter
        // too much for function correctness.
        if (inst->numSrcRegs() == 0) {
            inst->setCanIssue();
        }

        // This current instruction is valid, so add it into the decode
        // queue.  The next instruction may not be valid, so check to
        // see if branches were predicted correctly.
        toRename->insts[toRenameIndex] = inst;

        ++(toRename->size);
        ++toRenameIndex;
        ++decodeDecodedInsts;
        --insts_available;

        // Ensure that if it was predicted as a branch, it really is a
        // branch.
        if (inst->predTaken() && !inst->isControl()) {
            panic("Instruction predicted as a branch!");

            ++decodeControlMispred;

            // Might want to set some sort of boolean and just do
            // a check at the end
            squash(inst, inst->threadNumber);

            break;
        }

        // Go ahead and compute any PC-relative branches.
        if (inst->isDirectCtrl() && inst->isUncondCtrl()) {
            ++decodeBranchResolved;
            inst->setNextPC(inst->branchTarget());

            if (inst->mispredicted()) {
                ++decodeBranchMispred;

                // Might want to set some sort of boolean and just do
                // a check at the end
                squash(inst, inst->threadNumber);
                inst->setPredTarg(inst->branchTarget());

                break;
            }
        }
    }

    // If we didn't process all instructions, then we will need to block
    // and put all those instructions into the skid buffer.
    if (!insts_to_decode.empty()) {
        block(tid);
    }

    // Record that decode has written to the time buffer for activity
    // tracking.
    if (toRenameIndex) {
        wroteToTimeBuffer = true;
    }
}
