/*
 * Copyright (c) 2022-2023 The University of Edinburgh
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

#include "cpu/o3/bac.hh"

#include <algorithm>

#include "arch/generic/pcstate.hh"
#include "base/trace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/ftq.hh"
#include "cpu/o3/limits.hh"
#include "debug/Activity.hh"
#include "debug/BAC.hh"
#include "debug/Branch.hh"
#include "debug/Drain.hh"
#include "debug/Fetch.hh"
#include "debug/O3PipeView.hh"
#include "params/BaseO3CPU.hh"
#include "sim/full_system.hh"

using namespace gem5::branch_prediction;

namespace gem5
{

namespace o3
{

BAC::BAC(CPU *_cpu, const BaseO3CPUParams &params)
    : cpu(_cpu),
      bpu(params.branchPred),
      ftq(nullptr),
      wroteToTimeBuffer(false),
      decoupledFrontEnd(params.decoupledFrontEnd),
      fetchToBacDelay(params.fetchToBacDelay),
      decodeToFetchDelay(params.decodeToFetchDelay),
      commitToFetchDelay(params.commitToFetchDelay),
      bacToFetchDelay(params.bacToFetchDelay),
      fetchTargetWidth(params.fetchTargetWidth),
      minInstSize(params.minInstSize),
      numThreads(params.numThreads),
      stats(_cpu,this)
{
    fatal_if(decoupledFrontEnd && (fetchTargetWidth < params.fetchBufferSize),
            "Fetch target width should be larger than fetch buffer size!");

    for (int i = 0; i < MaxThreads; i++) {
        bacPC[i].reset(params.isa[0]->newPCState());
        stalls[i] = {false, false, false};
    }

    assert(bpu!=nullptr);
}

std::string
BAC::name() const {
    return cpu->name() + ".bac";
}


void
BAC::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    // TODO:
    timeBuffer = time_buffer;

    // Create wires to get information from proper places in time buffer.
    fromFetch = timeBuffer->getWire(-fetchToBacDelay);
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

void
BAC::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

void
BAC::setFetchTargetQueue(FTQ * _ptr)
{
    // Set pointer to the fetch target queue
    ftq = _ptr;
}

void
BAC::startupStage()
{
    resetStage();
    // For decoupled BPU the BAC need to start together with fetch
    // so it must start up in active state.
    switchToActive();
}


void
BAC::clearStates(ThreadID tid)
{
    bacStatus[tid] = Running;
    set(bacPC[tid], cpu->pcState(tid));

    stalls[tid].fetch = false;
    stalls[tid].drain = false;
    stalls[tid].bpu = false;

    assert(ftq!=nullptr);
    ftq->resetState();
}


void
BAC::resetStage()
{
    // Setup PC and nextPC with initial state.
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        bacStatus[tid] = Running;
        set(bacPC[tid], cpu->pcState(tid));

        stalls[tid].fetch = false;
        stalls[tid].drain = false;
        stalls[tid].bpu = false;
    }

    assert(ftq!=nullptr);
    ftq->resetState();

    wroteToTimeBuffer = false;
    _status = Inactive;
}


void
BAC::drainResume()
{
    DPRINTF(Drain, "Resume from draining.\n");
    for (ThreadID i = 0; i < numThreads; ++i) {
        stalls[i].drain = false;
    }
}

void
BAC::drainSanityCheck() const
{
    assert(isDrained());

    for (ThreadID i = 0; i < numThreads; ++i) {
        assert(bacStatus[i] == Idle || stalls[i].drain);
        assert(ftq->isEmpty(i));
    }

    bpu->drainSanityCheck();
}

bool
BAC::isDrained() const
{
    // Make sure the FTQ is empty and the state of all threads is idle.
    for (ThreadID i = 0; i < numThreads; ++i) {
        // Verify FTQs are drained
        if (!ftq->isEmpty(i))
            return false;

        // Return false if not idle or drain stalled
        if (bacStatus[i] != Idle) {
            return false;
        }
    }
    return true;
}


void
BAC::drainStall(ThreadID tid)
{
    assert(cpu->isDraining());
    assert(!stalls[tid].drain);
    DPRINTF(Drain, "%i: Thread drained.\n", tid);
    stalls[tid].drain = true;
}


void
BAC::switchToActive()
{
    if (_status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");
        cpu->activateStage(CPU::BACIdx);
        _status = Active;
    }
}

void
BAC::switchToInactive()
{
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");
        cpu->deactivateStage(CPU::BACIdx);
        _status = Inactive;
    }
}


bool
BAC::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    if (stalls[tid].fetch) {
        DPRINTF(BAC,"[tid:%i] Fetch stall detected.\n",tid);
        ret_val = true;
    }

    if (stalls[tid].bpu) {
        DPRINTF(BAC,"[tid:%i] BPU stall detected.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

void
BAC::updateBACStatus()
{
    //Check Running
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (bacStatus[tid] == Running ||
            bacStatus[tid] == Squashing) {

            if (_status == Inactive) {
                DPRINTF(Activity, "[tid:%i] Activating stage.\n",tid);

                cpu->activateStage(CPU::BACIdx);
            }

            _status = Active;
            return;
        }
    }

    // Stage is switching from active to inactive, notify CPU of it.
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(CPU::BACIdx);
    }

    _status = Inactive;
}


bool
BAC::checkAndUpdateBPUSignals(ThreadID tid)
{
    // Check squash signals from commit.
    if (fromCommit->commitInfo[tid].squash) {

        DPRINTF(BAC, "[tid:%i] Squashing from commit. PC = %s\n",
                        tid, *fromCommit->commitInfo[tid].pc);

        // In any case, squash the FTQ and the branch histories in the
        // FTQ first.
        squashBpuHistories(tid);
        squash(*fromCommit->commitInfo[tid].pc, tid);

        // If it was a branch mispredict on a control instruction, update the
        // branch predictor with that instruction, otherwise just kill the
        // invalid state we generated in after sequence number
        if (fromCommit->commitInfo[tid].mispredictInst &&
            fromCommit->commitInfo[tid].mispredictInst->isControl()) {

            bpu->squash(fromCommit->commitInfo[tid].doneSeqNum,
                        *fromCommit->commitInfo[tid].pc,
                        fromCommit->commitInfo[tid].branchTaken, tid, true);
            stats.branchMisspredict++;
            stats.squashBranchCommit++;
        } else {
            bpu->squash(fromCommit->commitInfo[tid].doneSeqNum, tid);
            if (fromCommit->commitInfo[tid].mispredictInst) {
                DPRINTF(BAC, "[tid:%i] Squashing due to mispredict of "
                        "non-control instruction: %s\n",tid,
                        fromCommit->commitInfo[tid]
                            .mispredictInst->staticInst->disassemble(
                                fromCommit->commitInfo[tid]
                            .mispredictInst->pcState().instAddr()));
            } else {
                DPRINTF(BAC, "[tid:%i] Squashing due to "
                "mispredict of non-control instruction: %s\n",tid);
            }
            stats.noBranchMisspredict++;
        }
        return true;

    } else if (fromCommit->commitInfo[tid].doneSeqNum) {
        // Update the branch predictor if it wasn't a squashed instruction
        // that was broadcasted.
        bpu->update(fromCommit->commitInfo[tid].doneSeqNum, tid);
    }

    // Check squash signals from decode.
    if (fromDecode->decodeInfo[tid].squash) {
        DPRINTF(Fetch, "[tid:%i] Squashing from decode. PC = %s\n",
                        tid, *fromDecode->decodeInfo[tid].nextPC);

        // Squash.
        squashBpuHistories(tid);
        squash(*fromDecode->decodeInfo[tid].nextPC, tid);

        // Update the branch predictor.
        if (fromDecode->decodeInfo[tid].branchMispredict) {

            bpu->squash(fromDecode->decodeInfo[tid].doneSeqNum,
                    *fromDecode->decodeInfo[tid].nextPC,
                    fromDecode->decodeInfo[tid].branchTaken, tid, false);
            stats.branchMisspredict++;
            stats.squashBranchDecode++;
        } else {
            bpu->squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              tid);
            stats.noBranchMisspredict++;
        }
        return true;
    }


    // Check squash signals from fetch.
    if (fromFetch->fetchInfo[tid].squash
        && bacStatus[tid] != Squashing) {
        DPRINTF(BAC, "Squashing from fetch with PC = %s\n",
                *fromFetch->fetchInfo[tid].nextPC);

        // Squash unless we're already squashing
        squashBpuHistories(tid);
        squash(*fromFetch->fetchInfo[tid].nextPC, tid);
        return true;
    }
    return false;
}




bool
BAC::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is.
    // Check stall signals, block if necessary.
    if (checkAndUpdateBPUSignals(tid)) {
        return true;
    }

    // Check stalls
    if (stalls[tid].drain) {
        assert(cpu->isDraining());
        DPRINTF(BAC,"[tid:%i] Drain stall detected.\n",tid);
        // Squash BPU histories and disable the FTQ.
        squashBpuHistories(tid);
        ftq->squash(tid);

        bacStatus[tid] = Idle;
        return true;
    }

    if (checkStall(tid)) {
        // return block(tid);
        bacStatus[tid] = Blocked;
        return false;
    }

    // If at this point the FTQ is still invalid we need to wait for
    // A resteer/squash signal.
    if (!ftq->isValid(tid) && bacStatus[tid] != Idle) {
        DPRINTF(BAC, "[tid:%i] FTQ is invalid. Wait for resteer.\n", tid);

        bacStatus[tid] = Idle;
        return true;
    }

    // Check if the FTQ got blocked or unblocked
    if ((bacStatus[tid] == Running) && ftq->isLocked(tid)) {

        DPRINTF(BAC, "[tid:%i] FTQ is locked\n", tid);
        bacStatus[tid] = FTQLocked;
        return true;
    }
    if ((bacStatus[tid] == FTQLocked) && !ftq->isLocked(tid)) {

        DPRINTF(BAC, "[tid:%i] FTQ not locked anymore -> Running\n", tid);
        bacStatus[tid] = Running;
        return true;
    }

    // Check if the FTQ became free in that cycle.
    if ((bacStatus[tid] == FTQFull) && !ftq->isFull(tid)) {

        DPRINTF(BAC, "[tid:%i] FTQ not full anymore -> Running\n", tid);
        bacStatus[tid] = Running;
        return true;
    }

    if (bacStatus[tid] == Squashing) {

        // Switch status to running after squashing FTQ and setting the PC.
        DPRINTF(BAC, "[tid:%i] Done squashing, switching to running.\n", tid);
        bacStatus[tid] = Running;
        return true;
    }

    // Now all stall/squash conditions are checked.
    // Attempt to run the BAC if not already running.
    if (ftq->isValid(tid) &&
            ((bacStatus[tid] == Idle) || (bacStatus[tid] == Blocked))) {

        DPRINTF(BAC, "[tid:%i] Attempt to run\n", tid);
        bacStatus[tid] = Running;
        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause BAC to change its status.  BAC remains the same as before.
    return false;
}




void
BAC::squashBpuHistories(ThreadID tid)
{
    if (!decoupledFrontEnd) return;

    DPRINTF(BAC, "%s(tid:%i): FTQ sz: %i\n", tid, __func__, ftq->size(tid));

    unsigned n_fts = ftq->size(tid);
    if (n_fts == 0) return;

    // Iterate over the FTQ in reverse order to
    // revert all predictions made.
    ftq->forAllBackward(tid,
        [this, tid](FetchTargetPtr &ft)
        {
            if (ft->bpu_history) {
                auto hist = static_cast<BPredUnit::PredictorHistory*>
                                                    (ft->bpu_history);
                bpu->squashHistory(tid, hist);
                assert(hist == nullptr);
                ft->bpu_history = nullptr;
            }
        });
}

void
BAC::squash(const PCStateBase &new_pc, ThreadID tid)
{
    if (!decoupledFrontEnd) return;

    DPRINTF(BAC, "[tid:%i] Squashing FTQ.\n", tid);

    // Set status to squashing.
    bacStatus[tid] = Squashing;

    // Set the new PC
    set(bacPC[tid], new_pc);

    // Then squash all fetch targets
    ftq->squash(tid);
}


void
BAC::tick()
{
    bool activity = false;
    bool status_change = false;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

if (decoupledFrontEnd) {
    // FDP ---------------------------------------------
    // In the decoupled frontend the BAC stage is active as all
    // others. Its main purpose is generate fetch targets by using
    // the branch predction unit.

    while (threads != end) {
        ThreadID tid = *threads++;

        // Check stall and squash signals first.
        status_change |= checkSignalsAndUpdate(tid);

        // Generate fetch targets if BAC is in running state
        if (bacStatus[tid] == Running) {
            generateFetchTargets(tid, status_change);
            activity = true;
        }
        profileCycle(tid);
    }

} else {
    // No FDP -------------------------------------------
    // In the non-decoupled frontend the BAC stage is passive and
    // only manages the branch prediction unit.
    // It is always idle.

    while (threads != end) {
        ThreadID tid = *threads++;

        // Update only the branch prediction signals.
        checkAndUpdateBPUSignals(tid);
        if (bacStatus[tid] != Idle) {
            bacStatus[tid] = Idle;
            status_change = true;
        }
    }
} // ----------------------------------------------------

    if (status_change) {
        updateBACStatus();
    }

    if (activity) {
        DPRINTF(Activity, "Activity this cycle.\n");

        cpu->activityThisCycle();
    }
}


FetchTargetPtr
BAC::newFetchTarget(ThreadID tid, const PCStateBase &start_pc)
{
    auto ft = std::make_shared<FetchTarget>(start_pc,
                                            cpu->getAndIncrementFTSeq());

    DPRINTF(BAC, "Create new fetch target ftn:%llu\n", ft->ftNum());
    stats.fetchTargets++;
    return ft;
}

bool
BAC::predict(ThreadID tid, const StaticInstPtr &inst,
             const FetchTargetPtr &ft, PCStateBase &pc)
{

    /** Perform the prediction. */
    BPredUnit::PredictorHistory* bpu_history = nullptr;
    bool taken  = bpu->predict(inst, ft->ftNum(), pc, tid, bpu_history);

    /** Push the prediction history to the fetch target.
     * The postFetch() function will move the history from the FTQ to the
     * main history of the BPU.
    */
    ft->bpu_history = static_cast<void*>(bpu_history);

    DPRINTF(Branch,"[tid:%i, ftn:%llu] History added.\n", tid, ft->ftNum());
    return taken;
}


void
BAC::generateFetchTargets(ThreadID tid, bool &status_change)
{
    /**
     * This function implements the head of the decoupled frontend.
     * Instead of waiting for the pre-decoding the current instruction, as
     * done in the standared front-end, the BTB is leveraged for finding
     * branches in the instruction stream.
     *
     * Starting from the current address we search all consecutive addresses
     * if a entry exits in the BTB. As soon as the BTB hits, we know we have
     * reached a branch instruction and make a prediction for the branch.
     * The start and end address of this so called fetch target is stored
     * together with the prediction in the FTQ.
     *
     * Depending on the prediction of the BPU the branch target or the
     * fallthrough address determine the start address for the next
     * fetch target and search cycle.
     */

    bool branch_found = false;
    bool predict_taken = false;

    // Get a reference to the current PC state for this thread.
    // The search itself is done on the instruction address to speed up
    // simulation time.
    PCStateBase &cur_pc = *bacPC[tid];
    Addr search_addr = cur_pc.instAddr();
    Addr start_addr = search_addr;

    // In each cycles a new fetch target is created starting with
    // the current PC.
    FetchTargetPtr curFT = newFetchTarget(tid, cur_pc);


    // Scan through the instruction stream and search for branches.
    // The BTB contains only branches where taken at least once.
    while (true) {

        // Check if the current search address can be found in the BTB
        // indicating the end of the branch.
        branch_found = bpu->BTBValid(tid, search_addr);

        // If its a branch stop searching
        if (branch_found) {
            break;
        }

        // If its not a branch check if the maximum search width is reached.
        // If yes stop searching.
        if ((search_addr - start_addr) >= fetchTargetWidth) {
            break;
        }

        // Continue searching.
        search_addr += minInstSize;
    }

    // Update the current PC to point to the last instruction
    // in the fetch target
    cur_pc.set(search_addr);


    // Search stopped either because a branch was found in instruction
    // stream or the maximum search width per cycle was reached.
    // In the first case make the branch prediction and in the later
    // advance the PC to start the search at the following address.

    // Make a copy of the current PC since the BPU will update it.
    std::unique_ptr<PCStateBase> next_pc(cur_pc.clone());
    StaticInstPtr staticInst = nullptr;

    if (branch_found) {
        // Branch found in instruction stream. As the current
        // BPU implementation required the static instruction we need to
        // look it up from the BTB.
        staticInst = bpu->BTBLookupInst(tid, cur_pc);
        assert(staticInst);

        // Now make the actual prediction. Note the BPU will advance
        // the PC to the next instruction.
        predict_taken = predict(tid, staticInst, curFT, *next_pc);

        DPRINTF(BAC, "[tid:%i, ftn:%llu] Branch found at PC %#x "
                "taken?:%i, target:%#x\n",
                tid, curFT->ftNum(), cur_pc.instAddr(),
                predict_taken, next_pc->instAddr());

        stats.branches++;
        if (predict_taken) {
            stats.predTakenBranches++;
        }

    } else {

        // Not a branch therefore we will continue the next FT at the
        // next address
        next_pc->set(cur_pc.instAddr() + minInstSize);
    }


    // Complete the fetch target if
    // - a branch is found
    // - or the maximum fetch bandwidth is reached.
    curFT->finalize(cur_pc, curFT->ftNum(), branch_found,
                        predict_taken, *next_pc);

    ftq->insert(tid, curFT);
    wroteToTimeBuffer = true;

    // Check whether the FTQ became full. In that case block until
    // fetch has consumed one.
    if (ftq->isFull(tid)) {
        DPRINTF(BAC, "FTQ full\n");
        bacStatus[tid] = FTQFull;
        status_change = true;
    }

    // x86 has some complex instruction like string copy where the branch
    // is not the last instruction or have several branches within the same
    // instruction. Those branches jump always! to itself. This messes up
    // the searching approach and will result in an infinite loop until the
    // branch is squashed.
    // We handle this by assuming only one branch per instruction and go
    // straight to the next address/instruction/fetch target. In case the
    // decoder finds more branches in this instruction we squash the FTQ.
    // (see postFetch())
    // This could be circumvented by using not only the PC but also the
    // microPC to make predictions. However, since such instructions are
    // rare this is not implemented.
    if (staticInst
        && staticInst->isMicroop() && !staticInst->isLastMicroop()) {
        stats.branchesNotLastuOp++;
        // The target is always to itself no matter if its taken or not.
        // assert(next_pc->instAddr() == search_addr);
        DPRINTF(BAC, "Branch detected which is not the last uOp %s. "
                    "Continue with next address.\n", cur_pc);

        next_pc->set(cur_pc.instAddr() + staticInst->size());
    }

    DPRINTF(BAC, "[tid:%i] [fn:%llu] %i addresses searched. "
            "Branch found:%i. Continue with PC:%s in next cycle\n",
            tid, curFT->ftNum(), (search_addr - start_addr),
            branch_found, *next_pc);

    stats.ftSizeDist.sample(search_addr - start_addr);

    // Finally set the BPU PC to the next FT in the next cycle
    set(cur_pc, *next_pc);

    // ftq->printFTQ(tid);
}



/// Post fetch part ------------------------------------------


bool
BAC::updatePreDecode(ThreadID tid, const InstSeqNum seqNum,
                     const StaticInstPtr &inst, PCStateBase &pc,
                     const FetchTargetPtr &ft)
{
    assert(ft != nullptr);
    // The PC must be in the range of the fetch target.
    assert(ft->inRange(pc.instAddr()));

    assert(ft->ftNum() == ftq->readHead(tid)->ftNum());
    BranchType brType = branch_prediction::getBranchType(inst);
    stats.preDecUpdate[brType]++;

    DPRINTF(BAC, "%s(tid:%i, sn:%lu, inst: %s, PC:%#x, FT[%llu, taken:%i, "
            "end:#%#x)\n", __func__, tid, seqNum,
            branch_prediction::toString(brType), pc.instAddr(), ft->ftNum(),
            ft->predTaken(), ft->endAddress());

    bool target_set = false;
    BPredUnit::PredictorHistory* hist = nullptr;

    // The fetch stage will call this function after pre-decoding an
    // instruction finds a branch instruction. Check if this is the exit
    // branch.
    if (ft->isExitBranch(pc.instAddr())
        && ft->bpu_history != nullptr) {

        // Pop the history from the FTQ to move it later to the
        // history buffer.
        hist = static_cast<BPredUnit::PredictorHistory*>(ft->bpu_history);
        ft->bpu_history = nullptr;

        DPRINTF(BAC, "Pop history from FT:%llu => sn:%llu, PC:%#x, taken:%i, "
                "target:%#x\n", ft->ftNum(), seqNum, hist->pc,
                hist->predTaken, hist->target->instAddr());

    }

    // Special cases ------------------------------------------------
    // We need to handle two corner cases for complex instructions
    // 1. For complex instructions it can happen that several branches with
    // different types exists in the same instruction. If the branch type
    // does not match with the type of the prediction history its invalid.
    // We squash everything  the history and we can make a fresh
    // prediction
    if (hist && (hist->type != brType)) {
        DPRINTF(Branch, "Branch types dont match. Delete history\n", tid);
        stats.typeMissmatch++;

        // Push the history back to the FTQ to allow it to be sqaushed
        // in correct order. Then squash all histories right away.
        ft->bpu_history = static_cast<void*>(hist);
        hist = nullptr;
        squashBpuHistories(tid);

        // Lock the FTQ. The complex instruction needs to
        // be completed before unlocking. Unlocking is performed by resetting
        // the BAC stage.
        ftq->lock(tid);
    }

    // 2. For complex instruction with more than one branches the history
    // the history is already used. We only predict the first branch in a
    // complex instruction (see createFetchTarget() function).
    // In that case we squash the FTQ and lock it until the full instruction
    // Afterwards the fetch stage will reset the BAC stage with a
    // bacResteer() call. Hence, operation for complex instructions is:
    // Detecting multi branch inst. -> lock FTQ util inst. done. -> reset BAC.
    //
    // Note we might end up here multiple times until the full instruction
    // is completed.
    if (inst->isMicroop() && !inst->isLastMicroop() && (hist == nullptr)) {

        DPRINTF(Branch, "No history for complex instruction found. \n");
        stats.multiBranchInst++;

        // First squash all histories that are already in the FTQ
        // to have a clean state.
        squashBpuHistories(tid);

        // Then lock the FTQ. The complex instruction needs to
        // be completed before unlocking. Unlocking is performed by
        // resetting the BAC stage with a bacResteer() call from the
        // fetch stage.
        ftq->lock(tid);

        // Finally we can make a fresh prediction.
        bpu->predict(inst, ft->ftNum(), pc, tid, hist);
        target_set = true;
    }


    // Normal case --------------------------------------------------
    // Check if we have a valid history. If not we need to create one.
    if (hist == nullptr) {
        DPRINTF(BAC, "[tid:%i, sn:%llu] No branch history for PC:%#x\n",
                tid, seqNum, pc.instAddr());
        stats.noHistType[brType]++;

        // The branch was not detected by the BAC stage in the first place
        // because the BTB did not had an entry for this PC. It can happen
        // if this is the first time the branch is encountered, the branch
        // was never taken before, or the entry got evicted.
        //
        // Create a "dummy" history object by assuming the branch is not
        // taken. This will allow the BPU to fix its histories and internal
        // state in case the assumption was wrong. It works because for
        // FDP we use "taken" history where not taken branches don't modify
        // the global history.

        hist = new BPredUnit::PredictorHistory(tid, seqNum,
                                               pc.instAddr(), inst);
        bpu->branchPlaceholder(tid, pc.instAddr(), inst->isUncondCtrl(),
                               hist->bpHistory);

        hist->predTaken = hist->condPred = false;
        hist->targetProvider = BPredUnit::TargetProvider::NoTarget;

        set(hist->target, std::unique_ptr<PCStateBase>(pc.clone()));
        inst->advancePC(*hist->target);

    }

    assert(hist != nullptr);
    assert(hist->type == brType);

    // Assign the branch instruction instance its sequence number
    // and push the history to the main history buffer.
    hist->seqNum = seqNum;
    bpu->predHist[tid].push_front(hist);

    // Finally update the current fetch PC if not already done.
    // For taken branches the target is stored in the FTQ. For not taken
    // branches we need to advance the PC.
    if (!target_set) {
        if (hist->predTaken) {
            set(pc, ft->readPredTarg());
        } else {
            inst->advancePC(pc);
        }
    }

    DPRINTF(BAC, "%s done. next PC:%s\n", __func__, pc);
    return hist->predTaken;
}


bool
BAC::updatePC(const DynInstPtr &inst,
              PCStateBase &fetch_pc, FetchTargetPtr &ft)
{
    // This function will update the fetch PC to the next instruction.
    // If the current instruction is a branch it will make
    // the branch prediction.
    bool predict_taken;
    ThreadID tid = inst->threadNumber;


    if (inst->isControl()) {
        // The instruction is a control instruction.

        if (decoupledFrontEnd) {
            // With a decoupled front-end the branch prediction was done
            // while creating the fetch target. Now update the prediction
            // with the information from the predecoding.
            predict_taken = updatePreDecode(tid, inst->seqNum,
                                            inst->staticInst, fetch_pc, ft);
        } else {
            // With a coupled front-end we need to make the branch prediction
            // here.
            predict_taken = bpu->predict(inst->staticInst, inst->seqNum,
                                         fetch_pc, tid);
        }

        DPRINTF(BAC, "[tid:%i] [sn:%llu] Branch at PC %#x "
                "predicted %s to go to %s\n",
                tid, inst->seqNum, inst->pcState().instAddr(),
                predict_taken ? "taken" : "not taken",
                fetch_pc);
        inst->setPredTarg(fetch_pc);
        inst->setPredTaken(predict_taken);

        ++stats.branches;

        if (predict_taken) {
            ++stats.predTakenBranches;
        }

    } else {

        // For non-branch instructions simply advance the PC.
        inst->staticInst->advancePC(fetch_pc);
        inst->setPredTarg(fetch_pc);
        inst->setPredTaken(false);
        predict_taken = false;
    }


    if (decoupledFrontEnd) {

        // For the decoupled front-end we need to check if this instruction
        // is the exit instruction of the fetch target. It does not need
        // to be a branch.
        // If the instruction is micro coded check if its the last uOp.
        // Also remove the fetch target if the FTQ became invalid.
        if ((ft->isExitInst(inst->pcState().instAddr())
                && (!inst->isMicroop() || inst->isLastMicroop()))
            || !ftq->isValid(tid)) {

            DPRINTF(BAC, "[tid:%i][ft:%llu] Reached end of Fetch Target\n",
                            tid, ft->ftNum());

            ft = nullptr;
        }
    }

    return predict_taken;
}


void
BAC::profileCycle(ThreadID tid)
{
    switch (bacStatus[tid]) {
    case Idle:
        stats.idleCycles++;
        break;
    case Running:
        stats.runCycles++;
        break;
    case Squashing:
        stats.squashCycles++;
        break;
    case FTQFull:
        stats.ftqFullCycles++;
        break;

    default:
        break;
    }
}


BAC::BACStats::BACStats(o3::CPU *cpu, BAC *bac)
    : statistics::Group(cpu, "bac"),

    ADD_STAT(idleCycles, statistics::units::Cycle::get(),
            "Number of cycles BAC is idle. (PC invalid)"),
    ADD_STAT(runCycles, statistics::units::Cycle::get(),
            "Number of cycles BAC is running"),
    ADD_STAT(squashCycles, statistics::units::Cycle::get(),
            "Number of cycles BAC is squashing"),
    ADD_STAT(ftqFullCycles, statistics::units::Cycle::get(),
            "Number of cycles BAC has spent waiting for FTQ to become free"),

    ADD_STAT(fetchTargets, statistics::units::Count::get(),
            "Number of fetch targets created "),
    ADD_STAT(branches, statistics::units::Count::get(),
            "Number of branches that BAC encountered"),
    ADD_STAT(predTakenBranches, statistics::units::Count::get(),
            "Number of branches that BAC predicted taken."),
    ADD_STAT(branchesNotLastuOp, statistics::units::Count::get(),
             "Number of branches that fetch encountered which are not the "
             "last uOp within a macrooperation. Jump to itself."),
    ADD_STAT(branchMisspredict, statistics::units::Count::get(),
            "Number of branches that BAC has predicted taken"),
    ADD_STAT(noBranchMisspredict, statistics::units::Count::get(),
            "Number of branches that BAC has predicted taken"),
    ADD_STAT(squashBranchDecode, statistics::units::Count::get(),
            "Number of branches squashed from decode"),
    ADD_STAT(squashBranchCommit, statistics::units::Count::get(),
            "Number of branches squashed from decode"),
    ADD_STAT(preDecUpdate, statistics::units::Count::get(),
            "Number of branches extracted from the predecoder"),
    ADD_STAT(noHistType, statistics::units::Count::get(),
            "Number and type of branches that where undetected by the BPU."),
    ADD_STAT(typeMissmatch, statistics::units::Count::get(),
            "Number branches where the branch type miss match"),
    ADD_STAT(multiBranchInst, statistics::units::Count::get(),
            "Number branches because its not the last branch."),
    ADD_STAT(ftSizeDist, statistics::units::Count::get(),
             "Number of bytes per fetch target")
{
    using namespace statistics;

    ftSizeDist
        .init(/* base value */ 0,
              /* last value */ bac->fetchTargetWidth,
              /* bucket size */ 4)
        .flags(statistics::pdf);

    preDecUpdate
        .init(enums::Num_BranchType)
        .flags(total | pdf);
    noHistType
        .init(enums::Num_BranchType)
        .flags(total | pdf);


    for (int i = 0; i < enums::Num_BranchType; i++) {
        preDecUpdate.subname(i, enums::BranchTypeStrings[i]);
        noHistType.subname(i, enums::BranchTypeStrings[i]);
    }
}

} // namespace branch_prediction
} // namespace gem5
