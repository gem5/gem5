/*
 * Copyright (c) 2011, 2014 ARM Limited
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

#include "cpu/pred/tournament.hh"

#include "base/bitfield.hh"
#include "base/intmath.hh"

namespace gem5
{

namespace branch_prediction
{

TournamentBP::TournamentBP(const TournamentBPParams &params)
    : BPredUnit(params),
      localPredictorSize(params.localPredictorSize),
      localCtrBits(params.localCtrBits),
      localCtrs(localPredictorSize, SatCounter8(localCtrBits)),
      localHistoryTableSize(params.localHistoryTableSize),
      localHistoryBits(ceilLog2(params.localPredictorSize)),
      globalPredictorSize(params.globalPredictorSize),
      globalCtrBits(params.globalCtrBits),
      globalCtrs(globalPredictorSize, SatCounter8(globalCtrBits)),
      globalHistory(params.numThreads, 0),
      globalHistoryBits(
          ceilLog2(params.globalPredictorSize) >
          ceilLog2(params.choicePredictorSize) ?
          ceilLog2(params.globalPredictorSize) :
          ceilLog2(params.choicePredictorSize)),
      choicePredictorSize(params.choicePredictorSize),
      choiceCtrBits(params.choiceCtrBits),
      choiceCtrs(choicePredictorSize, SatCounter8(choiceCtrBits))
{
    if (!isPowerOf2(localPredictorSize)) {
        fatal("Invalid local predictor size!\n");
    }

    if (!isPowerOf2(globalPredictorSize)) {
        fatal("Invalid global predictor size!\n");
    }

    localPredictorMask = mask(localHistoryBits);

    if (!isPowerOf2(localHistoryTableSize)) {
        fatal("Invalid local history table size!\n");
    }

    //Setup the history table for the local table
    localHistoryTable.resize(localHistoryTableSize);

    for (int i = 0; i < localHistoryTableSize; ++i)
        localHistoryTable[i] = 0;

    // Set up the global history mask
    // this is equivalent to mask(log2(globalPredictorSize)
    globalHistoryMask = globalPredictorSize - 1;

    if (!isPowerOf2(choicePredictorSize)) {
        fatal("Invalid choice predictor size!\n");
    }

    // Set up choiceHistoryMask
    // this is equivalent to mask(log2(choicePredictorSize)
    choiceHistoryMask = choicePredictorSize - 1;

    //Set up historyRegisterMask
    historyRegisterMask = mask(globalHistoryBits);

    //Check that predictors don't use more bits than they have available
    if (globalHistoryMask > historyRegisterMask) {
        fatal("Global predictor too large for global history bits!\n");
    }
    if (choiceHistoryMask > historyRegisterMask) {
        fatal("Choice predictor too large for global history bits!\n");
    }

    if (globalHistoryMask < historyRegisterMask &&
        choiceHistoryMask < historyRegisterMask) {
        inform("More global history bits than required by predictors\n");
    }

    // Set thresholds for the three predictors' counters
    // This is equivalent to (2^(Ctr))/2 - 1
    localThreshold  = (1ULL << (localCtrBits  - 1)) - 1;
    globalThreshold = (1ULL << (globalCtrBits - 1)) - 1;
    choiceThreshold = (1ULL << (choiceCtrBits - 1)) - 1;
}

inline
unsigned
TournamentBP::calcLocHistIdx(Addr &branch_addr)
{
    // Get low order bits after removing instruction offset.
    return (branch_addr >> instShiftAmt) & (localHistoryTableSize - 1);
}

inline
void
TournamentBP::updateGlobalHist(ThreadID tid, bool taken)
{
    globalHistory[tid] = (globalHistory[tid] << 1) | taken;
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

inline
void
TournamentBP::updateLocalHist(unsigned local_history_idx, bool taken)
{
    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1) | taken;
}

bool
TournamentBP::lookup(ThreadID tid, Addr pc, void * &bp_history)
{
    bool local_prediction;
    unsigned local_history_idx;
    unsigned local_predictor_idx;

    bool global_prediction;
    bool choice_prediction;

    //Lookup in the local predictor to get its branch prediction
    local_history_idx = calcLocHistIdx(pc);
    local_predictor_idx = localHistoryTable[local_history_idx]
        & localPredictorMask;
    local_prediction = localCtrs[local_predictor_idx] > localThreshold;

    //Lookup in the global predictor to get its branch prediction
    global_prediction = globalThreshold <
      globalCtrs[globalHistory[tid] & globalHistoryMask];

    //Lookup in the choice predictor to see which one to use
    choice_prediction = choiceThreshold <
      choiceCtrs[globalHistory[tid] & choiceHistoryMask];

    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid];
    history->localPredTaken = local_prediction;
    history->globalPredTaken = global_prediction;
    history->globalUsed = choice_prediction;
    history->localHistoryIdx = local_history_idx;
    history->localHistory = local_predictor_idx;
    bp_history = (void *)history;

    assert(local_history_idx < localHistoryTableSize);

    // Select and return the prediction
    // History update will be happen in the next function
    if (choice_prediction) {
        return global_prediction;
    } else {
        return local_prediction;
    }
}

void
TournamentBP::updateHistories(ThreadID tid, Addr pc, bool uncond,
                         bool taken, Addr target, void * &bp_history)
{
    assert(uncond || bp_history);
    if (uncond) {
        // Create BPHistory and pass it back to be recorded.
        BPHistory *history = new BPHistory;
        history->globalHistory = globalHistory[tid];
        history->localPredTaken = true;
        history->globalPredTaken = true;
        history->globalUsed = true;
        history->localHistoryIdx = invalidPredictorIndex;
        history->localHistory = invalidPredictorIndex;
        bp_history = static_cast<void *>(history);
    }

    // Update the global history for all branches
    updateGlobalHist(tid, taken);

    // Update the local history only for conditional branches
    if (!uncond) {
        auto history = static_cast<BPHistory *>(bp_history);
        updateLocalHist(history->localHistoryIdx, taken);
    }
}


void
TournamentBP::update(ThreadID tid, Addr pc, bool taken,
                     void * &bp_history, bool squashed,
                     const StaticInstPtr & inst, Addr target)
{
    assert(bp_history);

    BPHistory *history = static_cast<BPHistory *>(bp_history);

    unsigned local_history_idx = calcLocHistIdx(pc);

    assert(local_history_idx < localHistoryTableSize);

    // Unconditional branches do not use local history.
    bool old_local_pred_valid = history->localHistory !=
            invalidPredictorIndex;

    // If this is a misprediction, restore the speculatively
    // updated state (global history register and local history)
    // and update again.
    if (squashed) {
        // Global history restore and update
        globalHistory[tid] = (history->globalHistory << 1) | taken;
        globalHistory[tid] &= historyRegisterMask;

        // Local history restore and update.
        if (old_local_pred_valid) {
            localHistoryTable[local_history_idx] =
                        (history->localHistory << 1) | taken;
        }

        return;
    }

    unsigned old_local_pred_index = history->localHistory &
        localPredictorMask;

    assert(old_local_pred_index < localPredictorSize);

    // Update the choice predictor to tell it which one was correct if
    // there was a prediction.
    if (history->localPredTaken != history->globalPredTaken &&
        old_local_pred_valid)
    {
        // If the local prediction matches the actual outcome,
        // decrement the counter. Otherwise increment the
        // counter.
        unsigned choice_predictor_idx =
            history->globalHistory & choiceHistoryMask;
        if (history->localPredTaken == taken) {
            choiceCtrs[choice_predictor_idx]--;
        } else if (history->globalPredTaken == taken) {
            choiceCtrs[choice_predictor_idx]++;
        }
    }

    // Update the counters with the proper
    // resolution of the branch. Histories are updated
    // speculatively, restored upon squash() calls, and
    // recomputed upon update(squash = true) calls,
    // so they do not need to be updated.
    unsigned global_predictor_idx =
            history->globalHistory & globalHistoryMask;
    if (taken) {
        globalCtrs[global_predictor_idx]++;
        if (old_local_pred_valid) {
            localCtrs[old_local_pred_index]++;
        }
    } else {
        globalCtrs[global_predictor_idx]--;
        if (old_local_pred_valid) {
            localCtrs[old_local_pred_index]--;
        }
    }

    // We're done with this history, now delete it.
    delete history;
    bp_history = nullptr;
}

void
TournamentBP::squash(ThreadID tid, void * &bp_history)
{
    BPHistory *history = static_cast<BPHistory *>(bp_history);

    // Restore global history to state prior to this branch.
    globalHistory[tid] = history->globalHistory;

    // Restore local history
    if (history->localHistoryIdx != invalidPredictorIndex) {
        localHistoryTable[history->localHistoryIdx] = history->localHistory;
    }

    // Delete this BPHistory now that we're done with it.
    delete history;
    bp_history = nullptr;
}

#ifdef GEM5_DEBUG
int
TournamentBP::BPHistory::newCount = 0;
#endif

} // namespace branch_prediction
} // namespace gem5
