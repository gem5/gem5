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

#include "base/intmath.hh"
#include "cpu/o3/tournament_pred.hh"

TournamentBP::TournamentBP(unsigned _localPredictorSize,
                           unsigned _localCtrBits,
                           unsigned _localHistoryTableSize,
                           unsigned _localHistoryBits,
                           unsigned _globalPredictorSize,
                           unsigned _globalCtrBits,
                           unsigned _globalHistoryBits,
                           unsigned _choicePredictorSize,
                           unsigned _choiceCtrBits,
                           unsigned _instShiftAmt)
    : localPredictorSize(_localPredictorSize),
      localCtrBits(_localCtrBits),
      localHistoryTableSize(_localHistoryTableSize),
      localHistoryBits(_localHistoryBits),
      globalPredictorSize(_globalPredictorSize),
      globalCtrBits(_globalCtrBits),
      globalHistoryBits(_globalHistoryBits),
      choicePredictorSize(_globalPredictorSize),
      choiceCtrBits(_choiceCtrBits),
      instShiftAmt(_instShiftAmt)
{
    if (!isPowerOf2(localPredictorSize)) {
        fatal("Invalid local predictor size!\n");
    }

    //Setup the array of counters for the local predictor
    localCtrs.resize(localPredictorSize);

    for (int i = 0; i < localPredictorSize; ++i)
        localCtrs[i].setBits(localCtrBits);

    if (!isPowerOf2(localHistoryTableSize)) {
        fatal("Invalid local history table size!\n");
    }

    //Setup the history table for the local table
    localHistoryTable.resize(localHistoryTableSize);

    for (int i = 0; i < localHistoryTableSize; ++i)
        localHistoryTable[i] = 0;

    // Setup the local history mask
    localHistoryMask = (1 << localHistoryBits) - 1;

    if (!isPowerOf2(globalPredictorSize)) {
        fatal("Invalid global predictor size!\n");
    }

    //Setup the array of counters for the global predictor
    globalCtrs.resize(globalPredictorSize);

    for (int i = 0; i < globalPredictorSize; ++i)
        globalCtrs[i].setBits(globalCtrBits);

    //Clear the global history
    globalHistory = 0;
    // Setup the global history mask
    globalHistoryMask = (1 << globalHistoryBits) - 1;

    if (!isPowerOf2(choicePredictorSize)) {
        fatal("Invalid choice predictor size!\n");
    }

    //Setup the array of counters for the choice predictor
    choiceCtrs.resize(choicePredictorSize);

    for (int i = 0; i < choicePredictorSize; ++i)
        choiceCtrs[i].setBits(choiceCtrBits);

    // @todo: Allow for different thresholds between the predictors.
    threshold = (1 << (localCtrBits - 1)) - 1;
    threshold = threshold / 2;
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
TournamentBP::updateGlobalHistTaken()
{
    globalHistory = (globalHistory << 1) | 1;
    globalHistory = globalHistory & globalHistoryMask;
}

inline
void
TournamentBP::updateGlobalHistNotTaken()
{
    globalHistory = (globalHistory << 1);
    globalHistory = globalHistory & globalHistoryMask;
}

inline
void
TournamentBP::updateLocalHistTaken(unsigned local_history_idx)
{
    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1) | 1;
}

inline
void
TournamentBP::updateLocalHistNotTaken(unsigned local_history_idx)
{
    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1);
}

bool
TournamentBP::lookup(Addr &branch_addr, void * &bp_history)
{
    bool local_prediction;
    unsigned local_history_idx;
    unsigned local_predictor_idx;

    bool global_prediction;
    bool choice_prediction;

    //Lookup in the local predictor to get its branch prediction
    local_history_idx = calcLocHistIdx(branch_addr);
    local_predictor_idx = localHistoryTable[local_history_idx]
        & localHistoryMask;
    local_prediction = localCtrs[local_predictor_idx].read() > threshold;

    //Lookup in the global predictor to get its branch prediction
    global_prediction = globalCtrs[globalHistory].read() > threshold;

    //Lookup in the choice predictor to see which one to use
    choice_prediction = choiceCtrs[globalHistory].read() > threshold;

    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory;
    history->localPredTaken = local_prediction;
    history->globalPredTaken = global_prediction;
    history->globalUsed = choice_prediction;
    bp_history = (void *)history;

    assert(globalHistory < globalPredictorSize &&
           local_history_idx < localPredictorSize);

    // Commented code is for doing speculative update of counters and
    // all histories.
    if (choice_prediction) {
        if (global_prediction) {
//            updateHistoriesTaken(local_history_idx);
//            globalCtrs[globalHistory].increment();
//            localCtrs[local_history_idx].increment();
            updateGlobalHistTaken();
            return true;
        } else {
//            updateHistoriesNotTaken(local_history_idx);
//            globalCtrs[globalHistory].decrement();
//            localCtrs[local_history_idx].decrement();
            updateGlobalHistNotTaken();
            return false;
        }
    } else {
        if (local_prediction) {
//            updateHistoriesTaken(local_history_idx);
//            globalCtrs[globalHistory].increment();
//            localCtrs[local_history_idx].increment();
            updateGlobalHistTaken();
            return true;
        } else {
//            updateHistoriesNotTaken(local_history_idx);
//            globalCtrs[globalHistory].decrement();
//            localCtrs[local_history_idx].decrement();
            updateGlobalHistNotTaken();
            return false;
        }
    }
}

void
TournamentBP::uncondBr(void * &bp_history)
{
    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory;
    history->localPredTaken = true;
    history->globalPredTaken = true;
    bp_history = static_cast<void *>(history);

    updateGlobalHistTaken();
}

void
TournamentBP::update(Addr &branch_addr, bool taken, void *bp_history)
{
    unsigned local_history_idx;
    unsigned local_predictor_idx;
    unsigned local_predictor_hist;

    // Get the local predictor's current prediction
    local_history_idx = calcLocHistIdx(branch_addr);
    local_predictor_hist = localHistoryTable[local_history_idx];
    local_predictor_idx = local_predictor_hist & localHistoryMask;

    // Update the choice predictor to tell it which one was correct if
    // there was a prediction.
    if (bp_history) {
        BPHistory *history = static_cast<BPHistory *>(bp_history);
        if (history->localPredTaken != history->globalPredTaken) {
            // If the local prediction matches the actual outcome,
            // decerement the counter.  Otherwise increment the
            // counter.
            if (history->localPredTaken == taken) {
                choiceCtrs[globalHistory].decrement();
            } else if (history->globalPredTaken == taken){
                choiceCtrs[globalHistory].increment();
            }
        }

        // We're done with this history, now delete it.
        delete history;
    }

    assert(globalHistory < globalPredictorSize &&
           local_predictor_idx < localPredictorSize);

    // Update the counters and local history with the proper
    // resolution of the branch.  Global history is updated
    // speculatively and restored upon squash() calls, so it does not
    // need to be updated.
    if (taken) {
        localCtrs[local_predictor_idx].increment();
        globalCtrs[globalHistory].increment();

        updateLocalHistTaken(local_history_idx);
    } else {
        localCtrs[local_predictor_idx].decrement();
        globalCtrs[globalHistory].decrement();

        updateLocalHistNotTaken(local_history_idx);
    }
}

void
TournamentBP::squash(void *bp_history)
{
    BPHistory *history = static_cast<BPHistory *>(bp_history);

    // Restore global history to state prior to this branch.
    globalHistory = history->globalHistory;

    // Delete this BPHistory now that we're done with it.
    delete history;
}

#ifdef DEBUG
int
TournamentBP::BPHistory::newCount = 0;
#endif
