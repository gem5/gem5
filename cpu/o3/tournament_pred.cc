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
    //Should do checks here to make sure sizes are correct (powers of 2)

    //Setup the array of counters for the local predictor
    localCtrs.resize(localPredictorSize);

    for (int i = 0; i < localPredictorSize; ++i)
        localCtrs[i].setBits(localCtrBits);

    //Setup the history table for the local table
    localHistoryTable.resize(localHistoryTableSize);

    for (int i = 0; i < localHistoryTableSize; ++i)
        localHistoryTable[i] = 0;

    // Setup the local history mask
    localHistoryMask = (1 << localHistoryBits) - 1;

    //Setup the array of counters for the global predictor
    globalCtrs.resize(globalPredictorSize);

    for (int i = 0; i < globalPredictorSize; ++i)
        globalCtrs[i].setBits(globalCtrBits);

    //Clear the global history
    globalHistory = 0;
    // Setup the global history mask
    globalHistoryMask = (1 << globalHistoryBits) - 1;

    //Setup the array of counters for the choice predictor
    choiceCtrs.resize(choicePredictorSize);

    for (int i = 0; i < choicePredictorSize; ++i)
        choiceCtrs[i].setBits(choiceCtrBits);

    threshold = (1 << (localCtrBits - 1)) - 1;
    threshold = threshold / 2;
}

inline
unsigned
TournamentBP::calcLocHistIdx(Addr &branch_addr)
{
    return (branch_addr >> instShiftAmt) & (localHistoryTableSize - 1);
}

inline
void
TournamentBP::updateHistoriesTaken(unsigned local_history_idx)
{
    globalHistory = (globalHistory << 1) | 1;
    globalHistory = globalHistory & globalHistoryMask;

    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1) | 1;
}

inline
void
TournamentBP::updateHistoriesNotTaken(unsigned local_history_idx)
{
    globalHistory = (globalHistory << 1);
    globalHistory = globalHistory & globalHistoryMask;

    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1);
}

bool
TournamentBP::lookup(Addr &branch_addr)
{
    uint8_t local_prediction;
    unsigned local_history_idx;
    unsigned local_predictor_idx;

    uint8_t global_prediction;
    uint8_t choice_prediction;

    //Lookup in the local predictor to get its branch prediction
    local_history_idx = calcLocHistIdx(branch_addr);
    local_predictor_idx = localHistoryTable[local_history_idx]
        & localHistoryMask;
    local_prediction = localCtrs[local_predictor_idx].read();

    //Lookup in the global predictor to get its branch prediction
    global_prediction = globalCtrs[globalHistory].read();

    //Lookup in the choice predictor to see which one to use
    choice_prediction = choiceCtrs[globalHistory].read();

    //@todo Put a threshold value in for the three predictors that can
    // be set through the constructor (so this isn't hard coded).
    //Also should put some of this code into functions.
    if (choice_prediction > threshold) {
        if (global_prediction > threshold) {
            updateHistoriesTaken(local_history_idx);

            assert(globalHistory < globalPredictorSize &&
                   local_history_idx < localPredictorSize);

            globalCtrs[globalHistory].increment();
            localCtrs[local_history_idx].increment();

            return true;
        } else {
            updateHistoriesNotTaken(local_history_idx);

            assert(globalHistory < globalPredictorSize &&
                   local_history_idx < localPredictorSize);

            globalCtrs[globalHistory].decrement();
            localCtrs[local_history_idx].decrement();

            return false;
        }
    } else {
        if (local_prediction > threshold) {
            updateHistoriesTaken(local_history_idx);

            assert(globalHistory < globalPredictorSize &&
                   local_history_idx < localPredictorSize);

            globalCtrs[globalHistory].increment();
            localCtrs[local_history_idx].increment();

            return true;
        } else {
            updateHistoriesNotTaken(local_history_idx);

            assert(globalHistory < globalPredictorSize &&
                   local_history_idx < localPredictorSize);

            globalCtrs[globalHistory].decrement();
            localCtrs[local_history_idx].decrement();

            return false;
        }
    }
}

// Update the branch predictor if it predicted a branch wrong.
void
TournamentBP::update(Addr &branch_addr, unsigned correct_gh, bool taken)
{

    uint8_t local_prediction;
    unsigned local_history_idx;
    unsigned local_predictor_idx;
    bool local_pred_taken;

    uint8_t global_prediction;
    bool global_pred_taken;

    // Load the correct global history into the register.
    globalHistory = correct_gh;

    // Get the local predictor's current prediction, remove the incorrect
    // update, and update the local predictor
    local_history_idx = calcLocHistIdx(branch_addr);
    local_predictor_idx = localHistoryTable[local_history_idx];
    local_predictor_idx = (local_predictor_idx >> 1) & localHistoryMask;

    local_prediction = localCtrs[local_predictor_idx].read();
    local_pred_taken = local_prediction > threshold;

    //Get the global predictor's current prediction, and update the
    //global predictor
    global_prediction = globalCtrs[globalHistory].read();
    global_pred_taken = global_prediction > threshold;

    //Update the choice predictor to tell it which one was correct
    if (local_pred_taken != global_pred_taken) {
        //If the local prediction matches the actual outcome, decerement
        //the counter.  Otherwise increment the counter.
        if (local_pred_taken == taken) {
            choiceCtrs[globalHistory].decrement();
        } else {
            choiceCtrs[globalHistory].increment();
        }
    }

    if (taken) {
        assert(globalHistory < globalPredictorSize &&
               local_predictor_idx < localPredictorSize);

        localCtrs[local_predictor_idx].increment();
        globalCtrs[globalHistory].increment();

        globalHistory = (globalHistory << 1) | 1;
        globalHistory = globalHistory & globalHistoryMask;

        localHistoryTable[local_history_idx] |= 1;
    } else {
        assert(globalHistory < globalPredictorSize &&
               local_predictor_idx < localPredictorSize);

        localCtrs[local_predictor_idx].decrement();
        globalCtrs[globalHistory].decrement();

        globalHistory = (globalHistory << 1);
        globalHistory = globalHistory & globalHistoryMask;

        localHistoryTable[local_history_idx] &= ~1;
    }
}
