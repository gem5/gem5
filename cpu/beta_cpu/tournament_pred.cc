#include "cpu/beta_cpu/tournament_pred.hh"

TournamentBP::TournamentBP(unsigned _local_predictor_size,
                           unsigned _local_ctr_bits,
                           unsigned _local_history_table_size,
                           unsigned _local_history_bits,
                           unsigned _global_predictor_size,
                           unsigned _global_ctr_bits,
                           unsigned _global_history_bits,
                           unsigned _choice_predictor_size,
                           unsigned _choice_ctr_bits,
                           unsigned _instShiftAmt)
    : localPredictorSize(_local_predictor_size),
      localCtrBits(_local_ctr_bits),
      localHistoryTableSize(_local_history_table_size),
      localHistoryBits(_local_history_bits),
      globalPredictorSize(_global_predictor_size),
      globalCtrBits(_global_ctr_bits),
      globalHistoryBits(_global_history_bits),
      choicePredictorSize(_global_predictor_size),
      choiceCtrBits(_choice_ctr_bits),
      instShiftAmt(_instShiftAmt)
{
    //Should do checks here to make sure sizes are correct (powers of 2)

    //Setup the array of counters for the local predictor
    localCtrs = new SatCounter[localPredictorSize];

    for (int i = 0; i < localPredictorSize; ++i)
        localCtrs[i].setBits(localCtrBits);

    //Setup the history table for the local table
    localHistoryTable = new unsigned[localHistoryTableSize];

    for (int i = 0; i < localHistoryTableSize; ++i)
        localHistoryTable[i] = 0;

    // Setup the local history mask
    localHistoryMask = (1 << localHistoryBits) - 1;

    //Setup the array of counters for the global predictor
    globalCtrs = new SatCounter[globalPredictorSize];

    for (int i = 0; i < globalPredictorSize; ++i)
        globalCtrs[i].setBits(globalCtrBits);

    //Clear the global history
    globalHistory = 0;
    // Setup the global history mask
    globalHistoryMask = (1 << globalHistoryBits) - 1;

    //Setup the array of counters for the choice predictor
    choiceCtrs = new SatCounter[choicePredictorSize];

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
    }
    else {
        assert(globalHistory < globalPredictorSize &&
               local_predictor_idx < localPredictorSize);

        localCtrs[local_predictor_idx].decrement();
        globalCtrs[globalHistory].decrement();

        globalHistory = (globalHistory << 1);
        globalHistory = globalHistory & globalHistoryMask;

        localHistoryTable[local_history_idx] &= ~1;
    }
}
