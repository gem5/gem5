#include "cpu/beta_cpu/tournament_pred.hh"

TournamentBP::SatCounter::SatCounter(unsigned bits)
    : maxVal((1 << bits) - 1), counter(0)
{
}

TournamentBP::SatCounter::SatCounter(unsigned bits, unsigned initial_val)
    : maxVal((1 << bits) - 1), counter(initial_val)
{
    // Check to make sure initial value doesn't exceed the max counter value.
    if (initial_val > maxVal) {
        panic("BP: Initial counter value exceeds max size.");
    }
}

void
TournamentBP::SatCounter::increment()
{
    if (counter < maxVal) {
        ++counter;
    }
}

void
TournamentBP::SatCounter::decrement()
{
    if (counter > 0) {
        --counter;
    }
}

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
    : local_predictor_size(_local_predictor_size),
      local_ctr_bits(_local_ctr_bits),
      local_history_table_size(_local_history_table_size),
      local_history_bits(_local_history_bits),
      global_predictor_size(_global_predictor_size),
      global_ctr_bits(_global_ctr_bits),
      global_history_bits(_global_history_bits),
      choice_predictor_size(_global_predictor_size),
      choice_ctr_bits(_choice_ctr_bits),
      instShiftAmt(_instShiftAmt)
{
    //Should do checks here to make sure sizes are correct (powers of 2)

    //Setup the array of counters for the local predictor
    local_ctrs = new SatCounter[local_predictor_size](local_ctr_bits);
    //Setup the history table for the local table
    local_history_table = new unsigned[local_history_table_size](0);
    // Setup the local history mask
    localHistoryMask = (1 << local_history_bits) - 1;

    //Setup the array of counters for the global predictor
    global_ctrs = new SatCounter[global_predictor_size](global_ctr_bits);
    //Clear the global history
    global_history = 0;
    // Setup the global history mask
    globalHistoryMask = (1 << global_history_bits) - 1;

    //Setup the array of counters for the choice predictor
    choice_ctrs = new SatCounter[choice_predictor_size](choice_ctr_bits);

    threshold = (1 << (local_ctr_bits - 1)) - 1;
    threshold = threshold / 2;
}

inline
unsigned
TournamentBP::calcLocHistIdx(Addr &branch_addr)
{
    return (branch_addr >> instShiftAmt) & (local_history_table_size - 1);
}

inline
void
TournamentBP::updateHistoriesTaken(unsigned local_history_idx)
{
    global_history = (global_history << 1) | 1;
    global_history = global_history & globalHistoryMask;

    local_history_table[local_history_idx] =
        (local_history_table[local_history_idx] << 1) | 1;
}

inline
void
TournamentBP::updateHistoriesNotTaken(unsigned local_history_idx)
{
    global_history = (global_history << 1);
    global_history = global_history & globalHistoryMask;

    local_history_table[local_history_idx] =
        (local_history_table[local_history_idx] << 1);
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
    local_predictor_idx = local_history_table[local_history_idx]
        & localHistoryMask;
    local_prediction = local_ctrs[local_predictor_idx].read();

    //Lookup in the global predictor to get its branch prediction
    global_prediction = global_ctrs[global_history].read();

    //Lookup in the choice predictor to see which one to use
    choice_prediction = choice_ctrs[global_history].read();

    //@todo Put a threshold value in for the three predictors that can
    // be set through the constructor (so this isn't hard coded).
    //Also should put some of this code into functions.
    if (choice_prediction > threshold) {
        if (global_prediction > threshold) {
            updateHistoriesTaken(local_history_idx);

            assert(global_history < global_predictor_size &&
                   local_history_idx < local_predictor_size);

            global_ctrs[global_history].increment();
            local_ctrs[local_history_idx].increment();

            return true;
        } else {
            updateHistoriesNotTaken(local_history_idx);

            assert(global_history < global_predictor_size &&
                   local_history_idx < local_predictor_size);

            global_ctrs[global_history].decrement();
            local_ctrs[local_history_idx].decrement();

            return false;
        }
    } else {
        if (local_prediction > threshold) {
            updateHistoriesTaken(local_history_idx);

            assert(global_history < global_predictor_size &&
                   local_history_idx < local_predictor_size);

            global_ctrs[global_history].increment();
            local_ctrs[local_history_idx].increment();

            return true;
        } else {
            updateHistoriesNotTaken(local_history_idx);

            assert(global_history < global_predictor_size &&
                   local_history_idx < local_predictor_size);

            global_ctrs[global_history].decrement();
            local_ctrs[local_history_idx].decrement();

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
    global_history = correct_gh;

    // Get the local predictor's current prediction, remove the incorrect
    // update, and update the local predictor
    local_history_idx = calcLocHistIdx(branch_addr);
    local_predictor_idx = local_history_table[local_history_idx];
    local_predictor_idx = (local_predictor_idx >> 1) & localHistoryMask;

    local_prediction = local_ctrs[local_predictor_idx].read();
    local_pred_taken = local_prediction > threshold;

    //Get the global predictor's current prediction, and update the
    //global predictor
    global_prediction = global_ctrs[global_history].read();
    global_pred_taken = global_prediction > threshold;

    //Update the choice predictor to tell it which one was correct
    if (local_pred_taken != global_pred_taken) {
        //If the local prediction matches the actual outcome, decerement
        //the counter.  Otherwise increment the counter.
        if (local_pred_taken == taken) {
            choice_ctrs[global_history].decrement();
        } else {
            choice_ctrs[global_history].increment();
        }
    }

    if (taken) {
        assert(global_history < global_predictor_size &&
               local_predictor_idx < local_predictor_size);

        local_ctrs[local_predictor_idx].increment();
        global_ctrs[global_history].increment();

        global_history = (global_history << 1) | 1;
        global_history = global_history & globalHistoryMask;

        local_history_table[local_history_idx] |= 1;
    }
    else {
        assert(global_history < global_predictor_size &&
               local_predictor_idx < local_predictor_size);

        local_ctrs[local_predictor_idx].decrement();
        global_ctrs[global_history].decrement();

        global_history = (global_history << 1);
        global_history = global_history & globalHistoryMask;

        local_history_table[local_history_idx] &= ~1;
    }
}
