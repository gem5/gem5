#ifndef __CPU_BETA_CPU_TOURNAMENT_PRED_HH__
#define __CPU_BETA_CPU_TOURNAMENT_PRED_HH__

// For Addr type.
#include "arch/alpha/isa_traits.hh"
#include "cpu/beta_cpu/sat_counter.hh"

class TournamentBP
{
  public:
    /**
     * Default branch predictor constructor.
     */
    TournamentBP(unsigned local_predictor_size,
                 unsigned local_ctr_bits,
                 unsigned local_history_table_size,
                 unsigned local_history_bits,
                 unsigned global_predictor_size,
                 unsigned global_history_bits,
                 unsigned global_ctr_bits,
                 unsigned choice_predictor_size,
                 unsigned choice_ctr_bits,
                 unsigned instShiftAmt);

    /**
     * Looks up the given address in the branch predictor and returns
     * a true/false value as to whether it is taken.
     * @param branch_addr The address of the branch to look up.
     * @return Whether or not the branch is taken.
     */
    bool lookup(Addr &branch_addr);

    /**
     * Updates the branch predictor with the actual result of a branch.
     * @param branch_addr The address of the branch to update.
     * @param taken Whether or not the branch was taken.
     */
    void update(Addr &branch_addr, unsigned global_history, bool taken);

    inline unsigned readGlobalHist() { return global_history; }

  private:

    inline bool getPrediction(uint8_t &count);

    inline unsigned calcLocHistIdx(Addr &branch_addr);

    inline void updateHistoriesTaken(unsigned local_history_idx);

    inline void updateHistoriesNotTaken(unsigned local_history_idx);

    /** Local counters. */
    SatCounter *local_ctrs;

    /** Size of the local predictor. */
    unsigned local_predictor_size;

    /** Number of bits of the local predictor's counters. */
    unsigned local_ctr_bits;

    /** Array of local history table entries. */
    unsigned *local_history_table;

    /** Size of the local history table. */
    unsigned local_history_table_size;

    /** Number of bits for each entry of the local history table.
     *  @todo Doesn't this come from the size of the local predictor?
     */
    unsigned local_history_bits;

    /** Mask to get the proper local history. */
    unsigned localHistoryMask;


    /** Array of counters that make up the global predictor. */
    SatCounter *global_ctrs;

    /** Size of the global predictor. */
    unsigned global_predictor_size;

    /** Number of bits of the global predictor's counters. */
    unsigned global_ctr_bits;

    /** Global history register. */
    unsigned global_history;

    /** Number of bits for the global history. */
    unsigned global_history_bits;

    /** Mask to get the proper global history. */
    unsigned globalHistoryMask;


    /** Array of counters that make up the choice predictor. */
    SatCounter *choice_ctrs;

    /** Size of the choice predictor (identical to the global predictor). */
    unsigned choice_predictor_size;

    /** Number of bits of the choice predictor's counters. */
    unsigned choice_ctr_bits;

    /** Number of bits to shift the instruction over to get rid of the word
     *  offset.
     */
    unsigned instShiftAmt;

    /** Threshold for the counter value; above the threshold is taken,
     *  equal to or below the threshold is not taken.
     */
    unsigned threshold;
};

#endif // __CPU_BETA_CPU_TOURNAMENT_PRED_HH__
