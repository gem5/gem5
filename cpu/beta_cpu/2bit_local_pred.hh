#ifndef __CPU_BETA_CPU_2BIT_LOCAL_PRED_HH__
#define __CPU_BETA_CPU_2BIT_LOCAL_PRED_HH__

// For Addr type.
#include "arch/alpha/isa_traits.hh"
#include "cpu/beta_cpu/sat_counter.hh"

class DefaultBP
{
  public:
    /**
     * Default branch predictor constructor.
     */
    DefaultBP(unsigned localPredictorSize, unsigned localCtrBits,
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
    void update(Addr &branch_addr, bool taken);

  private:

    /** Returns the taken/not taken prediction given the value of the
     *  counter.
     */
    inline bool getPrediction(uint8_t &count);

    /** Calculates the local index based on the PC. */
    inline unsigned getLocalIndex(Addr &PC);

    /** Array of counters that make up the local predictor. */
    SatCounter *localCtrs;

    /** Size of the local predictor. */
    unsigned localPredictorSize;

    /** Number of bits of the local predictor's counters. */
    unsigned localCtrBits;

    /** Number of bits to shift the PC when calculating index. */
    unsigned instShiftAmt;

    /** Mask to get index bits. */
    unsigned indexMask;
};

#endif // __CPU_BETA_CPU_2BIT_LOCAL_PRED_HH__
