#ifndef __2BIT_LOCAL_PRED_HH__
#define __2BIT_LOCAL_PRED_HH__

// For Addr type.
#include "arch/alpha/isa_traits.hh"

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

    inline bool getPrediction(uint8_t &count);

    inline unsigned getLocalIndex(Addr &PC);

    /**
     * Private counter class for the internal saturating counters.
     * Implements an n bit saturating counter and provides methods to
     * increment, decrement, and read it.
     * @todo Consider making this something that more closely mimics a
     * built in class so you can use ++ or --.
     */
    class SatCounter
    {
      public:
        /**
         * Constructor for the counter.
         * @param bits How many bits the counter will have.
         */
        SatCounter(unsigned bits);

        /**
         * Constructor for the counter.
         * @param bits How many bits the counter will have.
         * @param initial_val Starting value for each counter.
         */
        SatCounter(unsigned bits, unsigned initial_val);

        /**
         * Increments the counter's current value.
         */
        void increment();

        /**
         * Decrements the counter's current value.
         */
        void decrement();

        /**
         * Read the counter's value.
         */
        uint8_t read()
        {
            return counter;
        }

      private:
        uint8_t maxVal;
        uint8_t counter;
    };

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

#endif // __2BIT_LOCAL_PRED_HH__
