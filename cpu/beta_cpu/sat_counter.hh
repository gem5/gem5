#ifndef __CPU_BETA_CPU_SAT_COUNTER_HH__
#define __CPU_BETA_CPU_SAT_COUNTER_HH__

#include <stdint.h>

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
     */
    SatCounter();

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
     * Sets the number of bits.
     */
    void setBits(unsigned bits);

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
    const uint8_t read() const
    {
        return counter;
    }

  private:
    uint8_t maxVal;
    uint8_t counter;
};

#endif // __CPU_BETA_CPU_SAT_COUNTER_HH__
