#include "base/trace.hh"
#include "cpu/beta_cpu/2bit_local_pred.hh"

DefaultBP::SatCounter::SatCounter(unsigned bits)
    : maxVal((1 << bits) - 1), counter(0)
{
}

DefaultBP::SatCounter::SatCounter(unsigned bits, unsigned initial_val)
    : maxVal((1 << bits) - 1), counter(initial_val)
{
    // Check to make sure initial value doesn't exceed the max counter value.
    if (initial_val > maxVal) {
        panic("BP: Initial counter value exceeds max size.");
    }
}

void
DefaultBP::SatCounter::increment()
{
    if(counter < maxVal) {
        ++counter;
    }
}

void
DefaultBP::SatCounter::decrement()
{
    if(counter > 0) {
        --counter;
    }
}

DefaultBP::DefaultBP(unsigned _localPredictorSize,
                     unsigned _localCtrBits,
                     unsigned _instShiftAmt)
    : localPredictorSize(_localPredictorSize),
      localCtrBits(_localCtrBits),
      instShiftAmt(_instShiftAmt)
{
    // Should do checks here to make sure sizes are correct (powers of 2).

    // Setup the index mask.
    indexMask = localPredictorSize - 1;

    DPRINTF(Fetch, "Branch predictor: index mask: %#x\n", indexMask);

    // Setup the array of counters for the local predictor.
    localCtrs = new SatCounter[localPredictorSize](localCtrBits);

    DPRINTF(Fetch, "Branch predictor: local predictor size: %i\n",
            localPredictorSize);

    DPRINTF(Fetch, "Branch predictor: local counter bits: %i\n", localCtrBits);

    DPRINTF(Fetch, "Branch predictor: instruction shift amount: %i\n",
            instShiftAmt);
}

inline
bool
DefaultBP::getPrediction(uint8_t &count)
{
    // Get the MSB of the count
    return (count >> (localCtrBits - 1));
}

inline
unsigned
DefaultBP::getLocalIndex(Addr &branch_addr)
{
    return (branch_addr >> instShiftAmt) & indexMask;
}

bool
DefaultBP::lookup(Addr &branch_addr)
{
    uint8_t local_prediction;
    unsigned local_predictor_idx = getLocalIndex(branch_addr);

    DPRINTF(Fetch, "Branch predictor: Looking up index %#x\n",
            local_predictor_idx);

    local_prediction = localCtrs[local_predictor_idx].read();

    DPRINTF(Fetch, "Branch predictor: prediction is %i.\n",
            (int)local_prediction);

    return getPrediction(local_prediction);
}

void
DefaultBP::update(Addr &branch_addr, bool taken)
{
    unsigned local_predictor_idx;

    // Update the local predictor.
    local_predictor_idx = getLocalIndex(branch_addr);

    DPRINTF(Fetch, "Branch predictor: Looking up index %#x\n",
            local_predictor_idx);

    if (taken) {
        DPRINTF(Fetch, "Branch predictor: Branch updated as taken.\n");
        localCtrs[local_predictor_idx].increment();
    } else {
        DPRINTF(Fetch, "Branch predictor: Branch updated as not taken.\n");
        localCtrs[local_predictor_idx].decrement();
    }
}
