#include "base/trace.hh"
#include "cpu/beta_cpu/2bit_local_pred.hh"

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
    localCtrs = new SatCounter[localPredictorSize];

    for (int i = 0; i < localPredictorSize; ++i)
        localCtrs[i].setBits(_localCtrBits);

    DPRINTF(Fetch, "Branch predictor: local predictor size: %i\n",
            localPredictorSize);

    DPRINTF(Fetch, "Branch predictor: local counter bits: %i\n", localCtrBits);

    DPRINTF(Fetch, "Branch predictor: instruction shift amount: %i\n",
            instShiftAmt);
}

bool
DefaultBP::lookup(Addr &branch_addr)
{
    bool taken;
    uint8_t local_prediction;
    unsigned local_predictor_idx = getLocalIndex(branch_addr);

    DPRINTF(Fetch, "Branch predictor: Looking up index %#x\n",
            local_predictor_idx);

    assert(local_predictor_idx < localPredictorSize);

    local_prediction = localCtrs[local_predictor_idx].read();

    DPRINTF(Fetch, "Branch predictor: prediction is %i.\n",
            (int)local_prediction);

    taken = getPrediction(local_prediction);

#if 0
    // Speculative update.
    if (taken) {
        DPRINTF(Fetch, "Branch predictor: Branch updated as taken.\n");
        localCtrs[local_predictor_idx].increment();
    } else {
        DPRINTF(Fetch, "Branch predictor: Branch updated as not taken.\n");
        localCtrs[local_predictor_idx].decrement();
    }
#endif

    return taken;
}

void
DefaultBP::update(Addr &branch_addr, bool taken)
{
    unsigned local_predictor_idx;

    // Update the local predictor.
    local_predictor_idx = getLocalIndex(branch_addr);

    DPRINTF(Fetch, "Branch predictor: Looking up index %#x\n",
            local_predictor_idx);

    assert(local_predictor_idx < localPredictorSize);

    if (taken) {
        DPRINTF(Fetch, "Branch predictor: Branch updated as taken.\n");
        localCtrs[local_predictor_idx].increment();
    } else {
        DPRINTF(Fetch, "Branch predictor: Branch updated as not taken.\n");
        localCtrs[local_predictor_idx].decrement();
    }
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
