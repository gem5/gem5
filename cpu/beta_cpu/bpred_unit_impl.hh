
#include "cpu/beta_cpu/bpred_unit.hh"

template<class Impl>
DefaultBPredUnit<Impl>::DefaultBPredUnit(Params &params)
  : BP(params.localPredictorSize,
       params.localPredictorCtrBits,
       params.instShiftAmt),
    BTB(params.BTBEntries,
        params.BTBTagSize,
        params.instShiftAmt)
{
}
