
#ifndef __BPRED_UNIT_HH__
#define __BPRED_UNIT_HH__

// For Addr type.
#include "arch/alpha/isa_traits.hh"

#include "cpu/beta_cpu/2bit_local_pred.hh"
#include "cpu/beta_cpu/btb.hh"

/**
 * Basically a wrapper class to hold both the branch predictor
 * and the BTB.  Right now I'm unsure of the implementation; it would
 * be nicer to have something closer to the CPUPolicy or the Impl where
 * this is just typedefs, but it forces the upper level stages to be
 * aware of the constructors of the BP and the BTB.  The nicer thing
 * to do is have this templated on the Impl, accept the usual Params
 * object, and be able to call the constructors on the BP and BTB.
 */
template<class Impl>
class DefaultBPredUnit
{
  public:
    typedef typename Impl::Params Params;

    DefaultBPredUnit(Params &params);

    bool BPLookup(Addr &inst_PC)
    { return BP.lookup(inst_PC); }

    bool BTBValid(Addr &inst_PC)
    { return BTB.valid(inst_PC); }

    Addr BTBLookup(Addr &inst_PC)
    { return BTB.lookup(inst_PC); }

    void BPUpdate(Addr &inst_PC, bool taken)
    { BP.update(inst_PC, taken); }

    void BTBUpdate(Addr &inst_PC, Addr &target_PC)
    { BTB.update(inst_PC, target_PC); }

  private:

    DefaultBP BP;

    DefaultBTB BTB;

};

#endif // __BPRED_UNIT_HH__
