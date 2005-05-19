
#ifndef __BPRED_UNIT_HH__
#define __BPRED_UNIT_HH__

// For Addr type.
#include "arch/alpha/isa_traits.hh"
#include "base/statistics.hh"
#include "cpu/inst_seq.hh"

#include "cpu/beta_cpu/2bit_local_pred.hh"
#include "cpu/beta_cpu/tournament_pred.hh"
#include "cpu/beta_cpu/btb.hh"
#include "cpu/beta_cpu/ras.hh"

#include <list>

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
class TwobitBPredUnit
{
  public:
    typedef typename Impl::Params Params;
    typedef typename Impl::DynInstPtr DynInstPtr;

    TwobitBPredUnit(Params &params);

    void regStats();

    bool predict(DynInstPtr &inst, Addr &PC);

    void update(const InstSeqNum &done_sn);

    void squash(const InstSeqNum &squashed_sn);

    void squash(const InstSeqNum &squashed_sn, const Addr &corr_target,
                bool actually_taken);

    bool BPLookup(Addr &inst_PC)
    { return BP.lookup(inst_PC); }

    bool BTBValid(Addr &inst_PC)
    { return BTB.valid(inst_PC); }

    Addr BTBLookup(Addr &inst_PC)
    { return BTB.lookup(inst_PC); }

    // Will want to include global history.
    void BPUpdate(Addr &inst_PC, bool taken)
    { BP.update(inst_PC, taken); }

    void BTBUpdate(Addr &inst_PC, Addr &target_PC)
    { BTB.update(inst_PC, target_PC); }

  private:
    struct PredictorHistory {
        PredictorHistory(const InstSeqNum &seq_num, const Addr &inst_PC,
                         const bool pred_taken)
            : seqNum(seq_num), PC(inst_PC), predTaken(pred_taken),
              globalHistory(0), usedRAS(0), wasCall(0), RASIndex(0),
              RASTarget(0)
        { }

        InstSeqNum seqNum;

        Addr PC;

        bool predTaken;

        unsigned globalHistory;

        bool usedRAS;

        bool wasCall;

        unsigned RASIndex;

        Addr RASTarget;
    };

    std::list<PredictorHistory> predHist;

    DefaultBP BP;

    DefaultBTB BTB;

    ReturnAddrStack RAS;

    Stats::Scalar<> lookups;
    Stats::Scalar<> condPredicted;
    Stats::Scalar<> condIncorrect;
    Stats::Scalar<> BTBLookups;
    Stats::Scalar<> BTBHits;
    Stats::Scalar<> BTBCorrect;
    Stats::Scalar<> usedRAS;
    Stats::Scalar<> RASIncorrect;
};

#endif // __BPRED_UNIT_HH__
