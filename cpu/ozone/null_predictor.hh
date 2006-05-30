
#ifndef __CPU_OZONE_NULL_PREDICTOR_HH__
#define __CPU_OZONE_NULL_PREDICTOR_HH__

#include "arch/isa_traits.hh"
#include "cpu/inst_seq.hh"

template <class Impl>
class NullPredictor
{
  public:
    typedef typename Impl::Params Params;
    typedef typename Impl::DynInstPtr DynInstPtr;

    NullPredictor(Params *p) { }

    struct BPredInfo {
        BPredInfo()
            : PC(0), nextPC(0)
        { }

        BPredInfo(const Addr &pc, const Addr &next_pc)
            : PC(pc), nextPC(next_pc)
        { }

        Addr PC;
        Addr nextPC;
    };

    BPredInfo lookup(Addr &PC) { return BPredInfo(PC, PC+4); }

    void undo(BPredInfo &bp_info) { return; }

    /**
     * Predicts whether or not the instruction is a taken branch, and the
     * target of the branch if it is taken.
     * @param inst The branch instruction.
     * @param PC The predicted PC is passed back through this parameter.
     * @param tid The thread id.
     * @return Returns if the branch is taken or not.
     */
    bool predict(DynInstPtr &inst, Addr &PC, unsigned tid)
    { return false; }

    /**
     * Tells the branch predictor to commit any updates until the given
     * sequence number.
     * @param done_sn The sequence number to commit any older updates up until.
     * @param tid The thread id.
     */
    void update(const InstSeqNum &done_sn, unsigned tid) { }

    /**
     * Squashes all outstanding updates until a given sequence number.
     * @param squashed_sn The sequence number to squash any younger updates up
     * until.
     * @param tid The thread id.
     */
    void squash(const InstSeqNum &squashed_sn, unsigned tid) { }

    /**
     * Squashes all outstanding updates until a given sequence number, and
     * corrects that sn's update with the proper address and taken/not taken.
     * @param squashed_sn The sequence number to squash any younger updates up
     * until.
     * @param corr_target The correct branch target.
     * @param actually_taken The correct branch direction.
     * @param tid The thread id.
     */
    void squash(const InstSeqNum &squashed_sn, const Addr &corr_target,
                bool actually_taken, unsigned tid)
    { }

};

#endif // __CPU_OZONE_NULL_PREDICTOR_HH__
