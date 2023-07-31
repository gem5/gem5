/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010,2022-2023 The University of Edinburgh
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CPU_PRED_BPRED_UNIT_HH__
#define __CPU_PRED_BPRED_UNIT_HH__

#include <deque>

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/pred/branch_type.hh"
#include "cpu/pred/btb.hh"
#include "cpu/pred/indirect.hh"
#include "cpu/pred/ras.hh"
#include "cpu/static_inst.hh"
#include "enums/TargetProvider.hh"
#include "params/BranchPredictor.hh"
#include "sim/probe/pmu.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace o3 {
    class BAC;
}

namespace branch_prediction
{



/**
 * Basically a wrapper class to hold both the branch predictor
 * and the BTB.
 */
class BPredUnit : public SimObject
{
    friend class o3::BAC;

    typedef BranchPredictorParams Params;
    typedef enums::TargetProvider TargetProvider;

    /** Branch Predictor Unit (BPU) interface functions */
  public:



    /**
     * @param params The params object, that has the size of the BP and BTB.
     */
    BPredUnit(const Params &p);

    void regProbePoints() override;

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /**
     * Predicts whether or not the instruction is a taken branch, and the
     * target of the branch if it is taken.
     * @param inst The branch instruction.
     * @param PC The predicted PC is passed back through this parameter.
     * @param tid The thread id.
     * @return Returns if the branch is taken or not.
     */
    bool predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                 PCStateBase &pc, ThreadID tid);

    /**
     * Tells the branch predictor to commit any updates until the given
     * sequence number.
     * @param done_sn The sequence number to commit any older updates up until.
     * @param tid The thread id.
     */
    void update(const InstSeqNum &done_sn, ThreadID tid);

    /**
     * Squashes all outstanding updates until a given sequence number.
     * @param squashed_sn The sequence number to squash any younger updates up
     * until.
     * @param tid The thread id.
     */
    void squash(const InstSeqNum &squashed_sn, ThreadID tid);

    /**
     * Squashes all outstanding updates until a given sequence number, and
     * corrects that sn's update with the proper address and taken/not taken.
     * @param squashed_sn The sequence number to squash any younger updates up
     * until.
     * @param corr_target The correct branch target.
     * @param actually_taken The correct branch direction.
     * @param tid The thread id.
     * @param inst The static instruction that caused the misprediction
     * @param from_commit Indicate whether the squash is comming from commit
     *              or from decode. Its optional and used for statistics.
     */
    void squash(const InstSeqNum &squashed_sn,
                const PCStateBase &corr_target,
                bool actually_taken, ThreadID tid, bool from_commit=true);

  protected:

    /** *******************************************************
     * Interface functions to the conditional branch predictor
     *
    */

    /**
     * Looks up a given conditional branch PC of in the BP to see if it
     * is taken or not taken.
     * @param PC The PC to look up.
     * @param bpHistory Pointer that will be set to an object that
     * has the branch predictor state associated with the lookup.
     * @return Whether the branch is taken or not taken.
     */
    virtual bool lookup(ThreadID tid, Addr pc, void * &bpHistory) = 0;

    /**
     * Once done with the prediction this function updates the
     * path and global history. All branches call this function
     * including unconditional once.
     * @param PC The branch's PC that will be updated.
     * @param uncond Wheather or not this branch is an unconditional branch.
     * @param taken Whether or not the branch was taken
     * @param target The final target of branch. Some modern
     * predictors use the target in their history.
     * @param bpHistory Pointer that will be set to an object that
     * has the branch predictor state associated with the lookup.
     */
    virtual void updateHistories(ThreadID tid, Addr pc, bool uncond,
                            bool taken, Addr target, void * &bpHistory) = 0;

    /**
     * @param bpHistory Pointer to the history object.  The predictor
     * will need to update any state and delete the object.
     */
    virtual void squash(ThreadID tid, void * &bpHistory) = 0;


    /**
     * Updates the BP with taken/not taken information.
     * @param PC The branch's PC that will be updated.
     * @param taken Whether the branch was taken or not taken.
     * @param bpHistory Pointer to the branch predictor state that is
     * associated with the branch lookup that is being updated.
     * @param squashed Set to true when this function is called during a
     * squash operation.
     * @param inst Static instruction information
     * @param corrTarget The resolved target of the branch (only needed
     * for squashed branches)
     * @todo Make this update flexible enough to handle a global predictor.
     */
    virtual void update(ThreadID tid, Addr pc, bool taken,
                   void * &bpHistory, bool squashed,
                   const StaticInstPtr &inst, Addr corrTarget) = 0;

    /**
     * Special function for the decoupled front-end. In it there can be
     * branches which are not detected by the BPU in the first place as it
     * requires a BTB hit. This function will generage a placeholder for
     * such a branch once it is pre-decoded in the fetch stage. It will
     * only create the branch history object but not update BPU internal.
     * If the branch turns to be wrong then decode or commit will
     * be able to user the normal squash functionality to correct the branch.
     * Note that not all branch predictors implement this functionality.
     * @param PC The branch's PC.
     * @param uncond Wheather or not this branch is an unconditional branch.
     * @param bpHistory Pointer that will be set to an branch history object.
     */
    virtual void branchPlaceholder(ThreadID tid, Addr pc,
                                bool uncond, void * &bpHistory);

    /**
     * Looks up a given PC in the BTB to see if a matching entry exists.
     * @param inst_PC The PC to look up.
     * @return Whether the BTB contains the given PC.
     */
    bool BTBValid(ThreadID tid, Addr instPC)
    {
        return btb->valid(tid, instPC);
    }
    bool BTBValid(ThreadID tid, PCStateBase &instPC)
    {
        return BTBValid(tid, instPC.instAddr());
    }

    /**
     * Looks up a given PC in the BTB to get the predicted target. The PC may
     * be changed or deleted in the future, so it needs to be used immediately,
     * and/or copied for use later.
     * @param inst_PC The PC to look up.
     * @return The address of the target of the branch.
     */
    const PCStateBase *
    BTBLookup(ThreadID tid, PCStateBase &instPC)
    {
        return btb->lookup(tid, instPC.instAddr());
    }

    /**
     * Looks up a given PC in the BTB to get current static instruction
     * information. This is necessary in a decoupled frontend as
     * the information does not usually exist at that this point.
     * Only for instructions (branches) that hit in the BTB this information
     * is available as the BTB stores them together with the target.
     *
     * @param inst_PC The PC to look up.
     * @return The static instruction info of the given PC if existant.
     */
    const StaticInstPtr
    BTBLookupInst(ThreadID tid, Addr instPC)
    {
        return btb->lookupInst(tid, instPC);
    }
    const StaticInstPtr
    BTBLookupInst(ThreadID tid, PCStateBase &instPC)
    {
        return BTBLookupInst(tid, instPC.instAddr());
    }

    /**
     * Updates the BTB with the target of a branch.
     * @param inst_PC The branch's PC that will be updated.
     * @param target_PC The branch's target that will be added to the BTB.
     */
    void
    BTBUpdate(ThreadID tid, Addr instPC, const PCStateBase &target)
    {
        ++stats.BTBUpdates;
        return btb->update(tid, instPC, target);
    }


    void dump();

  private:
    struct PredictorHistory
    {
        /**
         * Makes a predictor history struct that contains any
         * information needed to update the predictor, BTB, and RAS.
         */
        PredictorHistory(ThreadID _tid, InstSeqNum sn, Addr _pc,
                         const StaticInstPtr & inst)
            : seqNum(sn), tid(_tid), pc(_pc),
              inst(inst), type(getBranchType(inst)),
              call(inst->isCall()), uncond(inst->isUncondCtrl()),
              predTaken(false), actuallyTaken(false), condPred(false),
              btbHit(false), targetProvider(TargetProvider::NoTarget),
              resteered(false), mispredict(false), target(nullptr),
              bpHistory(nullptr),
              indirectHistory(nullptr), rasHistory(nullptr)
        { }

        ~PredictorHistory()
        {
            assert(bpHistory == nullptr);
            assert(indirectHistory == nullptr);
            assert(rasHistory == nullptr);
        }

        PredictorHistory (const PredictorHistory&) = delete;
        PredictorHistory& operator= (const PredictorHistory&) = delete;

        bool
        operator==(const PredictorHistory &entry) const
        {
            return this->seqNum == entry.seqNum;
        }

        /** The sequence number for the predictor history entry. */
        InstSeqNum seqNum;

        /** The thread id. */
        const ThreadID tid;

        /** The PC associated with the sequence number. */
        const Addr pc;

        /** The branch instrction */
        const StaticInstPtr inst;

        /** The type of the branch */
        const BranchType type;

        /** Whether or not the instruction was a call. */
        const bool call;

        /** Was unconditional control */
        const bool uncond;

        /** Whether or not it was predicted taken. */
        bool predTaken;

        /** To record the actual outcome of the branch */
        bool actuallyTaken;

        /** The prediction of the conditional predictor */
        bool condPred;

        /** Was BTB hit at prediction time */
        bool btbHit;

        /** Which component provided the target */
        TargetProvider targetProvider;

        /** Resteered */
        bool resteered;

        /** The branch was corrected hence was mispredicted. */
        bool mispredict;

        /** The predicted target */
        std::unique_ptr<PCStateBase> target;

        /**
         * Pointer to the history objects passed back from the branch
         * predictor subcomponents.
         * It is used to update or restore state.
         * Respectively for conditional, indirect and RAS.
         */
        void *bpHistory = nullptr;

        void *indirectHistory = nullptr;

        void *rasHistory = nullptr;

    };

    typedef std::deque<PredictorHistory*> History;


    /**
     * Internal prediction function.
    */
    bool predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
               PCStateBase &pc, ThreadID tid, PredictorHistory* &bpu_history);

    /**
     * Squashes a particular branch instance. Revertes
     * all speculative updated state and deletes the history object
     * @param tid The thread id.
     * @param bpu_history The history to be squashed.
     */
    void squashHistory(ThreadID tid, PredictorHistory* &bpu_history);


    /**
     * Commit a particular branch
     * @param tid The thread id.
     * @param bpu_history The history of the branch to be commited.
     */
    void commitBranch(ThreadID tid, PredictorHistory* &bpu_history);



  protected:
    /** Number of the threads for which the branch history is maintained. */
    const unsigned numThreads;

    /** Requires the BTB to hit for returns and indirect branches. For an
     * advances front-end there is no other way to know about the branch. */
    const bool requiresBTBHit;

    /** Number of bits to shift instructions by for predictor addresses. */
    const unsigned instShiftAmt;

    /**
     * The per-thread predictor history. This is used to update the predictor
     * as instructions are committed, or restore it to the proper state after
     * a squash.
     */
    std::vector<History> predHist;

    /** The BTB. */
    BranchTargetBuffer * btb;

    /** The return address stack. */
    ReturnAddrStack * ras;

    /** The indirect target predictor. */
    IndirectPredictor * iPred;

    /** Statistics */
    struct BPredUnitStats : public statistics::Group
    {
        BPredUnitStats(statistics::Group *parent, BPredUnit *bp);

        /** Stat for number of BP lookups. */
        statistics::Vector2d lookups;
        /** Stat for branches that got squashed as an earlier branch was
         * mispredicted. */
        statistics::Vector2d squashes;
        /** Stat for branches that got corrected but not yet commited.
         * A corrected branch might not be commited in case an earlier
         * branch was mispredicted */
        statistics::Vector2d corrected;
        /** Stat for branches finally committed */
        statistics::Vector2d committed;
        /** Stat for the number of committed branches that where
         * mispredicted by the BPU */
        statistics::Vector2d mispredicted;
        /** Stat for the target provider of taken branches*/
        statistics::Vector2d targetProvider;
        /** Stat for branches where the target was incorrect or not
         * available at prediction */
        statistics::Vector2d targetWrong;
        /** Stat for branches squashed from decode */
        statistics::Vector2d earlyResteers;

        /** Stat for number of conditional branches predicted. */
        statistics::Scalar condPredicted;
        /** Stat for n of conditional branches predicted as taken. */
        statistics::Scalar condPredictedTaken;
        /** Stat for number of conditional branches predicted incorrectly. */
        statistics::Scalar condIncorrect;
        /** Stat for number of branches predicted incorrectly. */
        statistics::Scalar incorrect;
        /** Stat for number of BTB lookups. */
        statistics::Scalar BTBLookups;
        /** Stat for number of BTB updates. */
        statistics::Scalar BTBUpdates;
        /** Stat for number of BTB hits. */
        statistics::Scalar BTBHits;
        /** Stat for number for the ratio between BTB hits and BTB lookups. */
        statistics::Formula BTBHitRatio;
        /** Stat for number BTB misspredictions. No or wrong target found */
        statistics::Scalar BTBMispredicted;

        /** Stat for the number of indirect target lookups.*/
        statistics::Scalar indirectLookups;
        /** Stat for the number of indirect target hits.*/
        statistics::Scalar indirectHits;
        /** Stat for the number of indirect target misses.*/
        statistics::Scalar indirectMisses;
        /** Stat for the number of indirect target mispredictions.*/
        statistics::Scalar indirectMispredicted;

        /** Stat for the number of branches predicted taken but miss in BTB*/
        statistics::Scalar predTakenBTBMiss;
        /** Stat for the number of branches predicted not taken but
         * turn out to be taken*/
        statistics::Scalar NotTakenMispredicted;
        /** Stat for the number of branches predicted taken but turn
         * out to be not taken*/
        statistics::Scalar TakenMispredicted;


    } stats;

  protected:

    /**
     * @{
     * @name PMU Probe points.
     */

    /**
     * Helper method to instantiate probe points belonging to this
     * object.
     *
     * @param name Name of the probe point.
     * @return A unique_ptr to the new probe point.
     */
    probing::PMUUPtr pmuProbePoint(const char *name);


    /**
     * Branches seen by the branch predictor
     *
     * @note This counter includes speculative branches.
     */
    probing::PMUUPtr ppBranches;

    /** Miss-predicted branches */
    probing::PMUUPtr ppMisses;

    /** @} */
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_BPRED_UNIT_HH__
