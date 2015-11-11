/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010 The University of Edinburgh
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
 *
 * Authors: Kevin Lim
 *          Korey Sewell
 *          Timothy M. Jones
 *          Nilay Vaish
 */

#ifndef __CPU_PRED_BPRED_UNIT_HH__
#define __CPU_PRED_BPRED_UNIT_HH__

#include <deque>

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/pred/btb.hh"
#include "cpu/pred/ras.hh"
#include "cpu/inst_seq.hh"
#include "cpu/static_inst.hh"
#include "params/BranchPredictor.hh"
#include "sim/probe/pmu.hh"
#include "sim/sim_object.hh"

/**
 * Basically a wrapper class to hold both the branch predictor
 * and the BTB.
 */
class BPredUnit : public SimObject
{
  public:
      typedef BranchPredictorParams Params;
    /**
     * @param params The params object, that has the size of the BP and BTB.
     */
    BPredUnit(const Params *p);

    /**
     * Registers statistics.
     */
    void regStats() override;

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
                 TheISA::PCState &pc, ThreadID tid);
    bool predictInOrder(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                        int asid, TheISA::PCState &instPC,
                        TheISA::PCState &predPC, ThreadID tid);

    // @todo: Rename this function.
    virtual void uncondBranch(Addr pc, void * &bp_history) = 0;

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
     */
    void squash(const InstSeqNum &squashed_sn,
                const TheISA::PCState &corr_target,
                bool actually_taken, ThreadID tid);

    /**
     * @param bp_history Pointer to the history object.  The predictor
     * will need to update any state and delete the object.
     */
    virtual void squash(void *bp_history) = 0;

    /**
     * Looks up a given PC in the BP to see if it is taken or not taken.
     * @param inst_PC The PC to look up.
     * @param bp_history Pointer that will be set to an object that
     * has the branch predictor state associated with the lookup.
     * @return Whether the branch is taken or not taken.
     */
    virtual bool lookup(Addr instPC, void * &bp_history) = 0;

     /**
     * If a branch is not taken, because the BTB address is invalid or missing,
     * this function sets the appropriate counter in the global and local
     * predictors to not taken.
     * @param inst_PC The PC to look up the local predictor.
     * @param bp_history Pointer that will be set to an object that
     * has the branch predictor state associated with the lookup.
     */
    virtual void btbUpdate(Addr instPC, void * &bp_history) = 0;

    /**
     * Looks up a given PC in the BTB to see if a matching entry exists.
     * @param inst_PC The PC to look up.
     * @return Whether the BTB contains the given PC.
     */
    bool BTBValid(Addr instPC)
    { return BTB.valid(instPC, 0); }

    /**
     * Looks up a given PC in the BTB to get the predicted target.
     * @param inst_PC The PC to look up.
     * @return The address of the target of the branch.
     */
    TheISA::PCState BTBLookup(Addr instPC)
    { return BTB.lookup(instPC, 0); }

    /**
     * Updates the BP with taken/not taken information.
     * @param inst_PC The branch's PC that will be updated.
     * @param taken Whether the branch was taken or not taken.
     * @param bp_history Pointer to the branch predictor state that is
     * associated with the branch lookup that is being updated.
     * @param squashed Set to true when this function is called during a
     * squash operation.
     * @todo Make this update flexible enough to handle a global predictor.
     */
    virtual void update(Addr instPC, bool taken, void *bp_history,
                        bool squashed) = 0;
     /**
     * Deletes the associated history with a branch, performs no predictor
     * updates.  Used for branches that mispredict and update tables but
     * are still speculative and later retire.
     * @param bp_history History to delete associated with this predictor
     */
    virtual void retireSquashed(void *bp_history) = 0;

    /**
     * Updates the BTB with the target of a branch.
     * @param inst_PC The branch's PC that will be updated.
     * @param target_PC The branch's target that will be added to the BTB.
     */
    void BTBUpdate(Addr instPC, const TheISA::PCState &target)
    { BTB.update(instPC, target, 0); }

    void dump();

  private:
    struct PredictorHistory {
        /**
         * Makes a predictor history struct that contains any
         * information needed to update the predictor, BTB, and RAS.
         */
        PredictorHistory(const InstSeqNum &seq_num, Addr instPC,
                         bool pred_taken, void *bp_history,
                         ThreadID _tid)
            : seqNum(seq_num), pc(instPC), bpHistory(bp_history), RASTarget(0),
              RASIndex(0), tid(_tid), predTaken(pred_taken), usedRAS(0), pushedRAS(0),
              wasCall(0), wasReturn(0), wasSquashed(0)
        {}

        bool operator==(const PredictorHistory &entry) const {
            return this->seqNum == entry.seqNum;
        }

        /** The sequence number for the predictor history entry. */
        InstSeqNum seqNum;

        /** The PC associated with the sequence number. */
        Addr pc;

        /** Pointer to the history object passed back from the branch
         * predictor.  It is used to update or restore state of the
         * branch predictor.
         */
        void *bpHistory;

        /** The RAS target (only valid if a return). */
        TheISA::PCState RASTarget;

        /** The RAS index of the instruction (only valid if a call). */
        unsigned RASIndex;

        /** The thread id. */
        ThreadID tid;

        /** Whether or not it was predicted taken. */
        bool predTaken;

        /** Whether or not the RAS was used. */
        bool usedRAS;

        /* Whether or not the RAS was pushed */
        bool pushedRAS;

        /** Whether or not the instruction was a call. */
        bool wasCall;

        /** Whether or not the instruction was a return. */
        bool wasReturn;

        /** Whether this instruction has already mispredicted/updated bp */
        bool wasSquashed;
    };

    typedef std::deque<PredictorHistory> History;

    /** Number of the threads for which the branch history is maintained. */
    const unsigned numThreads;


    /**
     * The per-thread predictor history. This is used to update the predictor
     * as instructions are committed, or restore it to the proper state after
     * a squash.
     */
    std::vector<History> predHist;

    /** The BTB. */
    DefaultBTB BTB;

    /** The per-thread return address stack. */
    std::vector<ReturnAddrStack> RAS;

    /** Stat for number of BP lookups. */
    Stats::Scalar lookups;
    /** Stat for number of conditional branches predicted. */
    Stats::Scalar condPredicted;
    /** Stat for number of conditional branches predicted incorrectly. */
    Stats::Scalar condIncorrect;
    /** Stat for number of BTB lookups. */
    Stats::Scalar BTBLookups;
    /** Stat for number of BTB hits. */
    Stats::Scalar BTBHits;
    /** Stat for number of times the BTB is correct. */
    Stats::Scalar BTBCorrect;
    /** Stat for percent times an entry in BTB found. */
    Stats::Formula BTBHitPct;
    /** Stat for number of times the RAS is used to get a target. */
    Stats::Scalar usedRAS;
    /** Stat for number of times the RAS is incorrect. */
    Stats::Scalar RASIncorrect;

  protected:
    /** Number of bits to shift instructions by for predictor addresses. */
    const unsigned instShiftAmt;

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
    ProbePoints::PMUUPtr pmuProbePoint(const char *name);


    /**
     * Branches seen by the branch predictor
     *
     * @note This counter includes speculative branches.
     */
    ProbePoints::PMUUPtr ppBranches;

    /** Miss-predicted branches */
    ProbePoints::PMUUPtr ppMisses;

    /** @} */
};

#endif // __CPU_PRED_BPRED_UNIT_HH__
