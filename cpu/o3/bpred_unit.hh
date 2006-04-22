/*
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

#ifndef __CPU_O3_BPRED_UNIT_HH__
#define __CPU_O3_BPRED_UNIT_HH__

// For Addr type.
#include "arch/isa_traits.hh"
#include "base/statistics.hh"
#include "cpu/inst_seq.hh"

#include "cpu/o3/2bit_local_pred.hh"
#include "cpu/o3/btb.hh"
#include "cpu/o3/ras.hh"
#include "cpu/o3/tournament_pred.hh"

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

    /**
     * @param params The params object, that has the size of the BP and BTB.
     */
    TwobitBPredUnit(Params *params);

    /**
     * Registers statistics.
     */
    void regStats();

    /**
     * Predicts whether or not the instruction is a taken branch, and the
     * target of the branch if it is taken.
     * @param inst The branch instruction.
     * @param PC The predicted PC is passed back through this parameter.
     * @param tid The thread id.
     * @return Returns if the branch is taken or not.
     */
    bool predict(DynInstPtr &inst, Addr &PC, unsigned tid);

    /**
     * Tells the branch predictor to commit any updates until the given
     * sequence number.
     * @param done_sn The sequence number to commit any older updates up until.
     * @param tid The thread id.
     */
    void update(const InstSeqNum &done_sn, unsigned tid);

    /**
     * Squashes all outstanding updates until a given sequence number.
     * @param squashed_sn The sequence number to squash any younger updates up
     * until.
     * @param tid The thread id.
     */
    void squash(const InstSeqNum &squashed_sn, unsigned tid);

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
                bool actually_taken, unsigned tid);

    /**
     * Looks up a given PC in the BP to see if it is taken or not taken.
     * @param inst_PC The PC to look up.
     * @return Whether the branch is taken or not taken.
     */
    bool BPLookup(Addr &inst_PC)
    { return BP.lookup(inst_PC); }

    /**
     * Looks up a given PC in the BTB to see if a matching entry exists.
     * @param inst_PC The PC to look up.
     * @return Whether the BTB contains the given PC.
     */
    bool BTBValid(Addr &inst_PC)
    { return BTB.valid(inst_PC, 0); }

    /**
     * Looks up a given PC in the BTB to get the predicted target.
     * @param inst_PC The PC to look up.
     * @return The address of the target of the branch.
     */
    Addr BTBLookup(Addr &inst_PC)
    { return BTB.lookup(inst_PC, 0); }

    /**
     * Updates the BP with taken/not taken information.
     * @param inst_PC The branch's PC that will be updated.
     * @param taken Whether the branch was taken or not taken.
     * @todo Make this update flexible enough to handle a global predictor.
     */
    void BPUpdate(Addr &inst_PC, bool taken)
    { BP.update(inst_PC, taken); }

    /**
     * Updates the BTB with the target of a branch.
     * @param inst_PC The branch's PC that will be updated.
     * @param target_PC The branch's target that will be added to the BTB.
     */
    void BTBUpdate(Addr &inst_PC, Addr &target_PC)
    { BTB.update(inst_PC, target_PC,0); }

  private:
    struct PredictorHistory {
        /**
         * Makes a predictor history struct that contains a sequence number,
         * the PC of its instruction, and whether or not it was predicted
         * taken.
         */
        PredictorHistory(const InstSeqNum &seq_num, const Addr &inst_PC,
                         const bool pred_taken, const unsigned _tid)
            : seqNum(seq_num), PC(inst_PC), RASTarget(0), globalHistory(0),
              RASIndex(0), tid(_tid), predTaken(pred_taken), usedRAS(0),
              wasCall(0)
        { }

        /** The sequence number for the predictor history entry. */
        InstSeqNum seqNum;

        /** The PC associated with the sequence number. */
        Addr PC;

        /** The RAS target (only valid if a return). */
        Addr RASTarget;

        /** The global history at the time this entry was created. */
        unsigned globalHistory;

        /** The RAS index of the instruction (only valid if a call). */
        unsigned RASIndex;

        /** The thread id. */
        unsigned tid;

        /** Whether or not it was predicted taken. */
        bool predTaken;

        /** Whether or not the RAS was used. */
        bool usedRAS;

        /** Whether or not the instruction was a call. */
        bool wasCall;
    };

    typedef std::list<PredictorHistory> History;

    /**
     * The per-thread predictor history. This is used to update the predictor
     * as instructions are committed, or restore it to the proper state after
     * a squash.
     */
    History predHist[Impl::MaxThreads];

    /** The branch predictor. */
    DefaultBP BP;

    /** The BTB. */
    DefaultBTB BTB;

    /** The per-thread return address stack. */
    ReturnAddrStack RAS[Impl::MaxThreads];

    /** Stat for number of BP lookups. */
    Stats::Scalar<> lookups;
    /** Stat for number of conditional branches predicted. */
    Stats::Scalar<> condPredicted;
    /** Stat for number of conditional branches predicted incorrectly. */
    Stats::Scalar<> condIncorrect;
    /** Stat for number of BTB lookups. */
    Stats::Scalar<> BTBLookups;
    /** Stat for number of BTB hits. */
    Stats::Scalar<> BTBHits;
    /** Stat for number of times the BTB is correct. */
    Stats::Scalar<> BTBCorrect;
    /** Stat for number of times the RAS is used to get a target. */
    Stats::Scalar<> usedRAS;
    /** Stat for number of times the RAS is incorrect. */
    Stats::Scalar<> RASIncorrect;
};

#endif // __CPU_O3_BPRED_UNIT_HH__
