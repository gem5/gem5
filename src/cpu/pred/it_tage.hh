/*
 * Copyright (c) 2023 University of Edinburgh
 *
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

/* @file
 * Implementation of the ITTAGE indirect branch predictor
 * Reference: https://jilp.org/jwac-2/program/cbp3_07_seznec.pdf
 * Author: André Seznec
 */

#ifndef __CPU_PRED_ITTAGE_HH__
#define __CPU_PRED_ITTAGE_HH__

#include <tuple>
#include <vector>

#include "base/random.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/pred/indirect.hh"
#include "cpu/pred/tage_base.hh"
#include "enums/BranchType.hh"
#include "params/ITTAGE.hh"
#include "params/ITTAGE_TAGE.hh"

namespace gem5
{

namespace branch_prediction
{

class ITTAGE;

/** Base TAGE with target array */
class ITTAGE_TAGE : public TAGEBase
{
    typedef enums::BranchType BranchType;
    friend ITTAGE;

  public:
    ITTAGE_TAGE(const ITTAGE_TAGEParams &p);

  private:
    struct TgtEntry
    {
        std::unique_ptr<PCStateBase> target;
        TgtEntry() : target(nullptr) {}
    };

    /** Extended TAGE gtable by the target field */
    TgtEntry **tgtTable;

    // more provider types
    enum
    {
        BTB = TAGEBase::LAST_TAGE_PROVIDER_TYPE + 1,
        LAST_LTAGE_PROVIDER_TYPE = BTB
    };

    struct ITTAGEBranchInfo : public TAGEBase::BranchInfo
    {
        ITTAGEBranchInfo(TAGEBase &tage, Addr pc, bool conditional)
            : TAGEBase::BranchInfo(tage, pc, conditional),
              predTarget(nullptr),
              corrTarget(nullptr),
              longestMatchTarget(MaxAddr),
              altTarget(MaxAddr),
              indirect(false),
              type(BranchType::NoBranch),
              taken(false),
              squash(false)
        {}

        ~ITTAGEBranchInfo() {}

        /** Predicted and correct target of a branch */
        std::unique_ptr<PCStateBase> predTarget;
        std::unique_ptr<PCStateBase> corrTarget;

        /** Predicted targets for alt and provider components */
        Addr longestMatchTarget;
        Addr altTarget;

        /** A prediction for this branch was made */
        bool indirect;
        BranchType type;
        bool taken;
        bool squash;
    };

    ITTAGEBranchInfo *makeITTBranchInfo(Addr pc, BranchType type);

    void ittTagePredict(ThreadID tid, Addr branch_pc, ITTAGEBranchInfo *bi);

    /**
     * Internal history update function. This function shifts
     * nBits into the global history vector. If the update
     * is speculative the functions makes a copy of the
     * GHR to rollback.
     * @param tid The thread ID to select the histories to update.
     * @param speculative Whether the update is speculative or not
     * @param hist The bit vector with n bits that will be shifted
     * into the global history vector.
     * @param nBits The number of bits to be updated
     * @param bi Wrapping pointer to BranchInfo (to allow
     * storing derived class prediction information in the
     * base class).
     */
    void ittUpdateHistories(ThreadID tid, bool speculative, uint64_t hist,
                            uint8_t nBits, ITTAGEBranchInfo *bi);

    void updateIndirect(ThreadID tid, ITTAGEBranchInfo *bi, int nrand,
                        const PCStateBase &target);

    bool allocateEntry(int idx, TAGEBase::BranchInfo *bi, bool taken) override;

    /**
     * Handles the update of the TAGE entries
     */
    void ittHandleTAGEUpdate(Addr branch_pc, ITTAGEBranchInfo *bi);

    void ittUpdateStats(const PCStateBase &target, ITTAGEBranchInfo *bi);

    struct ITTAGE_TAGEStats : public statistics::Group
    {
        ITTAGE_TAGEStats(statistics::Group *parent);
        statistics::Scalar btbProviderCorrect;
        statistics::Scalar btbProviderWrong;
        statistics::Scalar targetReplacements;
    } istats;
};

class ITTAGE : public IndirectPredictor
{
    typedef enums::BranchType BranchType;

  protected:
    ITTAGE_TAGE *itage;

    Random::RandomPtr rng = Random::genRandom();

  public:
    ITTAGE(const ITTAGEParams &params);
    ~ITTAGE() = default;

    /** Interface function to override */
    void reset() override;

    const PCStateBase *lookup(ThreadID tid, InstSeqNum sn, Addr pc,
                              void *&indirect_history) override;
    void update(ThreadID tid, InstSeqNum sn, Addr pc, bool squash, bool taken,
                const PCStateBase &target, BranchType brType,
                void *&indirect_history) override;
    void squash(ThreadID tid, InstSeqNum seq_num,
                void *&indirect_history) override;
    void commit(ThreadID tid, InstSeqNum seq_num,
                void *&indirect_history) override;

  private:
    /** Number of bits in the history for different branch types */
    const unsigned histBitsConditional;
    const unsigned histBitsIndBranch;
    const unsigned histBitsIndCall;
    const unsigned histBitsCall;

    inline bool
    isIndirectNoReturn(BranchType type)
    {
        return (type == BranchType::CallIndirect) ||
               (type == BranchType::IndirectUncond);
    }

    /**
     * Calculates the history update based on the instruction type
     * @param inst The branch instruction.
     * @param pc Branch pc.
     * @param taken branch direction.
     * @param target branch target.
     * @return A tuple: 1. A bit vector to update the history with
     *                  2. The number of bits in the bit vector
     */
    std::tuple<uint64_t, uint8_t> calculateHistUpdate(BranchType type, Addr pc,
                                                      bool taken, Addr target);
    void genIndirectInfo(ThreadID tid, InstSeqNum sn, Addr pc,
                         BranchType brType, void *&iHistory);

  protected:
    struct ITTAGEStats : public statistics::Group
    {
        ITTAGEStats(statistics::Group *parent);

        statistics::Scalar lookups;
        statistics::Scalar hits;
        statistics::Scalar misses;
        statistics::Scalar targetRecords;
        statistics::Scalar indirectRecords;
        statistics::Scalar mispredictIndCall;
        statistics::Scalar mispredictIndJump;
        statistics::Scalar mispredictTotal;
        statistics::Scalar updates;
        statistics::Scalar updateSquashed;
        statistics::Scalar updateIndSquashed;
    } stats;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_ITTAGE_HH__
