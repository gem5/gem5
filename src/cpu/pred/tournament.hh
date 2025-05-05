/*
 * Copyright (c) 2011, 2014 ARM Limited
 * Copyright (c) 2022-2023 The University of Edinburgh
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#ifndef __CPU_PRED_TOURNAMENT_PRED_HH__
#define __CPU_PRED_TOURNAMENT_PRED_HH__

#include <vector>

#include "base/sat_counter.hh"
#include "base/types.hh"
#include "cpu/pred/bpred_unit.hh"
#include "params/TournamentBP.hh"

namespace gem5
{

namespace branch_prediction
{

/**
 * Implements a tournament branch predictor, hopefully identical to the one
 * used in the 21264.  It has a local predictor, which uses a local history
 * table to index into a table of counters, and a global predictor, which
 * uses a global history to index into a table of counters.  A choice
 * predictor chooses between the two.  Both the global history register
 * and the selected local history are speculatively updated.
 */
class TournamentBP : public BPredUnit
{
  public:
    /**
     * Default branch predictor constructor.
     */
    TournamentBP(const TournamentBPParams &params);

    // Base class methods.
    bool lookup(ThreadID tid, Addr pc, void* &bp_history) override;
    void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target, const StaticInstPtr &inst,
                         void * &bp_history) override;
    void update(ThreadID tid, Addr pc, bool taken, void * &bp_history,
                bool squashed, const StaticInstPtr & inst,
                Addr target) override;
    void squash(ThreadID tid, void * &bp_history) override;

  private:
    /**
     * Returns if the branch should be taken or not, given a counter
     * value.
     * @param count The counter value.
     */
    inline bool getPrediction(uint8_t &count);

    /**
     * Returns the local history index, given a branch address.
     * @param branch_addr The branch's PC address.
     */
    inline unsigned calcLocHistIdx(Addr &branch_addr);

    /** Updates global history with the given direction
     * @param taken Whether or not the branch was taken
    */
    inline void updateGlobalHist(ThreadID tid, bool taken);

    /**
     * Updates local histories.
     * @param local_history_idx The local history table entry that
     * will be updated.
     * @param taken Whether or not the branch was taken.
     */
    inline void updateLocalHist(unsigned local_history_idx, bool taken);

    /**
     * The branch history information that is created upon predicting
     * a branch.  It will be passed back upon updating and squashing,
     * when the BP can use this information to update/restore its
     * state properly.
     */
    struct BPHistory
    {
#ifdef GEM5_DEBUG
        BPHistory()
        { newCount++; }
        ~BPHistory()
        { newCount--; }

        static int newCount;
#endif
        unsigned globalHistory;
        unsigned localHistoryIdx;
        unsigned localHistory;
        bool localPredTaken;
        bool globalPredTaken;
        bool globalUsed;
    };

    /** Flag for invalid predictor index */
    static const int invalidPredictorIndex = -1;
    /** Number of counters in the local predictor. */
    unsigned localPredictorSize;

    /** Mask to truncate values stored in the local history table. */
    unsigned localPredictorMask;

    /** Number of bits of the local predictor's counters. */
    unsigned localCtrBits;

    /** Local counters. */
    std::vector<SatCounter8> localCtrs;

    /** Array of local history table entries. */
    std::vector<unsigned> localHistoryTable;

    /** Number of entries in the local history table. */
    unsigned localHistoryTableSize;

    /** Number of bits for each entry of the local history table. */
    unsigned localHistoryBits;

    /** Number of entries in the global predictor. */
    unsigned globalPredictorSize;

    /** Number of bits of the global predictor's counters. */
    unsigned globalCtrBits;

    /** Array of counters that make up the global predictor. */
    std::vector<SatCounter8> globalCtrs;

    /** Global history register. Contains as much history as specified by
     *  globalHistoryBits. Actual number of bits used is determined by
     *  globalHistoryMask and choiceHistoryMask. */
    std::vector<unsigned> globalHistory;

    /** Number of bits for the global history. Determines maximum number of
        entries in global and choice predictor tables. */
    unsigned globalHistoryBits;

    /** Mask to apply to globalHistory to access global history table.
     *  Based on globalPredictorSize.*/
    unsigned globalHistoryMask;

    /** Mask to apply to globalHistory to access choice history table.
     *  Based on choicePredictorSize.*/
    unsigned choiceHistoryMask;

    /** Mask to control how much history is stored. All of it might not be
     *  used. */
    unsigned historyRegisterMask;

    /** Number of entries in the choice predictor. */
    unsigned choicePredictorSize;

    /** Number of bits in the choice predictor's counters. */
    unsigned choiceCtrBits;

    /** Array of counters that make up the choice predictor. */
    std::vector<SatCounter8> choiceCtrs;

    /** Thresholds for the counter value; above the threshold is taken,
     *  equal to or below the threshold is not taken.
     */
    unsigned localThreshold;
    unsigned globalThreshold;
    unsigned choiceThreshold;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_TOURNAMENT_PRED_HH__
