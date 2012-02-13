/*
 * Copyright (c) 2011 ARM Limited
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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_TOURNAMENT_PRED_HH__
#define __CPU_O3_TOURNAMENT_PRED_HH__

#include <vector>

#include "base/types.hh"
#include "cpu/o3/sat_counter.hh"

/**
 * Implements a tournament branch predictor, hopefully identical to the one
 * used in the 21264.  It has a local predictor, which uses a local history
 * table to index into a table of counters, and a global predictor, which
 * uses a global history to index into a table of counters.  A choice
 * predictor chooses between the two.  Only the global history register
 * is speculatively updated, the rest are updated upon branches committing
 * or misspeculating.
 */
class TournamentBP
{
  public:
    /**
     * Default branch predictor constructor.
     */
    TournamentBP(unsigned localPredictorSize,
                 unsigned localCtrBits,
                 unsigned localHistoryTableSize,
                 unsigned localHistoryBits,
                 unsigned globalPredictorSize,
                 unsigned globalHistoryBits,
                 unsigned globalCtrBits,
                 unsigned choicePredictorSize,
                 unsigned choiceCtrBits,
                 unsigned instShiftAmt);

    /**
     * Looks up the given address in the branch predictor and returns
     * a true/false value as to whether it is taken.  Also creates a
     * BPHistory object to store any state it will need on squash/update.
     * @param branch_addr The address of the branch to look up.
     * @param bp_history Pointer that will be set to the BPHistory object.
     * @return Whether or not the branch is taken.
     */
    bool lookup(Addr &branch_addr, void * &bp_history);

    /**
     * Records that there was an unconditional branch, and modifies
     * the bp history to point to an object that has the previous
     * global history stored in it.
     * @param bp_history Pointer that will be set to the BPHistory object.
     */
    void uncondBr(void * &bp_history);
    /**
     * Updates the branch predictor to Not Taken if a BTB entry is
     * invalid or not found.
     * @param branch_addr The address of the branch to look up.
     * @param bp_history Pointer to any bp history state.
     * @return Whether or not the branch is taken.
     */
    void BTBUpdate(Addr &branch_addr, void * &bp_history);
    /**
     * Updates the branch predictor with the actual result of a branch.
     * @param branch_addr The address of the branch to update.
     * @param taken Whether or not the branch was taken.
     * @param bp_history Pointer to the BPHistory object that was created
     * when the branch was predicted.
     * @param squashed is set when this function is called during a squash
     * operation.
     */
    void update(Addr &branch_addr, bool taken, void *bp_history, bool squashed);

    /**
     * Restores the global branch history on a squash.
     * @param bp_history Pointer to the BPHistory object that has the
     * previous global branch history in it.
     */
    void squash(void *bp_history);

    /** Returns the global history. */
    inline unsigned readGlobalHist() { return globalHistory; }

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

    /** Updates global history as taken. */
    inline void updateGlobalHistTaken();

    /** Updates global history as not taken. */
    inline void updateGlobalHistNotTaken();

    /**
     * Updates local histories as taken.
     * @param local_history_idx The local history table entry that
     * will be updated.
     */
    inline void updateLocalHistTaken(unsigned local_history_idx);

    /**
     * Updates local histories as not taken.
     * @param local_history_idx The local history table entry that
     * will be updated.
     */
    inline void updateLocalHistNotTaken(unsigned local_history_idx);

    /**
     * The branch history information that is created upon predicting
     * a branch.  It will be passed back upon updating and squashing,
     * when the BP can use this information to update/restore its
     * state properly.
     */
    struct BPHistory {
#ifdef DEBUG
        BPHistory()
        { newCount++; }
        ~BPHistory()
        { newCount--; }

        static int newCount;
#endif
        unsigned globalHistory;
        unsigned localHistory;
        bool localPredTaken;
        bool globalPredTaken;
        bool globalUsed;
    };

    /** Flag for invalid predictor index */
    static const int invalidPredictorIndex = -1;
    /** Local counters. */
    std::vector<SatCounter> localCtrs;

    /** Size of the local predictor. */
    unsigned localPredictorSize;

    /** Mask to get the proper index bits into the predictor. */
    unsigned localPredictorMask;

    /** Number of bits of the local predictor's counters. */
    unsigned localCtrBits;

    /** Array of local history table entries. */
    std::vector<unsigned> localHistoryTable;

    /** Size of the local history table. */
    unsigned localHistoryTableSize;

    /** Number of bits for each entry of the local history table.
     *  @todo Doesn't this come from the size of the local predictor?
     */
    unsigned localHistoryBits;

    /** Mask to get the proper local history. */
    unsigned localHistoryMask;

    /** Array of counters that make up the global predictor. */
    std::vector<SatCounter> globalCtrs;

    /** Size of the global predictor. */
    unsigned globalPredictorSize;

    /** Number of bits of the global predictor's counters. */
    unsigned globalCtrBits;

    /** Global history register. */
    unsigned globalHistory;

    /** Number of bits for the global history. */
    unsigned globalHistoryBits;

    /** Mask to get the proper global history. */
    unsigned globalHistoryMask;

    /** Array of counters that make up the choice predictor. */
    std::vector<SatCounter> choiceCtrs;

    /** Size of the choice predictor (identical to the global predictor). */
    unsigned choicePredictorSize;

    /** Number of bits of the choice predictor's counters. */
    unsigned choiceCtrBits;

    /** Number of bits to shift the instruction over to get rid of the word
     *  offset.
     */
    unsigned instShiftAmt;

    /** Threshold for the counter value; above the threshold is taken,
     *  equal to or below the threshold is not taken.
     */
    unsigned threshold;
};

#endif // __CPU_O3_TOURNAMENT_PRED_HH__
