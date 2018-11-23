/*
 * Copyright (c) 2014 The University of Wisconsin
 *
 * Copyright (c) 2006 INRIA (Institut National de Recherche en
 * Informatique et en Automatique  / French National Research Institute
 * for Computer Science and Applied Mathematics)
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
 *
 * Authors: Vignyan Reddy, Dibakar Gope and Arthur Perais,
 * from André Seznec's code.
 */

/* @file
 * Implementation of a L-TAGE branch predictor. TAGE is a global-history based
 * branch predictor. It features a PC-indexed bimodal predictor and N
 * partially tagged tables, indexed with a hash of the PC and the global
 * branch history. The different lengths of global branch history used to
 * index the partially tagged tables grow geometrically. A small path history
 * is also used in the hash. L-TAGE also features a loop predictor that records
 * iteration count of loops and predicts accordingly.
 *
 * All TAGE tables are accessed in parallel, and the one using the longest
 * history that matches provides the prediction (some exceptions apply).
 * Entries are allocated in components using a longer history than the
 * one that predicted when the prediction is incorrect.
 */

#ifndef __CPU_PRED_LTAGE
#define __CPU_PRED_LTAGE


#include <vector>

#include "base/types.hh"
#include "cpu/pred/tage.hh"
#include "params/LTAGE.hh"

class LTAGE: public TAGE
{
  public:
    LTAGE(const LTAGEParams *params);

    // Base class methods.
    void squash(ThreadID tid, void *bp_history) override;

    void regStats() override;

  private:
    // Prediction Structures
    // Loop Predictor Entry
    struct LoopEntry
    {
        uint16_t numIter;
        uint16_t currentIter;
        uint16_t currentIterSpec; // only for useSpeculation
        uint8_t confidence;
        uint16_t tag;
        uint8_t age;
        bool dir; // only for useDirectionBit

        LoopEntry() : numIter(0), currentIter(0), currentIterSpec(0),
                      confidence(0), tag(0), age(0), dir(0) { }
    };

    // more provider types
    enum {
        LOOP = LAST_TAGE_PROVIDER_TYPE + 1
    };

    // Primary branch history entry
    struct LTageBranchInfo : public TageBranchInfo
    {
        uint16_t loopTag;
        uint16_t currentIter;

        bool loopPred;
        bool loopPredValid;
        int  loopIndex;
        int  loopLowPcBits;  // only for useHashing
        int loopHit;

        LTageBranchInfo(int sz)
            : TageBranchInfo(sz),
              loopTag(0), currentIter(0),
              loopPred(false),
              loopPredValid(false), loopIndex(0), loopLowPcBits(0), loopHit(0)
        {}
    };

    /**
     * Computes the index used to access the
     * loop predictor.
     * @param pc_in The unshifted branch PC.
     */
    int lindex(Addr pc_in) const;

    /**
     * Computes the index used to access the
     * ltable structures.
     * It may take hashing into account
     * @param index Result of lindex function
     * @param lowPcBits PC bits masked with set size
     * @param way Way to be used
     */
    int finallindex(int lindex, int lowPcBits, int way) const;

    /**
     * Get a branch prediction from the loop
     * predictor.
     * @param pc The unshifted branch PC.
     * @param bi Pointer to information on the
     * prediction.
     * @param speculative Use speculative number of iterations
     */
    bool getLoop(Addr pc, LTageBranchInfo* bi, bool speculative) const;

   /**
    * Updates the loop predictor.
    * @param pc The unshifted branch PC.
    * @param taken The actual branch outcome.
    * @param bi Pointer to information on the
    * prediction recorded at prediction time.
    */
    void loopUpdate(Addr pc, bool Taken, LTageBranchInfo* bi);

    /**
     * Speculatively updates the loop predictor
     * iteration count (only for useSpeculation).
     * @param taken The predicted branch outcome.
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     */
    void specLoopUpdate(bool taken, LTageBranchInfo* bi);

    /**
     * Update LTAGE for conditional branches.
     * @param branch_pc The unshifted branch PC.
     * @param taken Actual branch outcome.
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     * @nrand Random int number from 0 to 3
     */
    void condBranchUpdate(
        Addr branch_pc, bool taken, TageBranchInfo* bi, int nrand) override;

    /**
     * Get a branch prediction from LTAGE. *NOT* an override of
     * BpredUnit::predict().
     * @param tid The thread ID to select the global
     * histories to use.
     * @param branch_pc The unshifted branch PC.
     * @param cond_branch True if the branch is conditional.
     * @param b Reference to wrapping pointer to allow storing
     * derived class prediction information in the base class.
     */
    bool predict(
        ThreadID tid, Addr branch_pc, bool cond_branch, void* &b) override;

    /**
     * Restores speculatively updated path and direction histories.
     * Also recomputes compressed (folded) histories based on the
     * correct branch outcome.
     * This version of squash() is called once on a branch misprediction.
     * @param tid The Thread ID to select the histories to rollback.
     * @param taken The correct branch outcome.
     * @param bp_history Wrapping pointer to TageBranchInfo (to allow
     * storing derived class prediction information in the
     * base class).
     * @post bp_history points to valid memory.
     */
    void squash(
        ThreadID tid, bool taken, void *bp_history) override;

    /**
     * Update the stats
     * @param taken Actual branch outcome
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     */
    void updateStats(bool taken, TageBranchInfo* bi) override;

    const unsigned logSizeLoopPred;
    const unsigned loopTableAgeBits;
    const unsigned loopTableConfidenceBits;
    const unsigned loopTableTagBits;
    const unsigned loopTableIterBits;
    const unsigned logLoopTableAssoc;
    const uint8_t confidenceThreshold;
    const uint16_t loopTagMask;
    const uint16_t loopNumIterMask;
    const int loopSetMask;

    LoopEntry *ltable;

    int8_t loopUseCounter;
    unsigned withLoopBits;

    const bool useDirectionBit;
    const bool useSpeculation;
    const bool useHashing;

    // stats
    Stats::Scalar loopPredictorCorrect;
    Stats::Scalar loopPredictorWrong;
};

#endif // __CPU_PRED_LTAGE
