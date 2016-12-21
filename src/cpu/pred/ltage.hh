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
#include "cpu/pred/bpred_unit.hh"
#include "params/LTAGE.hh"

class LTAGE: public BPredUnit
{
  public:
    LTAGE(const LTAGEParams *params);

    // Base class methods.
    void uncondBranch(ThreadID tid, Addr br_pc, void* &bp_history) override;
    bool lookup(ThreadID tid, Addr branch_addr, void* &bp_history) override;
    void btbUpdate(ThreadID tid, Addr branch_addr, void* &bp_history) override;
    void update(ThreadID tid, Addr branch_addr, bool taken, void *bp_history,
                bool squashed) override;
    void squash(ThreadID tid, void *bp_history) override;
    unsigned getGHR(ThreadID tid, void *bp_history) const override;

  private:
    // Prediction Structures
    // Loop Predictor Entry
    struct LoopEntry
    {
        uint16_t numIter;
        uint16_t currentIter;
        uint16_t currentIterSpec;
        uint8_t confidence;
        uint16_t tag;
        uint8_t age;
        bool dir;

        LoopEntry() : numIter(0), currentIter(0), currentIterSpec(0),
                      confidence(0), tag(0), age(0), dir(0) { }
    };

    // Bimodal Predictor Entry
    struct BimodalEntry
    {
        uint8_t pred;
        uint8_t hyst;

        BimodalEntry() : pred(0), hyst(1) { }
    };

    // Tage Entry
    struct TageEntry
    {
        int8_t ctr;
        uint16_t tag;
        int8_t u;
        TageEntry() : ctr(0), tag(0), u(0) { }
    };

    // Folded History Table - compressed history
    // to mix with instruction PC to index partially
    // tagged tables.
    struct FoldedHistory
    {
        unsigned comp;
        int compLength;
        int origLength;
        int outpoint;

        void init(int original_length, int compressed_length)
        {
            comp = 0;
            origLength = original_length;
            compLength = compressed_length;
            outpoint = original_length % compressed_length;
        }

        void update(uint8_t * h)
        {
            comp = (comp << 1) | h[0];
            comp ^= h[origLength] << outpoint;
            comp ^= (comp >> compLength);
            comp &= (ULL(1) << compLength) - 1;
        }
    };

    // Primary branch history entry
    struct BranchInfo
    {
        int pathHist;
        int ptGhist;
        int hitBank;
        int hitBankIndex;
        int altBank;
        int altBankIndex;
        int bimodalIndex;
        int loopTag;
        uint16_t currentIter;

        bool tagePred;
        bool altTaken;
        bool loopPred;
        bool loopPredValid;
        int  loopIndex;
        int loopHit;
        bool condBranch;
        bool longestMatchPred;
        bool pseudoNewAlloc;
        Addr branchPC;

        // Pointer to dynamically allocated storage
        // to save table indices and folded histories.
        // To do one call to new instead of five.
        int *storage;

        // Pointers to actual saved array within the dynamically
        // allocated storage.
        int *tableIndices;
        int *tableTags;
        int *ci;
        int *ct0;
        int *ct1;

        BranchInfo(int sz)
            : pathHist(0), ptGhist(0),
              hitBank(0), hitBankIndex(0),
              altBank(0), altBankIndex(0),
              bimodalIndex(0), loopTag(0), currentIter(0),
              tagePred(false), altTaken(false), loopPred(false),
              loopPredValid(false), loopIndex(0), loopHit(0),
              condBranch(false), longestMatchPred(false),
              pseudoNewAlloc(false), branchPC(0)
        {
            storage = new int [sz * 5];
            tableIndices = storage;
            tableTags = storage + sz;
            ci = tableTags + sz;
            ct0 = ci + sz;
            ct1 = ct0 + sz;
        }

        ~BranchInfo()
        {
            delete[] storage;
        }
    };

    /**
     * Computes the index used to access the
     * bimodal table.
     * @param pc_in The unshifted branch PC.
     */
    int bindex(Addr pc_in) const;

    /**
     * Computes the index used to access the
     * loop predictor.
     * @param pc_in The unshifted branch PC.
     */
    int lindex(Addr pc_in) const;

    /**
     * Computes the index used to access a
     * partially tagged table.
     * @param tid The thread ID used to select the
     * global histories to use.
     * @param pc The unshifted branch PC.
     * @param bank The partially tagged table to access.
     */
    inline int gindex(ThreadID tid, Addr pc, int bank) const;

    /**
     * Utility function to shuffle the path history
     * depending on which tagged table we are accessing.
     * @param phist The path history.
     * @param size Number of path history bits to use.
     * @param bank The partially tagged table to access.
     */
    int F(int phist, int size, int bank) const;

    /**
     * Computes the partial tag of a tagged table.
     * @param tid the thread ID used to select the
     * global histories to use.
     * @param pc The unshifted branch PC.
     * @param bank The partially tagged table to access.
     */
    inline uint16_t gtag(ThreadID tid, Addr pc, int bank) const;

    /**
     * Updates a direction counter based on the actual
     * branch outcome.
     * @param ctr Reference to counter to update.
     * @param taken Actual branch outcome.
     * @param nbits Counter width.
     */
    void ctrUpdate(int8_t & ctr, bool taken, int nbits);

    /**
     * Get a branch prediction from the bimodal
     * predictor.
     * @param pc The unshifted branch PC.
     * @param bi Pointer to information on the
     * prediction.
     */
    bool getBimodePred(Addr pc, BranchInfo* bi) const;

    /**
     * Updates the bimodal predictor.
     * @param pc The unshifted branch PC.
     * @param taken The actual branch outcome.
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     */
    void baseUpdate(Addr pc, bool taken, BranchInfo* bi);

    /**
     * Get a branch prediction from the loop
     * predictor.
     * @param pc The unshifted branch PC.
     * @param bi Pointer to information on the
     * prediction.
     */
    bool getLoop(Addr pc, BranchInfo* bi) const;

   /**
    * Updates the loop predictor.
    * @param pc The unshifted branch PC.
    * @param taken The actual branch outcome.
    * @param bi Pointer to information on the
    * prediction recorded at prediction time.
    */
    void loopUpdate(Addr pc, bool Taken, BranchInfo* bi);

   /**
    * (Speculatively) updates the global branch history.
    * @param h Reference to pointer to global branch history.
    * @param dir (Predicted) outcome to update the histories
    * with.
    * @param tab
    * @param PT Reference to path history.
    */
    void updateGHist(uint8_t * &h, bool dir, uint8_t * tab, int &PT);

    /**
     * Get a branch prediction from L-TAGE. *NOT* an override of
     * BpredUnit::predict().
     * @param tid The thread ID to select the global
     * histories to use.
     * @param branch_pc The unshifted branch PC.
     * @param cond_branch True if the branch is conditional.
     * @param b Reference to wrapping pointer to allow storing
     * derived class prediction information in the base class.
     */
    bool predict(ThreadID tid, Addr branch_pc, bool cond_branch, void* &b);

    /**
     * Update L-TAGE. Called at execute to repair histories on a misprediction
     * and at commit to update the tables.
     * @param tid The thread ID to select the global
     * histories to use.
     * @param branch_pc The unshifted branch PC.
     * @param taken Actual branch outcome.
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     */
    void update(ThreadID tid, Addr branch_pc, bool taken, BranchInfo* bi);

   /**
    * (Speculatively) updates global histories (path and direction).
    * Also recomputes compressed (folded) histories based on the
    * branch direction.
    * @param tid The thread ID to select the histories
    * to update.
    * @param branch_pc The unshifted branch PC.
    * @param taken (Predicted) branch direction.
    * @param b Wrapping pointer to BranchInfo (to allow
    * storing derived class prediction information in the
    * base class).
    */
    void updateHistories(ThreadID tid, Addr branch_pc, bool taken, void* b);

    /**
     * Restores speculatively updated path and direction histories.
     * Also recomputes compressed (folded) histories based on the
     * correct branch outcome.
     * This version of squash() is called once on a branch misprediction.
     * @param tid The Thread ID to select the histories to rollback.
     * @param taken The correct branch outcome.
     * @param bp_history Wrapping pointer to BranchInfo (to allow
     * storing derived class prediction information in the
     * base class).
     * @post bp_history points to valid memory.
     */
    void squash(ThreadID tid, bool taken, void *bp_history);

    /**
     * Speculatively updates the loop predictor
     * iteration count.
     * @param pc The unshifted branch PC.
     * @param taken The predicted branch outcome.
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     */
    void specLoopUpdate(Addr pc, bool taken, BranchInfo* bi);

    const unsigned logSizeBiMP;
    const unsigned logSizeTagTables;
    const unsigned logSizeLoopPred;
    const unsigned nHistoryTables;
    const unsigned tagTableCounterBits;
    const unsigned histBufferSize;
    const unsigned minHist;
    const unsigned maxHist;
    const unsigned minTagWidth;

    BimodalEntry *btable;
    TageEntry **gtable;
    LoopEntry *ltable;

    // Keep per-thread histories to
    // support SMT.
    struct ThreadHistory {
        // Speculative path history
        // (LSB of branch address)
        int pathHist;

        // Speculative branch direction
        // history (circular buffer)
        // @TODO Convert to std::vector<bool>
        uint8_t *globalHistory;

        // Pointer to most recent branch outcome
        uint8_t* gHist;

        // Index to most recent branch outcome
        int ptGhist;

        // Speculative folded histories.
        FoldedHistory *computeIndices;
        FoldedHistory *computeTags[2];
    };

    std::vector<ThreadHistory> threadHistory;

    int tagWidths[15];
    int tagTableSizes[15];
    int *histLengths;
    int *tableIndices;
    int *tableTags;

    int8_t loopUseCounter;
    int8_t useAltPredForNewlyAllocated;
    int tCounter;
    int logTick;
};

#endif // __CPU_PRED_LTAGE
