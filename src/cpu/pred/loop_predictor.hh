/*
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
 */

#ifndef __CPU_PRED_LOOP_PREDICTOR_HH__
#define __CPU_PRED_LOOP_PREDICTOR_HH__

#include "base/statistics.hh"
#include "base/types.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct LoopPredictorParams;

namespace branch_prediction
{

class LoopPredictor : public SimObject
{
  protected:
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

    LoopEntry *ltable;

    int8_t loopUseCounter;
    unsigned withLoopBits;

    const bool useDirectionBit;
    const bool useSpeculation;
    const bool useHashing;
    const bool restrictAllocation;
    const unsigned initialLoopIter;
    const unsigned initialLoopAge;
    const bool optionalAgeReset;

    struct LoopPredictorStats : public statistics::Group
    {
        LoopPredictorStats(statistics::Group *parent);
        statistics::Scalar used;
        statistics::Scalar correct;
        statistics::Scalar wrong;
    } stats;

    /**
     * Updates an unsigned counter based on up/down parameter
     * @param ctr Reference to counter to update.
     * @param up Boolean indicating if the counter is incremented/decremented
     * If true it is incremented, if false it is decremented
     * @param nbits Counter width.
     */
    static inline void unsignedCtrUpdate(uint8_t &ctr, bool up, unsigned nbits)
    {
        assert(nbits <= sizeof(uint8_t) << 3);
        if (up) {
            if (ctr < ((1 << nbits) - 1))
                ctr++;
        } else {
            if (ctr)
                ctr--;
        }
    }
    static inline void signedCtrUpdate(int8_t &ctr, bool up, unsigned nbits)
    {
        if (up) {
            if (ctr < ((1 << (nbits - 1)) - 1))
                ctr++;
        } else {
            if (ctr > -(1 << (nbits - 1)))
                ctr--;
        }
    }
  public:
    // Primary branch history entry
    struct BranchInfo
    {
        uint16_t loopTag;
        uint16_t currentIter;

        bool loopPred;
        bool loopPredValid;
        bool loopPredUsed;
        int  loopIndex;
        int  loopIndexB;  // only for useHashing
        int loopHit;
        bool predTaken;

        BranchInfo()
            : loopTag(0), currentIter(0),
              loopPred(false),
              loopPredValid(false), loopIndex(0), loopIndexB(0), loopHit(0),
              predTaken(false)
        {}
    };

    /**
     * Computes the index used to access the
     * loop predictor.
     * @param pc_in The unshifted branch PC.
     * @param instShiftAmt Shift the pc by as many bits
     */
    int lindex(Addr pc_in, unsigned instShiftAmt) const;

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
     * @param instShiftAmt Shift the pc by as many bits (if hashing is not
     * used)
     * @result the result of the prediction, if it could be predicted
     */
    bool getLoop(Addr pc, BranchInfo* bi, bool speculative,
                 unsigned instShiftAmt) const;

   /**
    * Updates the loop predictor.
    * @param pc The unshifted branch PC.
    * @param taken The actual branch outcome.
    * @param bi Pointer to information on the
    * prediction recorded at prediction time.
    * @param tage_pred tage prediction of the branch
    */
    void loopUpdate(Addr pc, bool Taken, BranchInfo* bi, bool tage_pred);

    /**
     * Speculatively updates the loop predictor
     * iteration count (only for useSpeculation).
     * @param taken The predicted branch outcome.
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     */
    void specLoopUpdate(bool taken, BranchInfo* bi);

    /**
     * Update LTAGE for conditional branches.
     * @param branch_pc The unshifted branch PC.
     * @param taken Actual branch outcome.
     * @param tage_pred Prediction from TAGE
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     * @param instShiftAmt Number of bits to shift instructions
     */
    void condBranchUpdate(ThreadID tid, Addr branch_pc, bool taken,
        bool tage_pred, BranchInfo* bi, unsigned instShiftAmt);

    /**
     * Get the loop prediction
     * @param tid The thread ID to select the global
     * histories to use.
     * @param branch_pc The unshifted branch PC.
     * @param cond_branch True if the branch is conditional.
     * @param bi Reference to wrapping pointer to allow storing
     * derived class prediction information in the base class.
     * @param prev_pred_taken Result of the TAGE prediction
     * @param instShiftAmt Shift the pc by as many bits
     * @param instShiftAmt Shift the pc by as many bits
     * @result the prediction, true if taken
     */
    bool loopPredict(
        ThreadID tid, Addr branch_pc, bool cond_branch,
        BranchInfo* bi, bool prev_pred_taken, unsigned instShiftAmt);

    /**
     * Update the stats
     * @param taken Actual branch outcome
     * @param bi Pointer to information on the prediction
     * recorded at prediction time.
     */
    void updateStats(bool taken, BranchInfo* bi);

    void squashLoop(BranchInfo * bi);

    void squash(ThreadID tid, BranchInfo *bi);

    virtual bool calcConf(int index) const;

    virtual bool optionalAgeInc() const;

    virtual BranchInfo *makeBranchInfo();

    /**
     * Gets the value of the loop use counter
     * @return the loop use counter value
     */
    int8_t getLoopUseCounter() const
    {
        return loopUseCounter;
    }

    /**
     * Initialize the loop predictor
     */
    void init() override;

    LoopPredictor(const LoopPredictorParams &p);

    size_t getSizeInBits() const;
};

} // namespace branch_prediction
} // namespace gem5

#endif//__CPU_PRED_LOOP_PREDICTOR_HH__
