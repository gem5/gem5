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
 */

#include "cpu/pred/loop_predictor.hh"

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/LTage.hh"
#include "params/LoopPredictor.hh"

namespace gem5
{

namespace branch_prediction
{

LoopPredictor::LoopPredictor(const LoopPredictorParams &p)
  : SimObject(p), logSizeLoopPred(p.logSizeLoopPred),
    loopTableAgeBits(p.loopTableAgeBits),
    loopTableConfidenceBits(p.loopTableConfidenceBits),
    loopTableTagBits(p.loopTableTagBits),
    loopTableIterBits(p.loopTableIterBits),
    logLoopTableAssoc(p.logLoopTableAssoc),
    confidenceThreshold((1 << loopTableConfidenceBits) - 1),
    loopTagMask((1 << loopTableTagBits) - 1),
    loopNumIterMask((1 << loopTableIterBits) - 1),
    loopSetMask((1 << (logSizeLoopPred - logLoopTableAssoc)) - 1),
    loopUseCounter(-1),
    withLoopBits(p.withLoopBits),
    useDirectionBit(p.useDirectionBit),
    useSpeculation(p.useSpeculation),
    useHashing(p.useHashing),
    restrictAllocation(p.restrictAllocation),
    initialLoopIter(p.initialLoopIter),
    initialLoopAge(p.initialLoopAge),
    optionalAgeReset(p.optionalAgeReset),
    stats(this)
{
    assert(initialLoopAge <= ((1 << loopTableAgeBits) - 1));
}

void
LoopPredictor::init()
{
    // we use uint16_t type for these vales, so they cannot be more than
    // 16 bits
    assert(loopTableTagBits <= 16);
    assert(loopTableIterBits <= 16);

    assert(logSizeLoopPred >= logLoopTableAssoc);

    ltable = new LoopEntry[1ULL << logSizeLoopPred];
}

LoopPredictor::BranchInfo*
LoopPredictor::makeBranchInfo()
{
    return new BranchInfo();
}

int
LoopPredictor::lindex(Addr pc_in, unsigned instShiftAmt) const
{
    // The loop table is implemented as a linear table
    // If associativity is N (N being 1 << logLoopTableAssoc),
    // the first N entries are for set 0, the next N entries are for set 1,
    // and so on.
    // Thus, this function calculates the set and then it gets left shifted
    // by logLoopTableAssoc in order to return the index of the first of the
    // N entries of the set
    Addr pc = pc_in >> instShiftAmt;
    if (useHashing) {
        pc ^= pc_in;
    }
    return ((pc & loopSetMask) << logLoopTableAssoc);
}

int
LoopPredictor::finallindex(int index, int lowPcBits, int way) const
{
    return (useHashing ? (index ^ ((lowPcBits >> way) << logLoopTableAssoc)) :
                         (index))
           + way;
}

//loop prediction: only used if high confidence
bool
LoopPredictor::getLoop(Addr pc, BranchInfo* bi, bool speculative,
                       unsigned instShiftAmt) const
{
    bi->loopHit = -1;
    bi->loopPredValid = false;
    bi->loopIndex = lindex(pc, instShiftAmt);

    if (useHashing) {
        unsigned pcShift = logSizeLoopPred - logLoopTableAssoc;
        bi->loopIndexB = (pc >> pcShift) & loopSetMask;
        bi->loopTag = (pc >> pcShift) ^ (pc >> (pcShift + loopTableTagBits));
        bi->loopTag &= loopTagMask;
    } else {
        unsigned pcShift = instShiftAmt + logSizeLoopPred - logLoopTableAssoc;
        bi->loopTag = (pc >> pcShift) & loopTagMask;
        // bi->loopIndexB is not used without hash
    }

    for (int i = 0; i < (1 << logLoopTableAssoc); i++) {
        int idx = finallindex(bi->loopIndex, bi->loopIndexB, i);
        if (ltable[idx].tag == bi->loopTag) {
            bi->loopHit = i;
            bi->loopPredValid = calcConf(idx);

            uint16_t iter = speculative ? ltable[idx].currentIterSpec
                                        : ltable[idx].currentIter;

            if ((iter + 1) == ltable[idx].numIter) {
                return useDirectionBit ? !(ltable[idx].dir) : false;
            } else {
                return useDirectionBit ? (ltable[idx].dir) : true;
            }
        }
    }
    return false;
}

bool
LoopPredictor::calcConf(int index) const
{
    return ltable[index].confidence == confidenceThreshold;
}

void
LoopPredictor::specLoopUpdate(bool taken, BranchInfo* bi)
{
    if (bi->loopHit>=0) {
        int index = finallindex(bi->loopIndex, bi->loopIndexB, bi->loopHit);
        if (taken != ltable[index].dir) {
            ltable[index].currentIterSpec = 0;
        } else {
            ltable[index].currentIterSpec =
                (ltable[index].currentIterSpec + 1) & loopNumIterMask;
        }
    }
}

bool
LoopPredictor::optionalAgeInc() const
{
    return false;
}

void
LoopPredictor::loopUpdate(Addr pc, bool taken, BranchInfo* bi, bool tage_pred)
{
    int idx = finallindex(bi->loopIndex, bi->loopIndexB, bi->loopHit);
    if (bi->loopHit >= 0) {
        //already a hit
        if (bi->loopPredValid) {
            if (taken != bi->loopPred) {
                // free the entry
                ltable[idx].numIter = 0;
                ltable[idx].age = 0;
                ltable[idx].confidence = 0;
                ltable[idx].currentIter = 0;
                return;
            } else if (bi->loopPred != tage_pred || optionalAgeInc()) {
                DPRINTF(LTage, "Loop Prediction success:%lx\n",pc);
                unsignedCtrUpdate(ltable[idx].age, true, loopTableAgeBits);
            }
        }

        ltable[idx].currentIter =
            (ltable[idx].currentIter + 1) & loopNumIterMask;
        if (ltable[idx].currentIter > ltable[idx].numIter) {
            ltable[idx].confidence = 0;
            if (ltable[idx].numIter != 0) {
                // free the entry
                ltable[idx].numIter = 0;
                if (optionalAgeReset) {
                    ltable[idx].age = 0;
                }
            }
        }

        if (taken != (useDirectionBit ? ltable[idx].dir : true)) {
            if (ltable[idx].currentIter == ltable[idx].numIter) {
                DPRINTF(LTage, "Loop End predicted successfully:%lx\n", pc);
                unsignedCtrUpdate(ltable[idx].confidence, true,
                                  loopTableConfidenceBits);
                //just do not predict when the loop count is 1 or 2
                if (ltable[idx].numIter < 3) {
                    // free the entry
                    ltable[idx].dir = taken; // ignored if no useDirectionBit
                    ltable[idx].numIter = 0;
                    ltable[idx].age = 0;
                    ltable[idx].confidence = 0;
                }
            } else {
                DPRINTF(LTage, "Loop End predicted incorrectly:%lx\n", pc);
                if (ltable[idx].numIter == 0) {
                    // first complete nest;
                    ltable[idx].confidence = 0;
                    ltable[idx].numIter = ltable[idx].currentIter;
                } else {
                    //not the same number of iterations as last time: free the
                    //entry
                    ltable[idx].numIter = 0;
                    if (optionalAgeReset) {
                        ltable[idx].age = 0;
                    }
                    ltable[idx].confidence = 0;
                }
            }
            ltable[idx].currentIter = 0;
        }

    } else if (useDirectionBit ? (bi->predTaken != taken) : taken) {
        if ((random_mt.random<int>() & 3) == 0 || !restrictAllocation) {
            //try to allocate an entry on taken branch
            int nrand = random_mt.random<int>();
            for (int i = 0; i < (1 << logLoopTableAssoc); i++) {
                int loop_hit = (nrand + i) & ((1 << logLoopTableAssoc) - 1);
                idx = finallindex(bi->loopIndex, bi->loopIndexB, loop_hit);
                if (ltable[idx].age == 0) {
                    DPRINTF(LTage,
                            "Allocating loop pred entry for branch %lx\n",
                            pc);
                    ltable[idx].dir = !taken; // ignored if no useDirectionBit
                    ltable[idx].tag = bi->loopTag;
                    ltable[idx].numIter = 0;
                    ltable[idx].age = initialLoopAge;
                    ltable[idx].confidence = 0;
                    ltable[idx].currentIter = initialLoopIter;
                    break;

                } else {
                    ltable[idx].age--;
                }
                if (restrictAllocation) {
                    break;
                }
            }
        }
    }
}

bool
LoopPredictor::loopPredict(ThreadID tid, Addr branch_pc, bool cond_branch,
                   BranchInfo* bi, bool prev_pred_taken, unsigned instShiftAmt)
{
    bool pred_taken = prev_pred_taken;
    if (cond_branch) {
        // loop prediction
        bi->loopPred = getLoop(branch_pc, bi, useSpeculation, instShiftAmt);

        if ((loopUseCounter >= 0) && bi->loopPredValid) {
            pred_taken = bi->loopPred;
            bi->loopPredUsed = true;
        }

        if (useSpeculation) {
            specLoopUpdate(pred_taken, bi);
        }
    }

    return pred_taken;
}

void
LoopPredictor::squash(ThreadID tid, BranchInfo *bi)
{
    if (bi->loopHit >= 0) {
        int idx = finallindex(bi->loopIndex,
                bi->loopIndexB,
                bi->loopHit);
        ltable[idx].currentIterSpec = bi->currentIter;
    }
}

void
LoopPredictor::squashLoop(BranchInfo* bi)
{
    if (bi->loopHit >= 0) {
        int idx = finallindex(bi->loopIndex,
                bi->loopIndexB,
                bi->loopHit);
        ltable[idx].currentIterSpec = bi->currentIter;
    }
}

void
LoopPredictor::updateStats(bool taken, BranchInfo* bi)
{
    if (taken == bi->loopPred) {
        stats.correct++;
    } else {
        stats.wrong++;
    }
}

void
LoopPredictor::condBranchUpdate(ThreadID tid, Addr branch_pc, bool taken,
                                bool tage_pred, BranchInfo* bi,
                                unsigned instShiftAmt)
{
    if (useSpeculation) {
        // recalculate loop prediction without speculation
        // It is ok to overwrite the loop prediction fields in bi
        // as the stats have already been updated with the previous
        // values
        bi->loopPred = getLoop(branch_pc, bi, false, instShiftAmt);
    }

    if (bi->loopPredValid) {
        if (bi->predTaken != bi->loopPred) {
            signedCtrUpdate(loopUseCounter,
                      (bi->loopPred == taken),
                      withLoopBits);
        }
    }

    loopUpdate(branch_pc, taken, bi, tage_pred);
}

LoopPredictor::LoopPredictorStats::LoopPredictorStats(
    statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(correct, statistics::units::Count::get(),
               "Number of times the loop predictor is the provider and the "
               "prediction is correct"),
      ADD_STAT(wrong, statistics::units::Count::get(),
               "Number of times the loop predictor is the provider and the "
               "prediction is wrong")
{
}

size_t
LoopPredictor::getSizeInBits() const
{
    return (1ULL << logSizeLoopPred) *
        ((useSpeculation ? 3 : 2) * loopTableIterBits +
        loopTableConfidenceBits + loopTableTagBits +
        loopTableAgeBits + useDirectionBit);
}

} // namespace branch_prediction
} // namespace gem5
