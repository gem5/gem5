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
 * Implementation of a L-TAGE branch predictor
 */

#include "cpu/pred/ltage.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"
#include "debug/LTage.hh"

LTAGE::LTAGE(const LTAGEParams *params)
  : TAGE(params),
    logSizeLoopPred(params->logSizeLoopPred),
    loopTableAgeBits(params->loopTableAgeBits),
    loopTableConfidenceBits(params->loopTableConfidenceBits),
    loopTableTagBits(params->loopTableTagBits),
    loopTableIterBits(params->loopTableIterBits),
    logLoopTableAssoc(params->logLoopTableAssoc),
    confidenceThreshold((1 << loopTableConfidenceBits) - 1),
    loopTagMask((1 << loopTableTagBits) - 1),
    loopNumIterMask((1 << loopTableIterBits) - 1),
    loopSetMask((1 << (logSizeLoopPred - logLoopTableAssoc)) - 1),
    loopUseCounter(0),
    withLoopBits(params->withLoopBits),
    useDirectionBit(params->useDirectionBit),
    useSpeculation(params->useSpeculation),
    useHashing(params->useHashing)
{
    // we use uint16_t type for these vales, so they cannot be more than
    // 16 bits
    assert(loopTableTagBits <= 16);
    assert(loopTableIterBits <= 16);

    assert(logSizeLoopPred >= logLoopTableAssoc);

    ltable = new LoopEntry[ULL(1) << logSizeLoopPred];
}

int
LTAGE::lindex(Addr pc_in) const
{
    // The loop table is implemented as a linear table
    // If associativity is N (N being 1 << logLoopTableAssoc),
    // the first N entries are for set 0, the next N entries are for set 1,
    // and so on.
    // Thus, this function calculates the set and then it gets left shifted
    // by logLoopTableAssoc in order to return the index of the first of the
    // N entries of the set
    Addr mask = (ULL(1) << (logSizeLoopPred - logLoopTableAssoc)) - 1;
    Addr pc = pc_in >> instShiftAmt;
    if (useHashing) {
        // copied from TAGE-SC-L
        // (http://www.jilp.org/cbp2016/code/AndreSeznecLimited.tar.gz)
        pc ^= (pc_in >> (instShiftAmt + logLoopTableAssoc));
    }
    return ((pc & mask) << logLoopTableAssoc);
}

int
LTAGE::finallindex(int index, int lowPcBits, int way) const
{
    // copied from TAGE-SC-L
    // (http://www.jilp.org/cbp2016/code/AndreSeznecLimited.tar.gz)
    return (useHashing ? (index ^ ((lowPcBits >> way) << logLoopTableAssoc)) :
                         (index))
           + way;
}

//loop prediction: only used if high confidence
bool
LTAGE::getLoop(Addr pc, LTageBranchInfo* bi, bool speculative) const
{
    bi->loopHit = -1;
    bi->loopPredValid = false;
    bi->loopIndex = lindex(pc);
    unsigned pcShift = instShiftAmt + logSizeLoopPred - logLoopTableAssoc;
    bi->loopTag = ((pc) >> pcShift) & loopTagMask;

    if (useHashing) {
        bi->loopTag ^= ((pc >> (pcShift + logSizeLoopPred)) & loopTagMask);
        bi->loopLowPcBits = (pc >> pcShift) & loopSetMask;
    }

    for (int i = 0; i < (1 << logLoopTableAssoc); i++) {
        int idx = finallindex(bi->loopIndex, bi->loopLowPcBits, i);
        if (ltable[idx].tag == bi->loopTag) {
            bi->loopHit = i;
            bi->loopPredValid =
                ltable[idx].confidence == confidenceThreshold;

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

void
LTAGE::specLoopUpdate(bool taken, LTageBranchInfo* bi)
{
    if (bi->loopHit>=0) {
        int index = finallindex(bi->loopIndex, bi->loopLowPcBits, bi->loopHit);
        if (taken != ltable[index].dir) {
            ltable[index].currentIterSpec = 0;
        } else {
            ltable[index].currentIterSpec =
                (ltable[index].currentIterSpec + 1) & loopNumIterMask;
        }
    }
}

void
LTAGE::loopUpdate(Addr pc, bool taken, LTageBranchInfo* bi)
{
    int idx = finallindex(bi->loopIndex, bi->loopLowPcBits, bi->loopHit);
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
            } else if (bi->loopPred != bi->tagePred) {
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
                ltable[idx].age = 0;
                ltable[idx].confidence = 0;
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
                    ltable[idx].age = 0;
                    ltable[idx].confidence = 0;
                }
            }
            ltable[idx].currentIter = 0;
        }

    } else if (useDirectionBit ?
                ((bi->loopPredValid ? bi->loopPred : bi->tagePred) != taken) :
                taken) {
        //try to allocate an entry on taken branch
        int nrand = random_mt.random<int>();
        for (int i = 0; i < (1 << logLoopTableAssoc); i++) {
            int loop_hit = (nrand + i) & ((1 << logLoopTableAssoc) - 1);
            idx = bi->loopIndex + loop_hit;
            if (ltable[idx].age == 0) {
                DPRINTF(LTage, "Allocating loop pred entry for branch %lx\n",
                        pc);
                ltable[idx].dir = !taken; // ignored if no useDirectionBit
                ltable[idx].tag = bi->loopTag;
                ltable[idx].numIter = 0;
                ltable[idx].age = (1 << loopTableAgeBits) - 1;
                ltable[idx].confidence = 0;
                ltable[idx].currentIter = 1;
                break;

            }
            else
                ltable[idx].age--;
        }
    }

}

//prediction
bool
LTAGE::predict(ThreadID tid, Addr branch_pc, bool cond_branch, void* &b)
{
    LTageBranchInfo *bi = new LTageBranchInfo(nHistoryTables+1);
    b = (void*)(bi);

    bool pred_taken = tagePredict(tid, branch_pc, cond_branch, bi);

    if (cond_branch) {
        // loop prediction
        bi->loopPred = getLoop(branch_pc, bi, useSpeculation);

        if ((loopUseCounter >= 0) && bi->loopPredValid) {
            pred_taken = bi->loopPred;
            bi->provider = LOOP;
        }
        DPRINTF(LTage, "Predict for %lx: taken?:%d, loopTaken?:%d, "
                "loopValid?:%d, loopUseCounter:%d, tagePred:%d, altPred:%d\n",
                branch_pc, pred_taken, bi->loopPred, bi->loopPredValid,
                loopUseCounter, bi->tagePred, bi->altTaken);

        if (useSpeculation) {
            specLoopUpdate(pred_taken, bi);
        }
    }

    return pred_taken;
}

void
LTAGE::condBranchUpdate(Addr branch_pc, bool taken,
                        TageBranchInfo* tage_bi, int nrand)
{
    LTageBranchInfo* bi = static_cast<LTageBranchInfo*>(tage_bi);

    if (useSpeculation) {
        // recalculate loop prediction without speculation
        // It is ok to overwrite the loop prediction fields in bi
        // as the stats have already been updated with the previous
        // values
        bi->loopPred = getLoop(branch_pc, bi, false);
    }

    if (bi->loopPredValid) {
        if (bi->tagePred != bi->loopPred) {
            ctrUpdate(loopUseCounter,
                      (bi->loopPred == taken),
                      withLoopBits);
        }
    }

    loopUpdate(branch_pc, taken, bi);

    TAGE::condBranchUpdate(branch_pc, taken, bi, nrand);
}

void
LTAGE::squash(ThreadID tid, bool taken, void *bp_history)
{
    TAGE::squash(tid, taken, bp_history);

    LTageBranchInfo* bi = (LTageBranchInfo*)(bp_history);

    if (bi->condBranch) {
        if (bi->loopHit >= 0) {
            int idx = finallindex(bi->loopIndex,
                                  bi->loopLowPcBits,
                                  bi->loopHit);
            ltable[idx].currentIterSpec = bi->currentIter;
        }
    }
}

void
LTAGE::squash(ThreadID tid, void *bp_history)
{
    LTageBranchInfo* bi = (LTageBranchInfo*)(bp_history);
    if (bi->condBranch) {
        if (bi->loopHit >= 0) {
            int idx = finallindex(bi->loopIndex,
                                  bi->loopLowPcBits,
                                  bi->loopHit);
            ltable[idx].currentIterSpec = bi->currentIter;
        }
    }

    TAGE::squash(tid, bp_history);
}


void
LTAGE::updateStats(bool taken, TageBranchInfo* bi)
{
    TAGE::updateStats(taken, bi);

    LTageBranchInfo * ltage_bi = static_cast<LTageBranchInfo *>(bi);

    if (ltage_bi->provider == LOOP) {
        if (taken == ltage_bi->loopPred) {
            loopPredictorCorrect++;
        } else {
            loopPredictorWrong++;
        }
    }
}



void
LTAGE::regStats()
{
    TAGE::regStats();

    loopPredictorCorrect
        .name(name() + ".loopPredictorCorrect")
        .desc("Number of times the loop predictor is the provider and "
              "the prediction is correct");

    loopPredictorWrong
        .name(name() + ".loopPredictorWrong")
        .desc("Number of times the loop predictor is the provider and "
              "the prediction is wrong");
}



LTAGE*
LTAGEParams::create()
{
    return new LTAGE(this);
}
