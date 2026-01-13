/*
 * Copyright (c) 2023 The University of Edinburgh
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
 * Copyright 2019 Texas A&M University
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Daniel A. Jiménez
 *  Adapted to gem5 by: Javier Bueno Hedo
 *
 */

/*
 * Multiperspective Perceptron Predictor with TAGE (by Daniel A. Jiménez)
 */

#include "cpu/pred/multiperspective_perceptron_tage.hh"

#include "base/random.hh"

namespace gem5
{

namespace branch_prediction
{

void
MPP_TAGE::calculateParameters()
{
   assert(tunedHistoryLengths.size() == (nHistoryTables+1));
   for (int i = 0; i <= nHistoryTables; i += 1) {
      histLengths[i] = tunedHistoryLengths[i];
   }
}

void
MPP_TAGE::handleTAGEUpdate(Addr branch_pc, bool taken,
                           TAGEBase::BranchInfo* bi)
{
    if (bi->hitBank > 0) {
        if (abs (2 * gtable[bi->hitBank][bi->hitBankIndex].ctr + 1) == 1) {
            if (bi->longestMatchPred != taken) {
                // acts as a protection
                if (bi->altBank > 0) {
                    ctrUpdate(gtable[bi->altBank][bi->altBankIndex].ctr, taken,
                              tagTableCounterBits);
                }
                if (bi->altBank == 0){
                    baseUpdate(branch_pc, taken, bi);
                }
            }
        }

        ctrUpdate(gtable[bi->hitBank][bi->hitBankIndex].ctr, taken,
                  tagTableCounterBits);

        //sign changes: no way it can have been useful
        if (abs (2 * gtable[bi->hitBank][bi->hitBankIndex].ctr + 1) == 1) {
            gtable[bi->hitBank][bi->hitBankIndex].u = 0;
        }
    } else {
        baseUpdate(branch_pc, taken, bi);
    }

    if ((bi->longestMatchPred != bi->altTaken) &&
        (bi->longestMatchPred == taken) &&
        (gtable[bi->hitBank][bi->hitBankIndex].u < (1 << tagTableUBits) -1)) {
            gtable[bi->hitBank][bi->hitBankIndex].u++;
    }
}

void
MPP_TAGE::handleAllocAndUReset(bool alloc, bool taken,
                               TAGEBase::BranchInfo* bi, int nrand)
{
    if (!alloc) {
        return;
    }

    int a = 1;

    if ((rng->random<int>() & 127) < 32) {
        a = 2;
    }
    int dep = bi->hitBank + a;

    int penalty = 0;
    int numAllocated = 0;
    int T = 1;

    for (int i = dep; i <= nHistoryTables; i += 1) {
        if (noSkip[i]) {
            if (gtable[i][bi->tableIndices[i]].u == 0) {
                gtable[i][bi->tableIndices[i]].tag = bi->tableTags[i];
                gtable[i][bi->tableIndices[i]].ctr = taken ? 0 : -1;
                numAllocated++;
                if (T <= 0) {
                    break;
                }
                i += 1;
                T -= 1;
            } else {
                penalty++;
            }
        } else { assert(false); }
    }

    tCounter += (penalty - numAllocated);

    handleUReset();
}

void
MPP_TAGE::handleUReset()
{
    //just the best formula for the Championship:
    //In practice when one out of two entries are useful
    if (tCounter < 0) {
        tCounter = 0;
    }

    if (tCounter >= ((1ULL << logUResetPeriod))) {
        // Update the u bits for the short tags table
        for (int i = 1; i <= nHistoryTables; i++) {
            for (int j = 0; j < (1ULL << logTagTableSizes[i]); j++) {
                resetUctr(gtable[i][j].u);
            }
        }

        tCounter = 0;
    }
}

void
MPP_TAGE::resetUctr(uint8_t &u)
{
    // On real HW it should be u >>= 1 instead of if > 0 then u--
    if (u > 0) {
        u--;
    }
}


int
MPP_TAGE::bindex(Addr pc_in) const
{
    uint32_t pc = (uint32_t) pc_in;
    return ((pc ^ (pc >> 4)) & ((1ULL << (logTagTableSizes[0])) - 1));
}

unsigned
MPP_TAGE::getUseAltIdx(TAGEBase::BranchInfo* bi, Addr branch_pc)
{
    uint32_t hpc = ((uint32_t) branch_pc);
    hpc = (hpc ^(hpc >> 4));
    return 2 * ((hpc & ((numUseAltOnNa/2)-1)) ^ bi->longestMatchPred) +
           ((bi->hitBank > (nHistoryTables / 3)) ? 1 : 0);
}

void
MPP_TAGE::adjustAlloc(bool & alloc, bool taken, bool pred_taken)
{
    // Do not allocate too often if the prediction is ok
    if ((taken == pred_taken) && ((rng->random<int>() & 31) != 0)) {
        alloc = false;
    }
}

void
MPP_TAGE::updateHistories(ThreadID tid, Addr branch_pc, bool speculative,
                          bool taken, Addr target, const StaticInstPtr &inst,
                          TAGEBase::BranchInfo* bi)
{
    if (speculative != speculativeHistUpdate) {
        return;
    }
    // speculation is not implemented
    assert(! speculative);

    int brtype = inst->isDirectCtrl() ? 0 : 2;
    if (! inst->isUncondCtrl()) {
        ++brtype;
    }

    // TAGE update
    int tmp = (branch_pc << 1) + taken;
    int path = branch_pc;

    int maxt = (brtype & 1) ? 1 : 4;

    // Update path history
    for (int t = 0; t < maxt; t++) {
        int pathbit = (path & 127);
        path >>= 1;
        threadHistory[tid].pathHist
                        = (threadHistory[tid].pathHist << 1) ^ pathbit;
    }

    // Update global history
    updateGHist(tid, tmp, maxt);
}

bool
MPP_TAGE::isHighConfidence(TAGEBase::BranchInfo *bi) const
{
    if (bi->hitBank > 0) {
        return (abs(2 * gtable[bi->hitBank][bi->hitBankIndex].ctr + 1)) >=
               ((1 << tagTableCounterBits) - 1);
    } else {
        int bim = (btablePrediction[bi->bimodalIndex] << 1)
            + btableHysteresis[bi->bimodalIndex >> logRatioBiModalHystEntries];
        return (bim == 0) || (bim == 3);
    }

}

bool
MPP_LoopPredictor::calcConf(int index) const
{
    return LoopPredictor::calcConf(index) ||
           (ltable[index].confidence * ltable[index].numIter > 128);
}

bool
MPP_LoopPredictor::optionalAgeInc() const
{
    return ((rng->random<int>() & 7) == 0);
}

MPP_StatisticalCorrector::MPP_StatisticalCorrector(
        const MPP_StatisticalCorrectorParams &p) : StatisticalCorrector(p),
    thirdH(0), pnb(p.pnb), logPnb(p.logPnb), pm(p.pm), gnb(p.gnb),
    logGnb(p.logGnb), gm(p.gm)
{
    initGEHLTable(pnb, pm, pgehl, logPnb, wp, -1);
    initGEHLTable(gnb, gm, ggehl, logGnb, wg, -1);

    for (int8_t &pos : wl) {
        pos = -1;
    }
}

void
MPP_StatisticalCorrector::initBias()
{
    for (int j = 0; j < (1 << logBias); j++) {
        if (j & 1) {
            bias[j] = 15;
            biasSK[j] = 15;
        } else {
            bias[j] = -16;
            biasSK[j] = -16;
        }
    }
}

unsigned
MPP_StatisticalCorrector::getIndBias(Addr branch_pc,
        StatisticalCorrector::BranchInfo* bi, bool bias) const
{
    unsigned int truncated_pc = branch_pc;
    return ((truncated_pc << 1) + bi->predBeforeSC) & ((1 << logBias) - 1);
}

unsigned
MPP_StatisticalCorrector::getIndBiasSK(Addr branch_pc,
        StatisticalCorrector::BranchInfo* bi) const
{
    return (((branch_pc ^ (branch_pc >> (logBias - 1))) << 1)
            + bi->predBeforeSC) & ((1 << logBias) - 1);
}

unsigned
MPP_StatisticalCorrector::getIndBiasBank(Addr branch_pc,
        StatisticalCorrector::BranchInfo* bi, int hitBank, int altBank) const
{
    return 0;
}

int
MPP_StatisticalCorrector::gIndexLogsSubstr(int nbr, int i)
{
    return (i >= (nbr - 2)) ? 1 : 0;
}

unsigned
MPP_StatisticalCorrector::getIndUpd(Addr branch_pc) const
{
    return ((branch_pc ^ (branch_pc >> 4)) & ((1 << (logSizeUp)) - 1));
}

void
MPP_StatisticalCorrector::gUpdate(Addr branch_pc, bool taken, int64_t hist,
                   std::vector<int> & length, std::vector<int8_t> * tab,
                   int nbr, int logs, std::vector<int8_t> & w,
                   StatisticalCorrector::BranchInfo* bi)
{
    for (int i = 0; i < nbr; i++) {
        int64_t bhist = hist & ((int64_t) ((1 << length[i]) - 1));
        int64_t index = gIndex(branch_pc, bhist, logs, nbr, i);
        ctrUpdate(tab[i][index], taken, scCountersWidth - (i < (nbr - 1)));
    }
}

bool
MPP_StatisticalCorrector::scPredict(ThreadID tid, Addr branch_pc,
        bool cond_branch, StatisticalCorrector::BranchInfo* bi,
        bool prev_pred_taken, bool bias_bit, bool use_conf_ctr,
        int8_t conf_ctr, unsigned conf_bits, int hitBank, int altBank,
        int init_lsum)
{
    bool pred_taken = prev_pred_taken;
    if (cond_branch) {

        bi->predBeforeSC = prev_pred_taken;

        int lsum = init_lsum;

        getBiasLSUM(branch_pc, bi, lsum);

        int thres = gPredictions(tid, branch_pc, bi, lsum);

        // These will be needed at update time
        bi->lsum = lsum;
        bi->thres = thres;
        bi->scPred = (lsum >= 0);

        if (pred_taken != bi->scPred) {
            pred_taken = bi->scPred;

            if (bi->highConf /* comes from tage prediction */) {
              if ((abs(lsum) < thres / 3))
                pred_taken = (firstH < 0) ? bi->scPred : prev_pred_taken;
              else if ((abs(lsum) < 2 * thres / 3))
                pred_taken = (secondH < 0) ? bi->scPred : prev_pred_taken;
              else if ((abs(lsum) < thres))
                pred_taken = (thirdH < 0) ? bi->scPred : prev_pred_taken;
            }
        }
    }

    return pred_taken;
}

MultiperspectivePerceptronTAGE::MultiperspectivePerceptronTAGE(
    const MultiperspectivePerceptronTAGEParams &p)
  : MultiperspectivePerceptron(p), tage(p.tage),
    loopPredictor(p.loop_predictor),
    statisticalCorrector(p.statistical_corrector)
{
    fatal_if(tage->isSpeculativeUpdateEnabled(),
        "Speculative updates support is not implemented");
}

void
MultiperspectivePerceptronTAGE::init()
{
    tage->init();
    int numBitsTage = tage->getSizeInBits();
    int numBitsLoopPred = loopPredictor->getSizeInBits();
    int numBitsStatisticalCorrector = statisticalCorrector->getSizeInBits();

    setExtraBits(numBitsTage + numBitsLoopPred + numBitsStatisticalCorrector);
    MultiperspectivePerceptron::init();
}


unsigned int
MultiperspectivePerceptronTAGE::getIndex(ThreadID tid, MPPTAGEBranchInfo &bi,
        const HistorySpec &spec, int index) const
{
    // get the hash for the feature
    unsigned int g = spec.getHash(tid, bi.getPC(), bi.getPC() >> 2, index);
    // shift it and xor it with the hashed PC
    unsigned long long int h = g;
    h <<= 20;
    h ^= (bi.getPC() ^ (bi.getPC() >> 2));

    // maybe xor in an IMLI counter
    if ((1ull << index) & imli_mask1) {
        h += threadData[tid]->imli_counter[0];
    }
    if ((1ull << index) & imli_mask4) {
        h += threadData[tid]->imli_counter[3];
    }

    // return it modulo the table size
    return h % table_sizes[index];
}


int
MultiperspectivePerceptronTAGE::computePartialSum(ThreadID tid,
                                                  MPPTAGEBranchInfo &bi) const
{
    int yout = 0;
    for (int i = 0; i < specs.size(); i += 1) {
        yout += specs[i]->coeff *
            threadData[tid]->tables[i][getIndex(tid, bi, *specs[i], i)];
    }
    return yout;
}

void
MultiperspectivePerceptronTAGE::updatePartial(ThreadID tid,
                                              MPPTAGEBranchInfo &bi,
                                              bool taken)
{
    // update tables
    for (int i = 0; i < specs.size(); i += 1) {
        unsigned int idx = getIndex(tid, bi, *specs[i], i);
        short int *c =
            &threadData[tid]->tables[i][idx];
        short int max_weight = (1 << (specs[i]->width - 1)) - 1;
        short int min_weight = -(1 << (specs[i]->width - 1));
        if (taken) {
            if (*c < max_weight) {
                *c += 1;
            }
        } else {
            if (*c > min_weight) {
                *c -= 1;
            }
        }
    }
}

void
MultiperspectivePerceptronTAGE::updateHistories(ThreadID tid,
                                                MPPTAGEBranchInfo &bi,
                                                bool taken)
{
    unsigned int hpc = (bi.getPC() ^ (bi.getPC() >> 2));
    unsigned int pc = bi.getPC();

    // update recency stack
    unsigned short recency_pc = pc >> 2;
    threadData[tid]->insertRecency(recency_pc, assoc);

    // update acyclic history
    threadData[tid]->updateAcyclic(taken, hpc);

    // update modpath histories
    for (int ii = 0; ii < modpath_indices.size(); ii +=1) {
        int i = modpath_indices[ii];
        if (hpc % (i + 2) == 0) {
            memmove(&threadData[tid]->modpath_histories[i][1],
                    &threadData[tid]->modpath_histories[i][0],
                    sizeof(unsigned short int) * (modpath_lengths[ii] - 1));
            threadData[tid]->modpath_histories[i][0] = hpc;
        }
    }

    // update modulo histories
    for (int ii = 0; ii < modhist_indices.size(); ii += 1) {
        int i = modhist_indices[ii];
        if (hpc % (i + 2) == 0) {
            for (int j = modhist_lengths[ii] - 1; j > 0; j -= 1) {
                threadData[tid]->mod_histories[i][j] =
                    threadData[tid]->mod_histories[i][j-1];
            }
            threadData[tid]->mod_histories[i][0] = taken;
        }
    }

    // update blurry history
    std::vector<std::vector<unsigned int>> &blurrypath_histories =
        threadData[tid]->blurrypath_histories;
    for (int i = 0; i < blurrypath_histories.size(); i += 1)
    {
        if (blurrypath_histories[i].size() > 0) {
            unsigned int z = pc >> i;
            if (blurrypath_histories[i][0] != z) {
                memmove(&blurrypath_histories[i][1],
                        &blurrypath_histories[i][0],
                        sizeof(unsigned int) *
                        (blurrypath_histories[i].size() - 1));
                blurrypath_histories[i][0] = z;
            }
        }
    }
}

bool
MultiperspectivePerceptronTAGE::lookup(ThreadID tid, Addr instPC,
                                   void * &bp_history)
{
    MPPTAGEBranchInfo *bi =
        new MPPTAGEBranchInfo(instPC, pcshift, true, *tage, *loopPredictor,
                              *statisticalCorrector);
    bp_history = (void *)bi;
    bool pred_taken = tage->tagePredict(tid, instPC, true, bi->tageBranchInfo);

    pred_taken = loopPredictor->loopPredict(tid, instPC, true,
            bi->lpBranchInfo, pred_taken, instShiftAmt);

    bi->scBranchInfo->highConf = tage->isHighConfidence(bi->tageBranchInfo);

    int init_lsum = 22;
    if (!pred_taken) {
        init_lsum = -init_lsum;
    }
    init_lsum += computePartialSum(tid, *bi);

    pred_taken = statisticalCorrector->scPredict(tid, instPC, true,
            bi->scBranchInfo, pred_taken, false /* bias_bit: unused */,
            false /* use_tage_ctr: unused */, 0 /* conf_ctr: unused */,
            0 /* conf_bits: unused */, 0 /* hitBank: unused */,
            0 /* altBank: unused */, init_lsum);
    bi->predictedTaken = pred_taken;
    bi->lpBranchInfo->predTaken = pred_taken;
    return pred_taken;
}


void
MPP_StatisticalCorrector::condBranchUpdate(ThreadID tid, Addr branch_pc,
        bool taken, StatisticalCorrector::BranchInfo *bi, Addr target,
        bool bias_bit, int hitBank, int altBank)
{
    bool scPred = (bi->lsum >= 0);

    if (bi->predBeforeSC != scPred) {
        if (abs(bi->lsum) < bi->thres) {
            if (bi->highConf) {
                if (abs(bi->lsum) < bi->thres / 3) {
                    ctrUpdate(firstH, (bi->predBeforeSC == taken),
                              chooserConfWidth);
                } else if (abs(bi->lsum) < 2 * bi->thres / 3) {
                    ctrUpdate(secondH, (bi->predBeforeSC == taken),
                              chooserConfWidth);
                } else if (abs(bi->lsum) < bi->thres) {
                    ctrUpdate(thirdH, (bi->predBeforeSC == taken),
                              chooserConfWidth);
                }
            }
        }
    }

    if ((scPred != taken) || ((abs(bi->lsum) < bi->thres))) {

        ctrUpdate(pUpdateThreshold[getIndUpd(branch_pc)], (scPred != taken),
                  pUpdateThresholdWidth + 1); //+1 because the sign is ignored
        if (pUpdateThreshold[getIndUpd(branch_pc)] < 0)
            pUpdateThreshold[getIndUpd(branch_pc)] = 0;

        unsigned indBias = getIndBias(branch_pc, bi, false);
        unsigned indBiasSK = getIndBiasSK(branch_pc, bi);

        ctrUpdate(bias[indBias], taken, scCountersWidth);
        ctrUpdate(biasSK[indBiasSK], taken, scCountersWidth);

        gUpdates(tid, branch_pc, taken, bi);
    }
}

void
MultiperspectivePerceptronTAGE::update(ThreadID tid, Addr pc, bool taken,
                                   void * &bp_history, bool squashed,
                                   const StaticInstPtr & inst,
                                   Addr target)
{
    assert(bp_history);
    MPPTAGEBranchInfo *bi = static_cast<MPPTAGEBranchInfo*>(bp_history);

    if (squashed) {
        if (tage->isSpeculativeUpdateEnabled()) {
            // This restores the global history, then update it
            // and recomputes the folded histories.
            tage->squash(tid, taken, target, inst, bi->tageBranchInfo);
            if (bi->tageBranchInfo->condBranch) {
                loopPredictor->squashLoop(bi->lpBranchInfo);
            }
        }
        return;
    }

    if (bi->isUnconditional()) {
        tage->updateHistories(tid, pc, false, taken, target,
                              inst, bi->tageBranchInfo);
        statisticalCorrector->updateHistories(pc, false, inst, taken,
                                              bi->scBranchInfo, target,
                                              tage->getPathHist(tid, false));
    } else {
        tage->updateStats(taken, bi->tageBranchInfo);
        loopPredictor->updateStats(taken, bi->lpBranchInfo);
        statisticalCorrector->updateStats(taken, bi->scBranchInfo);

        loopPredictor->condBranchUpdate(tid, pc, taken,
                bi->tageBranchInfo->tagePred, bi->lpBranchInfo, instShiftAmt);

        bool scPred = (bi->scBranchInfo->lsum >= 0);
        if ((scPred != taken) ||
            ((abs(bi->scBranchInfo->lsum) < bi->scBranchInfo->thres))) {
            updatePartial(tid, *bi, taken);
        }
        statisticalCorrector->condBranchUpdate(tid, pc, taken,
                                               bi->scBranchInfo, target,
                                               false /* bias_bit: unused */,
                                               0 /* hitBank: unused */,
                                               0 /* altBank: unused*/);

        tage->condBranchUpdate(tid, pc, taken, bi->tageBranchInfo,
                               rng->random<int>(), target,
                               bi->predictedTaken, true);

        updateHistories(tid, *bi, taken);

        if (!tage->isSpeculativeUpdateEnabled()) {
            if (inst->isCondCtrl() && inst->isDirectCtrl()
                && !inst->isCall() && !inst->isReturn()) {
                uint32_t truncated_target = target;
                uint32_t truncated_pc = pc;
                if (truncated_target < truncated_pc) {
                    if (!taken) {
                        threadData[tid]->imli_counter[0] = 0;
                    } else {
                        threadData[tid]->imli_counter[0] += 1;
                    }
                } else {
                    if (taken) {
                        threadData[tid]->imli_counter[3] = 0;
                    } else {
                        threadData[tid]->imli_counter[3] += 1;
                    }
                }
            }

            tage->updateHistories(tid, pc, false, taken, target,
                                  inst, bi->tageBranchInfo);
            statisticalCorrector->updateHistories(pc, false, inst, taken,
                                                  bi->scBranchInfo, target,
                                                  tage->getPathHist(tid,
                                                                    false));
        }
    }
    delete bi;
    bp_history = nullptr;
}

void
MultiperspectivePerceptronTAGE::updateHistories(ThreadID tid, Addr pc,
                                    bool uncond, bool taken, Addr target,
                                    const StaticInstPtr &inst,
                                    void * &bp_history)
{
    assert(uncond || bp_history);

    // For perceptron there is no speculative history correction.
    // Conditional branches are done.
    if (!uncond) return;

    MPPTAGEBranchInfo *bi =
        new MPPTAGEBranchInfo(pc, pcshift, false, *tage, *loopPredictor,
                              *statisticalCorrector);
    bp_history = (void *) bi;
}

void
MultiperspectivePerceptronTAGE::squash(ThreadID tid, void * &bp_history)
{
    assert(bp_history);
    MPPTAGEBranchInfo *bi = static_cast<MPPTAGEBranchInfo*>(bp_history);
    delete bi;
    bp_history = nullptr;
}

} // namespace branch_prediction
} // namespace gem5
