/*
 * Copyright (c) 2018 Metempsy Technology Consulting
 * Copyright (c) 2024 Technical University of Munich
 * All rights reserved.
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
 * Author: André Seznec, Pau Cabre, Javier Bueno
 *
 */

/*
 * Statistical corrector base class
 */

#include "cpu/pred/statistical_corrector.hh"

#include "params/StatisticalCorrector.hh"

namespace gem5
{

namespace branch_prediction
{

StatisticalCorrector::StatisticalCorrector(
    const StatisticalCorrectorParams &p)
  : SimObject(p),
    logBias(p.logBias),
    logSizeUp(p.logSizeUp),
    logSizeUps(logSizeUp / 2),
    numEntriesFirstLocalHistories(p.numEntriesFirstLocalHistories),
    bwnb(p.bwnb),
    logBwnb(p.logBwnb),
    bwm(p.bwm),
    lnb(p.lnb),
    logLnb(p.logLnb),
    lm(p.lm),
    inb(p.inb),
    logInb(p.logInb),
    im(p.im),
    chooserConfWidth(p.chooserConfWidth),
    updateThresholdWidth(p.updateThresholdWidth),
    pUpdateThresholdWidth(p.pUpdateThresholdWidth),
    extraWeightsWidth(p.extraWeightsWidth),
    scCountersWidth(p.scCountersWidth),
    instShiftAmt(p.instShiftAmt),
    speculativeHistUpdate(p.speculativeHistUpdate),
    firstH(0),
    secondH(0),
    stats(this)
{
    wb.resize(1 << logSizeUps, 4);

    initGEHLTable(lnb, lm, lgehl, logLnb, wl, p.lWeightInitValue);
    initGEHLTable(bwnb, bwm, bwgehl, logBwnb, wbw, p.bwWeightInitValue);
    initGEHLTable(inb, im, igehl, logInb, wi, p.iWeightInitValue);

    updateThreshold = 35 << 3;

    pUpdateThreshold.resize(1 << logSizeUp, p.initialUpdateThresholdValue);

    bias.resize(1 << logBias);
    biasSK.resize(1 << logBias);
    biasBank.resize(1 << logBias);
}

StatisticalCorrector::BranchInfo*
StatisticalCorrector::makeBranchInfo()
{
    return new BranchInfo();
}

StatisticalCorrector::SCThreadHistory*
StatisticalCorrector::makeThreadHistory()
{
    return new SCThreadHistory(instShiftAmt);
}

void
StatisticalCorrector::initBias()
{
    for (int j = 0; j < (1 << logBias); j++) {
        switch (j & 3) {
          case 0:
            bias[j] = -32;
            biasSK[j] = -8;
            biasBank[j] = -32;
            break;
          case 1:
            bias[j] = 31;
            biasSK[j] = 7;
            biasBank[j] = 31;
            break;
          case 2:
            bias[j] = -1;
            biasSK[j] = -32;
            biasBank[j] = -1;
            break;
          case 3:
            bias[j] = 0;
            biasSK[j] = 31;
            biasBank[j] = 0;
            break;
        }
    }
}

void
StatisticalCorrector::initGEHLTable(unsigned numLenghts,
                                    std::vector<int> lengths,
                                    std::vector<int8_t> *&table,
                                    unsigned logNumEntries,
                                    std::vector<int8_t> &w, int8_t wInitValue)
{
    assert(lengths.size() == numLenghts);
    if (numLenghts == 0) {
        return;
    }
    table = new std::vector<int8_t> [numLenghts];
    for (int i = 0; i < numLenghts; ++i) {
        table[i].resize(1 << logNumEntries, 0);
        for (int j = 0; j < ((1 << logNumEntries) - 1); ++j) {
            if (! (j & 1)) {
                table[i][j] = -1;
            }
        }
    }

    w.resize(1 << logSizeUps, wInitValue);
}

unsigned
StatisticalCorrector::getIndBias(Addr branch_pc, BranchInfo* bi,
                                 bool bias) const
{
    Addr shifted_pc = branch_pc >> instShiftAmt;
    return (((((shifted_pc ^ (shifted_pc >> 2)) << 1)
            ^ (bi->lowConf & bias)) << 1) +  bi->predBeforeSC)
            & ((1 << logBias) - 1);
}

unsigned
StatisticalCorrector::getIndBiasSK(Addr branch_pc, BranchInfo *bi) const
{
    Addr shifted_pc = branch_pc >> instShiftAmt;
    return (((((shifted_pc ^ (shifted_pc >> (logBias - 2))) << 1)
            ^ bi->highConf) << 1) + bi->predBeforeSC)
            & ((1 << logBias) - 1);
}

unsigned
StatisticalCorrector::getIndUpd(Addr branch_pc) const
{
    Addr shifted_pc = branch_pc >> instShiftAmt;
    return ((shifted_pc ^ (shifted_pc >> 2)) & ((1 << logSizeUp) - 1));
}

unsigned
StatisticalCorrector::getIndUpds(Addr branch_pc) const
{
    Addr shifted_pc = branch_pc >> instShiftAmt;
    return ((shifted_pc ^ (shifted_pc >> 2)) & ((1 << logSizeUps) - 1));
}

int64_t
StatisticalCorrector::gIndex(Addr branch_pc, int64_t bhist, int logs, int nbr,
                             int i)
{
    return (((int64_t) branch_pc >> instShiftAmt) ^
             bhist ^ (bhist >> (8 - i)) ^
            (bhist >> (16 - 2 * i)) ^ (bhist >> (24 - 3 * i)) ^
            (bhist >> (32 - 3 * i)) ^ (bhist >> (40 - 4 * i))) &
           ((1 << (logs - gIndexLogsSubstr(nbr, i))) - 1);
}

int
StatisticalCorrector::gPredict(Addr branch_pc, int64_t hist,
                               std::vector<int> &length,
                               std::vector<int8_t> *tab, int nbr, int logs,
                               std::vector<int8_t> &w)
{
    int percsum = 0;
    for (int i = 0; i < nbr; i++) {
        int64_t bhist = hist & ((int64_t) ((1 << length[i]) - 1));
        int64_t index = gIndex(branch_pc, bhist, logs, nbr, i);
        int8_t ctr = tab[i][index];
        percsum += (2 * ctr + 1);
    }
    percsum = (1 + (w[getIndUpds(branch_pc)] >= 0)) * percsum;
    return percsum;
}

void
StatisticalCorrector::gUpdate(Addr branch_pc, bool taken, int64_t hist,
                              std::vector<int> &length,
                              std::vector<int8_t> *tab, int nbr, int logs,
                              std::vector<int8_t> &w, BranchInfo *bi)
{
    int percsum = 0;
    for (int i = 0; i < nbr; i++) {
        int64_t bhist = hist & ((int64_t) ((1 << length[i]) - 1));
        int64_t index = gIndex(branch_pc, bhist, logs, nbr, i);
        percsum += (2 * tab[i][index] + 1);
        ctrUpdate(tab[i][index], taken, scCountersWidth);
    }

    int xsum = bi->lsum - ((w[getIndUpds(branch_pc)] >= 0)) * percsum;
    if ((xsum + percsum >= 0) != (xsum >= 0)) {
        ctrUpdate(w[getIndUpds(branch_pc)], ((percsum >= 0) == taken),
                  extraWeightsWidth);
    }
}

bool
StatisticalCorrector::scPredict(ThreadID tid, Addr branch_pc, bool cond_branch,
                                BranchInfo *bi, bool prev_pred_taken,
                                bool bias_bit, bool use_conf_ctr,
                                int8_t conf_ctr, unsigned conf_bits,
                                int hitBank, int altBank, int init_lsum)
{
    bool pred_taken = prev_pred_taken;
    if (cond_branch) {

        bi->predBeforeSC = prev_pred_taken;
        int lsum = init_lsum;

        lsum += calcBias(branch_pc, bi, bias_bit,
                         use_conf_ctr, conf_ctr, conf_bits, hitBank, altBank);

        // Record the history state to be able to recover
        // The gPredictions function will use the recorded state to
        // make the predictions
        scRecordHistState(branch_pc, bi);

        int thres = gPredictions(tid, branch_pc, bi, lsum);

        // These will be needed at update time
        bi->lsum = lsum;
        bi->thres = thres;

        bool scPred = (lsum >= 0);

        if (pred_taken != scPred) {
            bool useScPred = true;
            //Choser uses TAGE confidence and |LSUM|
            if (bi->highConf) {
                if (abs (lsum) < (thres / 4)) {
                    useScPred = false;
                } else if (abs (lsum) < (thres / 2)) {
                    useScPred = (secondH < 0);
                }
            }

            if (bi->medConf) {
                if (abs (lsum) < (thres / 4)) {
                    useScPred = (firstH < 0);
                }
            }

            bi->usedScPred = useScPred;
            if (useScPred) {
                pred_taken = scPred;
                bi->scPred = scPred;
            }
        }
    }

    return pred_taken;
}

int
StatisticalCorrector::calcBias(Addr branch_pc, BranchInfo *bi, bool bias_bit,
                               bool use_conf_ctr, int8_t conf_ctr,
                               unsigned conf_bits, int hitBank, int altBank)
{
    // first calc/update the confidences from the TAGE prediction
    if (use_conf_ctr) {
        bi->lowConf = (abs(2 * conf_ctr + 1) == 1);
        bi->medConf = (abs(2 * conf_ctr + 1) == 5);
        bi->highConf = (abs(2 * conf_ctr + 1) >= (1<<conf_bits) - 1);
    }

    int lsum = 0;

    int8_t ctr = bias[getIndBias(branch_pc, bi, bias_bit)];
    lsum += (2 * ctr + 1);
    ctr = biasSK[getIndBiasSK(branch_pc, bi)];
    lsum += (2 * ctr + 1);
    ctr = biasBank[getIndBiasBank(branch_pc, bi, hitBank, altBank)];
    lsum += (2 * ctr + 1);

    lsum = (1 + (wb[getIndUpds(branch_pc)] >= 0)) * lsum;
    return lsum;
}


void
StatisticalCorrector::updateHistories(Addr branch_pc, bool speculative,
                                      const StaticInstPtr &inst, bool taken,
                                      BranchInfo *bi, Addr target,
                                      int64_t phist)
{
    if (speculative != speculativeHistUpdate) {
        return;
    }

    // If this is the first time we see this branch record the current
    // state of the history to be able to recover.
    if (speculativeHistUpdate && (!bi->modified)) {
        scRecordHistState(branch_pc, bi);
    }

    // In case the branch already updated the history
    // we need to revert the previous update first.
    if (bi->modified) {
        scRestoreHistState(bi);
    }

    // Update the SC histories
    scHistoryUpdate(branch_pc, inst, taken, target, phist);
    bi->modified = true;
}

void
StatisticalCorrector::scHistoryUpdate(Addr branch_pc,
                                      const StaticInstPtr &inst, bool taken,
                                      Addr target, int64_t phist)
{
    // Update the path history from TAGE
    scHistory->pHist = phist;
    int brtype = inst->isDirectCtrl() ? 0 : 2;
    if (! inst->isUncondCtrl()) {
        ++brtype;
    }
    // Non speculative SC histories update
    if (brtype & 1) {
        if (target < branch_pc) {
            //This branch corresponds to a loop
            if (!taken) {
                //exit of the "loop"
                scHistory->imliCount = 0;
            } else {
                if (scHistory->imliCount < ((1 << im[0]) - 1)) {
                    scHistory->imliCount++;
                }
            }
        }

        scHistory->bwHist = (scHistory->bwHist << 1) +
                                (taken & (target < branch_pc));
        scHistory->updateLocalHistory(1, branch_pc, taken);
    }
}

void
StatisticalCorrector::scRecordHistState(Addr branch_pc, BranchInfo *bi)
{
    bi->pc = branch_pc;
    bi->bwHist = scHistory->bwHist;
    bi->imliCount = scHistory->imliCount;
    bi->pHist = scHistory->pHist;
    bi->localHistories[1] = scHistory->getLocalHistory(1, branch_pc);
}

bool
StatisticalCorrector::scRestoreHistState(BranchInfo *bi)
{
    if (!bi->modified) {
        return false;
    }
    scHistory->bwHist = bi->bwHist;
    scHistory->imliCount = bi->imliCount;
    scHistory->setLocalHistory(1, bi->pc, bi->localHistories[1]);
    return true;
}


void
StatisticalCorrector::condBranchUpdate(ThreadID tid, Addr branch_pc,
                                       bool taken, BranchInfo *bi,
                                       Addr corrTarget, bool b, int hitBank,
                                       int altBank)
{

    if (speculativeHistUpdate) {
        // For speculative updates we recalculate the lsum as it might
        // have changed since the last update

        bi->lsum = calcBias(branch_pc, bi, b, false/* was already recorded*/,
                            0, 0, hitBank, altBank);
        bi->thres = gPredictions(tid, branch_pc, bi, bi->lsum);
    }

    bool scPred = (bi->lsum >= 0);

    if (bi->predBeforeSC != scPred) {
        if (abs(bi->lsum) < bi->thres) {
            if (bi->highConf) {
                if ((abs(bi->lsum) < bi->thres / 2)) {
                    if ((abs(bi->lsum) >= bi->thres / 4)) {
                        ctrUpdate(secondH, (bi->predBeforeSC == taken),
                                  chooserConfWidth);
                    }
                }
            }
        }
        if (bi->medConf) {
            if ((abs(bi->lsum) < bi->thres / 4)) {
                ctrUpdate(firstH, (bi->predBeforeSC == taken),
                          chooserConfWidth);
            }
        }
    }

    if ((scPred != taken) || ((abs(bi->lsum) < bi->thres))) {
        ctrUpdate(updateThreshold, (scPred != taken), updateThresholdWidth);
        ctrUpdate(pUpdateThreshold[getIndUpd(branch_pc)], (scPred != taken),
                  pUpdateThresholdWidth);

        unsigned indUpds = getIndUpds(branch_pc);
        unsigned indBias = getIndBias(branch_pc, bi, b);
        unsigned indBiasSK = getIndBiasSK(branch_pc, bi);
        unsigned indBiasBank = getIndBiasBank(branch_pc, bi, hitBank, altBank);

        int xsum = bi->lsum -
                      ((wb[indUpds] >= 0) * ((2 * bias[indBias] + 1) +
                          (2 * biasSK[indBiasSK] + 1) +
                          (2 * biasBank[indBiasBank] + 1)));

        if ((xsum + ((2 * bias[indBias] + 1) + (2 * biasSK[indBiasSK] + 1) +
            (2 * biasBank[indBiasBank] + 1)) >= 0) != (xsum >= 0))
        {
            ctrUpdate(wb[indUpds],
                      (((2 * bias[indBias] + 1) +
                        (2 * biasSK[indBiasSK] + 1) +
                        (2 * biasBank[indBiasBank] + 1) >= 0) == taken),
                      extraWeightsWidth);
        }

        ctrUpdate(bias[indBias], taken, scCountersWidth);
        ctrUpdate(biasSK[indBiasSK], taken, scCountersWidth);
        ctrUpdate(biasBank[indBiasBank], taken, scCountersWidth);

        gUpdates(tid, branch_pc, taken, bi);
    }
}

void
StatisticalCorrector::updateStats(bool taken, BranchInfo *bi)
{
    if (taken == bi->scPred) {
        stats.correct++;
    } else {
        stats.wrong++;
    }
}

void
StatisticalCorrector::init()
{
    scHistory = makeThreadHistory();
    initBias();
}

size_t
StatisticalCorrector::getSizeInBits() const
{
    // Not implemented
    return 0;
}

StatisticalCorrector::StatisticalCorrectorStats::StatisticalCorrectorStats(
    statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(correct, statistics::units::Count::get(),
               "Number of time the SC predictor is the provider and the "
               "prediction is correct"),
      ADD_STAT(wrong, statistics::units::Count::get(),
               "Number of time the SC predictor is the provider and the "
               "prediction is wrong")
{
}

} // namespace branch_prediction
} // namespace gem5
