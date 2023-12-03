/*
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
 * 64 KB version
 */

#include "cpu/pred/multiperspective_perceptron_tage_64KB.hh"

namespace gem5
{

namespace branch_prediction
{

MPP_StatisticalCorrector_64KB::MPP_StatisticalCorrector_64KB(
    const MPP_StatisticalCorrector_64KBParams &p)
    : MPP_StatisticalCorrector(p),
      numEntriesSecondLocalHistories(p.numEntriesSecondLocalHistories),
      numEntriesThirdLocalHistories(p.numEntriesThirdLocalHistories),
      snb(p.snb),
      logSnb(p.logSnb),
      sm(p.sm),
      tnb(p.tnb),
      logTnb(p.logTnb),
      tm(p.tm)
{
    initGEHLTable(snb, sm, sgehl, logSnb, ws, -1);
    initGEHLTable(tnb, tm, tgehl, logTnb, wt, -1);
}

MPP_StatisticalCorrector_64KB::SCThreadHistory *
MPP_StatisticalCorrector_64KB::makeThreadHistory()
{
    MPP_SCThreadHistory *sh = new MPP_SCThreadHistory();

    sh->setNumOrdinalHistories(3);
    sh->initLocalHistory(1, numEntriesFirstLocalHistories, 4);
    sh->initLocalHistory(2, numEntriesSecondLocalHistories, 5);
    sh->initLocalHistory(3, numEntriesThirdLocalHistories, 3);

    return sh;
}

void
MPP_StatisticalCorrector_64KB::getBiasLSUM(
    Addr branch_pc, StatisticalCorrector::BranchInfo *bi, int &lsum) const
{
    int8_t ctr = bias[getIndBias(branch_pc, bi, false /* unused */)];
    lsum += 2.09 * ctr;
    ctr = biasSK[getIndBiasSK(branch_pc, bi)];
    lsum += 2.08 * ctr;
}

int
MPP_StatisticalCorrector_64KB::gPredictions(
    ThreadID tid, Addr branch_pc, StatisticalCorrector::BranchInfo *bi,
    int &lsum, int64_t phist)
{
    MPP_SCThreadHistory *sh = static_cast<MPP_SCThreadHistory *>(scHistory);
    unsigned int pc = branch_pc;
    lsum += gPredict((pc << 1) + bi->predBeforeSC, sh->globalHist << 11, gm,
                     ggehl, gnb, logGnb, wg);

    // Local History #1
    lsum += 2.02 * gPredict(branch_pc, sh->getLocalHistory(1, branch_pc), lm,
                            lgehl, lnb, logLnb, wl);
    if (sh->getLocalHistory(1, branch_pc) == 2047)
        lsum += 4;
    if (sh->getLocalHistory(1, branch_pc) == 0)
        lsum -= 4;

    // Local History #3
    lsum += gPredict(branch_pc, sh->getLocalHistory(3, branch_pc) << 11, tm,
                     tgehl, tnb, logTnb, wt);

    // Local History #2
    lsum += gPredict(branch_pc, sh->getLocalHistory(2, branch_pc), sm, sgehl,
                     snb, logSnb, ws);

    lsum += gPredict(branch_pc, sh->getHistoryStackEntry(), pm, pgehl, pnb,
                     logPnb, wp);

    int thres = pUpdateThreshold[getIndUpd(branch_pc)];

    return thres;
}

void
MPP_StatisticalCorrector_64KB::gUpdates(ThreadID tid, Addr pc, bool taken,
                                        StatisticalCorrector::BranchInfo *bi,
                                        int64_t phist)
{
    MPP_SCThreadHistory *sh = static_cast<MPP_SCThreadHistory *>(scHistory);

    gUpdate((pc << 1) + bi->predBeforeSC, taken, sh->globalHist << 11, gm,
            ggehl, gnb, logGnb, wg, bi);

    gUpdate(pc, taken, sh->getLocalHistory(1, pc), lm, lgehl, lnb, logLnb, wl,
            bi);

    gUpdate(pc, taken, sh->getLocalHistory(2, pc), sm, sgehl, snb, logSnb, ws,
            bi);

    gUpdate(pc, taken, sh->getLocalHistory(3, pc) << 11, tm, tgehl, tnb,
            logTnb, wt, bi);

    gUpdate(pc, taken, sh->getHistoryStackEntry(), pm, pgehl, pnb, logPnb, wp,
            bi);
}

void
MPP_StatisticalCorrector_64KB::scHistoryUpdate(
    Addr branch_pc, const StaticInstPtr &inst, bool taken,
    StatisticalCorrector::BranchInfo *bi, Addr corrTarget)
{
    int brtype = inst->isDirectCtrl() ? 0 : 2;
    if (!inst->isUncondCtrl()) {
        ++brtype;
    }

    MPP_SCThreadHistory *sh = static_cast<MPP_SCThreadHistory *>(scHistory);

    if (brtype & 1) {
        sh->globalHist = (sh->globalHist << 1) + taken;
        sh->updateLocalHistory(2, branch_pc, taken,
                               (branch_pc ^ (branch_pc >> 4)) & 15);
        sh->updateLocalHistory(3, branch_pc, taken);
    }
    sh->updateHistoryStack(corrTarget, taken, inst->isCall(),
                           inst->isReturn());

    StatisticalCorrector::scHistoryUpdate(branch_pc, inst, taken, bi,
                                          corrTarget);
}

size_t
MPP_StatisticalCorrector_64KB::getSizeInBits() const
{
    size_t bits = 16; // global histories

    bits += (1 << logSizeUp) * pUpdateThresholdWidth;

    bits += scCountersWidth * 2 * (1 << logBias); // 2 bias arrays

    bits += (gnb - 2) * (1 << logGnb) * (scCountersWidth - 1) +
            (1 << (logGnb - 1)) * (2 * scCountersWidth - 1);

    bits += (pnb - 2) * (1 << logPnb) * (scCountersWidth - 1) +
            (1 << (logPnb - 1)) * (2 * scCountersWidth - 1);

    bits += (lnb - 2) * (1 << logLnb) * (scCountersWidth - 1) +
            (1 << (logLnb - 1)) * (2 * scCountersWidth - 1);

    bits += numEntriesFirstLocalHistories * lm[0];

    bits += (snb - 2) * (1 << logSnb) * (scCountersWidth - 1) +
            (1 << (logSnb - 1)) * (2 * scCountersWidth - 1);

    bits += numEntriesSecondLocalHistories * sm[0];

    bits += (tnb - 2) * (1 << logTnb) * (scCountersWidth - 1) +
            (1 << (logTnb - 1)) * (2 * scCountersWidth - 1);

    /* tm[0] is artificially increased by 11 to accomodate IMLI */
    bits += numEntriesThirdLocalHistories * (tm[0] - 11);

    bits += 16 * 16; // History stack
    bits += 4;       // History stack pointer

    bits += 3 * chooserConfWidth; // 3 chooser counters

    return bits;
}

MultiperspectivePerceptronTAGE64KB::MultiperspectivePerceptronTAGE64KB(
    const MultiperspectivePerceptronTAGE64KBParams &p)
    : MultiperspectivePerceptronTAGE(p)
{}

void
MultiperspectivePerceptronTAGE64KB::createSpecs()
{
    addSpec(new BLURRYPATH(5, 15, -1, 2.25, 0, 6, *this));
    addSpec(new BLURRYPATH(8, 10, -1, 2.25, 0, 6, *this));
    addSpec(new RECENCYPOS(31, 3.5, 0, 6, *this));
    addSpec(new GHISTMODPATH(3, 7, 1, 2.24, 0, 6, *this));
    addSpec(new MODPATH(3, 20, 3, 2.24, 0, 6, *this));
    addSpec(new IMLI(1, 2.23, 0, 6, *this));
    addSpec(new IMLI(4, 1.98, 0, 6, *this));
    addSpec(new RECENCY(9, 3, -1, 2.51, 0, 6, *this));
    addSpec(new ACYCLIC(12, -1, -1, 2.0, 0, 6, *this));
}

} // namespace branch_prediction
} // namespace gem5
