/*
 * Copyright (c) 2018 Metempsy Technology Consulting
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
 * TAGE-SC-L branch predictor base class (devised by Andre Seznec)
 * It consits of a TAGE + a statistical corrector (SC) + a loop predictor (L)
 */

#include "cpu/pred/tage_sc_l.hh"

#include "base/random.hh"
#include "debug/TageSCL.hh"

bool
TAGE_SC_L_LoopPredictor::calcConf(int index) const
{
    return LoopPredictor::calcConf(index) ||
           (ltable[index].confidence * ltable[index].numIter > 128);
}

bool
TAGE_SC_L_LoopPredictor::optionalAgeInc() const
{
    return (random_mt.random<int>() & 7) == 0;
}

TAGE_SC_L_LoopPredictor *
TAGE_SC_L_LoopPredictorParams::create()
{
    return new TAGE_SC_L_LoopPredictor(this);
}

TAGE_SC_L::TAGE_SC_L(const TAGE_SC_LParams *p)
  : LTAGE(p), statisticalCorrector(p->statistical_corrector)
{
}

TAGEBase::BranchInfo*
TAGE_SC_L_TAGE::makeBranchInfo()
{
    return new BranchInfo(*this);
}
void
TAGE_SC_L_TAGE::calculateParameters()
{
    unsigned numHistLengths = nHistoryTables/2;
    histLengths[1] = minHist;
    histLengths[numHistLengths] = maxHist;

    // This calculates the different history lenghts
    // there are only numHistLengths different lengths
    // They are initially set to the lower half of histLengths
    for (int i = 2; i <= numHistLengths; i++) {
        histLengths[i] = (int) (((double) minHist *
                       pow ((double) (maxHist) / (double) minHist,
                           (double) (i - 1) / (double) ((numHistLengths - 1))))
                       + 0.5);
    }

    // This copies and duplicates the values from the lower half of the table
    // Ex: 4, 6, 9, 13 would get exanded to 4, 4, 6, 6, 9, 9, 13, 13
    for (int i = nHistoryTables; i > 1; i--)
    {
        histLengths[i] = histLengths[(i + 1) / 2];
    }

    for (int i = 1; i <= nHistoryTables; i++)
    {
        tagTableTagWidths.push_back(
            (i < firstLongTagTable) ? shortTagsSize : longTagsSize);

        logTagTableSizes.push_back(logTagTableSize);
    }
}

void
TAGE_SC_L_TAGE::buildTageTables()
{
    // Trick! We only allocate entries for tables 1 and firstLongTagTable and
    // make the other tables point to these allocated entries

    gtable[1] = new TageEntry[shortTagsTageFactor * (1 << logTagTableSize)];
    gtable[firstLongTagTable] =
        new TageEntry[longTagsTageFactor * (1 << logTagTableSize)];
    for (int i = 2; i < firstLongTagTable; ++i) {
        gtable[i] = gtable[1];
    }
    for (int i = firstLongTagTable + 1; i <= nHistoryTables; ++i) {
        gtable[i] = gtable[firstLongTagTable];
    }
}

void
TAGE_SC_L_TAGE::calculateIndicesAndTags(
    ThreadID tid, Addr pc, TAGEBase::BranchInfo* bi)
{
    // computes the table addresses and the partial tags

    for (int i = 1; i <= nHistoryTables; i += 2) {
        tableIndices[i] = gindex(tid, pc, i);
        tableTags[i] = gtag(tid, pc, i);
        tableTags[i + 1] = tableTags[i];
        tableIndices[i + 1] = tableIndices[i] ^
                             (tableTags[i] & ((1 << logTagTableSizes[i]) - 1));

        bi->tableTags[i] = tableTags[i];
        bi->tableTags[i+1] = tableTags[i+1];
    }

    Addr t = (pc ^ (threadHistory[tid].pathHist &
                    ((1 << histLengths[firstLongTagTable]) - 1)))
             % longTagsTageFactor;

    for (int i = firstLongTagTable; i <= nHistoryTables; i++) {
        if (noSkip[i]) {
            tableIndices[i] += (t << logTagTableSizes[i]);
            bi->tableIndices[i] = tableIndices[i];
            t++;
            t = t % longTagsTageFactor;
        }
    }

    t = (pc ^ (threadHistory[tid].pathHist & ((1 << histLengths[1]) - 1)))
        % shortTagsTageFactor;

    for (int i = 1; i <= firstLongTagTable - 1; i++) {
        if (noSkip[i]) {
            tableIndices[i] += (t << logTagTableSizes[i]);
            bi->tableIndices[i] = tableIndices[i];
            t++;
            t = t % shortTagsTageFactor;
        }
    }
}

unsigned
TAGE_SC_L_TAGE::getUseAltIdx(TAGEBase::BranchInfo* bi)
{
    BranchInfo *tbi = static_cast<BranchInfo *>(bi);
    unsigned idx;
    idx = ((((bi->hitBank-1)/8)<<1)+tbi->altConf) % (numUseAltOnNa-1);
    return idx;
}

int
TAGE_SC_L_TAGE::gindex(ThreadID tid, Addr pc, int bank) const
{
    int index;
    int hlen = (histLengths[bank] > pathHistBits) ? pathHistBits :
                                                    histLengths[bank];
    unsigned int shortPc = pc;

    // pc is not shifted by instShiftAmt in this implementation
    index = shortPc ^
            (shortPc >> ((int) abs(logTagTableSizes[bank] - bank) + 1)) ^
            threadHistory[tid].computeIndices[bank].comp ^
            F(threadHistory[tid].pathHist, hlen, bank);

    index = gindex_ext(index, bank);

    return (index & ((ULL(1) << (logTagTableSizes[bank])) - 1));
}

int
TAGE_SC_L_TAGE::F(int a, int size, int bank) const
{
    int a1, a2;

    a = a & ((ULL(1) << size) - 1);
    a1 = (a & ((ULL(1) << logTagTableSizes[bank]) - 1));
    a2 = (a >> logTagTableSizes[bank]);

    if (bank < logTagTableSizes[bank]) {
        a2 = ((a2 << bank) & ((ULL(1) << logTagTableSizes[bank]) - 1))
             + (a2 >> (logTagTableSizes[bank] - bank));
    }

    a = a1 ^ a2;

    if (bank < logTagTableSizes[bank]) {
        a = ((a << bank) & ((ULL(1) << logTagTableSizes[bank]) - 1))
            + (a >> (logTagTableSizes[bank] - bank));
    }

    return a;
}

int
TAGE_SC_L_TAGE::bindex(Addr pc) const
{
    return ((pc ^ (pc >> instShiftAmt)) &
            ((ULL(1) << (logTagTableSizes[0])) - 1));
}

void
TAGE_SC_L_TAGE::updatePathAndGlobalHistory(
    ThreadHistory& tHist, int brtype, bool taken, Addr branch_pc, Addr target)
{
    // TAGE update
    int tmp = ((branch_pc ^ (branch_pc >> instShiftAmt))) ^ taken;
    int path = branch_pc ^ (branch_pc >> instShiftAmt)
                         ^ (branch_pc >> (instShiftAmt+2));
    if ((brtype == 3) & taken) {
         tmp = (tmp ^ (target >> instShiftAmt));
         path = path ^ (target >> instShiftAmt) ^ (target >> (instShiftAmt+2));
    }

    // some branch types use 3 bits in global history, the others just 2
    int maxt = (brtype == 2) ? 3 : 2;

    for (int t = 0; t < maxt; t++) {
        bool dir = (tmp & 1);
        tmp >>= 1;
        int pathbit = (path & 127);
        path >>= 1;
        updateGHist(tHist.gHist, dir, tHist.globalHistory, tHist.ptGhist);
        tHist.pathHist = (tHist.pathHist << 1) ^ pathbit;
        if (truncatePathHist) {
            // The 8KB implementation does not do this truncation
            tHist.pathHist = (tHist.pathHist & ((ULL(1) << pathHistBits) - 1));
        }
        for (int i = 1; i <= nHistoryTables; i++) {
            tHist.computeIndices[i].update(tHist.gHist);
            tHist.computeTags[0][i].update(tHist.gHist);
            tHist.computeTags[1][i].update(tHist.gHist);
        }
    }
}

void
TAGE_SC_L_TAGE::updateHistories(
    ThreadID tid, Addr branch_pc, bool taken, TAGEBase::BranchInfo* b,
    bool speculative, const StaticInstPtr &inst, Addr target)
{
    if (speculative != speculativeHistUpdate) {
        return;
    }
    // speculation is not implemented
    assert(! speculative);

    ThreadHistory& tHist = threadHistory[tid];

    int brtype = inst->isDirectCtrl() ? 0 : 2;
    if (! inst->isUncondCtrl()) {
        ++brtype;
    }
    updatePathAndGlobalHistory(tHist, brtype, taken, branch_pc, target);

    DPRINTF(TageSCL, "Updating global histories with branch:%lx; taken?:%d, "
            "path Hist: %x; pointer:%d\n", branch_pc, taken, tHist.pathHist,
            tHist.ptGhist);
}

void
TAGE_SC_L_TAGE::squash(ThreadID tid, bool taken, TAGEBase::BranchInfo *bi,
                       Addr target)
{
    fatal("Speculation is not implemented");
}

void
TAGE_SC_L_TAGE::adjustAlloc(bool & alloc, bool taken, bool pred_taken)
{
    // Do not allocate too often if the prediction is ok
    if ((taken == pred_taken) && ((random_mt.random<int>() & 31) != 0)) {
        alloc = false;
    }
}

int
TAGE_SC_L_TAGE::calcDep(TAGEBase::BranchInfo* bi)
{
    int a = 1;
    if ((random_mt.random<int>() & 127) < 32) {
        a = 2;
    }
    return ((((bi->hitBank - 1 + 2 * a) & 0xffe)) ^
            (random_mt.random<int>() & 1));
}

void
TAGE_SC_L_TAGE::handleUReset()
{
    //just the best formula for the Championship:
    //In practice when one out of two entries are useful
    if (tCounter < 0) {
        tCounter = 0;
    }

    if (tCounter >= ((ULL(1) << logUResetPeriod))) {
        // Update the u bits for the short tags table
        for (int j = 0; j < (shortTagsTageFactor*(1<<logTagTableSize)); j++) {
            resetUctr(gtable[1][j].u);
        }

        // Update the u bits for the long tags table
        for (int j = 0; j < (longTagsTageFactor*(1<<logTagTableSize)); j++) {
            resetUctr(gtable[firstLongTagTable][j].u);
        }

        tCounter = 0;
    }
}

bool
TAGE_SC_L_TAGE::getBimodePred(Addr pc, TAGEBase::BranchInfo* tage_bi) const
{
    TAGE_SC_L_TAGE::BranchInfo *bi =
        static_cast<TAGE_SC_L_TAGE::BranchInfo *>(tage_bi);

    int bim = (btablePrediction[bi->bimodalIndex] << 1)
        + btableHysteresis[bi->bimodalIndex >> logRatioBiModalHystEntries];

    bi->highConf = (bim == 0) || (bim == 3);
    bi->lowConf = ! bi->highConf;
    bi->altConf = bi->highConf;
    bi->medConf = false;
    return TAGEBase::getBimodePred(pc, tage_bi);
}

void
TAGE_SC_L_TAGE::extraAltCalc(TAGEBase::BranchInfo* bi)
{
    TAGE_SC_L_TAGE::BranchInfo *tage_scl_bi =
        static_cast<TAGE_SC_L_TAGE::BranchInfo *>(bi);
    int8_t ctr = gtable[bi->altBank][bi->altBankIndex].ctr;
    tage_scl_bi->altConf = (abs(2*ctr + 1) > 1);
}

bool
TAGE_SC_L::predict(ThreadID tid, Addr branch_pc, bool cond_branch, void* &b)
{
    TageSCLBranchInfo *bi = new TageSCLBranchInfo(*tage,
                                                  *statisticalCorrector,
                                                  *loopPredictor);
    b = (void*)(bi);

    bool pred_taken = tage->tagePredict(tid, branch_pc, cond_branch,
                                        bi->tageBranchInfo);
    pred_taken = loopPredictor->loopPredict(tid, branch_pc, cond_branch,
                                            bi->lpBranchInfo, pred_taken,
                                            instShiftAmt);

    if (bi->lpBranchInfo->loopPredUsed) {
        bi->tageBranchInfo->provider = LOOP;
    }

    TAGE_SC_L_TAGE::BranchInfo* tage_scl_bi =
        static_cast<TAGE_SC_L_TAGE::BranchInfo *>(bi->tageBranchInfo);

    // Copy the confidences computed by TAGE
    bi->scBranchInfo->lowConf = tage_scl_bi->lowConf;
    bi->scBranchInfo->highConf = tage_scl_bi->highConf;
    bi->scBranchInfo->altConf = tage_scl_bi->altConf;
    bi->scBranchInfo->medConf = tage_scl_bi->medConf;

    bool use_tage_ctr = bi->tageBranchInfo->hitBank > 0;
    int8_t tage_ctr = use_tage_ctr ?
        tage->getCtr(tage_scl_bi->hitBank, tage_scl_bi->hitBankIndex) : 0;
    bool bias = (bi->tageBranchInfo->longestMatchPred !=
                 bi->tageBranchInfo->altTaken);

    pred_taken = statisticalCorrector->scPredict(tid, branch_pc, cond_branch,
            bi->scBranchInfo, pred_taken, bias, use_tage_ctr, tage_ctr,
            tage->getTageCtrBits(), bi->tageBranchInfo->hitBank,
            bi->tageBranchInfo->altBank, tage->getPathHist(tid));

    if (bi->scBranchInfo->usedScPred) {
        bi->tageBranchInfo->provider = SC;
    }

    // record final prediction
    bi->lpBranchInfo->predTaken = pred_taken;

    return pred_taken;
}

void
TAGE_SC_L::update(ThreadID tid, Addr branch_pc, bool taken, void *bp_history,
        bool squashed, const StaticInstPtr & inst, Addr corrTarget)
{
    assert(bp_history);

    TageSCLBranchInfo* bi = static_cast<TageSCLBranchInfo*>(bp_history);
    TAGE_SC_L_TAGE::BranchInfo* tage_bi =
        static_cast<TAGE_SC_L_TAGE::BranchInfo *>(bi->tageBranchInfo);

    assert(corrTarget != MaxAddr);

    if (squashed) {
        if (tage->isSpeculativeUpdateEnabled()) {
            // This restores the global history, then update it
            // and recomputes the folded histories.
            tage->squash(tid, taken, tage_bi, corrTarget);
            if (bi->tageBranchInfo->condBranch) {
                loopPredictor->squashLoop(bi->lpBranchInfo);
            }
        }
        return;
    }

    int nrand = random_mt.random<int>() & 3;
    if (tage_bi->condBranch) {
        DPRINTF(TageSCL, "Updating tables for branch:%lx; taken?:%d\n",
                branch_pc, taken);
        tage->updateStats(taken, bi->tageBranchInfo);

        loopPredictor->updateStats(taken, bi->lpBranchInfo);

        statisticalCorrector->updateStats(taken, bi->scBranchInfo);

        bool bias = (bi->tageBranchInfo->longestMatchPred !=
                     bi->tageBranchInfo->altTaken);
        statisticalCorrector->condBranchUpdate(tid, branch_pc, taken,
            bi->scBranchInfo, corrTarget, bias, bi->tageBranchInfo->hitBank,
            bi->tageBranchInfo->altBank, tage->getPathHist(tid));

        loopPredictor->condBranchUpdate(tid, branch_pc, taken,
                bi->tageBranchInfo->tagePred, bi->lpBranchInfo, instShiftAmt);

        tage->condBranchUpdate(tid, branch_pc, taken, bi->tageBranchInfo,
                               nrand, corrTarget, bi->lpBranchInfo->predTaken);
    }

    if (!tage->isSpeculativeUpdateEnabled()) {
        int brtype = inst->isDirectCtrl() ? 0 : 2;
        if (! inst->isUncondCtrl()) {
            ++brtype;
        }

        statisticalCorrector->scHistoryUpdate(branch_pc, brtype, taken,
                                              bi->scBranchInfo, corrTarget);

        tage->updateHistories(tid, branch_pc, taken, bi->tageBranchInfo, false,
                              inst, corrTarget);
    }

    delete bi;
}

void
TAGE_SC_L::regStats()
{
    LTAGE::regStats();
}
