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
 * 8KB TAGE-SC-L branch predictor (devised by Andre Seznec)
 */

#include "cpu/pred/tage_sc_l_8KB.hh"

#include "base/random.hh"
#include "debug/TageSCL.hh"

TAGE_SC_L_8KB_StatisticalCorrector::TAGE_SC_L_8KB_StatisticalCorrector(
    TAGE_SC_L_8KB_StatisticalCorrectorParams *p)
  : StatisticalCorrector(p),
    gnb(p->gnb),
    logGnb(p->logGnb),
    gm(p->gm)
{
    initGEHLTable(gnb, gm, ggehl, logGnb, wg, 7);
}

TAGE_SC_L_8KB_StatisticalCorrector::SCThreadHistory *
TAGE_SC_L_8KB_StatisticalCorrector::makeThreadHistory()
{
    SC_8KB_ThreadHistory *sh = new SC_8KB_ThreadHistory();
    sh->setNumOrdinalHistories(1);
    sh->initLocalHistory(1, numEntriesFirstLocalHistories, 2);
    return sh;
}

unsigned
TAGE_SC_L_8KB_StatisticalCorrector::getIndBiasBank(Addr branch_pc,
        BranchInfo* bi, int hitBank, int altBank) const
{
    return (bi->predBeforeSC + (((hitBank+1)/4)<<4) + (bi->highConf<<1) +
            (bi->lowConf <<2) +((altBank!=0)<<3)) & ((1<<logBias) -1);
}

int
TAGE_SC_L_8KB_StatisticalCorrector::gPredictions(
    ThreadID tid, Addr branch_pc, BranchInfo* bi, int & lsum, int64_t phist)
{
    SC_8KB_ThreadHistory *sh = static_cast<SC_8KB_ThreadHistory *>(scHistory);
    lsum += gPredict(
        branch_pc, sh->globalHist, gm, ggehl, gnb, logGnb, wg);

    lsum += gPredict(
        branch_pc, sh->bwHist, bwm, bwgehl, bwnb, logBwnb, wbw);

    // only 1 local history here
    lsum += gPredict(
        branch_pc, sh->getLocalHistory(1, branch_pc), lm,
        lgehl, lnb, logLnb, wl);

    lsum += gPredict(
        branch_pc, sh->imliCount, im, igehl, inb, logInb, wi);

    int thres = (updateThreshold>>3)+pUpdateThreshold[getIndUpd(branch_pc)];

    return thres;
}

int TAGE_SC_L_8KB_StatisticalCorrector::gIndexLogsSubstr(int nbr, int i)
{
    return 0;
}

void
TAGE_SC_L_8KB_StatisticalCorrector::scHistoryUpdate(Addr branch_pc, int brtype,
    bool taken, BranchInfo * tage_bi, Addr corrTarget)
{
    // Non speculative SC histories update
    if (brtype & 1) {
        SC_8KB_ThreadHistory *sh =
            static_cast<SC_8KB_ThreadHistory *>(scHistory);
        sh->globalHist = (sh->globalHist << 1) + taken;
    }

    StatisticalCorrector::scHistoryUpdate(branch_pc, brtype, taken, tage_bi,
                                          corrTarget);
}

void
TAGE_SC_L_8KB_StatisticalCorrector::gUpdates(ThreadID tid, Addr pc, bool taken,
        BranchInfo* bi, int64_t phist)
{
    SC_8KB_ThreadHistory *sh = static_cast<SC_8KB_ThreadHistory *>(scHistory);
    gUpdate(pc, taken, sh->globalHist, gm, ggehl, gnb, logGnb, wg, bi);
    gUpdate(pc, taken, sh->bwHist, bwm, bwgehl, bwnb, logBwnb, wbw, bi);
    gUpdate(pc, taken, sh->getLocalHistory(1, pc), lm, lgehl, lnb, logLnb, wl,
            bi);
    gUpdate(pc, taken, sh->imliCount, im, igehl, inb, logInb, wi, bi);
}

TAGE_SC_L_8KB_StatisticalCorrector*
TAGE_SC_L_8KB_StatisticalCorrectorParams::create()
{
    return new TAGE_SC_L_8KB_StatisticalCorrector(this);
}

TAGE_SC_L_8KB::TAGE_SC_L_8KB(const TAGE_SC_L_8KBParams *params)
  : TAGE_SC_L(params)
{
}

void
TAGE_SC_L_TAGE_8KB::initFoldedHistories(ThreadHistory & history)
{
    // Some hardcoded values are used here
    // (they do not seem to depend on any parameter)
    for (int i = 1; i <= nHistoryTables; i++) {
        history.computeIndices[i].init(
            histLengths[i], 17 + (2 * ((i - 1) / 2) % 4));
        history.computeTags[0][i].init(
            history.computeIndices[i].origLength, 13);
        history.computeTags[1][i].init(
            history.computeIndices[i].origLength, 11);
        DPRINTF(TageSCL, "HistLength:%d, TTSize:%d, TTTWidth:%d\n",
                histLengths[i], logTagTableSizes[i], tagTableTagWidths[i]);
    }
}

int
TAGE_SC_L_TAGE_8KB::gindex_ext(int index, int bank) const
{
    return (index ^ (index >> logTagTableSizes[bank])
                  ^ (index >> 2 * logTagTableSizes[bank]));
}

uint16_t
TAGE_SC_L_TAGE_8KB::gtag(ThreadID tid, Addr pc, int bank) const
{
    int tag = (threadHistory[tid].computeIndices[bank - 1].comp << 2) ^ pc ^
              (pc >> instShiftAmt) ^
              threadHistory[tid].computeIndices[bank].comp;
    int hlen = (histLengths[bank] > pathHistBits) ? pathHistBits :
                                                    histLengths[bank];

    tag = (tag >> 1) ^ ((tag & 1) << 10) ^
           F(threadHistory[tid].pathHist, hlen, bank);
    tag ^= threadHistory[tid].computeTags[0][bank].comp ^
           (threadHistory[tid].computeTags[1][bank].comp << 1);

    return ((tag ^ (tag >> tagTableTagWidths[bank]))
            & ((ULL(1) << tagTableTagWidths[bank]) - 1));
}

void
TAGE_SC_L_TAGE_8KB::handleAllocAndUReset(
    bool alloc, bool taken, TAGEBase::BranchInfo* bi, int nrand)
{
    if (!alloc) {
        return;
    }

    int penalty = 0;
    int truePen = 0;
    int numAllocated = 0;
    bool maxAllocReached = false;

    for (int I = calcDep(bi); I < nHistoryTables; I += 2) {
        // Handle the 2-way associativity for allocation
        for (int j = 0; j < 2; ++j) {
            int i = ((j == 0) ? I : (I ^ 1)) + 1;
            if (i > nHistoryTables) {
                break;
            }
            if (noSkip[i]) {
                if (gtable[i][bi->tableIndices[i]].u == 0) {
                    gtable[i][bi->tableIndices[i]].u =
                        ((random_mt.random<int>() & 31) == 0);
                    // protect randomly from fast replacement
                    gtable[i][bi->tableIndices[i]].tag = bi->tableTags[i];
                    gtable[i][bi->tableIndices[i]].ctr = taken ? 0 : -1;
                    numAllocated++;

                    if (numAllocated == maxNumAlloc) {
                        maxAllocReached = true;
                        break;
                    }
                    I += 2;
                } else {
                    int8_t ctr = gtable[i][bi->tableIndices[i]].ctr;
                    if ((gtable[i][bi->tableIndices[i]].u == 1) &
                        (abs (2 * ctr + 1) == 1)) {
                        if ((random_mt.random<int>() & 7) == 0) {
                            gtable[i][bi->tableIndices[i]].u = 0;
                        }
                    } else {
                        truePen++;
                    }
                    penalty++;
                }
            } else {
                break;
            }
        }
        if (maxAllocReached) {
            break;
        }
    }

    tCounter += (truePen + penalty - 5 * numAllocated);

    handleUReset();
}

void
TAGE_SC_L_TAGE_8KB::resetUctr(uint8_t & u)
{
    // On real HW it should be u >>= 1 instead of if > 0 then u--
    if (u > 0) {
        u--;
    }
}

void
TAGE_SC_L_TAGE_8KB::handleTAGEUpdate(Addr branch_pc, bool taken,
                                     TAGEBase::BranchInfo* bi)
{
    if (bi->hitBank > 0) {
        if (abs (2 * gtable[bi->hitBank][bi->hitBankIndex].ctr + 1) == 1) {
            if (bi->longestMatchPred != taken) { // acts as a protection
                if (bi->altBank > 0) {
                    int8_t ctr = gtable[bi->altBank][bi->altBankIndex].ctr;
                    if (abs (2 * ctr + 1) == 1) {
                        gtable[bi->altBank][bi->altBankIndex].u = 0;
                    }

                    //just mute from protected to unprotected
                    ctrUpdate(gtable[bi->altBank][bi->altBankIndex].ctr, taken,
                              tagTableCounterBits);
                    ctr = gtable[bi->altBank][bi->altBankIndex].ctr;
                    if (abs (2 * ctr + 1) == 1) {
                        gtable[bi->altBank][bi->altBankIndex].u = 0;
                    }
                }
                if (bi->altBank == 0) {
                    baseUpdate(branch_pc, taken, bi);
                }
            }
        }

        //just mute from protected to unprotected
        if (abs (2 * gtable[bi->hitBank][bi->hitBankIndex].ctr + 1) == 1) {
            gtable[bi->hitBank][bi->hitBankIndex].u = 0;
        }

        ctrUpdate(gtable[bi->hitBank][bi->hitBankIndex].ctr, taken,
                  tagTableCounterBits);

        //sign changes: no way it can have been useful
        if (abs (2 * gtable[bi->hitBank][bi->hitBankIndex].ctr + 1) == 1) {
            gtable[bi->hitBank][bi->hitBankIndex].u = 0;
        }

        if (bi->altTaken == taken) {
            if (bi->altBank > 0) {
                int8_t ctr = gtable[bi->altBank][bi->altBankIndex].ctr;
                if (abs (2*ctr + 1) == 7) {
                    if (gtable[bi->hitBank][bi->hitBankIndex].u == 1) {
                        if (bi->longestMatchPred == taken) {
                            gtable[bi->hitBank][bi->hitBankIndex].u = 0;
                        }
                    }
                }
            }
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

TAGE_SC_L_TAGE_8KB*
TAGE_SC_L_TAGE_8KBParams::create()
{
    return new TAGE_SC_L_TAGE_8KB(this);
}

TAGE_SC_L_8KB*
TAGE_SC_L_8KBParams::create()
{
    return new TAGE_SC_L_8KB(this);
}
