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
 * Implementation of a TAGE branch predictor
 */

#include "cpu/pred/tage_base.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/Fetch.hh"
#include "debug/Tage.hh"

TAGEBase::TAGEBase(const TAGEBaseParams *p)
   : SimObject(p),
     logRatioBiModalHystEntries(p->logRatioBiModalHystEntries),
     nHistoryTables(p->nHistoryTables),
     tagTableCounterBits(p->tagTableCounterBits),
     tagTableUBits(p->tagTableUBits),
     histBufferSize(p->histBufferSize),
     minHist(p->minHist),
     maxHist(p->maxHist),
     pathHistBits(p->pathHistBits),
     tagTableTagWidths(p->tagTableTagWidths),
     logTagTableSizes(p->logTagTableSizes),
     threadHistory(p->numThreads),
     logUResetPeriod(p->logUResetPeriod),
     initialTCounterValue(p->initialTCounterValue),
     numUseAltOnNa(p->numUseAltOnNa),
     useAltOnNaBits(p->useAltOnNaBits),
     maxNumAlloc(p->maxNumAlloc),
     noSkip(p->noSkip),
     speculativeHistUpdate(p->speculativeHistUpdate),
     instShiftAmt(p->instShiftAmt),
     initialized(false)
{
    if (noSkip.empty()) {
        // Set all the table to enabled by default
        noSkip.resize(nHistoryTables + 1, true);
    }
}

TAGEBase::BranchInfo*
TAGEBase::makeBranchInfo() {
    return new BranchInfo(*this);
}

void
TAGEBase::init()
{
    if (initialized) {
       return;
    }
    // Current method for periodically resetting the u counter bits only
    // works for 1 or 2 bits
    // Also make sure that it is not 0
    assert(tagTableUBits <= 2 && (tagTableUBits > 0));

    // we use int type for the path history, so it cannot be more than
    // its size
    assert(pathHistBits <= (sizeof(int)*8));

    // initialize the counter to half of the period
    assert(logUResetPeriod != 0);
    tCounter = initialTCounterValue;

    assert(histBufferSize > maxHist * 2);

    useAltPredForNewlyAllocated.resize(numUseAltOnNa, 0);

    for (auto& history : threadHistory) {
        history.pathHist = 0;
        history.globalHistory = new uint8_t[histBufferSize];
        history.gHist = history.globalHistory;
        memset(history.gHist, 0, histBufferSize);
        history.ptGhist = 0;
    }

    histLengths = new int [nHistoryTables+1];

    calculateParameters();

    assert(tagTableTagWidths.size() == (nHistoryTables+1));
    assert(logTagTableSizes.size() == (nHistoryTables+1));

    // First entry is for the Bimodal table and it is untagged in this
    // implementation
    assert(tagTableTagWidths[0] == 0);

    for (auto& history : threadHistory) {
        history.computeIndices = new FoldedHistory[nHistoryTables+1];
        history.computeTags[0] = new FoldedHistory[nHistoryTables+1];
        history.computeTags[1] = new FoldedHistory[nHistoryTables+1];

        initFoldedHistories(history);
    }

    const uint64_t bimodalTableSize = ULL(1) << logTagTableSizes[0];
    btablePrediction.resize(bimodalTableSize, false);
    btableHysteresis.resize(bimodalTableSize >> logRatioBiModalHystEntries,
                            true);

    gtable = new TageEntry*[nHistoryTables + 1];
    buildTageTables();

    tableIndices = new int [nHistoryTables+1];
    tableTags = new int [nHistoryTables+1];
    initialized = true;
}

void
TAGEBase::initFoldedHistories(ThreadHistory & history)
{
    for (int i = 1; i <= nHistoryTables; i++) {
        history.computeIndices[i].init(
            histLengths[i], (logTagTableSizes[i]));
        history.computeTags[0][i].init(
            history.computeIndices[i].origLength, tagTableTagWidths[i]);
        history.computeTags[1][i].init(
            history.computeIndices[i].origLength, tagTableTagWidths[i]-1);
        DPRINTF(Tage, "HistLength:%d, TTSize:%d, TTTWidth:%d\n",
                histLengths[i], logTagTableSizes[i], tagTableTagWidths[i]);
    }
}

void
TAGEBase::buildTageTables()
{
    for (int i = 1; i <= nHistoryTables; i++) {
        gtable[i] = new TageEntry[1<<(logTagTableSizes[i])];
    }
}

void
TAGEBase::calculateParameters()
{
    histLengths[1] = minHist;
    histLengths[nHistoryTables] = maxHist;

    for (int i = 2; i <= nHistoryTables; i++) {
        histLengths[i] = (int) (((double) minHist *
                       pow ((double) (maxHist) / (double) minHist,
                           (double) (i - 1) / (double) ((nHistoryTables- 1))))
                       + 0.5);
    }
}

void
TAGEBase::btbUpdate(ThreadID tid, Addr branch_pc, BranchInfo* &bi)
{
    if (speculativeHistUpdate) {
        ThreadHistory& tHist = threadHistory[tid];
        DPRINTF(Tage, "BTB miss resets prediction: %lx\n", branch_pc);
        assert(tHist.gHist == &tHist.globalHistory[tHist.ptGhist]);
        tHist.gHist[0] = 0;
        for (int i = 1; i <= nHistoryTables; i++) {
            tHist.computeIndices[i].comp = bi->ci[i];
            tHist.computeTags[0][i].comp = bi->ct0[i];
            tHist.computeTags[1][i].comp = bi->ct1[i];
            tHist.computeIndices[i].update(tHist.gHist);
            tHist.computeTags[0][i].update(tHist.gHist);
            tHist.computeTags[1][i].update(tHist.gHist);
        }
    }
}

int
TAGEBase::bindex(Addr pc_in) const
{
    return ((pc_in >> instShiftAmt) & ((ULL(1) << (logTagTableSizes[0])) - 1));
}

int
TAGEBase::F(int A, int size, int bank) const
{
    int A1, A2;

    A = A & ((ULL(1) << size) - 1);
    A1 = (A & ((ULL(1) << logTagTableSizes[bank]) - 1));
    A2 = (A >> logTagTableSizes[bank]);
    A2 = ((A2 << bank) & ((ULL(1) << logTagTableSizes[bank]) - 1))
       + (A2 >> (logTagTableSizes[bank] - bank));
    A = A1 ^ A2;
    A = ((A << bank) & ((ULL(1) << logTagTableSizes[bank]) - 1))
      + (A >> (logTagTableSizes[bank] - bank));
    return (A);
}

// gindex computes a full hash of pc, ghist and pathHist
int
TAGEBase::gindex(ThreadID tid, Addr pc, int bank) const
{
    int index;
    int hlen = (histLengths[bank] > pathHistBits) ? pathHistBits :
                                                    histLengths[bank];
    const unsigned int shiftedPc = pc >> instShiftAmt;
    index =
        shiftedPc ^
        (shiftedPc >> ((int) abs(logTagTableSizes[bank] - bank) + 1)) ^
        threadHistory[tid].computeIndices[bank].comp ^
        F(threadHistory[tid].pathHist, hlen, bank);

    return (index & ((ULL(1) << (logTagTableSizes[bank])) - 1));
}


// Tag computation
uint16_t
TAGEBase::gtag(ThreadID tid, Addr pc, int bank) const
{
    int tag = (pc >> instShiftAmt) ^
              threadHistory[tid].computeTags[0][bank].comp ^
              (threadHistory[tid].computeTags[1][bank].comp << 1);

    return (tag & ((ULL(1) << tagTableTagWidths[bank]) - 1));
}


// Up-down saturating counter
template<typename T>
void
TAGEBase::ctrUpdate(T & ctr, bool taken, int nbits)
{
    assert(nbits <= sizeof(T) << 3);
    if (taken) {
        if (ctr < ((1 << (nbits - 1)) - 1))
            ctr++;
    } else {
        if (ctr > -(1 << (nbits - 1)))
            ctr--;
    }
}

// int8_t and int versions of this function may be needed
template void TAGEBase::ctrUpdate(int8_t & ctr, bool taken, int nbits);
template void TAGEBase::ctrUpdate(int & ctr, bool taken, int nbits);

// Up-down unsigned saturating counter
void
TAGEBase::unsignedCtrUpdate(uint8_t & ctr, bool up, unsigned nbits)
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

// Bimodal prediction
bool
TAGEBase::getBimodePred(Addr pc, BranchInfo* bi) const
{
    return btablePrediction[bi->bimodalIndex];
}


// Update the bimodal predictor: a hysteresis bit is shared among N prediction
// bits (N = 2 ^ logRatioBiModalHystEntries)
void
TAGEBase::baseUpdate(Addr pc, bool taken, BranchInfo* bi)
{
    int inter = (btablePrediction[bi->bimodalIndex] << 1)
        + btableHysteresis[bi->bimodalIndex >> logRatioBiModalHystEntries];
    if (taken) {
        if (inter < 3)
            inter++;
    } else if (inter > 0) {
        inter--;
    }
    const bool pred = inter >> 1;
    const bool hyst = inter & 1;
    btablePrediction[bi->bimodalIndex] = pred;
    btableHysteresis[bi->bimodalIndex >> logRatioBiModalHystEntries] = hyst;
    DPRINTF(Tage, "Updating branch %lx, pred:%d, hyst:%d\n", pc, pred, hyst);
}

// shifting the global history:  we manage the history in a big table in order
// to reduce simulation time
void
TAGEBase::updateGHist(uint8_t * &h, bool dir, uint8_t * tab, int &pt)
{
    if (pt == 0) {
        DPRINTF(Tage, "Rolling over the histories\n");
         // Copy beginning of globalHistoryBuffer to end, such that
         // the last maxHist outcomes are still reachable
         // through pt[0 .. maxHist - 1].
         for (int i = 0; i < maxHist; i++)
             tab[histBufferSize - maxHist + i] = tab[i];
         pt =  histBufferSize - maxHist;
         h = &tab[pt];
    }
    pt--;
    h--;
    h[0] = (dir) ? 1 : 0;
}

void
TAGEBase::calculateIndicesAndTags(ThreadID tid, Addr branch_pc,
                                  BranchInfo* bi)
{
    // computes the table addresses and the partial tags
    for (int i = 1; i <= nHistoryTables; i++) {
        tableIndices[i] = gindex(tid, branch_pc, i);
        bi->tableIndices[i] = tableIndices[i];
        tableTags[i] = gtag(tid, branch_pc, i);
        bi->tableTags[i] = tableTags[i];
    }
}

unsigned
TAGEBase::getUseAltIdx(BranchInfo* bi, Addr branch_pc)
{
    // There is only 1 counter on the base TAGE implementation
    return 0;
}

bool
TAGEBase::tagePredict(ThreadID tid, Addr branch_pc,
              bool cond_branch, BranchInfo* bi)
{
    Addr pc = branch_pc;
    bool pred_taken = true;

    if (cond_branch) {
        // TAGE prediction

        calculateIndicesAndTags(tid, pc, bi);

        bi->bimodalIndex = bindex(pc);

        bi->hitBank = 0;
        bi->altBank = 0;
        //Look for the bank with longest matching history
        for (int i = nHistoryTables; i > 0; i--) {
            if (noSkip[i] &&
                gtable[i][tableIndices[i]].tag == tableTags[i]) {
                bi->hitBank = i;
                bi->hitBankIndex = tableIndices[bi->hitBank];
                break;
            }
        }
        //Look for the alternate bank
        for (int i = bi->hitBank - 1; i > 0; i--) {
            if (noSkip[i] &&
                gtable[i][tableIndices[i]].tag == tableTags[i]) {
                bi->altBank = i;
                bi->altBankIndex = tableIndices[bi->altBank];
                break;
            }
        }
        //computes the prediction and the alternate prediction
        if (bi->hitBank > 0) {
            if (bi->altBank > 0) {
                bi->altTaken =
                    gtable[bi->altBank][tableIndices[bi->altBank]].ctr >= 0;
                extraAltCalc(bi);
            }else {
                bi->altTaken = getBimodePred(pc, bi);
            }

            bi->longestMatchPred =
                gtable[bi->hitBank][tableIndices[bi->hitBank]].ctr >= 0;
            bi->pseudoNewAlloc =
                abs(2 * gtable[bi->hitBank][bi->hitBankIndex].ctr + 1) <= 1;

            //if the entry is recognized as a newly allocated entry and
            //useAltPredForNewlyAllocated is positive use the alternate
            //prediction
            if ((useAltPredForNewlyAllocated[getUseAltIdx(bi, branch_pc)] < 0)
                || ! bi->pseudoNewAlloc) {
                bi->tagePred = bi->longestMatchPred;
                bi->provider = TAGE_LONGEST_MATCH;
            } else {
                bi->tagePred = bi->altTaken;
                bi->provider = bi->altBank ? TAGE_ALT_MATCH
                                           : BIMODAL_ALT_MATCH;
            }
        } else {
            bi->altTaken = getBimodePred(pc, bi);
            bi->tagePred = bi->altTaken;
            bi->longestMatchPred = bi->altTaken;
            bi->provider = BIMODAL_ONLY;
        }
        //end TAGE prediction

        pred_taken = (bi->tagePred);
        DPRINTF(Tage, "Predict for %lx: taken?:%d, tagePred:%d, altPred:%d\n",
                branch_pc, pred_taken, bi->tagePred, bi->altTaken);
    }
    bi->branchPC = branch_pc;
    bi->condBranch = cond_branch;
    return pred_taken;
}

void
TAGEBase::adjustAlloc(bool & alloc, bool taken, bool pred_taken)
{
    // Nothing for this base class implementation
}

void
TAGEBase::handleAllocAndUReset(bool alloc, bool taken, BranchInfo* bi,
                           int nrand)
{
    if (alloc) {
        // is there some "unuseful" entry to allocate
        uint8_t min = 1;
        for (int i = nHistoryTables; i > bi->hitBank; i--) {
            if (gtable[i][bi->tableIndices[i]].u < min) {
                min = gtable[i][bi->tableIndices[i]].u;
            }
        }

        // we allocate an entry with a longer history
        // to  avoid ping-pong, we do not choose systematically the next
        // entry, but among the 3 next entries
        int Y = nrand &
            ((ULL(1) << (nHistoryTables - bi->hitBank - 1)) - 1);
        int X = bi->hitBank + 1;
        if (Y & 1) {
            X++;
            if (Y & 2)
                X++;
        }
        // No entry available, forces one to be available
        if (min > 0) {
            gtable[X][bi->tableIndices[X]].u = 0;
        }


        //Allocate entries
        unsigned numAllocated = 0;
        for (int i = X; i <= nHistoryTables; i++) {
            if ((gtable[i][bi->tableIndices[i]].u == 0)) {
                gtable[i][bi->tableIndices[i]].tag = bi->tableTags[i];
                gtable[i][bi->tableIndices[i]].ctr = (taken) ? 0 : -1;
                ++numAllocated;
                if (numAllocated == maxNumAlloc) {
                    break;
                }
            }
        }
    }

    tCounter++;

    handleUReset();
}

void
TAGEBase::handleUReset()
{
    //periodic reset of u: reset is not complete but bit by bit
    if ((tCounter & ((ULL(1) << logUResetPeriod) - 1)) == 0) {
        // reset least significant bit
        // most significant bit becomes least significant bit
        for (int i = 1; i <= nHistoryTables; i++) {
            for (int j = 0; j < (ULL(1) << logTagTableSizes[i]); j++) {
                resetUctr(gtable[i][j].u);
            }
        }
    }
}

void
TAGEBase::resetUctr(uint8_t & u)
{
    u >>= 1;
}

void
TAGEBase::condBranchUpdate(ThreadID tid, Addr branch_pc, bool taken,
    BranchInfo* bi, int nrand, Addr corrTarget, bool pred, bool preAdjustAlloc)
{
    // TAGE UPDATE
    // try to allocate a  new entries only if prediction was wrong
    bool alloc = (bi->tagePred != taken) && (bi->hitBank < nHistoryTables);

    if (preAdjustAlloc) {
        adjustAlloc(alloc, taken, pred);
    }

    if (bi->hitBank > 0) {
        // Manage the selection between longest matching and alternate
        // matching for "pseudo"-newly allocated longest matching entry
        bool PseudoNewAlloc = bi->pseudoNewAlloc;
        // an entry is considered as newly allocated if its prediction
        // counter is weak
        if (PseudoNewAlloc) {
            if (bi->longestMatchPred == taken) {
                alloc = false;
            }
            // if it was delivering the correct prediction, no need to
            // allocate new entry even if the overall prediction was false
            if (bi->longestMatchPred != bi->altTaken) {
                ctrUpdate(
                    useAltPredForNewlyAllocated[getUseAltIdx(bi, branch_pc)],
                    bi->altTaken == taken, useAltOnNaBits);
            }
        }
    }

    if (!preAdjustAlloc) {
        adjustAlloc(alloc, taken, pred);
    }

    handleAllocAndUReset(alloc, taken, bi, nrand);

    handleTAGEUpdate(branch_pc, taken, bi);
}

void
TAGEBase::handleTAGEUpdate(Addr branch_pc, bool taken, BranchInfo* bi)
{
    if (bi->hitBank > 0) {
        DPRINTF(Tage, "Updating tag table entry (%d,%d) for branch %lx\n",
                bi->hitBank, bi->hitBankIndex, branch_pc);
        ctrUpdate(gtable[bi->hitBank][bi->hitBankIndex].ctr, taken,
                  tagTableCounterBits);
        // if the provider entry is not certified to be useful also update
        // the alternate prediction
        if (gtable[bi->hitBank][bi->hitBankIndex].u == 0) {
            if (bi->altBank > 0) {
                ctrUpdate(gtable[bi->altBank][bi->altBankIndex].ctr, taken,
                          tagTableCounterBits);
                DPRINTF(Tage, "Updating tag table entry (%d,%d) for"
                        " branch %lx\n", bi->hitBank, bi->hitBankIndex,
                        branch_pc);
            }
            if (bi->altBank == 0) {
                baseUpdate(branch_pc, taken, bi);
            }
        }

        // update the u counter
        if (bi->tagePred != bi->altTaken) {
            unsignedCtrUpdate(gtable[bi->hitBank][bi->hitBankIndex].u,
                              bi->tagePred == taken, tagTableUBits);
        }
    } else {
        baseUpdate(branch_pc, taken, bi);
    }
}

void
TAGEBase::updateHistories(ThreadID tid, Addr branch_pc, bool taken,
                          BranchInfo* bi, bool speculative,
                          const StaticInstPtr &inst, Addr target)
{
    if (speculative != speculativeHistUpdate) {
        return;
    }
    ThreadHistory& tHist = threadHistory[tid];
    //  UPDATE HISTORIES
    bool pathbit = ((branch_pc >> instShiftAmt) & 1);
    //on a squash, return pointers to this and recompute indices.
    //update user history
    updateGHist(tHist.gHist, taken, tHist.globalHistory, tHist.ptGhist);
    tHist.pathHist = (tHist.pathHist << 1) + pathbit;
    tHist.pathHist = (tHist.pathHist & ((ULL(1) << pathHistBits) - 1));

    if (speculative) {
        bi->ptGhist = tHist.ptGhist;
        bi->pathHist = tHist.pathHist;
    }

    //prepare next index and tag computations for user branchs
    for (int i = 1; i <= nHistoryTables; i++)
    {
        if (speculative) {
            bi->ci[i]  = tHist.computeIndices[i].comp;
            bi->ct0[i] = tHist.computeTags[0][i].comp;
            bi->ct1[i] = tHist.computeTags[1][i].comp;
        }
        tHist.computeIndices[i].update(tHist.gHist);
        tHist.computeTags[0][i].update(tHist.gHist);
        tHist.computeTags[1][i].update(tHist.gHist);
    }
    DPRINTF(Tage, "Updating global histories with branch:%lx; taken?:%d, "
            "path Hist: %x; pointer:%d\n", branch_pc, taken, tHist.pathHist,
            tHist.ptGhist);
    assert(threadHistory[tid].gHist ==
            &threadHistory[tid].globalHistory[threadHistory[tid].ptGhist]);
}

void
TAGEBase::squash(ThreadID tid, bool taken, TAGEBase::BranchInfo *bi,
                 Addr target)
{
    if (!speculativeHistUpdate) {
        /* If there are no speculative updates, no actions are needed */
        return;
    }

    ThreadHistory& tHist = threadHistory[tid];
    DPRINTF(Tage, "Restoring branch info: %lx; taken? %d; PathHistory:%x, "
            "pointer:%d\n", bi->branchPC,taken, bi->pathHist, bi->ptGhist);
    tHist.pathHist = bi->pathHist;
    tHist.ptGhist = bi->ptGhist;
    tHist.gHist = &(tHist.globalHistory[tHist.ptGhist]);
    tHist.gHist[0] = (taken ? 1 : 0);
    for (int i = 1; i <= nHistoryTables; i++) {
        tHist.computeIndices[i].comp = bi->ci[i];
        tHist.computeTags[0][i].comp = bi->ct0[i];
        tHist.computeTags[1][i].comp = bi->ct1[i];
        tHist.computeIndices[i].update(tHist.gHist);
        tHist.computeTags[0][i].update(tHist.gHist);
        tHist.computeTags[1][i].update(tHist.gHist);
    }
}

void
TAGEBase::extraAltCalc(BranchInfo* bi)
{
    // do nothing. This is only used in some derived classes
    return;
}

void
TAGEBase::updateStats(bool taken, BranchInfo* bi)
{
    if (taken == bi->tagePred) {
        // correct prediction
        switch (bi->provider) {
          case BIMODAL_ONLY: tageBimodalProviderCorrect++; break;
          case TAGE_LONGEST_MATCH: tageLongestMatchProviderCorrect++; break;
          case BIMODAL_ALT_MATCH: bimodalAltMatchProviderCorrect++; break;
          case TAGE_ALT_MATCH: tageAltMatchProviderCorrect++; break;
        }
    } else {
        // wrong prediction
        switch (bi->provider) {
          case BIMODAL_ONLY: tageBimodalProviderWrong++; break;
          case TAGE_LONGEST_MATCH:
            tageLongestMatchProviderWrong++;
            if (bi->altTaken == taken) {
                tageAltMatchProviderWouldHaveHit++;
            }
            break;
          case BIMODAL_ALT_MATCH:
            bimodalAltMatchProviderWrong++;
            break;
          case TAGE_ALT_MATCH:
            tageAltMatchProviderWrong++;
            break;
        }

        switch (bi->provider) {
          case BIMODAL_ALT_MATCH:
          case TAGE_ALT_MATCH:
            if (bi->longestMatchPred == taken) {
                tageLongestMatchProviderWouldHaveHit++;
            }
        }
    }

    switch (bi->provider) {
      case TAGE_LONGEST_MATCH:
      case TAGE_ALT_MATCH:
        tageLongestMatchProvider[bi->hitBank]++;
        tageAltMatchProvider[bi->altBank]++;
        break;
    }
}

unsigned
TAGEBase::getGHR(ThreadID tid, BranchInfo *bi) const
{
    unsigned val = 0;
    for (unsigned i = 0; i < 32; i++) {
        // Make sure we don't go out of bounds
        int gh_offset = bi->ptGhist + i;
        assert(&(threadHistory[tid].globalHistory[gh_offset]) <
               threadHistory[tid].globalHistory + histBufferSize);
        val |= ((threadHistory[tid].globalHistory[gh_offset] & 0x1) << i);
    }

    return val;
}

void
TAGEBase::regStats()
{
    tageLongestMatchProviderCorrect
        .name(name() + ".tageLongestMatchProviderCorrect")
        .desc("Number of times TAGE Longest Match is the provider and "
              "the prediction is correct");

    tageAltMatchProviderCorrect
        .name(name() + ".tageAltMatchProviderCorrect")
        .desc("Number of times TAGE Alt Match is the provider and "
              "the prediction is correct");

    bimodalAltMatchProviderCorrect
        .name(name() + ".bimodalAltMatchProviderCorrect")
        .desc("Number of times TAGE Alt Match is the bimodal and it is the "
              "provider and the prediction is correct");

    tageBimodalProviderCorrect
        .name(name() + ".tageBimodalProviderCorrect")
        .desc("Number of times there are no hits on the TAGE tables "
              "and the bimodal prediction is correct");

    tageLongestMatchProviderWrong
        .name(name() + ".tageLongestMatchProviderWrong")
        .desc("Number of times TAGE Longest Match is the provider and "
              "the prediction is wrong");

    tageAltMatchProviderWrong
        .name(name() + ".tageAltMatchProviderWrong")
        .desc("Number of times TAGE Alt Match is the provider and "
              "the prediction is wrong");

    bimodalAltMatchProviderWrong
        .name(name() + ".bimodalAltMatchProviderWrong")
        .desc("Number of times TAGE Alt Match is the bimodal and it is the "
              "provider and the prediction is wrong");

    tageBimodalProviderWrong
        .name(name() + ".tageBimodalProviderWrong")
        .desc("Number of times there are no hits on the TAGE tables "
              "and the bimodal prediction is wrong");

    tageAltMatchProviderWouldHaveHit
        .name(name() + ".tageAltMatchProviderWouldHaveHit")
        .desc("Number of times TAGE Longest Match is the provider, "
              "the prediction is wrong and Alt Match prediction was correct");

    tageLongestMatchProviderWouldHaveHit
        .name(name() + ".tageLongestMatchProviderWouldHaveHit")
        .desc("Number of times TAGE Alt Match is the provider, the "
              "prediction is wrong and Longest Match prediction was correct");

    tageLongestMatchProvider
        .init(nHistoryTables + 1)
        .name(name() + ".tageLongestMatchProvider")
        .desc("TAGE provider for longest match");

    tageAltMatchProvider
        .init(nHistoryTables + 1)
        .name(name() + ".tageAltMatchProvider")
        .desc("TAGE provider for alt match");
}

int8_t
TAGEBase::getCtr(int hitBank, int hitBankIndex) const
{
    return gtable[hitBank][hitBankIndex].ctr;
}

unsigned
TAGEBase::getTageCtrBits() const
{
    return tagTableCounterBits;
}

int
TAGEBase::getPathHist(ThreadID tid) const
{
    return threadHistory[tid].pathHist;
}

bool
TAGEBase::isSpeculativeUpdateEnabled() const
{
    return speculativeHistUpdate;
}

size_t
TAGEBase::getSizeInBits() const {
    size_t bits = 0;
    for (int i = 1; i <= nHistoryTables; i++) {
        bits += (1 << logTagTableSizes[i]) *
            (tagTableCounterBits + tagTableUBits + tagTableTagWidths[i]);
    }
    uint64_t bimodalTableSize = ULL(1) << logTagTableSizes[0];
    bits += numUseAltOnNa * useAltOnNaBits;
    bits += bimodalTableSize;
    bits += (bimodalTableSize >> logRatioBiModalHystEntries);
    bits += histLengths[nHistoryTables];
    bits += pathHistBits;
    bits += logUResetPeriod;
    return bits;
}

TAGEBase*
TAGEBaseParams::create()
{
    return new TAGEBase(this);
}
