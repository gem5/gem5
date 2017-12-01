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
  : BPredUnit(params),
    logSizeBiMP(params->logSizeBiMP),
    logSizeTagTables(params->logSizeTagTables),
    logSizeLoopPred(params->logSizeLoopPred),
    nHistoryTables(params->nHistoryTables),
    tagTableCounterBits(params->tagTableCounterBits),
    histBufferSize(params->histBufferSize),
    minHist(params->minHist),
    maxHist(params->maxHist),
    minTagWidth(params->minTagWidth),
    threadHistory(params->numThreads)
{
    assert(params->histBufferSize > params->maxHist * 2);
    useAltPredForNewlyAllocated = 0;
    logTick = 19;
    tCounter = ULL(1) << (logTick - 1);

    for (auto& history : threadHistory) {
        history.pathHist = 0;
        history.globalHistory = new uint8_t[histBufferSize];
        history.gHist = history.globalHistory;
        memset(history.gHist, 0, histBufferSize);
        history.ptGhist = 0;
    }

    histLengths = new int [nHistoryTables+1];
    histLengths[1] = minHist;
    histLengths[nHistoryTables] = maxHist;

    for (int i = 2; i <= nHistoryTables; i++) {
        histLengths[i] = (int) (((double) minHist *
                    pow ((double) (maxHist) / (double) minHist,
                        (double) (i - 1) / (double) ((nHistoryTables- 1))))
                    + 0.5);
    }

    tagWidths[1] = minTagWidth;
    tagWidths[2] = minTagWidth;
    tagWidths[3] = minTagWidth + 1;
    tagWidths[4] = minTagWidth + 1;
    tagWidths[5] = minTagWidth + 2;
    tagWidths[6] = minTagWidth + 3;
    tagWidths[7] = minTagWidth + 4;
    tagWidths[8] = minTagWidth + 5;
    tagWidths[9] = minTagWidth + 5;
    tagWidths[10] = minTagWidth + 6;
    tagWidths[11] = minTagWidth + 7;
    tagWidths[12] = minTagWidth + 8;

    for (int i = 1; i <= 2; i++)
        tagTableSizes[i] = logSizeTagTables - 1;
    for (int i = 3; i <= 6; i++)
        tagTableSizes[i] = logSizeTagTables;
    for (int i = 7; i <= 10; i++)
        tagTableSizes[i] = logSizeTagTables - 1;
    for (int i = 11; i <= 12; i++)
        tagTableSizes[i] = logSizeTagTables - 2;

    for (auto& history : threadHistory) {
        history.computeIndices = new FoldedHistory[nHistoryTables+1];
        history.computeTags[0] = new FoldedHistory[nHistoryTables+1];
        history.computeTags[1] = new FoldedHistory[nHistoryTables+1];

        for (int i = 1; i <= nHistoryTables; i++) {
            history.computeIndices[i].init(histLengths[i], (tagTableSizes[i]));
            history.computeTags[0][i].init(
                history.computeIndices[i].origLength, tagWidths[i]);
            history.computeTags[1][i].init(
                history.computeIndices[i].origLength, tagWidths[i] - 1);
            DPRINTF(LTage, "HistLength:%d, TTSize:%d, TTTWidth:%d\n",
                    histLengths[i], tagTableSizes[i], tagWidths[i]);
        }
    }

    btable = new BimodalEntry[ULL(1) << logSizeBiMP];
    ltable = new LoopEntry[ULL(1) << logSizeLoopPred];
    gtable = new TageEntry*[nHistoryTables + 1];
    for (int i = 1; i <= nHistoryTables; i++) {
        gtable[i] = new TageEntry[1<<(tagTableSizes[i])];
    }

    tableIndices = new int [nHistoryTables+1];
    tableTags = new int [nHistoryTables+1];

    loopUseCounter = 0;
}

int
LTAGE::bindex(Addr pc_in) const
{
    return ((pc_in) & ((ULL(1) << (logSizeBiMP)) - 1));
}

int
LTAGE::lindex(Addr pc_in) const
{
    return (((pc_in) & ((ULL(1) << (logSizeLoopPred - 2)) - 1)) << 2);
}

int
LTAGE::F(int A, int size, int bank) const
{
    int A1, A2;

    A = A & ((ULL(1) << size) - 1);
    A1 = (A & ((ULL(1) << tagTableSizes[bank]) - 1));
    A2 = (A >> tagTableSizes[bank]);
    A2 = ((A2 << bank) & ((ULL(1) << tagTableSizes[bank]) - 1))
       + (A2 >> (tagTableSizes[bank] - bank));
    A = A1 ^ A2;
    A = ((A << bank) & ((ULL(1) << tagTableSizes[bank]) - 1))
      + (A >> (tagTableSizes[bank] - bank));
    return (A);
}


// gindex computes a full hash of pc, ghist and pathHist
int
LTAGE::gindex(ThreadID tid, Addr pc, int bank) const
{
    int index;
    int hlen = (histLengths[bank] > 16) ? 16 : histLengths[bank];
    index =
        (pc) ^ ((pc) >> ((int) abs(tagTableSizes[bank] - bank) + 1)) ^
        threadHistory[tid].computeIndices[bank].comp ^
        F(threadHistory[tid].pathHist, hlen, bank);

    return (index & ((ULL(1) << (tagTableSizes[bank])) - 1));
}


// Tag computation
uint16_t
LTAGE::gtag(ThreadID tid, Addr pc, int bank) const
{
    int tag = (pc) ^ threadHistory[tid].computeTags[0][bank].comp
                   ^ (threadHistory[tid].computeTags[1][bank].comp << 1);

    return (tag & ((ULL(1) << tagWidths[bank]) - 1));
}


// Up-down saturating counter
void
LTAGE::ctrUpdate(int8_t & ctr, bool taken, int nbits)
{
    assert(nbits <= sizeof(int8_t) << 3);
    if (taken) {
        if (ctr < ((1 << (nbits - 1)) - 1))
            ctr++;
    } else {
        if (ctr > -(1 << (nbits - 1)))
            ctr--;
    }
}

// Bimodal prediction
bool
LTAGE::getBimodePred(Addr pc, BranchInfo* bi) const
{
    return (btable[bi->bimodalIndex].pred > 0);
}


// Update the bimodal predictor: a hysteresis bit is shared among 4 prediction
// bits
void
LTAGE::baseUpdate(Addr pc, bool taken, BranchInfo* bi)
{
    int inter = (btable[bi->bimodalIndex].pred << 1)
              + btable[bi->bimodalIndex ].hyst;
    if (taken) {
        if (inter < 3)
            inter++;
    } else if (inter > 0) {
        inter--;
    }
    btable[bi->bimodalIndex].pred = inter >> 1;
    btable[bi->bimodalIndex].hyst = (inter & 1);
    DPRINTF(LTage, "Updating branch %lx, pred:%d, hyst:%d\n",
            pc, btable[bi->bimodalIndex].pred,btable[bi->bimodalIndex].hyst);
}


//loop prediction: only used if high confidence
bool
LTAGE::getLoop(Addr pc, BranchInfo* bi) const
{
    bi->loopHit = -1;
    bi->loopPredValid = false;
    bi->loopIndex = lindex(pc);
    bi->loopTag = ((pc) >> (logSizeLoopPred - 2));

    for (int i = 0; i < 4; i++) {
        if (ltable[bi->loopIndex + i].tag == bi->loopTag) {
            bi->loopHit = i;
            bi->loopPredValid = (ltable[bi->loopIndex + i].confidence >= 3);
            bi->currentIter = ltable[bi->loopIndex + i].currentIterSpec;
            if (ltable[bi->loopIndex + i].currentIterSpec + 1 ==
                ltable[bi->loopIndex + i].numIter) {
                return !(ltable[bi->loopIndex + i].dir);
            }else {
                return (ltable[bi->loopIndex + i].dir);
            }
        }
    }
    return false;
}

void
LTAGE::specLoopUpdate(Addr pc, bool taken, BranchInfo* bi)
{
    if (bi->loopHit>=0) {
        int index = lindex(pc);
        if (taken != ltable[index].dir) {
            ltable[index].currentIterSpec = 0;
        } else {
            ltable[index].currentIterSpec++;
        }
    }
}

void
LTAGE::loopUpdate(Addr pc, bool taken, BranchInfo* bi)
{
    int idx = bi->loopIndex + bi->loopHit;
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
                if (ltable[idx].age < 7)
                    ltable[idx].age++;
            }
        }

        ltable[idx].currentIter++;
        if (ltable[idx].currentIter > ltable[idx].numIter) {
            ltable[idx].confidence = 0;
            if (ltable[idx].numIter != 0) {
                // free the entry
                ltable[idx].numIter = 0;
                ltable[idx].age = 0;
                ltable[idx].confidence = 0;
            }
        }

        if (taken != ltable[idx].dir) {
            if (ltable[idx].currentIter == ltable[idx].numIter) {
                DPRINTF(LTage, "Loop End predicted successfully:%lx\n", pc);

                if (ltable[idx].confidence < 7) {
                    ltable[idx].confidence++;
                }
                //just do not predict when the loop count is 1 or 2
                if (ltable[idx].numIter < 3) {
                    // free the entry
                    ltable[idx].dir = taken;
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

    } else if (taken) {
        //try to allocate an entry on taken branch
        int nrand = random_mt.random<int>();
        for (int i = 0; i < 4; i++) {
            int loop_hit = (nrand + i) & 3;
            idx = bi->loopIndex + loop_hit;
            if (ltable[idx].age == 0) {
                DPRINTF(LTage, "Allocating loop pred entry for branch %lx\n",
                        pc);
                ltable[idx].dir = !taken;
                ltable[idx].tag = bi->loopTag;
                ltable[idx].numIter = 0;
                ltable[idx].age = 7;
                ltable[idx].confidence = 0;
                ltable[idx].currentIter = 1;
                break;

            }
            else
                ltable[idx].age--;
        }
    }

}

// shifting the global history:  we manage the history in a big table in order
// to reduce simulation time
void
LTAGE::updateGHist(uint8_t * &h, bool dir, uint8_t * tab, int &pt)
{
    if (pt == 0) {
        DPRINTF(LTage, "Rolling over the histories\n");
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

// Get GHR for hashing indirect predictor
// Build history backwards from pointer in
// bp_history.
unsigned
LTAGE::getGHR(ThreadID tid, void *bp_history) const
{
    BranchInfo* bi = static_cast<BranchInfo*>(bp_history);
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

//prediction
bool
LTAGE::predict(ThreadID tid, Addr branch_pc, bool cond_branch, void* &b)
{
    BranchInfo *bi = new BranchInfo(nHistoryTables+1);
    b = (void*)(bi);
    Addr pc = branch_pc;
    bool pred_taken = true;
    bi->loopHit = -1;

    if (cond_branch) {
        // TAGE prediction

        // computes the table addresses and the partial tags
        for (int i = 1; i <= nHistoryTables; i++) {
            tableIndices[i] = gindex(tid, pc, i);
            bi->tableIndices[i] = tableIndices[i];
            tableTags[i] = gtag(tid, pc, i);
            bi->tableTags[i] = tableTags[i];
        }

        bi->bimodalIndex = bindex(pc);

        bi->hitBank = 0;
        bi->altBank = 0;
        //Look for the bank with longest matching history
        for (int i = nHistoryTables; i > 0; i--) {
            if (gtable[i][tableIndices[i]].tag == tableTags[i]) {
                bi->hitBank = i;
                bi->hitBankIndex = tableIndices[bi->hitBank];
                break;
            }
        }
        //Look for the alternate bank
        for (int i = bi->hitBank - 1; i > 0; i--) {
            if (gtable[i][tableIndices[i]].tag == tableTags[i]) {
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
            if ((useAltPredForNewlyAllocated < 0)
                   || abs(2 *
                   gtable[bi->hitBank][tableIndices[bi->hitBank]].ctr + 1) > 1)
                bi->tagePred = bi->longestMatchPred;
            else
                bi->tagePred = bi->altTaken;
        } else {
            bi->altTaken = getBimodePred(pc, bi);
            bi->tagePred = bi->altTaken;
            bi->longestMatchPred = bi->altTaken;
        }
        //end TAGE prediction

        bi->loopPred = getLoop(pc, bi);	// loop prediction

        pred_taken = (((loopUseCounter >= 0) && bi->loopPredValid)) ?
                     (bi->loopPred): (bi->tagePred);
        DPRINTF(LTage, "Predict for %lx: taken?:%d, loopTaken?:%d, "
                "loopValid?:%d, loopUseCounter:%d, tagePred:%d, altPred:%d\n",
                branch_pc, pred_taken, bi->loopPred, bi->loopPredValid,
                loopUseCounter, bi->tagePred, bi->altTaken);
    }
    bi->branchPC = branch_pc;
    bi->condBranch = cond_branch;
    specLoopUpdate(branch_pc, pred_taken, bi);
    return pred_taken;
}

// PREDICTOR UPDATE
void
LTAGE::update(ThreadID tid, Addr branch_pc, bool taken, void* bp_history,
              bool squashed)
{
    assert(bp_history);

    BranchInfo *bi = static_cast<BranchInfo*>(bp_history);

    if (squashed) {
        // This restores the global history, then update it
        // and recomputes the folded histories.
        squash(tid, taken, bp_history);
        return;
    }

    int nrand  = random_mt.random<int>(0,3);
    Addr pc = branch_pc;
    if (bi->condBranch) {
        DPRINTF(LTage, "Updating tables for branch:%lx; taken?:%d\n",
                branch_pc, taken);
        // first update the loop predictor
        loopUpdate(pc, taken, bi);

        if (bi->loopPredValid) {
            if (bi->tagePred != bi->loopPred) {
                ctrUpdate(loopUseCounter, (bi->loopPred== taken), 7);
            }
        }

        // TAGE UPDATE
        // try to allocate a  new entries only if prediction was wrong
        bool longest_match_pred = false;
        bool alloc = (bi->tagePred != taken) && (bi->hitBank < nHistoryTables);
        if (bi->hitBank > 0) {
            // Manage the selection between longest matching and alternate
            // matching for "pseudo"-newly allocated longest matching entry
             longest_match_pred = bi->longestMatchPred;
            bool PseudoNewAlloc = bi->pseudoNewAlloc;
            // an entry is considered as newly allocated if its prediction
            // counter is weak
            if (PseudoNewAlloc) {
                if (longest_match_pred == taken) {
                    alloc = false;
                }
                // if it was delivering the correct prediction, no need to
                // allocate new entry even if the overall prediction was false
                if (longest_match_pred != bi->altTaken) {
                    ctrUpdate(useAltPredForNewlyAllocated,
                         bi->altTaken == taken, 4);
                }
            }
        }

        if (alloc) {
            // is there some "unuseful" entry to allocate
            int8_t min = 1;
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


            //Allocate only  one entry
            for (int i = X; i <= nHistoryTables; i++) {
                if ((gtable[i][bi->tableIndices[i]].u == 0)) {
                    gtable[i][bi->tableIndices[i]].tag = bi->tableTags[i];
                    gtable[i][bi->tableIndices[i]].ctr = (taken) ? 0 : -1;
                    gtable[i][bi->tableIndices[i]].u = 0; //?
                }
            }
        }
        //periodic reset of u: reset is not complete but bit by bit
        tCounter++;
        if ((tCounter & ((ULL(1) << logTick) - 1)) == 0) {
            // reset least significant bit
            // most significant bit becomes least significant bit
            for (int i = 1; i <= nHistoryTables; i++) {
                for (int j = 0; j < (ULL(1) << tagTableSizes[i]); j++) {
                    gtable[i][j].u = gtable[i][j].u >> 1;
                }
            }
        }

        if (bi->hitBank > 0) {
            DPRINTF(LTage, "Updating tag table entry (%d,%d) for branch %lx\n",
                    bi->hitBank, bi->hitBankIndex, branch_pc);
            ctrUpdate(gtable[bi->hitBank][bi->hitBankIndex].ctr, taken,
                      tagTableCounterBits);
            // if the provider entry is not certified to be useful also update
            // the alternate prediction
            if (gtable[bi->hitBank][bi->hitBankIndex].u == 0) {
                if (bi->altBank > 0) {
                    ctrUpdate(gtable[bi->altBank][bi->altBankIndex].ctr, taken,
                              tagTableCounterBits);
                    DPRINTF(LTage, "Updating tag table entry (%d,%d) for"
                            " branch %lx\n", bi->hitBank, bi->hitBankIndex,
                            branch_pc);
                }
                if (bi->altBank == 0) {
                    baseUpdate(pc, taken, bi);
                }
            }

            // update the u counter
            if (longest_match_pred != bi->altTaken) {
                if (longest_match_pred == taken) {
                    if (gtable[bi->hitBank][bi->hitBankIndex].u < 1) {
                        gtable[bi->hitBank][bi->hitBankIndex].u++;
                    }
                }
            }
        } else {
            baseUpdate(pc, taken, bi);
        }

        //END PREDICTOR UPDATE
    }
    if (!squashed) {
        delete bi;
    }
}

void
LTAGE::updateHistories(ThreadID tid, Addr branch_pc, bool taken, void* b)
{
    BranchInfo* bi = (BranchInfo*)(b);
    ThreadHistory& tHist = threadHistory[tid];
    //  UPDATE HISTORIES
    bool pathbit = ((branch_pc) & 1);
    //on a squash, return pointers to this and recompute indices.
    //update user history
    updateGHist(tHist.gHist, taken, tHist.globalHistory, tHist.ptGhist);
    tHist.pathHist = (tHist.pathHist << 1) + pathbit;
    tHist.pathHist = (tHist.pathHist & ((ULL(1) << 16) - 1));

    bi->ptGhist = tHist.ptGhist;
    bi->pathHist = tHist.pathHist;
    //prepare next index and tag computations for user branchs
    for (int i = 1; i <= nHistoryTables; i++)
    {
        bi->ci[i]  = tHist.computeIndices[i].comp;
        bi->ct0[i] = tHist.computeTags[0][i].comp;
        bi->ct1[i] = tHist.computeTags[1][i].comp;
        tHist.computeIndices[i].update(tHist.gHist);
        tHist.computeTags[0][i].update(tHist.gHist);
        tHist.computeTags[1][i].update(tHist.gHist);
    }
    DPRINTF(LTage, "Updating global histories with branch:%lx; taken?:%d, "
            "path Hist: %x; pointer:%d\n", branch_pc, taken, tHist.pathHist,
            tHist.ptGhist);
}

void
LTAGE::squash(ThreadID tid, bool taken, void *bp_history)
{
    BranchInfo* bi = (BranchInfo*)(bp_history);
    ThreadHistory& tHist = threadHistory[tid];
    DPRINTF(LTage, "Restoring branch info: %lx; taken? %d; PathHistory:%x, "
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

    if (bi->condBranch) {
        if (bi->loopHit >= 0) {
            int idx = bi->loopIndex + bi->loopHit;
            ltable[idx].currentIterSpec = bi->currentIter;
        }
    }

}

void
LTAGE::squash(ThreadID tid, void *bp_history)
{
    BranchInfo* bi = (BranchInfo*)(bp_history);
    DPRINTF(LTage, "Deleting branch info: %lx\n", bi->branchPC);
    if (bi->condBranch) {
        if (bi->loopHit >= 0) {
            int idx = bi->loopIndex + bi->loopHit;
            ltable[idx].currentIterSpec = bi->currentIter;
        }
    }

    delete bi;
}

bool
LTAGE::lookup(ThreadID tid, Addr branch_pc, void* &bp_history)
{
    bool retval = predict(tid, branch_pc, true, bp_history);

    DPRINTF(LTage, "Lookup branch: %lx; predict:%d\n", branch_pc, retval);
    updateHistories(tid, branch_pc, retval, bp_history);
    assert(threadHistory[tid].gHist ==
           &threadHistory[tid].globalHistory[threadHistory[tid].ptGhist]);

    return retval;
}

void
LTAGE::btbUpdate(ThreadID tid, Addr branch_pc, void* &bp_history)
{
    BranchInfo* bi = (BranchInfo*) bp_history;
    ThreadHistory& tHist = threadHistory[tid];
    DPRINTF(LTage, "BTB miss resets prediction: %lx\n", branch_pc);
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

void
LTAGE::uncondBranch(ThreadID tid, Addr br_pc, void* &bp_history)
{
    DPRINTF(LTage, "UnConditionalBranch: %lx\n", br_pc);
    predict(tid, br_pc, false, bp_history);
    updateHistories(tid, br_pc, true, bp_history);
    assert(threadHistory[tid].gHist ==
           &threadHistory[tid].globalHistory[threadHistory[tid].ptGhist]);
}

LTAGE*
LTAGEParams::create()
{
    return new LTAGE(this);
}
