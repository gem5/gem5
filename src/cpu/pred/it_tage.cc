/*
 * Copyright (c) 2023 University of Edinburgh
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

/* @file
 * Implementation of a ITTAGE indirect branch predictor
 */

#include "cpu/pred/it_tage.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "cpu/pred/branch_type.hh"
#include "debug/Indirect.hh"
#include "debug/Tage.hh"

namespace gem5
{

namespace branch_prediction
{

/** Internal TAGE component **/
ITTAGE_TAGE::ITTAGE_TAGE(const ITTAGE_TAGEParams &p)
    : TAGEBase(p), istats(this)
{
    TAGEBase::init();

    // Build the target TAGE table
    tgtTable = new TgtEntry *[nHistoryTables + 1];

    for (int i = 1; i <= nHistoryTables; i++) {
        tgtTable[i] = new TgtEntry[1 << (logTagTableSizes[i])];
    }
}

ITTAGE_TAGE::ITTAGEBranchInfo *
ITTAGE_TAGE::makeITTBranchInfo(Addr pc, BranchType type)
{
    return new ITTAGEBranchInfo(*this, pc, type == BranchType::DirectCond);
}

void
ITTAGE_TAGE::ittTagePredict(ThreadID tid, Addr branch_pc, ITTAGEBranchInfo *bi)
{
    Addr pc = branch_pc;

    // TAGE prediction
    calculateIndicesAndTags(tid, pc, bi);

    bi->bimodalIndex = bindex(pc);

    bi->hitBank = 0;
    bi->altBank = 0;
    // Look for the bank with longest matching history
    for (int i = nHistoryTables; i > 0; i--) {
        if (noSkip[i] && gtable[i][tableIndices[i]].tag == tableTags[i] &&
            tgtTable[i][tableIndices[i]].target != nullptr) {
            bi->hitBank = i;
            bi->hitBankIndex = tableIndices[bi->hitBank];
            break;
        }
    }
    // Look for the alternate bank
    for (int i = bi->hitBank - 1; i > 0; i--) {
        if (noSkip[i] && gtable[i][tableIndices[i]].tag == tableTags[i] &&
            tgtTable[i][tableIndices[i]].target != nullptr) {
            bi->altBank = i;
            bi->altBankIndex = tableIndices[bi->altBank];
            break;
        }
    }

    DPRINTF(Indirect, "Hit %lx: longest(%i,%i), alt(%i,%i)\n", branch_pc,
            bi->hitBank, bi->hitBankIndex, bi->altBank, bi->altBankIndex);

    bi->provider = BTB;
    bi->tagePred = false;
    bi->longestMatchPred = false;
    bi->predTarget = nullptr;

    // Select the provider component
    if (bi->hitBank > 0) {
        if (bi->altBank > 0) {
            bi->altTaken = gtable[bi->altBank][bi->altBankIndex].ctr > 0;
            bi->altTarget =
                tgtTable[bi->altBank][bi->altBankIndex].target->instAddr();
        } else {
            bi->altTaken = false;
        }

        bi->longestMatchPred = gtable[bi->hitBank][bi->hitBankIndex].ctr > 0;
        bi->longestMatchTarget =
            tgtTable[bi->hitBank][bi->hitBankIndex].target->instAddr();

        // if (the confidence counter is non-null or
        // USE ALT ON NA is negative) then the provider
        // component provides the prediction
        if ((useAltPredForNewlyAllocated[getUseAltIdx(bi, branch_pc)] < 0) ||
            bi->longestMatchPred) {

            bi->provider = TAGE_LONGEST_MATCH;
            bi->tagePred = bi->longestMatchPred;
        } else if (bi->altBank > 0) {
            // Otherwise the alternate component provides the prediction
            // if there was a hit by the alternate component
            bi->provider = TAGE_ALT_MATCH;
            bi->tagePred = bi->altTaken;
        } else {
            // If non of the predictors hit fall back to the BTB
            // as target provider.
        }
    }

    switch (bi->provider) {
        case TAGE_LONGEST_MATCH:
            set(bi->predTarget,
                tgtTable[bi->hitBank][bi->hitBankIndex].target);
            DPRINTF(Indirect, "Predict for %lx: provider:TAGE, target:%lx\n",
                    branch_pc, *(bi->predTarget));
            break;

        case TAGE_ALT_MATCH:
            set(bi->predTarget,
                tgtTable[bi->altBank][bi->altBankIndex].target);
            DPRINTF(Indirect,
                    "Predict for %lx: provider:ALT_TAGE, target:%lx\n",
                    branch_pc, *(bi->predTarget));
            break;

        case BTB:
            DPRINTF(Indirect, "Fallback to BTB for %lx\n", branch_pc);
            break;

        default:
            break;
    }

    // end TAGE prediction
}

void
ITTAGE_TAGE::ittUpdateHistories(ThreadID tid, bool speculative, uint64_t hist,
                                uint8_t nBits, ITTAGEBranchInfo *bi)
{
    if (speculative != speculativeHistUpdate) {
        return;
    }
    if (nBits == 0) {
        return;
    }

    // If this is the first time we see this branch record the current
    // state of the history to be able to recover.
    if (speculativeHistUpdate && (!bi->modified)) {
        recordHistState(tid, bi);
    }

    // In case the branch already updated the history
    // we need to revert the previous update first.
    if (bi->modified) {
        restoreHistState(tid, bi);
    }

    assert(bi->nGhist == 0);

    ThreadHistory &tHist = threadHistory[tid];

    bi->ghist = hist;
    bi->nGhist = nBits;

    //  Update the global history
    updateGHist(tid, bi->ghist, bi->nGhist);
    bi->modified = true;

    DPRINTF(Indirect, "%s(hist:%#x, nbits:%i) pc:%#x; ptr:%d, GHR:%#x\n",
            __func__, bi->ghist, bi->nGhist, bi->branchPC, tHist.ptGhist,
            getGHR(tid));
}

void
ITTAGE_TAGE::updateIndirect(ThreadID tid, ITTAGEBranchInfo *bi, int nrand,
                            const PCStateBase &target)
{
    // TAGE UPDATE
    // Try to allocate a  new entries only if prediction was wrong
    bool alloc = (bi->squash) && (bi->hitBank < nHistoryTables);

    if (bi->hitBank > 0) {
        // Manage the selection between longest matching and alternate
        // matching for "pseudo"-newly allocated longest matching entry
        bool PseudoNewAlloc = !bi->longestMatchPred;
        // an entry is considered as newly allocated if its prediction
        // counter is weak
        if (PseudoNewAlloc) {
            // If one of the provider provides the correct target
            // and if they are not the same update the new allocation counter
            if ((bi->longestMatchTarget != bi->altTarget) &&
                (((bi->longestMatchTarget == target.instAddr()) ||
                  (bi->altTarget == target.instAddr())))) {
                ctrUpdate(useAltPredForNewlyAllocated[getUseAltIdx(
                              bi, bi->branchPC)],
                          bi->altTarget == target.instAddr(), useAltOnNaBits);
            }
        }
    }

    handleAllocAndUReset(alloc, true, bi, nrand);

    ittHandleTAGEUpdate(bi->branchPC, bi);
}

bool
ITTAGE_TAGE::allocateEntry(int idx, TAGEBase::BranchInfo *bi, bool taken)
{
    if (TAGEBase::allocateEntry(idx, bi, taken)) {
        ITTAGEBranchInfo *b = static_cast<ITTAGEBranchInfo *>(bi);
        DPRINTF(Indirect, "%s table(%d,%d) PC:%#x, targ:%#x\n", __func__, idx,
                b->tableIndices[idx], bi->branchPC, b->corrTarget->instAddr());

        assert(idx <= nHistoryTables);
        assert(b->tableIndices[idx] < (1 << (logTagTableSizes[idx])));
        set(tgtTable[idx][b->tableIndices[idx]].target, b->corrTarget);
        return true;
    }
    return false;
}

void
ITTAGE_TAGE::ittHandleTAGEUpdate(Addr branch_pc, ITTAGEBranchInfo *bi)
{
    if (bi->hitBank > 0) {

        uint8_t &ctr = (uint8_t &)gtable[bi->hitBank][bi->hitBankIndex].ctr;

        DPRINTF(Indirect, "%s(%d,%d) for branch %lx: %i\n", __func__,
                bi->hitBank, bi->hitBankIndex, branch_pc, ctr);

        assert(bi->predTarget);

        // For ITTAGE we only use possitive counter values as confidence.
        // Increment in case the predicted target was correct.
        // Decrement the confidence otherwise
        unsignedCtrUpdate(ctr,
                          bi->longestMatchTarget == bi->corrTarget->instAddr(),
                          tagTableCounterBits);

        // If the confidence reach 0 replace the target.
        if (ctr == 0) {
            set(tgtTable[bi->hitBank][bi->hitBankIndex].target,
                bi->corrTarget);
            DPRINTF(Indirect, "Replace target (%d,%d) with %lx:\n",
                    bi->hitBank, bi->hitBankIndex, bi->corrTarget->instAddr());
            istats.targetReplacements++;
        }

        // update the u counter
        if (bi->longestMatchTarget != bi->altTarget) {
            unsignedCtrUpdate(gtable[bi->hitBank][bi->hitBankIndex].u,
                              bi->longestMatchTarget ==
                                  bi->corrTarget->instAddr(),
                              tagTableUBits);

            DPRINTF(Indirect,
                    "Updating the useful bit (%d,%d) for"
                    " branch %lx: %i\n",
                    bi->hitBank, bi->hitBankIndex, branch_pc,
                    gtable[bi->hitBank][bi->hitBankIndex].u);
        }
    }
}

void
ITTAGE_TAGE::ittUpdateStats(const PCStateBase &target, ITTAGEBranchInfo *bi)
{
    if (!bi->squash) {
        // correct prediction
        switch (bi->provider) {
            case BIMODAL_ONLY:
            case BIMODAL_ALT_MATCH:
                assert(false);
                break;

            case TAGE_LONGEST_MATCH:
                stats.longestMatchProviderCorrect++;
                break;
            case TAGE_ALT_MATCH:
                stats.altMatchProviderCorrect++;
                break;
            case BTB:
                istats.btbProviderCorrect++;
                break;
        }
    } else {
        // wrong prediction
        switch (bi->provider) {
            case BIMODAL_ONLY:
            case BIMODAL_ALT_MATCH:
                assert(false);
                break;

            case TAGE_LONGEST_MATCH:
                stats.longestMatchProviderWrong++;
                if (bi->altTarget == target.instAddr()) {
                    stats.altMatchProviderWouldHaveHit++;
                }
                break;
            case TAGE_ALT_MATCH:
                stats.altMatchProviderWrong++;
                if (bi->longestMatchTarget == target.instAddr()) {
                    stats.longestMatchProviderWouldHaveHit++;
                }
                break;
            case BTB:
                istats.btbProviderWrong++;
                break;
        }
    }

    switch (bi->provider) {
        case TAGE_LONGEST_MATCH:
        case TAGE_ALT_MATCH:
            stats.longestMatchProvider[bi->hitBank]++;
            stats.altMatchProvider[bi->altBank]++;
            break;
    }
}

ITTAGE_TAGE::ITTAGE_TAGEStats::ITTAGE_TAGEStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(btbProviderCorrect, statistics::units::Count::get(),
               "Number of times the BTB (fallback) is the provider and the "
               "prediction is correct"),
      ADD_STAT(btbProviderWrong, statistics::units::Count::get(),
               "Number of times the BTB (fallback) is the provider and the "
               "prediction is wrong"),
      ADD_STAT(targetReplacements, statistics::units::Count::get(),
               "Number of times the target was replaced")
{}

/*************** INDIRECT TAGE ****************/

ITTAGE::ITTAGE(const ITTAGEParams &params)
    : IndirectPredictor(params),
      itage(params.itage),
      histBitsConditional(params.histBitsConditional),
      histBitsIndBranch(params.histBitsIndBranch),
      histBitsIndCall(params.histBitsIndCall),
      histBitsCall(params.histBitsCall),
      stats(this)
{
    if (histBitsConditional > 1) {
        panic("Only support one bit of direction history");
    }
}

void
ITTAGE::reset()
{
    DPRINTF(Indirect, "Reset Indirect predictor\n");
}

const PCStateBase *
ITTAGE::lookup(ThreadID tid, InstSeqNum sn, Addr pc, void *&indirect_history)
{
    assert(indirect_history == nullptr);

    stats.lookups++;
    genIndirectInfo(tid, sn, pc, BranchType::NoBranch, indirect_history);

    auto bi = static_cast<ITTAGE_TAGE::ITTAGEBranchInfo *>(indirect_history);

    // First make the TAGE prediction.
    itage->ittTagePredict(tid, pc, bi);

    // If there was a TAGE it use the entry from the TAGE array.
    // Otherwise return null and the BTB target will be taken.
    if (bi->provider == ITTAGE_TAGE::BTB) {
        stats.misses++;
        return nullptr;
    }
    stats.hits++;
    return bi->predTarget.get();
}

void
ITTAGE::update(ThreadID tid, InstSeqNum sn, Addr pc, bool squash, bool taken,
               const PCStateBase &target, BranchType brType,
               void *&indirect_history)
{
    const auto [hist, nBits] =
        calculateHistUpdate(brType, pc, taken, target.instAddr());

    // Not all branches update the history
    if (!nBits) {
        return;
    }

    stats.updates++;

    // If there is no history we did not use the indirect predictor yet.
    // Create one
    if (indirect_history == nullptr) {
        genIndirectInfo(tid, sn, pc, brType, indirect_history);
    }

    auto bi = static_cast<ITTAGE_TAGE::ITTAGEBranchInfo *>(indirect_history);
    assert(bi != nullptr);

    DPRINTF(Indirect,
            "%s(tid:%lu, sn:%lu, PC:%#x, squash:%i, targ:%#x, "
            "taken:%i type:%s)\n",
            __func__, tid, sn, bi->branchPC, squash, target.instAddr(), taken,
            toString(brType));

    assert(bi->branchPC > 0);

    // If the target is provided by the
    // BTB record it for the later update.
    if (bi->provider == ITTAGE_TAGE::BTB && bi->predTarget == nullptr) {
        set(bi->predTarget, target);
    }

    // Update the branch information
    bi->indirect = isIndirectNoReturn(brType);
    bi->type = brType;
    bi->taken = taken;
    set(bi->corrTarget, target);
    if (squash) {
        bi->squash = true;
        stats.updateSquashed++;
        if (bi->indirect) {
            stats.updateIndSquashed++;
        }
    }

    // Speculative update the history
    itage->ittUpdateHistories(tid, true, hist, nBits, bi);
}

void
ITTAGE::squash(ThreadID tid, InstSeqNum sn, void *&indirect_history)
{
    // If no history was created there is no need to delete it
    if (indirect_history == nullptr) {
        return;
    }

    // If restore the history state if the
    // branch modified it.
    auto bi = static_cast<ITTAGE_TAGE::ITTAGEBranchInfo *>(indirect_history);
    itage->restoreHistState(tid, bi);

    delete bi;
    indirect_history = nullptr;
}

void
ITTAGE::commit(ThreadID tid, InstSeqNum sn, void *&indirect_history)
{
    if (indirect_history == nullptr) {
        return;
    }

    auto bi = static_cast<ITTAGE_TAGE::ITTAGEBranchInfo *>(indirect_history);
    assert(bi->branchPC > 0);

    DPRINTF(Indirect,
            "%s(tid:%lu, sn:%lu, BI[PC:%#x, targ:%#x, taken:%#x, "
            "type:%s])\n",
            __func__, tid, sn, bi->branchPC, bi->corrTarget->instAddr(),
            bi->taken, toString(bi->type));

    int nrand = rng->random<int>() & 3;
    if (bi->indirect) {
        stats.indirectRecords++;

        if (bi->squash) {
            if (bi->type == BranchType::CallIndirect) {
                stats.mispredictIndCall++;
            } else {
                stats.mispredictIndJump++;
            }
            stats.mispredictTotal++;
        }

        DPRINTF(Indirect, "Updating ITTAGE for PC:%#x, mispredict:%i, %s\n",
                bi->branchPC, bi->squash, toString(bi->type));
        itage->ittUpdateStats(*bi->corrTarget, bi);
        itage->updateIndirect(tid, bi, nrand, *bi->corrTarget);
    }

    // Optional non speculative update of the histories
    const auto [hist, nBits] = calculateHistUpdate(
        bi->type, bi->branchPC, bi->taken, bi->corrTarget->instAddr());

    itage->ittUpdateHistories(tid, false, hist, nBits, bi);

    // The branch is done
    delete bi;
    indirect_history = nullptr;
}

void
ITTAGE::genIndirectInfo(ThreadID tid, InstSeqNum sn, Addr pc,
                        BranchType brType, void *&iHistory)
{
    // Record the GHR as it was before this prediction
    // It will be used to recover the history in case this prediction is
    // wrong or belongs to bad path
    auto *bi = itage->makeITTBranchInfo(pc, brType);

    bi->type = brType;
    bi->indirect = isIndirectNoReturn(brType);
    bi->taken = false;
    bi->provider = ITTAGE_TAGE::BTB;

    if (bi->indirect) {
        itage->calculateIndicesAndTags(tid, pc, bi);
    }

    iHistory = (void *)(bi);
}

std::tuple<uint64_t, uint8_t>
ITTAGE::calculateHistUpdate(BranchType type, Addr pc, bool taken, Addr target)
{
    uint8_t nBits = 0;
    uint64_t hist = target ^ (target >> 3) ^ pc;

    switch (type) {
        case BranchType::DirectCond:
            nBits = histBitsConditional;
            hist = taken ? 1 : 0;
            break;

        case BranchType::IndirectUncond:
            nBits = histBitsIndBranch;
            break;
        case BranchType::CallDirect:
            nBits = histBitsCall;
            break;
        case BranchType::CallIndirect:
            nBits = histBitsIndCall;
            break;

        default:
            break;
    }

    hist &= (1 << nBits) - 1;
    return std::make_tuple(hist, nBits);
}

ITTAGE::ITTAGEStats::ITTAGEStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(lookups, statistics::units::Count::get(), "Number of lookups"),
      ADD_STAT(hits, statistics::units::Count::get(),
               "Number of hits of a tag"),
      ADD_STAT(misses, statistics::units::Count::get(), "Number of misses"),
      ADD_STAT(targetRecords, statistics::units::Count::get(),
               "Number of targets that where recorded/installed in the cache"),
      ADD_STAT(indirectRecords, statistics::units::Count::get(),
               "Number of indirect branches/calls recorded in the"
               " indirect hist"),
      ADD_STAT(mispredictIndCall, statistics::units::Count::get(),
               "Number of mispredicted indirect calls"),
      ADD_STAT(mispredictIndJump, statistics::units::Count::get(),
               "Number of mispredicted indirect jumps"),
      ADD_STAT(mispredictTotal, statistics::units::Count::get(),
               "Number of all mispredicted indirect branches"),
      ADD_STAT(updates, statistics::units::Count::get(),
               "Number of all mispredicted indirect branches"),
      ADD_STAT(updateSquashed, statistics::units::Count::get(),
               "Number of all mispredicted indirect branches"),
      ADD_STAT(updateIndSquashed, statistics::units::Count::get(),
               "Number of all mispredicted indirect branches")
{}

} // namespace branch_prediction
} // namespace gem5
