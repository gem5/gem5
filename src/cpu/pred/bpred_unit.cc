/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010,2022-2023 The University of Edinburgh
 * Copyright (c) 2012 Mark D. Hill and David A. Wood
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "cpu/pred/bpred_unit.hh"

#include <algorithm>

#include "arch/generic/pcstate.hh"
#include "base/compiler.hh"
#include "base/trace.hh"
#include "debug/Branch.hh"

namespace gem5
{

namespace branch_prediction
{

BPredUnit::BPredUnit(const Params &params)
    : SimObject(params),
      numThreads(params.numThreads),
      requiresBTBHit(params.requiresBTBHit),
      instShiftAmt(params.instShiftAmt),
      predHist(numThreads),
      btb(params.BTB),
      ras(params.RAS),
      iPred(params.indirectBranchPred),
      stats(this,this)
{
}


probing::PMUUPtr
BPredUnit::pmuProbePoint(const char *name)
{
    probing::PMUUPtr ptr;
    ptr.reset(new probing::PMU(getProbeManager(), name));

    return ptr;
}

void
BPredUnit::regProbePoints()
{
    ppBranches = pmuProbePoint("Branches");
    ppMisses = pmuProbePoint("Misses");
}

void
BPredUnit::drainSanityCheck() const
{
    // We shouldn't have any outstanding requests when we resume from
    // a drained system.
    for ([[maybe_unused]] const auto& ph : predHist)
        assert(ph.empty());
}

void
BPredUnit::branchPlaceholder(ThreadID tid, Addr pc,
                                bool uncond, void * &bpHistory)
{
    panic("Not implemented for this BP!\n");
}

bool
BPredUnit::predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                   PCStateBase &pc, ThreadID tid)
{
    /** Perform the prediction. */
    PredictorHistory* bpu_history = nullptr;
    bool taken  = predict(inst, seqNum, pc, tid, bpu_history);

    assert(bpu_history!=nullptr);

    /** Push the record into the history buffer */
    predHist[tid].push_front(bpu_history);

    DPRINTF(Branch, "[tid:%i] [sn:%llu] History entry added. "
            "predHist.size(): %i\n", tid, seqNum, predHist[tid].size());

    return taken;
}




bool
BPredUnit::predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                   PCStateBase &pc, ThreadID tid, PredictorHistory* &hist)
{
    assert(hist==nullptr);

    BranchType brType = getBranchType(inst);
    hist = new PredictorHistory(tid, seqNum, pc.instAddr(), inst);

    stats.lookups[tid][brType]++;
    ppBranches->notify(1);

    // See if branch predictor predicts taken.
    // If so, get its target addr either from the BTB or the RAS.
    // Save off record of branch stuff so the RAS can be fixed
    // up once it's done.


    /* -----------------------------------------------
     * Get branch direction
     * -----------------------------------------------
     * Lookup the direction predictor for every
     * conditional branch. For unconditional branches
     * the direction is always taken
     */

    if (inst->isUncondCtrl()) {
        // Unconditional branches -----
        hist->condPred = true;
    } else {
        // Conditional branches -------
        ++stats.condPredicted;
        hist->condPred = lookup(tid, pc.instAddr(), hist->bpHistory);

        if (hist->condPred) {
            ++stats.condPredictedTaken;
        }
    }
    hist->predTaken = hist->condPred;

    DPRINTF(Branch,
            "[tid:%i, sn:%llu] Branch predictor predicted %i for PC:%#x %s\n",
            tid, seqNum, hist->condPred, hist->pc, toString(brType));


    // The direction is done now get the target address
    // from BTB, RAS or indirect predictor.
    hist->targetProvider = TargetProvider::NoTarget;

    /* -----------------------------------------------
     * Branch Target Buffer (BTB)
     * -----------------------------------------------
     * First check for a BTB hit. This will be done
     * regardless of whether the RAS or the indirect
     * predictor provide the final target. That is
     * necessary as modern front-end do not have a
     * chance to detect a branch without a BTB hit.
     */
    stats.BTBLookups++;
    const PCStateBase * btb_target = btb->lookup(tid, pc.instAddr(), brType);
    if (btb_target) {
        stats.BTBHits++;
        hist->btbHit = true;

        if (hist->predTaken) {
            hist->targetProvider = TargetProvider::BTB;
            set(hist->target, btb_target);
        }
    }

    DPRINTF(Branch, "[tid:%i, sn:%llu] PC:%#x BTB:%s\n",
            tid, seqNum, hist->pc,  (hist->btbHit) ? "hit" : "miss");


    // If we model a decoupled front-end the BP requires a BTB hit
    // otherwise it cannot detect a branch.
    const bool allow_btb_override =
                    (hist->btbHit || !requiresBTBHit) ? true : false;


    /* -----------------------------------------------
     * Return Address Stack (RAS)
     * -----------------------------------------------
     * Perform RAS operations for calls and returns.
     * Calls: push their RETURN address onto
     *    the RAS.
     * Return: pop the the return address from the
     *    top of the RAS.
     */
    if (ras && allow_btb_override) {
        if (inst->isCall()) {
            // Incase of a call build the return address and
            // push it to the RAS.
            auto return_addr = inst->buildRetPC(pc, pc);
            if (inst->size()) {
                return_addr->set(pc.instAddr() + inst->size());
            }
            ras->push(tid, *return_addr, hist->rasHistory);

            DPRINTF(Branch, "[tid:%i] [sn:%llu] Instr. %s was "
                    "a call, push return address %s onto the RAS\n",
                    tid, seqNum, pc, *return_addr);

        }
        else if (inst->isReturn()) {

            // If it's a return from a function call, then look up the
            // RETURN address in the RAS.
            const PCStateBase *return_addr = ras->pop(tid, hist->rasHistory);
            if (return_addr) {

                // Set the target to the return address
                set(hist->target, *return_addr);
                hist->targetProvider = TargetProvider::RAS;

                DPRINTF(Branch, "[tid:%i] [sn:%llu] Instr. %s is a "
                        "return, RAS poped return addr: %s\n",
                        tid, seqNum, pc, *hist->target);
            }
        }
    }


    /* -----------------------------------------------
     *  Indirect Predictor
     * -----------------------------------------------
     * For indirect branches/calls check the indirect
     * predictor if one is available. Not for returns.
     * Note that depending on the implementation a
     * indirect predictor might only return a target
     * for an indirect branch with a changing target.
     * As most indirect branches have a static target
     * using the target from the BTB is the optimal
     * to save space in the indirect preditor itself.
     */
    if (iPred) {
        if (hist->predTaken && allow_btb_override &&
            inst->isIndirectCtrl() && !inst->isReturn()) {

            ++stats.indirectLookups;

            const PCStateBase *itarget = iPred->lookup(tid, seqNum,
                                                pc.instAddr(),
                                        hist->indirectHistory);

            if (itarget) {
                // Indirect predictor hit
                ++stats.indirectHits;
                hist->targetProvider = TargetProvider::Indirect;
                set(hist->target, *itarget);

                DPRINTF(Branch,
                        "[tid:%i, sn:%llu] Instruction %s predicted "
                        "indirect target is %s\n",
                        tid, seqNum, pc, *hist->target);
            } else {
                ++stats.indirectMisses;
                DPRINTF(Branch,
                        "[tid:%i, sn:%llu] PC:%#x no indirect target\n",
                        tid, seqNum, pc.instAddr());
            }
        }
    }


    /** ----------------------------------------------
     * Fallthrough
     * -----------------------------------------------
     * All the target predictors did their job.
     * If there is no target its either not taken or
     * a BTB miss. In that case we just fallthrough.
     * */
    if (hist->targetProvider == TargetProvider::NoTarget) {
        set(hist->target, pc);
        inst->advancePC(*hist->target);
        hist->predTaken = false;
    }
    stats.targetProvider[tid][hist->targetProvider]++;

    // The actual prediction is done.
    // For now the BPU assume its correct. The update
    // functions will correct the branch if needed.
    // If prediction and actual direction are the same
    // at commit the prediction was correct.
    hist->actuallyTaken = hist->predTaken;
    set(pc, *hist->target);

    DPRINTF(Branch, "%s(tid:%i, sn:%i, PC:%#x, %s) -> taken:%i, target:%s "
            "provider:%s\n", __func__, tid, seqNum, hist->pc,
            toString(brType), hist->predTaken, *hist->target,
            enums::TargetProviderStrings[hist->targetProvider]);


    /** ----------------------------------------------
     * Speculative history update
     * -----------------------------------------------
     * Now that the prediction is done the predictor
     * may update its histories speculative. (local
     * and global path). A later squash will revert
     * the history update if needed.
     * The actual prediction tables will updated once
     * we know the correct direction.
     **/
    updateHistories(tid, hist->pc, hist->uncond,
                    hist->predTaken, hist->target->instAddr(),
                    hist->bpHistory);


    if (iPred) {
        // Update the indirect predictor with the direction prediction
        iPred->update(tid, seqNum, hist->pc, false, hist->predTaken,
                      *hist->target, brType, hist->indirectHistory);
    }

    // dump();
    return hist->predTaken;
}


void
BPredUnit::update(const InstSeqNum &done_sn, ThreadID tid)
{
    DPRINTF(Branch, "[tid:%i] Committing branches until "
            "[sn:%llu]\n", tid, done_sn);

    // dump();

    while (!predHist[tid].empty() &&
            predHist[tid].back()->seqNum <= done_sn) {

        // Iterate from the back to front. Least recent
        // sequence number until the most recent done number
        commitBranch(tid, *predHist[tid].rbegin());

        delete predHist[tid].back();
        predHist[tid].pop_back();
        DPRINTF(Branch, "[tid:%i] [commit sn:%llu] pred_hist.size(): %i\n",
                tid, done_sn, predHist[tid].size());
    }
}

void
BPredUnit::commitBranch(ThreadID tid, PredictorHistory* &hist)
{

    stats.committed[tid][hist->type]++;
    if (hist->mispredict) {
        stats.mispredicted[tid][hist->type]++;
    }


    DPRINTF(Branch, "Commit branch: sn:%llu, PC:%#x %s, "
                    "pred:%i, taken:%i, target:%#x\n",
                hist->seqNum, hist->pc, toString(hist->type),
                hist->predTaken, hist->actuallyTaken,
                hist->target->instAddr());

    // Update the branch predictor with the correct results.
    update(tid, hist->pc,
                hist->actuallyTaken,
                hist->bpHistory, false,
                hist->inst,
                hist->target->instAddr());

    // Commite also Indirect predictor and RAS
    if (iPred) {
        iPred->commit(tid, hist->seqNum, hist->indirectHistory);
    }

    if (ras) {
        ras->commit(tid, hist->mispredict,
                         hist->type,
                         hist->rasHistory);
    }

    // Update the BTB with commited branches.
    // Install all taken
    if (hist->actuallyTaken) {

        DPRINTF(Branch,"[tid:%i] BTB Update called for [sn:%llu] "
                    "PC %#x -> T: %#x\n", tid,
                    hist->seqNum, hist->pc, hist->target->instAddr());

        stats.BTBUpdates++;
        btb->update(tid, hist->pc,
                        *hist->target,
                         hist->type,
                         hist->inst);
    }
}



void
BPredUnit::squash(const InstSeqNum &squashed_sn, ThreadID tid)
{

    while (!predHist[tid].empty() &&
            predHist[tid].front()->seqNum > squashed_sn) {

        auto hist = predHist[tid].begin();

        DPRINTF(Branch, "[tid:%i, squash sn:%llu] Removing history for "
                "sn:%llu, PC:%#x\n", tid, squashed_sn, (*hist)->seqNum,
                (*hist)->pc);

        squashHistory(tid, *hist);

        predHist[tid].pop_front();

        DPRINTF(Branch, "[tid:%i] [squash sn:%llu] pred_hist.size(): %i\n",
                tid, squashed_sn, predHist[tid].size());
    }
}



void
BPredUnit::squashHistory(ThreadID tid, PredictorHistory* &history)
{

    stats.squashes[tid][history->type]++;
    DPRINTF(Branch, "[tid:%i] [squash sn:%llu] Incorrect: %s\n",
                tid, history->seqNum,
                toString(history->type));


    if (history->rasHistory) {
        assert(ras);

        DPRINTF(Branch, "[tid:%i] [squash sn:%llu] Incorrect call/return "
                "PC %#x. Fix RAS.\n", tid, history->seqNum,
                history->pc);

        ras->squash(tid, history->rasHistory);
    }

    if (iPred) {
        iPred->squash(tid, history->seqNum,
                        history->indirectHistory);
    }

    // This call will delete the bpHistory.
    squash(tid, history->bpHistory);

    delete history; history = nullptr;
}


void
BPredUnit::squash(const InstSeqNum &squashed_sn,
                  const PCStateBase &corr_target,
                  bool actually_taken, ThreadID tid, bool from_commit)
{
    // Now that we know that a branch was mispredicted, we need to undo
    // all the branches that have been seen up until this branch and
    // fix up everything.
    // NOTE: This should be call conceivably in 2 scenarios:
    // (1) After an branch is executed, it updates its status in the ROB
    //     The commit stage then checks the ROB update and sends a signal to
    //     the fetch stage to squash history after the mispredict
    // (2) In the decode stage, you can find out early if a unconditional
    //     PC-relative, branch was predicted incorrectly. If so, a signal
    //     to the fetch stage is sent to squash history after the mispredict

    History &pred_hist = predHist[tid];

    ++stats.condIncorrect;
    ppMisses->notify(1);


    DPRINTF(Branch, "[tid:%i] Squash from %s start from sequence number %i, "
            "setting target to %s\n", tid, from_commit ? "commit" : "decode",
            squashed_sn, corr_target);

    // dump();

    // Squash All Branches AFTER this mispredicted branch
    // First the Prefetch history then the main history.
    squash(squashed_sn, tid);

    // If there's a squash due to a syscall, there may not be an entry
    // corresponding to the squash.  In that case, don't bother trying to
    // fix up the entry.
    if (!pred_hist.empty()) {

        PredictorHistory* const hist = *(pred_hist.begin());

        DPRINTF(Branch, "[tid:%i] [squash sn:%llu] Mispredicted: %s, PC:%#x\n",
                    tid, squashed_sn, toString(hist->type), hist->pc);

        // Update stats
        stats.corrected[tid][hist->type]++;
        if (hist->target &&
            (hist->target->instAddr() != corr_target.instAddr())) {
                stats.targetWrong[tid][hist->targetProvider]++;
        }

        ++stats.incorrect;
        // If the squash is comming from decode it can be
        // redirected earlier. Note that this branch might never get
        // committed as a preceeding branch was mispredicted
        if (!from_commit) {
            stats.earlyResteers[tid][hist->type]++;
        }

        if (actually_taken) {
            ++stats.NotTakenMispredicted;
        } else {
           ++stats.TakenMispredicted;
        }


        // There are separate functions for in-order and out-of-order
        // branch prediction, but not for update. Therefore, this
        // call should take into account that the mispredicted branch may
        // be on the wrong path (i.e., OoO execution), and that the counter
        // counter table(s) should not be updated. Thus, this call should
        // restore the state of the underlying predictor, for instance the
        // local/global histories. The counter tables will be updated when
        // the branch actually commits.

        // Remember the correct direction and target for the update at commit.
        hist->mispredict = true;
        hist->actuallyTaken = actually_taken;
        set(hist->target,  corr_target);

        // Correct Direction predictor ------------------
        update(tid, hist->pc, actually_taken, hist->bpHistory,
               true, hist->inst, corr_target.instAddr());


        // Correct Indirect predictor -------------------
        if (iPred) {
            iPred->update(tid, squashed_sn, hist->pc,
                            true, actually_taken, corr_target,
                            hist->type, hist->indirectHistory);
        }

        // Correct RAS ---------------------------------
        if (ras) {
            if (actually_taken) {
                // The branch was taken but the RAS was not updated
                // accordingly. Needs to be fixed.
                if (hist->call && (hist->rasHistory == nullptr)) {

                    // Incase of a call build the return address and
                    // push it to the RAS.
                    auto return_addr = hist->inst->buildRetPC(
                                                    corr_target, corr_target);
                    if (hist->inst->size()) {
                        return_addr->set(corr_target.instAddr()
                                         + hist->inst->size());
                    }
                    DPRINTF(Branch, "[tid:%i] [squash sn:%llu] "
                            "Incorrectly predicted call: [sn:%llu,PC:%#x] "
                            " Push return address %s onto RAS\n", tid,
                            squashed_sn, hist->seqNum, hist->pc,
                            *return_addr);
                    ras->push(tid, *return_addr, hist->rasHistory);
                }

                if (hist->type == BranchType::Return
                    && (hist->rasHistory == nullptr)) {
                    DPRINTF(Branch, "[tid:%i] [squash sn:%llu] "
                        "Incorrectly predicted return [sn:%llu] PC: %#x\n",
                        tid, squashed_sn, hist->seqNum, hist->pc);

                    ras->pop(tid, hist->rasHistory);
                }

            } else if (hist->rasHistory != nullptr) {
                // The branch was not taken but the RAS was modified.
                // Needs to be fixed.
                ras->squash(tid, hist->rasHistory);
            }
        }

        // Correct BTB ---------------------------------
        // Check if the misprediction was because there was a
        // BTB miss.
        if (actually_taken &&!hist->btbHit) {
            ++stats.BTBMispredicted;
            if (hist->condPred)
                ++stats.predTakenBTBMiss;

            btb->incorrectTarget(hist->pc, hist->type);

            DPRINTF(Branch,"[tid:%i] [squash sn:%llu] "
                "BTB miss PC %#x %s \n", tid, squashed_sn,
                hist->pc, toString(hist->type));
        }

    } else {
        DPRINTF(Branch, "[tid:%i] [sn:%llu] pred_hist empty, can't "
                "update\n", tid, squashed_sn);
    }
}


void
BPredUnit::dump()
{
    int i = 0;
    for (const auto& ph : predHist) {
        if (!ph.empty()) {
            auto hist = ph.begin();

            cprintf("predHist[%i].size(): %i\n", i++, ph.size());

            while (hist != ph.end()) {
                cprintf("sn:%llu], PC:%#x, tid:%i, predTaken:%i, "
                        "bpHistory:%#x, rasHistory:%#x\n",
                        (*hist)->seqNum, (*hist)->pc,
                        (*hist)->tid, (*hist)->predTaken,
                        (*hist)->bpHistory, (*hist)->rasHistory);
                hist++;
            }

            cprintf("\n");
        }
    }
}


BPredUnit::BPredUnitStats::BPredUnitStats(
                                statistics::Group *parent, BPredUnit *bp)
    : statistics::Group(parent),
      ADD_STAT(lookups, statistics::units::Count::get(),
              "Number of BP lookups"),
      ADD_STAT(squashes, statistics::units::Count::get(),
              "Number of branches that got squashed as an earlier branch was "
              "mispredicted."),
      ADD_STAT(corrected, statistics::units::Count::get(),
              "Number of branches that got corrected but not yet commited. "),
      ADD_STAT(committed, statistics::units::Count::get(),
              "Number of branches finally committed "),
      ADD_STAT(mispredicted, statistics::units::Count::get(),
              "Number of committed branches that where mispredicted."),
      ADD_STAT(targetProvider, statistics::units::Count::get(),
              "Number of targets each provider provided"),
      ADD_STAT(targetWrong, statistics::units::Count::get(),
              "Number of times the target was incorrect"),
      ADD_STAT(earlyResteers, statistics::units::Count::get(),
              "Number of branches that got squashed after decode."),

      ADD_STAT(condPredicted, statistics::units::Count::get(),
               "Number of conditional branches predicted"),
      ADD_STAT(condPredictedTaken, statistics::units::Count::get(),
               "Number of conditional branches predicted as taken"),
      ADD_STAT(condIncorrect, statistics::units::Count::get(),
               "Number of conditional branches incorrect"),
      ADD_STAT(incorrect, statistics::units::Count::get(),
               "Number of branches incorrect"),
      ADD_STAT(BTBLookups, statistics::units::Count::get(),
               "Number of BTB lookups"),
      ADD_STAT(BTBUpdates, statistics::units::Count::get(),
               "Number of BTB lookups"),
      ADD_STAT(BTBHits, statistics::units::Count::get(),
               "Number of BTB hits"),
      ADD_STAT(BTBHitRatio, statistics::units::Ratio::get(), "BTB Hit Ratio",
               BTBHits / BTBLookups),
      ADD_STAT(BTBMispredicted, statistics::units::Count::get(),
               "Number BTB misspredictions. No target found or target wrong"),
      ADD_STAT(indirectLookups, statistics::units::Count::get(),
               "Number of indirect predictor lookups."),
      ADD_STAT(indirectHits, statistics::units::Count::get(),
               "Number of indirect target hits."),
      ADD_STAT(indirectMisses, statistics::units::Count::get(),
               "Number of indirect misses."),
      ADD_STAT(indirectMispredicted, statistics::units::Count::get(),
               "Number of mispredicted indirect branches."),
      ADD_STAT(predTakenBTBMiss, statistics::units::Count::get(),
               "Number of branches predicted taken but miss in BTB"),
      ADD_STAT(NotTakenMispredicted, statistics::units::Count::get(),
               "Number branches predicted 'not taken' but turn out "
               "to be taken"),
      ADD_STAT(TakenMispredicted, statistics::units::Count::get(),
               "Number branches predicted taken but turn out to be not taken")
{
    using namespace statistics;
    BTBHitRatio.precision(6);

    lookups
        .init(bp->numThreads, enums::Num_BranchType)
        .flags(total | pdf);
    lookups.ysubnames(enums::BranchTypeStrings);

    squashes
        .init(bp->numThreads, enums::Num_BranchType)
        .flags(total | pdf);
    squashes.ysubnames(enums::BranchTypeStrings);

    corrected
        .init(bp->numThreads, enums::Num_BranchType)
        .flags(total | pdf);
    corrected.ysubnames(enums::BranchTypeStrings);

    committed
        .init(bp->numThreads, enums::Num_BranchType)
        .flags(total | pdf);
    committed.ysubnames(enums::BranchTypeStrings);

    mispredicted
        .init(bp->numThreads, enums::Num_BranchType)
        .flags(total | pdf);
    mispredicted.ysubnames(enums::BranchTypeStrings);

    targetProvider
        .init(bp->numThreads, enums::Num_TargetProvider)
        .flags(total | pdf);
    targetProvider.ysubnames(enums::TargetProviderStrings);

    targetWrong
        .init(bp->numThreads, enums::Num_BranchType)
        .flags(total | pdf);
    targetWrong.ysubnames(enums::BranchTypeStrings);

    earlyResteers
        .init(bp->numThreads, enums::Num_BranchType)
        .flags(total | pdf);
    earlyResteers.ysubnames(enums::BranchTypeStrings);
}

} // namespace branch_prediction
} // namespace gem5
