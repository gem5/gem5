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
      predHist(numThreads),
      btb(params.btb),
      RAS(numThreads),
      iPred(params.indirectBranchPred),
      stats(this),
      instShiftAmt(params.instShiftAmt)
{
    for (auto& r : RAS)
        r.init(params.RASSize);
}

BPredUnit::BPredUnitStats::BPredUnitStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(lookups, statistics::units::Count::get(),
              "Number of BP lookups"),
      ADD_STAT(condPredicted, statistics::units::Count::get(),
               "Number of conditional branches predicted"),
      ADD_STAT(condIncorrect, statistics::units::Count::get(),
               "Number of conditional branches incorrect"),
      ADD_STAT(BTBLookups, statistics::units::Count::get(),
               "Number of BTB lookups"),
      ADD_STAT(BTBUpdates, statistics::units::Count::get(),
               "Number of BTB updates"),
      ADD_STAT(BTBHits, statistics::units::Count::get(), "Number of BTB hits"),
      ADD_STAT(BTBHitRatio, statistics::units::Ratio::get(), "BTB Hit Ratio",
               BTBHits / BTBLookups),
      ADD_STAT(RASUsed, statistics::units::Count::get(),
               "Number of times the RAS was used to get a target."),
      ADD_STAT(RASIncorrect, statistics::units::Count::get(),
               "Number of incorrect RAS predictions."),
      ADD_STAT(indirectLookups, statistics::units::Count::get(),
               "Number of indirect predictor lookups."),
      ADD_STAT(indirectHits, statistics::units::Count::get(),
               "Number of indirect target hits."),
      ADD_STAT(indirectMisses, statistics::units::Count::get(),
               "Number of indirect misses."),
      ADD_STAT(indirectMispredicted, statistics::units::Count::get(),
               "Number of mispredicted indirect branches.")
{
    BTBHitRatio.precision(6);
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

bool
BPredUnit::predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                   PCStateBase &pc, ThreadID tid)
{
    // See if branch predictor predicts taken.
    // If so, get its target addr either from the BTB or the RAS.
    // Save off record of branch stuff so the RAS can be fixed
    // up once it's done.

    bool pred_taken = false;
    std::unique_ptr<PCStateBase> target(pc.clone());

    ++stats.lookups;
    ppBranches->notify(1);

    void *bp_history = NULL;
    void *indirect_history = NULL;

    if (inst->isUncondCtrl()) {
        DPRINTF(Branch, "[tid:%i] [sn:%llu] Unconditional control\n",
            tid,seqNum);
        pred_taken = true;
        // Tell the BP there was an unconditional branch.
        uncondBranch(tid, pc.instAddr(), bp_history);
    } else {
        ++stats.condPredicted;
        pred_taken = lookup(tid, pc.instAddr(), bp_history);

        DPRINTF(Branch, "[tid:%i] [sn:%llu] "
                "Branch predictor predicted %i for PC %s\n",
                tid, seqNum,  pred_taken, pc);
    }

    const bool orig_pred_taken = pred_taken;
    if (iPred) {
        iPred->genIndirectInfo(tid, indirect_history);
    }

    DPRINTF(Branch,
            "[tid:%i] [sn:%llu] Creating prediction history for PC %s\n",
            tid, seqNum, pc);

    PredictorHistory predict_record(seqNum, pc.instAddr(), pred_taken,
                                    bp_history, indirect_history, tid, inst);

    // Now lookup in the BTB or RAS.
    if (pred_taken) {
        // Note: The RAS may be both popped and pushed to
        //       support coroutines.
        if (inst->isReturn()) {
            ++stats.RASUsed;
            predict_record.wasReturn = true;
            // If it's a function return call, then look up the address
            // in the RAS.
            const PCStateBase *ras_top = RAS[tid].top();
            if (ras_top)
                set(target, inst->buildRetPC(pc, *ras_top));

            // Record the top entry of the RAS, and its index.
            predict_record.usedRAS = true;
            predict_record.RASIndex = RAS[tid].topIdx();
            set(predict_record.RASTarget, ras_top);

            RAS[tid].pop();

            DPRINTF(Branch, "[tid:%i] [sn:%llu] Instruction %s is a return, "
                    "RAS predicted target: %s, RAS index: %i\n",
                    tid, seqNum, pc, *target, predict_record.RASIndex);
        }

        if (inst->isCall()) {
            RAS[tid].push(pc);
            predict_record.pushedRAS = true;

            // Record that it was a call so that the top RAS entry can
            // be popped off if the speculation is incorrect.
            predict_record.wasCall = true;

            DPRINTF(Branch,
                    "[tid:%i] [sn:%llu] Instruction %s was a call, adding "
                    "%s to the RAS index: %i\n",
                    tid, seqNum, pc, pc, RAS[tid].topIdx());
        }

        // The target address is not predicted by RAS.
        // Thus, BTB/IndirectBranch Predictor is employed.
        if (!inst->isReturn()) {
            if (inst->isDirectCtrl() || !iPred) {
                ++stats.BTBLookups;
                // Check BTB on direct branches
                const PCStateBase * btb_target = btb->lookup(tid,
                                                       pc.instAddr(),
                                                       getBranchType(inst));
                if (btb_target) {
                    ++stats.BTBHits;
                    // If it's not a return, use the BTB to get target addr.
                    set(target, btb_target);
                    DPRINTF(Branch,
                            "[tid:%i] [sn:%llu] Instruction %s predicted "
                            "target is %s\n",
                            tid, seqNum, pc, *target);
                } else {
                    DPRINTF(Branch, "[tid:%i] [sn:%llu] BTB doesn't have a "
                            "valid entry\n", tid, seqNum);
                    pred_taken = false;
                    predict_record.predTaken = pred_taken;
                    // The Direction of the branch predictor is altered
                    // because the BTB did not have an entry
                    // The predictor needs to be updated accordingly
                    if (!inst->isCall() && !inst->isReturn()) {
                        btbUpdate(tid, pc.instAddr(), bp_history);
                        DPRINTF(Branch,
                                "[tid:%i] [sn:%llu] btbUpdate "
                                "called for %s\n",
                                tid, seqNum, pc);
                    } else if (inst->isCall() && !inst->isUncondCtrl()) {
                        RAS[tid].pop();
                        predict_record.pushedRAS = false;
                    }
                    inst->advancePC(*target);
                }
            } else {
                predict_record.wasIndirect = true;
                ++stats.indirectLookups;
                //Consult indirect predictor on indirect control
                if (iPred->lookup(pc.instAddr(), *target, tid)) {
                    // Indirect predictor hit
                    ++stats.indirectHits;
                    DPRINTF(Branch,
                            "[tid:%i] [sn:%llu] Instruction %s predicted "
                            "indirect target is %s\n",
                            tid, seqNum, pc, *target);
                } else {
                    ++stats.indirectMisses;
                    pred_taken = false;
                    predict_record.predTaken = pred_taken;
                    DPRINTF(Branch,
                            "[tid:%i] [sn:%llu] Instruction %s no indirect "
                            "target\n",
                            tid, seqNum, pc);
                    if (!inst->isCall() && !inst->isReturn()) {

                    } else if (inst->isCall() && !inst->isUncondCtrl()) {
                        RAS[tid].pop();
                        predict_record.pushedRAS = false;
                    }
                    inst->advancePC(*target);
                }
                iPred->recordIndirect(pc.instAddr(), target->instAddr(),
                        seqNum, tid);
            }
        }
    } else {
        if (inst->isReturn()) {
           predict_record.wasReturn = true;
        }
        inst->advancePC(*target);
    }
    predict_record.target = target->instAddr();

    set(pc, *target);

    if (iPred) {
        // Update the indirect predictor with the direction prediction
        // Note that this happens after indirect lookup, so it does not use
        // the new information
        // Note also that we use orig_pred_taken instead of pred_taken in
        // as this is the actual outcome of the direction prediction
        iPred->updateDirectionInfo(tid, orig_pred_taken);
    }

    predHist[tid].push_front(predict_record);

    DPRINTF(Branch,
            "[tid:%i] [sn:%llu] History entry added. "
            "predHist.size(): %i\n",
            tid, seqNum, predHist[tid].size());

    return pred_taken;
}

void
BPredUnit::update(const InstSeqNum &done_sn, ThreadID tid)
{
    DPRINTF(Branch, "[tid:%i] Committing branches until "
            "sn:%llu]\n", tid, done_sn);

    while (!predHist[tid].empty() &&
           predHist[tid].back().seqNum <= done_sn) {
        // Update the branch predictor with the correct results.
        update(tid, predHist[tid].back().pc,
                    predHist[tid].back().predTaken,
                    predHist[tid].back().bpHistory, false,
                    predHist[tid].back().inst,
                    predHist[tid].back().target);

        if (iPred) {
            iPred->commit(done_sn, tid, predHist[tid].back().indirectHistory);
        }

        predHist[tid].pop_back();
    }
}

void
BPredUnit::squash(const InstSeqNum &squashed_sn, ThreadID tid)
{
    History &pred_hist = predHist[tid];

    if (iPred) {
        iPred->squash(squashed_sn, tid);
    }

    while (!pred_hist.empty() &&
           pred_hist.front().seqNum > squashed_sn) {
        if (pred_hist.front().wasCall && pred_hist.front().pushedRAS) {
             // Was a call but predicated false. Pop RAS here
             DPRINTF(Branch, "[tid:%i] [squash sn:%llu] Squashing"
                     "  Call [sn:%llu] PC: %s Popping RAS\n", tid, squashed_sn,
                     pred_hist.front().seqNum, pred_hist.front().pc);
             RAS[tid].pop();
        }
        if (pred_hist.front().usedRAS) {
            if (pred_hist.front().RASTarget != nullptr) {
                DPRINTF(Branch, "[tid:%i] [squash sn:%llu]"
                        " Restoring top of RAS to: %i,"
                        " target: %s\n", tid, squashed_sn,
                        pred_hist.front().RASIndex,
                        *pred_hist.front().RASTarget);
            }
            else {
                DPRINTF(Branch, "[tid:%i] [squash sn:%llu]"
                        " Restoring top of RAS to: %i,"
                        " target: INVALID_TARGET\n", tid, squashed_sn,
                        pred_hist.front().RASIndex);
            }

            RAS[tid].restore(pred_hist.front().RASIndex,
                             pred_hist.front().RASTarget.get());
        }

        // This call should delete the bpHistory.
        squash(tid, pred_hist.front().bpHistory);
        if (iPred) {
            iPred->deleteIndirectInfo(tid, pred_hist.front().indirectHistory);
        }

        DPRINTF(Branch, "[tid:%i] [squash sn:%llu] "
                "Removing history for [sn:%llu] "
                "PC %#x\n", tid, squashed_sn, pred_hist.front().seqNum,
                pred_hist.front().pc);

        pred_hist.pop_front();

        DPRINTF(Branch, "[tid:%i] [squash sn:%llu] predHist.size(): %i\n",
                tid, squashed_sn, predHist[tid].size());
    }
}

void
BPredUnit::squash(const InstSeqNum &squashed_sn,
                  const PCStateBase &corr_target,
                  bool actually_taken, ThreadID tid)
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

    DPRINTF(Branch, "[tid:%i] Squashing from sequence number %i, "
            "setting target to %s\n", tid, squashed_sn, corr_target);

    // Squash All Branches AFTER this mispredicted branch
    squash(squashed_sn, tid);

    // If there's a squash due to a syscall, there may not be an entry
    // corresponding to the squash.  In that case, don't bother trying to
    // fix up the entry.
    if (!pred_hist.empty()) {

        auto hist_it = pred_hist.begin();
        //HistoryIt hist_it = find(pred_hist.begin(), pred_hist.end(),
        //                       squashed_sn);

        //assert(hist_it != pred_hist.end());
        if (pred_hist.front().seqNum != squashed_sn) {
            DPRINTF(Branch, "Front sn %i != Squash sn %i\n",
                    pred_hist.front().seqNum, squashed_sn);

            assert(pred_hist.front().seqNum == squashed_sn);
        }


        if ((*hist_it).usedRAS) {
            ++stats.RASIncorrect;
            DPRINTF(Branch,
                    "[tid:%i] [squash sn:%llu] Incorrect RAS [sn:%llu]\n",
                    tid, squashed_sn, hist_it->seqNum);
        }

        // There are separate functions for in-order and out-of-order
        // branch prediction, but not for update. Therefore, this
        // call should take into account that the mispredicted branch may
        // be on the wrong path (i.e., OoO execution), and that the counter
        // counter table(s) should not be updated. Thus, this call should
        // restore the state of the underlying predictor, for instance the
        // local/global histories. The counter tables will be updated when
        // the branch actually commits.

        // Remember the correct direction for the update at commit.
        pred_hist.front().predTaken = actually_taken;
        pred_hist.front().target = corr_target.instAddr();

        update(tid, (*hist_it).pc, actually_taken,
               pred_hist.front().bpHistory, true, pred_hist.front().inst,
               corr_target.instAddr());

        if (iPred) {
            iPred->changeDirectionPrediction(tid,
                pred_hist.front().indirectHistory, actually_taken);
        }

        if (actually_taken) {
            if (hist_it->wasReturn && !hist_it->usedRAS) {
                 DPRINTF(Branch, "[tid:%i] [squash sn:%llu] "
                        "Incorrectly predicted "
                        "return [sn:%llu] PC: %#x\n", tid, squashed_sn,
                        hist_it->seqNum,
                        hist_it->pc);
                 RAS[tid].pop();
                 hist_it->usedRAS = true;
            }
            if (hist_it->wasIndirect) {
                ++stats.indirectMispredicted;
                if (iPred) {
                    iPred->recordTarget(
                        hist_it->seqNum, pred_hist.front().indirectHistory,
                        corr_target, tid);
                }
            } else {
                DPRINTF(Branch,"[tid:%i] [squash sn:%llu] "
                        "BTB Update called for [sn:%llu] "
                        "PC %#x\n", tid, squashed_sn,
                        hist_it->seqNum, hist_it->pc);

                ++stats.BTBUpdates;
                btb->update(tid, hist_it->pc, corr_target,
                            getBranchType(hist_it->inst));
            }
        } else {
           //Actually not Taken
           if (hist_it->wasCall && hist_it->pushedRAS) {
                 //Was a Call but predicated false. Pop RAS here
                 DPRINTF(Branch,
                        "[tid:%i] [squash sn:%llu] "
                        "Incorrectly predicted "
                        "Call [sn:%llu] PC: %s Popping RAS\n",
                        tid, squashed_sn,
                        hist_it->seqNum, hist_it->pc);
                 RAS[tid].pop();
                 hist_it->pushedRAS = false;
           }
           if (hist_it->usedRAS) {

                std::string RASTarget;

                DPRINTF(Branch,
                        "[tid:%i] [squash sn:%llu] Incorrectly predicted "
                        "return [sn:%llu] PC: %#x Restoring RAS\n", tid,
                        squashed_sn,
                        hist_it->seqNum, hist_it->pc);
                if (hist_it->RASTarget) {
                    std::ostringstream os;
                    os << *hist_it->RASTarget.get();
                    RASTarget = os.str();
                } else {
                    RASTarget = "no RAS";
                }
                DPRINTF(Branch,
                        "[tid:%i] [squash sn:%llu] Restoring top of RAS "
                        "to: %i, target: %s\n", tid, squashed_sn,
                        hist_it->RASIndex, RASTarget.c_str());
                RAS[tid].restore(hist_it->RASIndex, hist_it->RASTarget.get());
                hist_it->usedRAS = false;
           }
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
            auto pred_hist_it = ph.begin();

            cprintf("predHist[%i].size(): %i\n", i++, ph.size());

            while (pred_hist_it != ph.end()) {
                cprintf("sn:%llu], PC:%#x, tid:%i, predTaken:%i, "
                        "bpHistory:%#x\n",
                        pred_hist_it->seqNum, pred_hist_it->pc,
                        pred_hist_it->tid, pred_hist_it->predTaken,
                        pred_hist_it->bpHistory);
                pred_hist_it++;
            }

            cprintf("\n");
        }
    }
}

} // namespace branch_prediction
} // namespace gem5
