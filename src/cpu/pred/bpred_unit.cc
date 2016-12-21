/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010 The University of Edinburgh
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
 *
 * Authors: Kevin Lim
 */

#include "cpu/pred/bpred_unit.hh"

#include <algorithm>

#include "arch/isa_traits.hh"
#include "arch/types.hh"
#include "arch/utility.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/Branch.hh"

BPredUnit::BPredUnit(const Params *params)
    : SimObject(params),
      numThreads(params->numThreads),
      predHist(numThreads),
      BTB(params->BTBEntries,
          params->BTBTagSize,
          params->instShiftAmt,
          params->numThreads),
      RAS(numThreads),
      useIndirect(params->useIndirect),
      iPred(params->indirectHashGHR,
            params->indirectHashTargets,
            params->indirectSets,
            params->indirectWays,
            params->indirectTagSize,
            params->indirectPathLength,
            params->instShiftAmt,
            params->numThreads),
      instShiftAmt(params->instShiftAmt)
{
    for (auto& r : RAS)
        r.init(params->RASSize);
}

void
BPredUnit::regStats()
{
    SimObject::regStats();

    lookups
        .name(name() + ".lookups")
        .desc("Number of BP lookups")
        ;

    condPredicted
        .name(name() + ".condPredicted")
        .desc("Number of conditional branches predicted")
        ;

    condIncorrect
        .name(name() + ".condIncorrect")
        .desc("Number of conditional branches incorrect")
        ;

    BTBLookups
        .name(name() + ".BTBLookups")
        .desc("Number of BTB lookups")
        ;

    BTBHits
        .name(name() + ".BTBHits")
        .desc("Number of BTB hits")
        ;

    BTBCorrect
        .name(name() + ".BTBCorrect")
        .desc("Number of correct BTB predictions (this stat may not "
              "work properly.")
        ;

    BTBHitPct
        .name(name() + ".BTBHitPct")
        .desc("BTB Hit Percentage")
        .precision(6);
    BTBHitPct = (BTBHits / BTBLookups) * 100;

    usedRAS
        .name(name() + ".usedRAS")
        .desc("Number of times the RAS was used to get a target.")
        ;

    RASIncorrect
        .name(name() + ".RASInCorrect")
        .desc("Number of incorrect RAS predictions.")
        ;

    indirectLookups
        .name(name() + ".indirectLookups")
        .desc("Number of indirect predictor lookups.")
        ;

    indirectHits
        .name(name() + ".indirectHits")
        .desc("Number of indirect target hits.")
        ;

    indirectMisses
        .name(name() + ".indirectMisses")
        .desc("Number of indirect misses.")
        ;

    indirectMispredicted
        .name(name() + "indirectMispredicted")
        .desc("Number of mispredicted indirect branches.")
        ;

}

ProbePoints::PMUUPtr
BPredUnit::pmuProbePoint(const char *name)
{
    ProbePoints::PMUUPtr ptr;
    ptr.reset(new ProbePoints::PMU(getProbeManager(), name));

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
    for (const auto& ph M5_VAR_USED : predHist)
        assert(ph.empty());
}

bool
BPredUnit::predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                   TheISA::PCState &pc, ThreadID tid)
{
    // See if branch predictor predicts taken.
    // If so, get its target addr either from the BTB or the RAS.
    // Save off record of branch stuff so the RAS can be fixed
    // up once it's done.

    bool pred_taken = false;
    TheISA::PCState target = pc;

    ++lookups;
    ppBranches->notify(1);

    void *bp_history = NULL;

    if (inst->isUncondCtrl()) {
        DPRINTF(Branch, "[tid:%i]: Unconditional control.\n", tid);
        pred_taken = true;
        // Tell the BP there was an unconditional branch.
        uncondBranch(tid, pc.instAddr(), bp_history);
    } else {
        ++condPredicted;
        pred_taken = lookup(tid, pc.instAddr(), bp_history);

        DPRINTF(Branch, "[tid:%i]: [sn:%i] Branch predictor"
                " predicted %i for PC %s\n", tid, seqNum,  pred_taken, pc);
    }

    DPRINTF(Branch, "[tid:%i]: [sn:%i] Creating prediction history "
            "for PC %s\n", tid, seqNum, pc);

    PredictorHistory predict_record(seqNum, pc.instAddr(),
                                    pred_taken, bp_history, tid);

    // Now lookup in the BTB or RAS.
    if (pred_taken) {
        if (inst->isReturn()) {
            ++usedRAS;
            predict_record.wasReturn = true;
            // If it's a function return call, then look up the address
            // in the RAS.
            TheISA::PCState rasTop = RAS[tid].top();
            target = TheISA::buildRetPC(pc, rasTop);

            // Record the top entry of the RAS, and its index.
            predict_record.usedRAS = true;
            predict_record.RASIndex = RAS[tid].topIdx();
            predict_record.RASTarget = rasTop;

            RAS[tid].pop();

            DPRINTF(Branch, "[tid:%i]: Instruction %s is a return, "
                    "RAS predicted target: %s, RAS index: %i.\n",
                    tid, pc, target, predict_record.RASIndex);
        } else {
            ++BTBLookups;

            if (inst->isCall()) {
                RAS[tid].push(pc);
                predict_record.pushedRAS = true;

                // Record that it was a call so that the top RAS entry can
                // be popped off if the speculation is incorrect.
                predict_record.wasCall = true;

                DPRINTF(Branch, "[tid:%i]: Instruction %s was a "
                        "call, adding %s to the RAS index: %i.\n",
                        tid, pc, pc, RAS[tid].topIdx());
            }

            if (inst->isDirectCtrl() || !useIndirect) {
                // Check BTB on direct branches
                if (BTB.valid(pc.instAddr(), tid)) {
                    ++BTBHits;

                    // If it's not a return, use the BTB to get target addr.
                    target = BTB.lookup(pc.instAddr(), tid);

                    DPRINTF(Branch, "[tid:%i]: Instruction %s predicted"
                            " target is %s.\n", tid, pc, target);

                } else {
                    DPRINTF(Branch, "[tid:%i]: BTB doesn't have a "
                            "valid entry.\n",tid);
                    pred_taken = false;
                    // The Direction of the branch predictor is altered
                    // because the BTB did not have an entry
                    // The predictor needs to be updated accordingly
                    if (!inst->isCall() && !inst->isReturn()) {
                        btbUpdate(tid, pc.instAddr(), bp_history);
                        DPRINTF(Branch, "[tid:%i]:[sn:%i] btbUpdate"
                                " called for %s\n", tid, seqNum, pc);
                    } else if (inst->isCall() && !inst->isUncondCtrl()) {
                        RAS[tid].pop();
                        predict_record.pushedRAS = false;
                    }
                    TheISA::advancePC(target, inst);
                }
            } else {
                predict_record.wasIndirect = true;
                ++indirectLookups;
                //Consult indirect predictor on indirect control
                if (iPred.lookup(pc.instAddr(), getGHR(tid, bp_history),
                        target, tid)) {
                    // Indirect predictor hit
                    ++indirectHits;
                    DPRINTF(Branch, "[tid:%i]: Instruction %s predicted "
                            "indirect target is %s.\n", tid, pc, target);
                } else {
                    ++indirectMisses;
                    pred_taken = false;
                    DPRINTF(Branch, "[tid:%i]: Instruction %s no indirect "
                            "target.\n", tid, pc);
                    if (!inst->isCall() && !inst->isReturn()) {

                    } else if (inst->isCall() && !inst->isUncondCtrl()) {
                        RAS[tid].pop();
                        predict_record.pushedRAS = false;
                    }
                    TheISA::advancePC(target, inst);
                }
                iPred.recordIndirect(pc.instAddr(), target.instAddr(), seqNum,
                        tid);
            }
        }
    } else {
        if (inst->isReturn()) {
           predict_record.wasReturn = true;
        }
        TheISA::advancePC(target, inst);
    }

    pc = target;

    predHist[tid].push_front(predict_record);

    DPRINTF(Branch, "[tid:%i]: [sn:%i]: History entry added."
            "predHist.size(): %i\n", tid, seqNum, predHist[tid].size());

    return pred_taken;
}

void
BPredUnit::update(const InstSeqNum &done_sn, ThreadID tid)
{
    DPRINTF(Branch, "[tid:%i]: Committing branches until "
            "[sn:%lli].\n", tid, done_sn);

    iPred.commit(done_sn, tid);
    while (!predHist[tid].empty() &&
           predHist[tid].back().seqNum <= done_sn) {
        // Update the branch predictor with the correct results.
        update(tid, predHist[tid].back().pc,
                    predHist[tid].back().predTaken,
                    predHist[tid].back().bpHistory, false);

        predHist[tid].pop_back();
    }
}

void
BPredUnit::squash(const InstSeqNum &squashed_sn, ThreadID tid)
{
    History &pred_hist = predHist[tid];

    iPred.squash(squashed_sn, tid);
    while (!pred_hist.empty() &&
           pred_hist.front().seqNum > squashed_sn) {
        if (pred_hist.front().usedRAS) {
            DPRINTF(Branch, "[tid:%i]: Restoring top of RAS to: %i,"
                    " target: %s.\n", tid,
                    pred_hist.front().RASIndex, pred_hist.front().RASTarget);

            RAS[tid].restore(pred_hist.front().RASIndex,
                             pred_hist.front().RASTarget);
        } else if (pred_hist.front().wasCall && pred_hist.front().pushedRAS) {
             // Was a call but predicated false. Pop RAS here
             DPRINTF(Branch, "[tid: %i] Squashing"
                     "  Call [sn:%i] PC: %s Popping RAS\n", tid,
                     pred_hist.front().seqNum, pred_hist.front().pc);
             RAS[tid].pop();
        }

        // This call should delete the bpHistory.
        squash(tid, pred_hist.front().bpHistory);

        DPRINTF(Branch, "[tid:%i]: Removing history for [sn:%i] "
                "PC %s.\n", tid, pred_hist.front().seqNum,
                pred_hist.front().pc);

        pred_hist.pop_front();

        DPRINTF(Branch, "[tid:%i]: predHist.size(): %i\n",
                tid, predHist[tid].size());
    }
}

void
BPredUnit::squash(const InstSeqNum &squashed_sn,
                  const TheISA::PCState &corrTarget,
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

    ++condIncorrect;
    ppMisses->notify(1);

    DPRINTF(Branch, "[tid:%i]: Squashing from sequence number %i, "
            "setting target to %s.\n", tid, squashed_sn, corrTarget);

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
            ++RASIncorrect;
            DPRINTF(Branch, "[tid:%i]: Incorrect RAS [sn:%i]\n",
                    tid, hist_it->seqNum);
        }

        // Get the underlying Global History Register
        unsigned ghr = getGHR(tid, hist_it->bpHistory);

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

        update(tid, (*hist_it).pc, actually_taken,
               pred_hist.front().bpHistory, true);

        if (actually_taken) {
            if (hist_it->wasReturn && !hist_it->usedRAS) {
                 DPRINTF(Branch, "[tid: %i] Incorrectly predicted"
                         "  return [sn:%i] PC: %s\n", tid, hist_it->seqNum,
                         hist_it->pc);
                 RAS[tid].pop();
                 hist_it->usedRAS = true;
            }
            if (hist_it->wasIndirect) {
                ++indirectMispredicted;
                iPred.recordTarget(hist_it->seqNum, ghr, corrTarget, tid);
            } else {
                DPRINTF(Branch,"[tid: %i] BTB Update called for [sn:%i]"
                        " PC: %s\n", tid,hist_it->seqNum, hist_it->pc);

                BTB.update((*hist_it).pc, corrTarget, tid);
            }
        } else {
           //Actually not Taken
           if (hist_it->usedRAS) {
                DPRINTF(Branch,"[tid: %i] Incorrectly predicted"
                        "  return [sn:%i] PC: %s Restoring RAS\n", tid,
                        hist_it->seqNum, hist_it->pc);
                DPRINTF(Branch, "[tid:%i]: Restoring top of RAS"
                        " to: %i, target: %s.\n", tid,
                        hist_it->RASIndex, hist_it->RASTarget);
                RAS[tid].restore(hist_it->RASIndex, hist_it->RASTarget);
                hist_it->usedRAS = false;
           } else if (hist_it->wasCall && hist_it->pushedRAS) {
                 //Was a Call but predicated false. Pop RAS here
                 DPRINTF(Branch, "[tid: %i] Incorrectly predicted"
                         "  Call [sn:%i] PC: %s Popping RAS\n", tid,
                         hist_it->seqNum, hist_it->pc);
                 RAS[tid].pop();
                 hist_it->pushedRAS = false;
           }
        }
    } else {
        DPRINTF(Branch, "[tid:%i]: [sn:%i] pred_hist empty, can't "
                "update.\n", tid, squashed_sn);
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
                cprintf("[sn:%lli], PC:%#x, tid:%i, predTaken:%i, "
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

