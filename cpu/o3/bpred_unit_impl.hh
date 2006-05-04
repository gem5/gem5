/*
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

#include "base/trace.hh"
#include "base/traceflags.hh"
#include "cpu/o3/bpred_unit.hh"

#include <vector>
#include <list>

using namespace std;

template<class Impl>
TwobitBPredUnit<Impl>::TwobitBPredUnit(Params *params)
  : BP(params->localPredictorSize,
       params->localCtrBits,
       params->instShiftAmt),
    BTB(params->BTBEntries,
        params->BTBTagSize,
        params->instShiftAmt)
{
    for (int i=0; i < Impl::MaxThreads; i++)
        RAS[i].init(params->RASSize);
}

template <class Impl>
void
TwobitBPredUnit<Impl>::regStats()
{
    lookups
        .name(name() + ".BPredUnit.lookups")
        .desc("Number of BP lookups")
        ;

    condPredicted
        .name(name() + ".BPredUnit.condPredicted")
        .desc("Number of conditional branches predicted")
        ;

    condIncorrect
        .name(name() + ".BPredUnit.condIncorrect")
        .desc("Number of conditional branches incorrect")
        ;

    BTBLookups
        .name(name() + ".BPredUnit.BTBLookups")
        .desc("Number of BTB lookups")
        ;

    BTBHits
        .name(name() + ".BPredUnit.BTBHits")
        .desc("Number of BTB hits")
        ;

    BTBCorrect
        .name(name() + ".BPredUnit.BTBCorrect")
        .desc("Number of correct BTB predictions (this stat may not "
              "work properly.")
        ;

    usedRAS
        .name(name() + ".BPredUnit.usedRAS")
        .desc("Number of times the RAS was used to get a target.")
        ;

    RASIncorrect
        .name(name() + ".BPredUnit.RASInCorrect")
        .desc("Number of incorrect RAS predictions.")
        ;
}

template <class Impl>
void
TwobitBPredUnit<Impl>::switchOut()
{
    for (int i = 0; i < Impl::MaxThreads; ++i) {
        predHist[i].clear();
    }
}

template <class Impl>
void
TwobitBPredUnit<Impl>::takeOverFrom()
{
    for (int i = 0; i < Impl::MaxThreads; ++i)
        RAS[i].reset();

    BP.reset();
    BTB.reset();
}

template <class Impl>
bool
TwobitBPredUnit<Impl>::predict(DynInstPtr &inst, Addr &PC, unsigned tid)
{
    // See if branch predictor predicts taken.
    // If so, get its target addr either from the BTB or the RAS.
    // Once that's done, speculatively update the predictor?
    // Save off record of branch stuff so the RAS can be fixed
    // up once it's done.

    using TheISA::MachInst;

    bool pred_taken = false;
    Addr target;

    ++lookups;

    if (inst->isUncondCtrl()) {
        DPRINTF(Fetch, "BranchPred: [tid:%i] Unconditional control.\n", tid);
        pred_taken = true;
    } else {
        ++condPredicted;

        pred_taken = BPLookup(PC);

        DPRINTF(Fetch, "BranchPred: [tid:%i]: Branch predictor predicted %i "
                "for PC %#x\n",
                tid, pred_taken, inst->readPC());
    }

    PredictorHistory predict_record(inst->seqNum, PC, pred_taken, tid);

    // Now lookup in the BTB or RAS.
    if (pred_taken) {
        if (inst->isReturn()) {
            ++usedRAS;

            // If it's a function return call, then look up the address
            // in the RAS.
            target = RAS[tid].top();

            // Record the top entry of the RAS, and its index.
            predict_record.usedRAS = true;
            predict_record.RASIndex = RAS[tid].topIdx();
            predict_record.RASTarget = target;

            assert(predict_record.RASIndex < 16);

            RAS[tid].pop();

            DPRINTF(Fetch, "BranchPred: [tid:%i]: Instruction %#x is a return, "
                    "RAS predicted target: %#x, RAS index: %i.\n",
                    tid, inst->readPC(), target, predict_record.RASIndex);
        } else {
            ++BTBLookups;

            if (inst->isCall()) {
                RAS[tid].push(PC + sizeof(MachInst));

                // Record that it was a call so that the top RAS entry can
                // be popped off if the speculation is incorrect.
                predict_record.wasCall = true;

                DPRINTF(Fetch, "BranchPred: [tid:%i] Instruction %#x was a call"
                        ", adding %#x to the RAS.\n",
                        tid, inst->readPC(), PC + sizeof(MachInst));
            }

            if (BTB.valid(PC, tid)) {
                ++BTBHits;

                //If it's anything else, use the BTB to get the target addr.
                target = BTB.lookup(PC, tid);

                DPRINTF(Fetch, "BranchPred: [tid:%i]: Instruction %#x predicted"
                        " target is %#x.\n",
                        tid, inst->readPC(), target);

            } else {
                DPRINTF(Fetch, "BranchPred: [tid:%i]: BTB doesn't have a "
                        "valid entry.\n",tid);
                pred_taken = false;
            }

        }
    }

    if (pred_taken) {
        // Set the PC and the instruction's predicted target.
        PC = target;
        inst->setPredTarg(target);
    } else {
        PC = PC + sizeof(MachInst);
        inst->setPredTarg(PC);
    }

    predHist[tid].push_front(predict_record);

    DPRINTF(Fetch, "[tid:%i] predHist.size(): %i\n", tid, predHist[tid].size());

    return pred_taken;
}

template <class Impl>
void
TwobitBPredUnit<Impl>::update(const InstSeqNum &done_sn, unsigned tid)
{
    DPRINTF(Fetch, "BranchPred: [tid:%i]: Commiting branches until sequence"
            "number %lli.\n", tid, done_sn);

    while (!predHist[tid].empty() &&
           predHist[tid].back().seqNum <= done_sn) {
        // Update the branch predictor with the correct results.
        BP.update(predHist[tid].back().PC,
                  predHist[tid].back().predTaken);

        predHist[tid].pop_back();
    }
}

template <class Impl>
void
TwobitBPredUnit<Impl>::squash(const InstSeqNum &squashed_sn, unsigned tid)
{
    History &pred_hist = predHist[tid];

    while (!pred_hist.empty() &&
           pred_hist.front().seqNum > squashed_sn) {
       if (pred_hist.front().usedRAS) {
            DPRINTF(Fetch, "BranchPred: [tid:%i]: Restoring top of RAS to: %i,"
                    " target: %#x.\n",
                    tid,
                    pred_hist.front().RASIndex,
                    pred_hist.front().RASTarget);

            RAS[tid].restore(pred_hist.front().RASIndex,
                             pred_hist.front().RASTarget);

        } else if (pred_hist.front().wasCall) {
            DPRINTF(Fetch, "BranchPred: [tid:%i]: Removing speculative entry added "
                    "to the RAS.\n",tid);

            RAS[tid].pop();
        }

        pred_hist.pop_front();
    }

}

template <class Impl>
void
TwobitBPredUnit<Impl>::squash(const InstSeqNum &squashed_sn,
                              const Addr &corr_target,
                              const bool actually_taken,
                              unsigned tid)
{
    // Now that we know that a branch was mispredicted, we need to undo
    // all the branches that have been seen up until this branch and
    // fix up everything.

    History &pred_hist = predHist[tid];

    ++condIncorrect;

    DPRINTF(Fetch, "BranchPred: [tid:%i]: Squashing from sequence number %i, "
            "setting target to %#x.\n",
            tid, squashed_sn, corr_target);

    while (!pred_hist.empty() &&
           pred_hist.front().seqNum > squashed_sn) {
        if (pred_hist.front().usedRAS) {
            DPRINTF(Fetch, "BranchPred: [tid:%i]: Restoring top of RAS to: %i, "
                    "target: %#x.\n",
                    tid,
                    pred_hist.front().RASIndex,
                    pred_hist.front().RASTarget);

            RAS[tid].restore(pred_hist.front().RASIndex,
                             pred_hist.front().RASTarget);
        } else if (pred_hist.front().wasCall) {
            DPRINTF(Fetch, "BranchPred: [tid:%i]: Removing speculative entry"
                    " added to the RAS.\n", tid);

            RAS[tid].pop();
        }

        pred_hist.pop_front();
    }

    // If there's a squash due to a syscall, there may not be an entry
    // corresponding to the squash.  In that case, don't bother trying to
    // fix up the entry.
    if (!pred_hist.empty()) {
        pred_hist.front().predTaken = actually_taken;

        if (pred_hist.front().usedRAS) {
            ++RASIncorrect;
        }

        BP.update(pred_hist.front().PC, actually_taken);

        BTB.update(pred_hist.front().PC, corr_target, tid);
        pred_hist.pop_front();
    }
}
