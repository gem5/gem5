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
 *
 * Authors: Kevin Lim
 */

#include <list>
#include <vector>

#include "base/trace.hh"
#include "base/traceflags.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/resources/bpred_unit.hh"

using namespace std;
using namespace ThePipeline;

BPredUnit::BPredUnit(Resource *_res, ThePipeline::Params *params)
    : res(_res), 
      BTB(params->BTBEntries, params->BTBTagSize, params->instShiftAmt)
{
    // Setup the selected predictor.
    if (params->predType == "local") {
        localBP = new LocalBP(params->localPredictorSize,
                              params->localCtrBits,
                              params->instShiftAmt);
        predictor = Local;
    } else if (params->predType == "tournament") {
        tournamentBP = new TournamentBP(params->localPredictorSize,
                                        params->localCtrBits,
                                        params->localHistoryTableSize,
                                        params->localHistoryBits,
                                        params->globalPredictorSize,
                                        params->globalHistoryBits,
                                        params->globalCtrBits,
                                        params->choicePredictorSize,
                                        params->choiceCtrBits,
                                        params->instShiftAmt);
        predictor = Tournament;
    } else {
        fatal("Invalid BP selected!");
    }

    for (int i=0; i < ThePipeline::MaxThreads; i++)
        RAS[i].init(params->RASSize);

    instSize = sizeof(TheISA::MachInst);
}

std::string
BPredUnit::name()
{
    return res->name();
}

void
BPredUnit::regStats()
{
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
}


void
BPredUnit::switchOut()
{
    // Clear any state upon switch out.
    for (int i = 0; i < ThePipeline::MaxThreads; ++i) {
        squash(0, i);
    }
}


void
BPredUnit::takeOverFrom()
{
    // Can reset all predictor state, but it's not necessarily better
    // than leaving it be.
/*
    for (int i = 0; i < ThePipeline::MaxThreads; ++i)
        RAS[i].reset();

    BP.reset();
    BTB.reset();
*/
}


bool
BPredUnit::predict(DynInstPtr &inst, Addr &pred_PC, ThreadID tid)
{
    // See if branch predictor predicts taken.
    // If so, get its target addr either from the BTB or the RAS.
    // Save off record of branch stuff so the RAS can be fixed
    // up once it's done.

    using TheISA::MachInst;
    
    int asid = inst->asid;
    bool pred_taken = false;
    Addr target;

    ++lookups;
    DPRINTF(InOrderBPred, "[tid:%i] [sn:%i] %s ... PC%#x doing branch "
            "prediction\n", tid, inst->seqNum,
            inst->staticInst->disassemble(inst->PC), inst->readPC());


    void *bp_history = NULL;

    if (inst->isUncondCtrl()) {
        DPRINTF(InOrderBPred, "[tid:%i] Unconditional control.\n",
                tid);
        pred_taken = true;
        // Tell the BP there was an unconditional branch.
        BPUncond(bp_history);

        if (inst->isReturn() && RAS[tid].empty()) {
            DPRINTF(InOrderBPred, "[tid:%i] RAS is empty, predicting "
                    "false.\n", tid);
            pred_taken = false;
        }
    } else {
        ++condPredicted;

        pred_taken = BPLookup(pred_PC, bp_history);

        DPRINTF(InOrderBPred, "[tid:%i]: Branch predictor predicted %i "
                "for PC %#x\n",
                tid, pred_taken, inst->readPC());
    }

    PredictorHistory predict_record(inst->seqNum, pred_PC, pred_taken,
                                    bp_history, tid);

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

            DPRINTF(InOrderBPred, "[tid:%i]: Instruction %#x is a return, "
                    "RAS predicted target: %#x, RAS index: %i.\n",
                    tid, inst->readPC(), target, predict_record.RASIndex);
        } else {
            ++BTBLookups;

            if (inst->isCall()) {

#if ISA_HAS_DELAY_SLOT
                Addr ras_pc = pred_PC + instSize; // Next Next PC
#else
                Addr ras_pc = pred_PC; // Next PC
#endif

                RAS[tid].push(ras_pc);

                // Record that it was a call so that the top RAS entry can
                // be popped off if the speculation is incorrect.
                predict_record.wasCall = true;

                DPRINTF(InOrderBPred, "[tid:%i]: Instruction %#x was a call"
                        ", adding %#x to the RAS index: %i.\n",
                        tid, inst->readPC(), ras_pc, RAS[tid].topIdx());
            }

            if (inst->isCall() &&
                inst->isUncondCtrl() &&
                inst->isDirectCtrl()) {
                target = inst->branchTarget();

                DPRINTF(InOrderBPred, "[tid:%i]: Setting %#x predicted"
                        " target to %#x.\n",
                        tid, inst->readPC(), target);
            } else if (BTB.valid(pred_PC, asid)) {
                ++BTBHits;

                // If it's not a return, use the BTB to get the target addr.
                target = BTB.lookup(pred_PC, asid);

                DPRINTF(InOrderBPred, "[tid:%i]: [asid:%i] Instruction %#x "
                        "predicted target is %#x.\n",
                        tid, asid, inst->readPC(), target);
            } else {
                DPRINTF(InOrderBPred, "[tid:%i]: BTB doesn't have a "
                        "valid entry.\n",tid);
                pred_taken = false;
            }
        }
    }

    if (pred_taken) {
        // Set the PC and the instruction's predicted target.
        pred_PC = target;
    } else {
#if ISA_HAS_DELAY_SLOT
        // This value will be inst->PC + 4 (nextPC)
        // Delay Slot archs need this to be inst->PC + 8 (nextNPC)
        // so we increment one more time here.
        pred_PC = pred_PC + instSize;
#endif
    }

    predHist[tid].push_front(predict_record);

    DPRINTF(InOrderBPred, "[tid:%i] [sn:%i] pushed onto front of predHist "
            "...predHist.size(): %i\n",
            tid, inst->seqNum, predHist[tid].size());

    inst->setBranchPred(pred_taken);

    return pred_taken;
}


void
BPredUnit::update(const InstSeqNum &done_sn, ThreadID tid)
{
    DPRINTF(Resource, "BranchPred: [tid:%i]: Commiting branches until sequence"
            "number %lli.\n", tid, done_sn);

    while (!predHist[tid].empty() &&
           predHist[tid].back().seqNum <= done_sn) {
        // Update the branch predictor with the correct results.
        BPUpdate(predHist[tid].back().PC,
                 predHist[tid].back().predTaken,
                 predHist[tid].back().bpHistory);

        predHist[tid].pop_back();
    }
}


void
BPredUnit::squash(const InstSeqNum &squashed_sn, ThreadID tid, ThreadID asid)
{
    History &pred_hist = predHist[tid];

    while (!pred_hist.empty() &&
           pred_hist.front().seqNum > squashed_sn) {
        if (pred_hist.front().usedRAS) {
            DPRINTF(InOrderBPred, "BranchPred: [tid:%i]: Restoring top of RAS "
                    "to: %i, target: %#x.\n",
                    tid,
                    pred_hist.front().RASIndex,
                    pred_hist.front().RASTarget);

            RAS[tid].restore(pred_hist.front().RASIndex,
                             pred_hist.front().RASTarget);

        } else if (pred_hist.front().wasCall) {
            DPRINTF(InOrderBPred, "BranchPred: [tid:%i]: Removing speculative "
                    "entry added to the RAS.\n",tid);

            RAS[tid].pop();
        }

        // This call should delete the bpHistory.
        BPSquash(pred_hist.front().bpHistory);

        pred_hist.pop_front();
    }

}


void
BPredUnit::squash(const InstSeqNum &squashed_sn,
                  const Addr &corr_target,
                  bool actually_taken,
                  ThreadID tid,
                  ThreadID asid)
{
    // Now that we know that a branch was mispredicted, we need to undo
    // all the branches that have been seen up until this branch and
    // fix up everything.

    History &pred_hist = predHist[tid];

    ++condIncorrect;

    DPRINTF(InOrderBPred, "[tid:%i]: Squashing from sequence number %i, "
            "setting target to %#x.\n",
            tid, squashed_sn, corr_target);

    squash(squashed_sn, tid);

    // If there's a squash due to a syscall, there may not be an entry
    // corresponding to the squash.  In that case, don't bother trying to
    // fix up the entry.
    if (!pred_hist.empty()) {
        HistoryIt hist_it = pred_hist.begin();
        //HistoryIt hist_it = find(pred_hist.begin(), pred_hist.end(),
        //                       squashed_sn);

        //assert(hist_it != pred_hist.end());
        if (pred_hist.front().seqNum != squashed_sn) {
            DPRINTF(InOrderBPred, "Front sn %i != Squash sn %i\n",
                    pred_hist.front().seqNum, squashed_sn);

            assert(pred_hist.front().seqNum == squashed_sn);
        }


        if ((*hist_it).usedRAS) {
            ++RASIncorrect;
        }

        BPUpdate((*hist_it).PC, actually_taken,
                 pred_hist.front().bpHistory);

        BTB.update((*hist_it).PC, corr_target, asid);

        DPRINTF(InOrderBPred, "[tid:%i]: Removing history for [sn:%i] "
                "PC %#x.\n", tid, (*hist_it).seqNum, (*hist_it).PC);

        pred_hist.erase(hist_it);

        DPRINTF(InOrderBPred, "[tid:%i]: predHist.size(): %i\n", tid,
                predHist[tid].size());

    } else {
        DPRINTF(InOrderBPred, "[tid:%i]: [sn:%i] pred_hist empty, can't "
                "update.\n", tid, squashed_sn);
    }
}


void
BPredUnit::BPUncond(void * &bp_history)
{
    // Only the tournament predictor cares about unconditional branches.
    if (predictor == Tournament) {
        tournamentBP->uncondBr(bp_history);
    }    
}


void
BPredUnit::BPSquash(void *bp_history)
{
    if (predictor == Local) {
        localBP->squash(bp_history);
    } else if (predictor == Tournament) {
        tournamentBP->squash(bp_history);
    } else {
        panic("Predictor type is unexpected value!");
    }    
}


bool
BPredUnit::BPLookup(Addr &inst_PC, void * &bp_history)
{
    if (predictor == Local) {
        return localBP->lookup(inst_PC, bp_history);
    } else if (predictor == Tournament) {
        return tournamentBP->lookup(inst_PC, bp_history);
    } else {
        panic("Predictor type is unexpected value!");
    }
}


void
BPredUnit::BPUpdate(Addr &inst_PC, bool taken, void *bp_history)
{
    if (predictor == Local) {
        localBP->update(inst_PC, taken, bp_history);
    } else if (predictor == Tournament) {
        tournamentBP->update(inst_PC, taken, bp_history);
    } else {
        panic("Predictor type is unexpected value!");
    }
}


void
BPredUnit::dump()
{
    /*typename History::iterator pred_hist_it;

    for (int i = 0; i < ThePipeline::MaxThreads; ++i) {
        if (!predHist[i].empty()) {
            pred_hist_it = predHist[i].begin();

            cprintf("predHist[%i].size(): %i\n", i, predHist[i].size());

            while (pred_hist_it != predHist[i].end()) {
                cprintf("[sn:%lli], PC:%#x, tid:%i, predTaken:%i, "
                        "bpHistory:%#x\n",
                        (*pred_hist_it).seqNum, (*pred_hist_it).PC,
                        (*pred_hist_it).tid, (*pred_hist_it).predTaken,
                        (*pred_hist_it).bpHistory);
                pred_hist_it++;
            }

            cprintf("\n");
        }
    }*/                
}
