/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 *
 */

#include "config/the_isa.hh"
#include "cpu/inorder/resources/branch_predictor.hh"
#include "debug/InOrderBPred.hh"
#include "debug/InOrderStage.hh"
#include "debug/Resource.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

BranchPredictor::BranchPredictor(std::string res_name, int res_id, int res_width,
                                 int res_latency, InOrderCPU *_cpu,
                                 ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      branchPred(this, params)
{
    instSize = sizeof(MachInst);
}

void
BranchPredictor::regStats()
{
    predictedTaken
        .name(name() + ".predictedTaken")
        .desc("Number of Branches Predicted As Taken (True).");

    predictedNotTaken
        .name(name() + ".predictedNotTaken")
        .desc("Number of Branches Predicted As Not Taken (False).");

    Resource::regStats();
   
    branchPred.regStats();
}

void
BranchPredictor::execute(int slot_num)
{
    ResourceRequest* bpred_req = reqs[slot_num];
    DynInstPtr inst = bpred_req->inst;
    if (inst->fault != NoFault) {
        DPRINTF(InOrderBPred,
                "[tid:%i]: [sn:%i]: Detected %s fault @ %x. Forwarding to "
                "next stage.\n", inst->readTid(), inst->seqNum, inst->fault->name(),
                inst->pcState());
        bpred_req->done();
        return;
    }

    if (!inst->isControl()) {
        DPRINTF(Resource, "Ignoring %s, not a control inst.\n",
                inst->instName());
        bpred_req->done();
        return;
    }

    ThreadID tid = inst->readTid();
    InstSeqNum seq_num = inst->seqNum;
    switch (bpred_req->cmd)
    {
      case PredictBranch:
        {
            if (inst->seqNum > cpu->squashSeqNum[tid] &&
                curTick() == cpu->lastSquashCycle[tid]) {
                DPRINTF(InOrderStage, "[tid:%u]: [sn:%i]: squashed, "
                        "skipping prediction \n", tid, inst->seqNum);
            } else {
                TheISA::PCState pred_PC = inst->pcState();
                TheISA::advancePC(pred_PC, inst->staticInst);

                if (inst->isControl()) {
                    // If not, the pred_PC be updated to pc+8
                    // If predicted, the pred_PC will be updated to new target
                    // value
                    bool predict_taken = branchPred.predict(inst, pred_PC, tid);

                    if (predict_taken) {
                        DPRINTF(InOrderBPred, "[tid:%i]: [sn:%i]: Branch "
                                "predicted true.\n", tid, seq_num);
                        predictedTaken++;
                    } else {
                        DPRINTF(InOrderBPred, "[tid:%i]: [sn:%i]: Branch "
                                "predicted false.\n", tid, seq_num);
                        predictedNotTaken++;
                    }

                    inst->setBranchPred(predict_taken);
                }

                //@todo: Check to see how hw_rei is handled here...how does PC,NPC get
                //       updated to compare mispredict against???
                inst->setPredTarg(pred_PC);
                DPRINTF(InOrderBPred, "[tid:%i]: [sn:%i]: %s Predicted PC is "
                        "%s.\n", tid, seq_num, inst->instName(), pred_PC);
            }

            bpred_req->done();
        }
        break;

      case UpdatePredictor:
        {
            if (inst->seqNum > cpu->squashSeqNum[tid] &&
                curTick() == cpu->lastSquashCycle[tid]) {
                DPRINTF(InOrderStage, "[tid:%u]: [sn:%i]: squashed, "
                        "skipping branch predictor update \n",
                        tid, inst->seqNum);
            } else {
                DPRINTF(InOrderBPred, "[tid:%i]: [sn:%i]: Updating "
                        "Branch Predictor.\n",
                        tid, seq_num);


                branchPred.update(seq_num, tid);
            }

            bpred_req->done();
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}

void
BranchPredictor::squash(DynInstPtr inst, int squash_stage,
                        InstSeqNum squash_seq_num, ThreadID tid)
{
    InstSeqNum bpred_squash_num = inst->seqNum;
    DPRINTF(InOrderBPred, "[tid:%i][sn:%i] Squashing...\n", tid,
            bpred_squash_num);

    // update due to branch resolution
    if (squash_stage >= ThePipeline::BackEndStartStage) {
        branchPred.squash(bpred_squash_num,
                          inst->pcState(),
                          inst->pcState().branching(),
                          tid);
    } else {
    // update due to predicted taken branch
        branchPred.squash(bpred_squash_num, tid);
    }
}

void
BranchPredictor::instGraduated(InstSeqNum seq_num, ThreadID tid)
{
    branchPred.update(seq_num, tid);
}
