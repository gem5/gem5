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
    // After this is working, change this to a reinterpret cast
    // for performance considerations
    ResourceRequest* bpred_req = reqMap[slot_num];

    DynInstPtr inst = bpred_req->inst;
    ThreadID tid = inst->readTid();
    int seq_num = inst->seqNum;
    //int stage_num = bpred_req->getStageNum();

    bpred_req->fault = NoFault;

    switch (bpred_req->cmd)
    {
      case PredictBranch:
        {
            if (inst->seqNum > cpu->squashSeqNum[tid] &&
                curTick() == cpu->lastSquashCycle[tid]) {
                DPRINTF(InOrderStage, "[tid:%u]: [sn:%i]: squashed, "
                        "skipping prediction \n", tid, inst->seqNum);
            } else {
                TheISA::PCState predPC = inst->pcState();
                TheISA::advancePC(predPC, inst->staticInst);

                if (inst->isControl()) {
                    // If not, the pred_PC be updated to pc+8
                    // If predicted, the pred_PC will be updated to new target
                    // value
                    bool predict_taken = branchPred.predict(inst, predPC, tid);

                    if (predict_taken) {
                        DPRINTF(InOrderBPred, "[tid:%i]: [sn:%i]: Branch "
                                "predicted true.\n", tid, seq_num);
                        predictedTaken++;
                    } else {
                        DPRINTF(InOrderBPred, "[tid:%i]: [sn:%i]: Branch "
                                "predicted false.\n", tid, seq_num);
                        predictedNotTaken++;
                    }

                    inst->setPredTarg(predPC);

                    inst->setBranchPred(predict_taken);

                    DPRINTF(InOrderBPred, "[tid:%i]: [sn:%i]: Predicted PC is "
                            "%s.\n", tid, seq_num, predPC);

                } else {
                    inst->setPredTarg(predPC);
                    //DPRINTF(InOrderBPred, "[tid:%i]: Ignoring [sn:%i] "
                    //      "because this isn't "
                    //      "a control instruction.\n", tid, seq_num);
                }
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
    DPRINTF(InOrderBPred, "[tid:%i][sn:%i] Squashing...\n", tid, inst->seqNum);

#if ISA_HAS_DELAY_SLOT
    // We need to squash the actual branch , NOT the delay slot
    // in the branch predictor
    squash_seq_num = squash_seq_num - 1;
#endif

    if (squash_stage >= ThePipeline::BackEndStartStage) {
        bool taken = inst->predTaken();
        branchPred.squash(squash_seq_num, inst->readPredTarg(), taken, tid);
    } else {
        branchPred.squash(squash_seq_num, tid);
    }
}

void
BranchPredictor::instGraduated(InstSeqNum seq_num, ThreadID tid)
{
    branchPred.update(seq_num, tid);
}
