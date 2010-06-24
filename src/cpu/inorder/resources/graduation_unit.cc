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

#include "cpu/inorder/resources/graduation_unit.hh"

using namespace ThePipeline;

GraduationUnit::GraduationUnit(std::string res_name, int res_id, int res_width,
                               int res_latency, InOrderCPU *_cpu,
                               ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      lastCycleGrad(0), numCycleGrad(0)
      
{
    for (ThreadID tid = 0; tid < ThePipeline::MaxThreads; tid++) {
        nonSpecInstActive[tid] = &cpu->nonSpecInstActive[tid];
        nonSpecSeqNum[tid] = &cpu->nonSpecSeqNum[tid];
    }
}

void
GraduationUnit::execute(int slot_num)
{
    ResourceRequest* grad_req = reqMap[slot_num];
    DynInstPtr inst = reqMap[slot_num]->inst;
    Fault fault = reqMap[slot_num]->fault;
    ThreadID tid = inst->readTid();
    int stage_num = inst->resSched.top()->stageNum;

    grad_req->fault = NoFault;

    switch (grad_req->cmd)
    {
      case GraduateInst:
        {
            // @TODO: Instructions should never really get to this point since
            // this should be handled through the request interface. Check to
            // make sure this happens and delete this code.
            if (lastCycleGrad != curTick) {
                lastCycleGrad = curTick;
                numCycleGrad = 0;
            } else if (numCycleGrad > width) {
                DPRINTF(InOrderGraduation,
                        "Graduation bandwidth reached for this cycle.\n");
                return;
            }

            // Make sure this is the last thing on the resource schedule
            assert(inst->resSched.size() == 1);

            DPRINTF(InOrderGraduation,
                    "[tid:%i] Graduating instruction [sn:%i].\n",
                    tid, inst->seqNum);

            // Release Non-Speculative "Block" on instructions that could not
            // execute because there was a non-speculative inst. active.
            // @TODO: Fix this functionality. Probably too conservative.
            if (inst->isNonSpeculative()) {
                *nonSpecInstActive[tid] = false;
                DPRINTF(InOrderGraduation,
                        "[tid:%i] Non-speculative inst [sn:%i] graduated\n",
                        tid, inst->seqNum);
            }

            if (inst->traceData) {
                inst->traceData->setStageCycle(stage_num, curTick);
            }

            // Tell CPU that instruction is finished processing
            cpu->instDone(inst, tid);

            //cpu->pipelineStage[stage_num]->toPrevStages->
            //stageInfo[stage_num][tid].doneSeqNum = inst->seqNum;

            grad_req->done();
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }

}
