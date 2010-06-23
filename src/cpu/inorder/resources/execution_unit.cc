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

#include <vector>
#include <list>
#include "cpu/inorder/resources/execution_unit.hh"
#include "cpu/inorder/resource_pool.hh"
#include "cpu/inorder/cpu.hh"

using namespace std;
using namespace ThePipeline;

ExecutionUnit::ExecutionUnit(string res_name, int res_id, int res_width,
                 int res_latency, InOrderCPU *_cpu, ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu)
{ }

void
ExecutionUnit::regStats()
{
    predictedTakenIncorrect
        .name(name() + ".predictedTakenIncorrect")
        .desc("Number of Branches Incorrectly Predicted As Taken.");

    predictedNotTakenIncorrect
        .name(name() + ".predictedNotTakenIncorrect")
        .desc("Number of Branches Incorrectly Predicted As Not Taken).");

    lastExecuteCycle = curTick;

    executions
        .name(name() + ".executions")
        .desc("Number of Instructions Executed.");

    Resource::regStats();
}

void
ExecutionUnit::execute(int slot_num)
{
    ResourceRequest* exec_req = reqMap[slot_num];
    DynInstPtr inst = reqMap[slot_num]->inst;
    Fault fault = reqMap[slot_num]->fault;
    ThreadID tid = inst->readTid();
    int seq_num = inst->seqNum;

    exec_req->fault = NoFault;

    DPRINTF(InOrderExecute, "[tid:%i] Executing [sn:%i] [PC:%#x] %s.\n",
            tid, seq_num, inst->readPC(), inst->instName());

    switch (exec_req->cmd)
    {
      case ExecuteInst:
        {
            if (curTick != lastExecuteCycle) {
                lastExecuteCycle = curTick;
            }


            if (inst->isMemRef()) {
                panic("%s not configured to handle memory ops.\n", resName);
            } else if (inst->isControl()) {
                // Evaluate Branch
                fault = inst->execute();
                executions++;

                inst->setExecuted();

                if (fault == NoFault) {
                    // If branch is mispredicted, then signal squash
                    // throughout all stages behind the pipeline stage
                    // that got squashed.
                    if (inst->mispredicted()) {
                        int stage_num = exec_req->getStageNum();
                        ThreadID tid = inst->readTid();

                        // If it's a branch ...
                        if (inst->isDirectCtrl()) {
                            assert(!inst->isIndirectCtrl());

                            if (inst->predTaken() && inst->isCondDelaySlot()) {
                                inst->bdelaySeqNum = seq_num;
                                inst->setPredTarg(inst->nextPC);

                                DPRINTF(InOrderExecute, "[tid:%i]: Conditional branch inst"
                                        "[sn:%i] PC %#x mispredicted as taken.\n", tid,
                                        seq_num, inst->PC);
                            } else if (!inst->predTaken() && inst->isCondDelaySlot()) {
                                inst->bdelaySeqNum = seq_num;
                                inst->setPredTarg(inst->nextPC);
                                inst->procDelaySlotOnMispred = true;

                                DPRINTF(InOrderExecute, "[tid:%i]: Conditional branch inst."
                                        "[sn:%i] PC %#x mispredicted as not taken.\n", tid,
                                        seq_num, inst->PC);
                            } else {
#if ISA_HAS_DELAY_SLOT
                                inst->bdelaySeqNum = seq_num + 1;
                                inst->setPredTarg(inst->nextNPC);
#else
                                inst->bdelaySeqNum = seq_num;
                                inst->setPredTarg(inst->nextPC);
#endif
                                DPRINTF(InOrderExecute, "[tid:%i]: Misprediction detected at "
                                        "[sn:%i] PC %#x,\n\t squashing after delay slot "
                                        "instruction [sn:%i].\n",
                                        tid, seq_num, inst->PC, inst->bdelaySeqNum);
                                DPRINTF(InOrderStall, "STALL: [tid:%i]: Branch "
                                        "misprediction at %#x\n", tid, inst->PC);
                            }

                            DPRINTF(InOrderExecute, "[tid:%i] Redirecting fetch to %#x.\n", tid,
                                    inst->readPredTarg());

                        } else if(inst->isIndirectCtrl()){
#if ISA_HAS_DELAY_SLOT
                            inst->setPredTarg(inst->nextNPC);
                            inst->bdelaySeqNum = seq_num + 1;
#else
                            inst->setPredTarg(inst->nextPC);
                            inst->bdelaySeqNum = seq_num;
#endif

                            DPRINTF(InOrderExecute, "[tid:%i] Redirecting fetch to %#x.\n", tid,
                                    inst->readPredTarg());
                        } else {
                            panic("Non-control instruction (%s) mispredicting?!!",
                                  inst->staticInst->getName());
                        }

                        DPRINTF(InOrderExecute, "[tid:%i] Squashing will start from stage %i.\n",
                                tid, stage_num);

                        cpu->pipelineStage[stage_num]->squashDueToBranch(inst, tid);

                        inst->squashingStage = stage_num;

                        // Squash throughout other resources
                        cpu->resPool->scheduleEvent((InOrderCPU::CPUEventType)ResourcePool::SquashAll,
                                                    inst, 0, 0, tid);

                        if (inst->predTaken()) {
                            predictedTakenIncorrect++;
                        } else {
                            predictedNotTakenIncorrect++;
                        }
                    } else {
                        DPRINTF(InOrderExecute, "[tid:%i]: [sn:%i]: Prediction Correct.\n",
                                inst->readTid(), seq_num);
                    }

                    exec_req->done();
                } else {
                    warn("inst [sn:%i] had a %s fault", seq_num, fault->name());
                }
            } else {
                // Regular ALU instruction
                fault = inst->execute();
                executions++;

                if (fault == NoFault) {
                    inst->setExecuted();

                    DPRINTF(InOrderExecute, "[tid:%i]: [sn:%i]: The result of execution is 0x%x.\n",
                            inst->readTid(), seq_num, (inst->resultType(0) == InOrderDynInst::Float) ?
                            inst->readFloatResult(0) : inst->readIntResult(0));

                    exec_req->done();
                } else {
                    warn("inst [sn:%i] had a %s fault", seq_num, fault->name());
                    cpu->trap(fault, tid);
                }
            }
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}


