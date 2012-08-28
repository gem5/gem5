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

#include <list>
#include <vector>

#include "cpu/inorder/resources/execution_unit.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/resource_pool.hh"
#include "debug/Fault.hh"
#include "debug/InOrderExecute.hh"
#include "debug/InOrderStall.hh"
#include "sim/full_system.hh"

using namespace std;
using namespace ThePipeline;

ExecutionUnit::ExecutionUnit(string res_name, int res_id, int res_width,
                             Cycles res_latency, InOrderCPU *_cpu,
                             ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      lastExecuteTick(0), lastControlTick(0)
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

    executions
        .name(name() + ".executions")
        .desc("Number of Instructions Executed.");

 
    predictedIncorrect
        .name(name() + ".mispredicted")
        .desc("Number of Branches Incorrectly Predicted");

    predictedCorrect
        .name(name() + ".predicted")
        .desc("Number of Branches Incorrectly Predicted");

    mispredictPct
        .name(name() + ".mispredictPct")
        .desc("Percentage of Incorrect Branches Predicts")
        .precision(6);
    mispredictPct = (predictedIncorrect / 
                     (predictedCorrect + predictedIncorrect)) * 100;

    Resource::regStats();
}

void
ExecutionUnit::execute(int slot_num)
{
    ResourceRequest* exec_req = reqs[slot_num];
    DynInstPtr inst = reqs[slot_num]->inst;
    if (inst->fault != NoFault) {
        DPRINTF(InOrderExecute,
                "[tid:%i]: [sn:%i]: Detected %s fault @ %x. Forwarding to "
                "next stage.\n", inst->readTid(), inst->seqNum, inst->fault->name(),
                inst->pcState());
        exec_req->done();
        return;
    }

    Fault fault = NoFault;
    Tick cur_tick = curTick();
    unsigned stage_num = exec_req->getStageNum();
    ThreadID tid = inst->readTid();
#if TRACING_ON
    InstSeqNum seq_num = inst->seqNum;
#endif

    switch (exec_req->cmd)
    {
      case ExecuteInst:
        {
            if (inst->isNop()) {
                DPRINTF(InOrderExecute, "[tid:%i] [sn:%i] [PC:%s] Ignoring execution"
                        "of %s.\n", inst->readTid(), seq_num, inst->pcState(),
                        inst->instName());
                inst->setExecuted();
                exec_req->done();
                return;
            } else {
                DPRINTF(InOrderExecute, "[tid:%i] Executing [sn:%i] [PC:%s] %s.\n",
                        inst->readTid(), seq_num, inst->pcState(), inst->instName());
            }

            if (cur_tick != lastExecuteTick) {
                lastExecuteTick = cur_tick;
            }

            //@todo: handle address generation here
            assert(!inst->isMemRef());

            if (inst->isControl()) {
                if (lastControlTick == cur_tick) {
                    DPRINTF(InOrderExecute, "Can not Execute More than One Control "
                            "Inst Per Cycle. Blocking Request.\n");
                    exec_req->done(false);
                    return;
                }
                lastControlTick = curTick();

                // Evaluate Branch
                fault = inst->execute();

                // Should unconditional control , pc relative count as an
                // execution??? Probably not.
                executions++;

                if (fault == NoFault) {
                    inst->setExecuted();

                    if (inst->mispredicted()) {
                        assert(inst->isControl());

                        // Set up Squash Generated By this Misprediction
                        TheISA::PCState pc = inst->pcState();
                        TheISA::advancePC(pc, inst->staticInst);
                        inst->setPredTarg(pc);
                        inst->setSquashInfo(stage_num);
                        setupSquash(inst, stage_num, tid);

                        DPRINTF(InOrderExecute, "[tid:%i]: [sn:%i] Squashing from "
                                "stage %i. Redirecting  fetch to %s.\n", tid,
                                inst->seqNum, stage_num, pc);
                        DPRINTF(InOrderStall, "STALL: [tid:%i]: Branch"
                                " misprediction at %s\n", tid, inst->pcState());

                        if (inst->predTaken()) {
                            predictedTakenIncorrect++;
                            DPRINTF(InOrderExecute, "[tid:%i] [sn:%i] %s ..."
                                    "PC %s ... Mispredicts! "
                                    "(Prediction: Taken)\n",
                                    tid, inst->seqNum,
                                    inst->staticInst->disassemble(
                                        inst->instAddr()),
                                    inst->pcState());
                        } else {
                            predictedNotTakenIncorrect++;
                            DPRINTF(InOrderExecute, "[tid:%i] [sn:%i] %s ..."
                                    "PC %s ... Mispredicts! "
                                    "(Prediction: Not Taken)\n",
                                    tid, inst->seqNum,
                                    inst->staticInst->disassemble(
                                        inst->instAddr()),
                                    inst->pcState());
                        }
                        predictedIncorrect++;
                    } else {
                        DPRINTF(InOrderExecute, "[tid:%i]: [sn:%i]: Prediction"
                                "Correct.\n", inst->readTid(), seq_num);
                        predictedCorrect++;
                    }

                    exec_req->done();
                } else {
                    DPRINTF(Fault, "[tid:%i]:[sn:%i]: Fault %s found\n",
                            inst->readTid(), inst->seqNum, fault->name());
                    inst->fault = fault;
                    exec_req->done();
                }
            } else {
                // Regular ALU instruction
                fault = inst->execute();
                executions++;

                if (fault == NoFault) {
                    inst->setExecuted();

#if TRACING_ON
                    for (int didx = 0; didx < inst->numDestRegs(); didx++)
                        if (inst->resultType(didx) == InOrderDynInst::Float ||
                            inst->resultType(didx) == InOrderDynInst::FloatBits ||
                            inst->resultType(didx) == InOrderDynInst::Double)
                            DPRINTF(InOrderExecute, "[tid:%i]: [sn:%i]: Dest result %i "
                                    "of FP execution is %08f (%x).\n", inst->readTid(),
                                    seq_num, didx, inst->readFloatResult(didx),
                                    inst->readFloatBitsResult(didx));
                        else
                            DPRINTF(InOrderExecute, "[tid:%i]: [sn:%i]: Dest result %i "
                                    "of Int execution is 0x%x.\n", inst->readTid(),
                                    seq_num, didx, inst->readIntResult(didx));
#endif

                    if (!FullSystem) {
                        // The Syscall might change the PC, so conservatively
                        // squash everything behing it
                        if (inst->isSyscall()) {
                            inst->setSquashInfo(stage_num);
                            setupSquash(inst, stage_num, tid);
                        }
                    }
                } else {
                    DPRINTF(InOrderExecute, "[tid:%i]: [sn:%i]: had a %s "
                            "fault.\n", inst->readTid(), seq_num, fault->name());
                    DPRINTF(Fault, "[tid:%i]:[sn:%i]: Fault %s found\n",
                            inst->readTid(), inst->seqNum, fault->name());
                    inst->fault = fault;
                }

                exec_req->done();
            }
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}


