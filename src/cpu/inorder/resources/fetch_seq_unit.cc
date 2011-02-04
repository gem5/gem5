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
#include "cpu/inorder/resources/fetch_seq_unit.hh"
#include "cpu/inorder/resource_pool.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

FetchSeqUnit::FetchSeqUnit(std::string res_name, int res_id, int res_width,
                           int res_latency, InOrderCPU *_cpu,
                           ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      instSize(sizeof(MachInst))
{
    for (ThreadID tid = 0; tid < ThePipeline::MaxThreads; tid++) {
        pcValid[tid] = false;
        pcBlockStage[tid] = 0;

        squashSeqNum[tid] = (InstSeqNum)-1;
        lastSquashCycle[tid] = 0;
    }
}

FetchSeqUnit::~FetchSeqUnit()
{
    delete [] resourceEvent;
}

void
FetchSeqUnit::init()
{
    resourceEvent = new FetchSeqEvent[width];

    initSlots();
}

void
FetchSeqUnit::execute(int slot_num)
{
    ResourceRequest* fs_req = reqMap[slot_num];
    DynInstPtr inst = fs_req->inst;
    ThreadID tid = inst->readTid();
    int stage_num = fs_req->getStageNum();
    int seq_num = inst->seqNum;

    fs_req->fault = NoFault;

    DPRINTF(InOrderFetchSeq, "[tid:%i]: Current PC is %s\n", tid,
            pc[tid]);

    switch (fs_req->cmd)
    {
      case AssignNextPC:
        {
            if (pcValid[tid]) {
                inst->pcState(pc[tid]);
                inst->setMemAddr(pc[tid].instAddr());

                // Advance to next PC (typically PC + 4)
                pc[tid].advance();

                inst->setSeqNum(cpu->getAndIncrementInstSeq(tid));

                DPRINTF(InOrderFetchSeq, "[tid:%i]: Assigning [sn:%i] to "
                        "PC %s\n", tid, inst->seqNum, inst->pcState());

                fs_req->done();
            } else {
                DPRINTF(InOrderStall, "STALL: [tid:%i]: NPC not valid\n", tid);
                fs_req->setCompleted(false);
            }
        }
        break;

      case UpdateTargetPC:
        {
            if (inst->isControl()) {
                // If it's a return, then we must wait for resolved address.
                if (inst->isReturn() && !inst->predTaken()) {
                    cpu->pipelineStage[stage_num]->
                        toPrevStages->stageBlock[stage_num][tid] = true;
                    pcValid[tid] = false;
                    pcBlockStage[tid] = stage_num;
                } else if (inst->isCondDelaySlot() && !inst->predTaken()) {
                // Not-Taken AND Conditional Control
                    DPRINTF(InOrderFetchSeq, "[tid:%i]: [sn:%i]: [PC:%s] "
                            "Predicted Not-Taken Cond. Delay inst. Skipping "
                            "delay slot and  Updating PC to %s\n",
                            tid, inst->seqNum, inst->pcState(),
                            inst->readPredTarg());

                    DPRINTF(InOrderFetchSeq, "[tid:%i] Setting up squash to "
                            "start from stage %i, after [sn:%i].\n", tid,
                            stage_num, seq_num);

                    inst->bdelaySeqNum = seq_num;
                    inst->squashingStage = stage_num;

                    squashAfterInst(inst, stage_num, tid);
                } else if (!inst->isCondDelaySlot() && !inst->predTaken()) {
                    // Not-Taken Control
                    DPRINTF(InOrderFetchSeq, "[tid:%i]: [sn:%i]: Predicted "
                            "Not-Taken Control "
                            "inst. updating PC to %s\n", tid, inst->seqNum,
                            inst->readPredTarg());
#if ISA_HAS_DELAY_SLOT
                    pc[tid] = inst->pcState();
                    advancePC(pc[tid], inst->staticInst);
#endif
                } else if (inst->predTaken()) {
                    // Taken Control
#if ISA_HAS_DELAY_SLOT
                    pc[tid] = inst->readPredTarg();

                    DPRINTF(InOrderFetchSeq, "[tid:%i]: [sn:%i] Updating delay"
                            " slot target to PC %s\n", tid, inst->seqNum,
                            inst->readPredTarg());
                    inst->bdelaySeqNum = seq_num + 1;
#else
                    inst->bdelaySeqNum = seq_num;
#endif

                    inst->squashingStage = stage_num;
                    DPRINTF(InOrderFetchSeq, "[tid:%i] Setting up squash to "
                            "start from stage %i, after [sn:%i].\n",
                            tid, stage_num, inst->bdelaySeqNum);

                    // Do Squashing
                    squashAfterInst(inst, stage_num, tid);
                }
            } else {
                DPRINTF(InOrderFetchSeq, "[tid:%i]: [sn:%i]: Ignoring branch "
                        "target update since then is not a control "
                        "instruction.\n", tid, inst->seqNum);
            }

            fs_req->done();
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}

inline void
FetchSeqUnit::squashAfterInst(DynInstPtr inst, int stage_num, ThreadID tid)
{
    // Squash In Pipeline Stage
    cpu->pipelineStage[stage_num]->squashDueToBranch(inst, tid);

    // Squash inside current resource, so if there needs to be fetching on
    // same cycle the fetch information will be correct.

    // Schedule Squash Through-out Resource Pool
    cpu->resPool->scheduleEvent(
            (InOrderCPU::CPUEventType)ResourcePool::SquashAll, inst, 0);
}

void
FetchSeqUnit::squash(DynInstPtr inst, int squash_stage,
                     InstSeqNum squash_seq_num, ThreadID tid)
{
    DPRINTF(InOrderFetchSeq, "[tid:%i]: Updating due to squash from stage %i."
            "\n", tid, squash_stage);

    InstSeqNum done_seq_num = inst->bdelaySeqNum;

    // Handles the case where we are squashing because of something that is
    // not a branch...like a memory stall
    TheISA::PCState newPC;
    if (inst->isControl()) {
        newPC = inst->readPredTarg();
    } else {
        TheISA::PCState thisPC = inst->pcState();
        assert(inst->staticInst);
        advancePC(thisPC, inst->staticInst);
        newPC = thisPC;
    }

    if (squashSeqNum[tid] <= done_seq_num &&
        lastSquashCycle[tid] == curTick()) {
        DPRINTF(InOrderFetchSeq, "[tid:%i]: Ignoring squash from stage %i, "
                "since there is an outstanding squash that is older.\n",
                tid, squash_stage);
    } else {
        squashSeqNum[tid] = done_seq_num;
        lastSquashCycle[tid] = curTick();

        // If The very next instruction number is the done seq. num,
        // then we haven't seen the delay slot yet ... if it isn't
        // the last done_seq_num then this is the delay slot inst.
        if (cpu->nextInstSeqNum(tid) != done_seq_num &&
            !inst->procDelaySlotOnMispred) {

            // Reset PC
            pc[tid] = newPC;
#if ISA_HAS_DELAY_SLOT
            TheISA::advancePC(pc[tid], inst->staticInst);
#endif

            DPRINTF(InOrderFetchSeq, "[tid:%i]: Setting PC to %s.\n",
                    tid, newPC);
        } else {
            assert(ISA_HAS_DELAY_SLOT);

            pc[tid] = (inst->procDelaySlotOnMispred) ?
                inst->branchTarget() : newPC;

            // Reset PC to Delay Slot Instruction
            if (inst->procDelaySlotOnMispred) {
                // Reset PC
                pc[tid] = newPC;
            }

        }

        // Unblock Any Stages Waiting for this information to be updated ...
        if (!pcValid[tid]) {
            cpu->pipelineStage[pcBlockStage[tid]]->
                toPrevStages->stageUnblock[pcBlockStage[tid]][tid] = true;
        }

        pcValid[tid] = true;
    }

    Resource::squash(inst, squash_stage, squash_seq_num, tid);
}

FetchSeqUnit::FetchSeqEvent::FetchSeqEvent()
    : ResourceEvent()
{ }

void
FetchSeqUnit::FetchSeqEvent::process()
{
    FetchSeqUnit* fs_res = dynamic_cast<FetchSeqUnit*>(resource);
    assert(fs_res);

    for (int i = 0; i < MaxThreads; i++) {
        fs_res->pc[i] = fs_res->cpu->pcState(i);
        DPRINTF(InOrderFetchSeq, "[tid:%i]: Setting PC: %s.\n",
                fs_res->pc[i]);

        fs_res->pcValid[i] = true;
    }
}


void
FetchSeqUnit::activateThread(ThreadID tid)
{
    pcValid[tid] = true;

    pc[tid] = cpu->pcState(tid);

    cpu->fetchPriorityList.push_back(tid);

    DPRINTF(InOrderFetchSeq, "[tid:%i]: Reading PC: %s.\n",
            tid, pc[tid]);
}

void
FetchSeqUnit::deactivateThread(ThreadID tid)
{
    pcValid[tid] = false;
    pcBlockStage[tid] = 0;

    squashSeqNum[tid] = (InstSeqNum)-1;
    lastSquashCycle[tid] = 0;

    list<ThreadID>::iterator thread_it = find(cpu->fetchPriorityList.begin(),
                                              cpu->fetchPriorityList.end(),
                                              tid);

    if (thread_it != cpu->fetchPriorityList.end())
        cpu->fetchPriorityList.erase(thread_it);
}

void
FetchSeqUnit::suspendThread(ThreadID tid)
{
    deactivateThread(tid);    
}

void
FetchSeqUnit::updateAfterContextSwitch(DynInstPtr inst, ThreadID tid)
{
    pcValid[tid] = true;

    if (cpu->thread[tid]->lastGradIsBranch) {
        /** This function assumes that the instruction causing the context
         *  switch was right after the branch. Thus, if it's not, then
         *  we are updating incorrectly here
         */
        assert(cpu->nextInstAddr(tid) == inst->instAddr());
        pc[tid] = cpu->thread[tid]->lastBranchPC;
    } else {
        pc[tid] = inst->pcState();
    }
    assert(inst->staticInst);
    advancePC(pc[tid], inst->staticInst);
    
    DPRINTF(InOrderFetchSeq, "[tid:%i]: Updating PCs due to Context Switch."
            "Assigning  PC: %s.\n", tid, pc[tid]);
}
