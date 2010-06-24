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
                       int res_latency, InOrderCPU *_cpu, ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      instSize(sizeof(MachInst))
{
    for (ThreadID tid = 0; tid < ThePipeline::MaxThreads; tid++) {
        delaySlotInfo[tid].numInsts = 0;
        delaySlotInfo[tid].targetReady = false;

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
    // After this is working, change this to a reinterpret cast
    // for performance considerations
    ResourceRequest* fs_req = reqMap[slot_num];
    DynInstPtr inst = fs_req->inst;
    ThreadID tid = inst->readTid();
    int stage_num = fs_req->getStageNum();
    int seq_num = inst->seqNum;

    fs_req->fault = NoFault;

    switch (fs_req->cmd)
    {
      case AssignNextPC:
        {
            if (pcValid[tid]) {

                if (delaySlotInfo[tid].targetReady &&
                    delaySlotInfo[tid].numInsts == 0) {
                    // Set PC to target
                    PC[tid] = delaySlotInfo[tid].targetAddr; //next_PC
                    nextPC[tid] = PC[tid] + instSize;        //next_NPC
                    nextNPC[tid] = PC[tid] + (2 * instSize);

                    delaySlotInfo[tid].targetReady = false;

                    DPRINTF(InOrderFetchSeq, "[tid:%i]: Setting PC to delay slot target\n",tid);
                }

                inst->setPC(PC[tid]);
                inst->setNextPC(PC[tid] + instSize);
                inst->setNextNPC(PC[tid] + (instSize * 2));

#if ISA_HAS_DELAY_SLOT
                inst->setPredTarg(inst->readNextNPC());
#else
                inst->setPredTarg(inst->readNextPC());
#endif
                inst->setMemAddr(PC[tid]);
                inst->setSeqNum(cpu->getAndIncrementInstSeq(tid));

                DPRINTF(InOrderFetchSeq, "[tid:%i]: Assigning [sn:%i] to PC %08p, NPC %08p, NNPC %08p\n", tid,
                        inst->seqNum, inst->readPC(), inst->readNextPC(), inst->readNextNPC());

                if (delaySlotInfo[tid].numInsts > 0) {
                    --delaySlotInfo[tid].numInsts;

                    // It's OK to set PC to target of branch
                    if (delaySlotInfo[tid].numInsts == 0) {
                        delaySlotInfo[tid].targetReady = true;
                    }

                    DPRINTF(InOrderFetchSeq, "[tid:%i]: %i delay slot inst(s) left to"
                            " process.\n", tid, delaySlotInfo[tid].numInsts);
                }

                PC[tid] = nextPC[tid];
                nextPC[tid] = nextNPC[tid];
                nextNPC[tid] += instSize;

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
                    cpu->pipelineStage[stage_num]->toPrevStages->stageBlock[stage_num][tid] = true;
                    pcValid[tid] = false;
                    pcBlockStage[tid] = stage_num;
                } else if (inst->isCondDelaySlot() && !inst->predTaken()) {
                // Not-Taken AND Conditional Control
                    DPRINTF(InOrderFetchSeq, "[tid:%i]: [sn:%i]: [PC:%08p] Predicted Not-Taken Cond. "
                            "Delay inst. Skipping delay slot and  Updating PC to %08p\n",
                            tid, inst->seqNum, inst->readPC(), inst->readPredTarg());

                    DPRINTF(InOrderFetchSeq, "[tid:%i] Setting up squash to start from stage %i, after [sn:%i].\n",
                            tid, stage_num, seq_num);

                    inst->bdelaySeqNum = seq_num;
                    inst->squashingStage = stage_num;

                    squashAfterInst(inst, stage_num, tid);
                } else if (!inst->isCondDelaySlot() && !inst->predTaken()) {
                    // Not-Taken Control
                    DPRINTF(InOrderFetchSeq, "[tid:%i]: [sn:%i]: Predicted Not-Taken Control "
                            "inst. updating PC to %08p\n", tid, inst->seqNum,
                            inst->readNextPC());
#if ISA_HAS_DELAY_SLOT
                    ++delaySlotInfo[tid].numInsts;
                    delaySlotInfo[tid].targetReady = false;
                    delaySlotInfo[tid].targetAddr = inst->readNextNPC();
#else
                    assert(delaySlotInfo[tid].numInsts == 0);
#endif
                } else if (inst->predTaken()) {
                    // Taken Control
#if ISA_HAS_DELAY_SLOT
                    ++delaySlotInfo[tid].numInsts;
                    delaySlotInfo[tid].targetReady = false;
                    delaySlotInfo[tid].targetAddr = inst->readPredTarg();

                    DPRINTF(InOrderFetchSeq, "[tid:%i]: [sn:%i] Updating delay slot target "
                            "to PC %08p\n", tid, inst->seqNum, inst->readPredTarg());
                    inst->bdelaySeqNum = seq_num + 1;
#else
                    inst->bdelaySeqNum = seq_num;
                    assert(delaySlotInfo[tid].numInsts == 0);
#endif

                    inst->squashingStage = stage_num;

                    DPRINTF(InOrderFetchSeq, "[tid:%i] Setting up squash to start from stage %i, after [sn:%i].\n",
                            tid, stage_num, inst->bdelaySeqNum);

                    // Do Squashing
                    squashAfterInst(inst, stage_num, tid);
                }
            } else {
                DPRINTF(InOrderFetchSeq, "[tid:%i]: [sn:%i]: Ignoring branch target update "
                        "since then is not a control instruction.\n", tid, inst->seqNum);
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

    // Squash inside current resource, so if there needs to be fetching on same cycle
    // the fetch information will be correct.
    // squash(inst, stage_num, inst->bdelaySeqNum, tid);

    // Schedule Squash Through-out Resource Pool
    cpu->resPool->scheduleEvent((InOrderCPU::CPUEventType)ResourcePool::SquashAll, inst, 0);
}
void
FetchSeqUnit::squash(DynInstPtr inst, int squash_stage,
                     InstSeqNum squash_seq_num, ThreadID tid)
{
    DPRINTF(InOrderFetchSeq, "[tid:%i]: Updating due to squash from stage %i.\n",
            tid, squash_stage);

    InstSeqNum done_seq_num = inst->bdelaySeqNum;

    // Handles the case where we are squashing because of something that is
    // not a branch...like a memory stall
    Addr new_PC = (inst->isControl()) ?
        inst->readPredTarg() : inst->readPC() + instSize;

    if (squashSeqNum[tid] <= done_seq_num &&
        lastSquashCycle[tid] == curTick) {
        DPRINTF(InOrderFetchSeq, "[tid:%i]: Ignoring squash from stage %i, since"
                "there is an outstanding squash that is older.\n",
                tid, squash_stage);
    } else {
        squashSeqNum[tid] = done_seq_num;
        lastSquashCycle[tid] = curTick;

        // If The very next instruction number is the done seq. num,
        // then we haven't seen the delay slot yet ... if it isn't
        // the last done_seq_num then this is the delay slot inst.
        if (cpu->nextInstSeqNum(tid) != done_seq_num &&
            !inst->procDelaySlotOnMispred) {
            delaySlotInfo[tid].numInsts = 0;
            delaySlotInfo[tid].targetReady = false;

            // Reset PC
            PC[tid] = new_PC;
            nextPC[tid] = new_PC + instSize;
            nextNPC[tid] = new_PC + (2 * instSize);

            DPRINTF(InOrderFetchSeq, "[tid:%i]: Setting PC to %08p.\n",
                    tid, PC[tid]);
        } else {
#if !ISA_HAS_DELAY_SLOT
            assert(0);
#endif

            delaySlotInfo[tid].numInsts = 1;
            delaySlotInfo[tid].targetReady = false;
            delaySlotInfo[tid].targetAddr = (inst->procDelaySlotOnMispred) ? inst->branchTarget() : new_PC;

            // Reset PC to Delay Slot Instruction
            if (inst->procDelaySlotOnMispred) {
                PC[tid] = new_PC;
                nextPC[tid] = new_PC + instSize;
                nextNPC[tid] = new_PC + (2 * instSize);
            }

        }

        // Unblock Any Stages Waiting for this information to be updated ...
        if (!pcValid[tid]) {
            cpu->pipelineStage[pcBlockStage[tid]]->toPrevStages->stageUnblock[pcBlockStage[tid]][tid] = true;
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

    for (int i=0; i < MaxThreads; i++) {
        fs_res->PC[i] = fs_res->cpu->readPC(i);
        fs_res->nextPC[i] = fs_res->cpu->readNextPC(i);
        fs_res->nextNPC[i] = fs_res->cpu->readNextNPC(i);
        DPRINTF(InOrderFetchSeq, "[tid:%i]: Setting PC:%08p NPC:%08p NNPC:%08p.\n",
                fs_res->PC[i], fs_res->nextPC[i], fs_res->nextNPC[i]);

        fs_res->pcValid[i] = true;
    }

    //cpu->fetchPriorityList.push_back(tid);
}


void
FetchSeqUnit::activateThread(ThreadID tid)
{
    pcValid[tid] = true;

    PC[tid] = cpu->readPC(tid);
    nextPC[tid] = cpu->readNextPC(tid);
    nextNPC[tid] = cpu->readNextNPC(tid);

    cpu->fetchPriorityList.push_back(tid);

    DPRINTF(InOrderFetchSeq, "[tid:%i]: Reading PC:%08p NPC:%08p NNPC:%08p.\n",
            tid, PC[tid], nextPC[tid], nextNPC[tid]);
}

void
FetchSeqUnit::deactivateThread(ThreadID tid)
{
    delaySlotInfo[tid].numInsts = 0;
    delaySlotInfo[tid].targetReady = false;

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
        assert(cpu->thread[tid]->lastBranchNextPC == inst->readPC());
        
        PC[tid] = cpu->thread[tid]->lastBranchNextNPC;
        nextPC[tid] = PC[tid] + instSize;
        nextNPC[tid] = nextPC[tid] + instSize;
    } else {
        PC[tid] = inst->readNextPC();
        nextPC[tid] = inst->readNextNPC();
        nextNPC[tid] = inst->readNextNPC() + instSize;        
    }
    
    DPRINTF(InOrderFetchSeq, "[tid:%i]: Updating PCs due to Context Switch."
            "Assigning  PC:%08p NPC:%08p NNPC:%08p.\n", tid, PC[tid], 
            nextPC[tid], nextNPC[tid]);
}
