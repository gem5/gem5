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
#include "debug/InOrderFetchSeq.hh"
#include "debug/InOrderStall.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

FetchSeqUnit::FetchSeqUnit(std::string res_name, int res_id, int res_width,
                           Cycles res_latency, InOrderCPU *_cpu,
                           ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      instSize(sizeof(MachInst))
{
    for (ThreadID tid = 0; tid < ThePipeline::MaxThreads; tid++) {
        pcValid[tid] = false;
        pcBlockStage[tid] = 0;

        //@todo: Use CPU's squashSeqNum here instead of maintaining our own
        // state
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

    for (int i = 0; i < width; i++) {
        reqs[i] = new ResourceRequest(this);
    }

    initSlots();
}

void
FetchSeqUnit::execute(int slot_num)
{
    ResourceRequest* fs_req = reqs[slot_num];
    DynInstPtr inst = fs_req->inst;
    ThreadID tid = inst->readTid();
    int stage_num = fs_req->getStageNum();

    if (inst->fault != NoFault) {
        DPRINTF(InOrderFetchSeq,
                "[tid:%i]: [sn:%i]: Detected %s fault @ %x. Forwarding to "
                "next stage.\n", tid, inst->seqNum, inst->fault->name(),
                inst->pcState());
        fs_req->done();
        return;
    }

    switch (fs_req->cmd)
    {
      case AssignNextPC:
        {
            DPRINTF(InOrderFetchSeq, "[tid:%i]: Current PC is %s\n", tid,
                    pc[tid]);

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
                fs_req->done(false);
            }
        }
        break;

      case UpdateTargetPC:
        {
            assert(!inst->isCondDelaySlot()  &&
                   "Not Handling Conditional Delay Slot");

            if (inst->isControl()) {
                if (inst->isReturn() && !inst->predTaken()) {
                    // If it's a return, then we must wait for resolved address.
                    // The Predictor will mark a return a false as "not taken"
                    // if there is no RAS entry
                    DPRINTF(InOrderFetchSeq, "[tid:%d]: Setting block signal "
                            "for stage %i.\n",
                            tid, stage_num);
                    cpu->pipelineStage[stage_num]->
                        toPrevStages->stageBlock[stage_num][tid] = true;
                    pcValid[tid] = false;
                    pcBlockStage[tid] = stage_num;
                } else if (inst->predTaken()) {
                    // Taken Control
                    inst->setSquashInfo(stage_num);
                    setupSquash(inst, stage_num, tid);

                    DPRINTF(InOrderFetchSeq, "[tid:%i] Setting up squash to "
                            "start from stage %i, after [sn:%i].\n",
                            tid, stage_num, inst->squashSeqNum);
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

void
FetchSeqUnit::squash(DynInstPtr inst, int squash_stage,
                     InstSeqNum squash_seq_num, ThreadID tid)
{
    DPRINTF(InOrderFetchSeq, "[tid:%i]: Updating due to squash from %s (%s) "
            "stage %i.\n", tid, inst->instName(), inst->pcState(),
            squash_stage);

    if (lastSquashCycle[tid] == curTick() &&
        squashSeqNum[tid] <= squash_seq_num) {
        DPRINTF(InOrderFetchSeq, "[tid:%i]: Ignoring squash from stage %i, "
                "since there is an outstanding squash that is older.\n",
                tid, squash_stage);
    } else {
        squashSeqNum[tid] = squash_seq_num;
        lastSquashCycle[tid] = curTick();

        if (inst->staticInst) {
            if (inst->fault != NoFault) {
                // A Trap Caused This Fault and will update the pc state
                // when done trapping
                DPRINTF(InOrderFetchSeq, "[tid:%i] Blocking due to fault @ "
                        "[sn:%i].%s %s \n", tid, inst->seqNum,
                        inst->instName(), inst->pcState());
                pcValid[tid] = false;
            } else {
                TheISA::PCState nextPC;
                assert(inst->staticInst);
                if (inst->isControl()) {
                    nextPC = inst->readPredTarg();

                    // If we are already fetching this PC then advance to next PC
                    // =======
                    // This should handle ISAs w/delay slots and annulled delay
                    // slots to figure out which is the next PC to fetch after
                    // a mispredict
                    DynInstPtr bdelay_inst = NULL;
                    ListIt bdelay_it;
                    if (inst->onInstList) {
                        bdelay_it = inst->getInstListIt();
                        bdelay_it++;
                    } else {
                        InstSeqNum branch_delay_num = inst->seqNum + 1;
                        bdelay_it = cpu->findInst(branch_delay_num, tid);
                    }

                    if (bdelay_it != cpu->instList[tid].end()) {
                        bdelay_inst = (*bdelay_it);
                    }

                    if (bdelay_inst) {
                        if (bdelay_inst->pc.instAddr() == nextPC.instAddr()) {
                            bdelay_inst->pc = nextPC;
                            advancePC(nextPC, inst->staticInst);
                            DPRINTF(InOrderFetchSeq, "Advanced PC to %s\n", nextPC);
                        }
                    }
                } else {
                    nextPC = inst->pcState();
                    advancePC(nextPC, inst->staticInst);
                }


                DPRINTF(InOrderFetchSeq, "[tid:%i]: Setting PC to %s.\n",
                        tid, nextPC);
                pc[tid] = nextPC;

                // Unblock Any Stages Waiting for this information to be updated ...
                if (!pcValid[tid]) {
                    DPRINTF(InOrderFetchSeq, "[tid:%d]: Setting unblock signal "
                            "for stage %i.\n",
                            tid, pcBlockStage[tid]);

                    // Need to use "fromNextStages" instead of "toPrevStages"
                    // because the timebuffer will have already have advanced
                    // in the tick function and this squash function will happen after
                    // the tick
                    cpu->pipelineStage[pcBlockStage[tid]]->
                        fromNextStages->stageUnblock[pcBlockStage[tid]][tid] = true;
                }

                pcValid[tid] = true;
            }
        }
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
FetchSeqUnit::trap(Fault fault, ThreadID tid, DynInstPtr inst)
{
    pcValid[tid] = true;
    pc[tid] = cpu->pcState(tid);
    DPRINTF(InOrderFetchSeq, "[tid:%i]: Trap updating to PC: "
            "%s.\n", tid, pc[tid]);
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
