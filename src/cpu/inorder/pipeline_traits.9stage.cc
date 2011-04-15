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

#include "cpu/inorder/resources/resource_list.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"

using namespace std;

namespace ThePipeline {


//@TODO: create my own Instruction Schedule Class
//that operates as a Priority QUEUE
int getNextPriority(DynInstPtr &inst, int stage_num)
{
    int cur_pri = 20;

    /*
    std::priority_queue<ScheduleEntry*, std::vector<ScheduleEntry*>,
        entryCompare>::iterator sked_it = inst->resSched.begin();

    std::priority_queue<ScheduleEntry*, std::vector<ScheduleEntry*>,
        entryCompare>::iterator sked_end = inst->resSched.end();

    while (sked_it != sked_end) {

        if (sked_it.top()->stageNum == stage_num) {
            cur_pri = sked_it.top()->priority;
        }

        sked_it++;
    }
    */

    return cur_pri;
}

void createFrontEndSchedule(DynInstPtr &inst)
{
    int stNum = 0;
    int stPri = 0;
    // Get Pointer to Instuction's Schedule
    ResSchedule *inst_sched = &inst->resSched;

    //
    // Stage 0
    // ---------------------------------------
    inst_sched->push(new ScheduleEntry(stNum, stPri, FetchSeq, FetchSeqUnit::AssignNextPC));
    stPri++;

    inst_sched->push(new ScheduleEntry(stNum, stPri, ITLB, TLBUnit::FetchLookup));
    stPri++;

    inst_sched->push(new ScheduleEntry(stNum, stPri, ICache, CacheUnit::InitiateFetch));
    stPri++;

    // Reset Priority / Update Next Stage Number
    stNum++;
    stPri = 0;

    //
    // Stage 1
    // ---------------------------------------
    inst_sched->push(new ScheduleEntry(stNum, stPri, ICache, CacheUnit::CompleteFetch));
    stPri++;

    inst_sched->push(new ScheduleEntry(stNum, stPri, Decode, DecodeUnit::DecodeInst));
    stPri++;

    inst_sched->push(new ScheduleEntry(stNum, stPri, BPred, BranchPredictor::PredictBranch));
    stPri++;

    inst_sched->push(new ScheduleEntry(stNum, stPri, FetchSeq, FetchSeqUnit::UpdateTargetPC));
    stPri++;

    if (inst->readTid() == 0)
        inst_sched->push(new ScheduleEntry(stNum, stPri, FetchBuff, InstBuffer::ScheduleOrBypass));
    else //if (inst->readTid() == 1)
        inst_sched->push(new ScheduleEntry(stNum, stPri, FetchBuff2, InstBuffer::ScheduleOrBypass));
    stPri++;

    // Reset Priority / Update Next Stage Number
    stNum++;
    stPri = 0;

    //
    // Stage 2
    // ---------------------------------------
    // Reset Priority / Update Next Stage Number
    stNum++;
    stPri = 0;
}

bool createBackEndSchedule(DynInstPtr &inst)
{
    if (!inst->staticInst) {
        return false;
    }

    std::string name = inst->staticInst->getName();

    int stNum = BackEndStartStage;
    int stPri = 0;

    // Get Pointer to Instuction's Schedule
    ResSchedule *inst_sched = &inst->resSched;

    //
    // Stage 3
    // ---------------------------------------
    // Set When Source Registers Should be read - Stage 4
    for (int idx=0; idx < inst->numSrcRegs(); idx++) {
        inst_sched->push(new ScheduleEntry(stNum, stPri, RegManager, UseDefUnit::ReadSrcReg, idx));
    }
    stPri++;

    // Reset Priority / Update Next Stage Number
    stPri = 0;
    stNum++;

    //
    // Stage 4
    // ---------------------------------------
    if (inst->isMemRef()) {
        inst_sched->push(new ScheduleEntry(stNum, stPri, AGEN, AGENUnit::GenerateAddr));
    }

    // Reset Priority / Update Next Stage Number
    stPri = 0;
    stNum++;

    //
    // Stage 5
    // ---------------------------------------
    // Execution Unit
    if (!inst->isNonSpeculative() && !inst->isMemRef()) {
        if (inst->opClass() == IntMultOp || inst->opClass() == IntDivOp) {
            inst_sched->push(new ScheduleEntry(stNum, stPri++, MDU, MultDivUnit::MultDiv));
        } else {
            inst_sched->push(new ScheduleEntry(stNum, stPri, ExecUnit, ExecutionUnit::ExecuteInst));
        }
    }
    stPri++;

    // DCache Initiate Access
    if (inst->isMemRef()) {
        inst_sched->push(new ScheduleEntry(stNum, stPri, DTLB, TLBUnit::DataLookup));
        stPri++;

        if (inst->isLoad()) {
            inst_sched->push(new ScheduleEntry(stNum, stPri, DCache, CacheUnit::InitiateReadData));
        } else if (inst->isStore()) {
            inst_sched->push(new ScheduleEntry(stNum, stPri, DCache, CacheUnit::InitiateWriteData));
        }
    }

    // Reset Priority / Update Next Stage Number
    stPri = 0;
    stNum++;

    //
    // Stage 6
    // ---------------------------------------
    // DCache Complete Access
    if (inst->isMemRef()) {
        if (inst->isLoad()) {
            inst_sched->push(new ScheduleEntry(stNum, stPri, DCache, CacheUnit::CompleteReadData));
        } else if (inst->isStore()) {
            inst_sched->push(new ScheduleEntry(stNum, stPri, DCache, CacheUnit::CompleteWriteData));
        }
    }

    // Reset Priority / Update Next Stage Number
    stPri = 0;
    stNum++;

    //
    // Stage 7
    // ---------------------------------------
    // Reset Priority / Update Next Stage Number
    stPri = 0;
    stNum++;

    //
    // Stage 8
    // ---------------------------------------
    // NonSpeculative Execution
    if (inst->isNonSpeculative() ) {
        if (inst->isMemRef())
            fatal("Schedule doesnt handle Non-Speculative Memory Instructions.\n");

        inst_sched->push(new ScheduleEntry(stNum, stPri, ExecUnit, ExecutionUnit::ExecuteInst));
        stPri++;
    }

    // Write Back to Register File
    for (int idx=0; idx < inst->numDestRegs(); idx++) {
        inst_sched->push(new ScheduleEntry(stNum, stPri, RegManager, UseDefUnit::WriteDestReg, idx));
        stPri++;
    }

    // Graduate Instructions
    inst_sched->push(new ScheduleEntry(stNum, stPri, Grad, GraduationUnit::GraduateInst));
    stPri++;

    // Reset Priority / Update Next Stage Number
    stPri = 0;
    stNum++;

    return true;
}

};
