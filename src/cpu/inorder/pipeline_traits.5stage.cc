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
    // IF - Stage 0
    // ---------------------------------------
    inst_sched->push(new ScheduleEntry(stNum, stPri++, FetchSeq, FetchSeqUnit::AssignNextPC));
    inst_sched->push(new ScheduleEntry(stNum, stPri++, ITLB,     TLBUnit::FetchLookup));
    inst_sched->push(new ScheduleEntry(stNum, stPri++, ICache,   CacheUnit::InitiateFetch));

    //
    // DE - Stage 1
    // ---------------------------------------
    stNum++; stPri = 0;
    inst_sched->push(new ScheduleEntry(stNum, stPri++, ICache,   CacheUnit::CompleteFetch));
    inst_sched->push(new ScheduleEntry(stNum, stPri++, Decode,   DecodeUnit::DecodeInst));
    inst_sched->push(new ScheduleEntry(stNum, stPri++, BPred,    BranchPredictor::PredictBranch));
    inst_sched->push(new ScheduleEntry(stNum, stPri++, FetchSeq, FetchSeqUnit::UpdateTargetPC));

}

bool createBackEndSchedule(DynInstPtr &inst)
{
    if (!inst->staticInst) {
        return false;
    }

    int stNum = BackEndStartStage;
    int stPri = 0;

    // Get Pointer to Instuction's Schedule
    ResSchedule *inst_sched = &inst->resSched;

    //
    // EX - Stage 2
    // ---------------------------------------
    for (int idx=0; idx < inst->numSrcRegs(); idx++) {
        if (!idx || !inst->isStore())
            inst_sched->push(new ScheduleEntry(stNum, stPri++, RegManager, UseDefUnit::ReadSrcReg, idx));
    }

    if ( inst->isNonSpeculative() ) {
        // skip execution of non speculative insts until later
    } else if (inst->isMemRef()) {
        inst_sched->push(new ScheduleEntry(stNum, stPri++, AGEN, AGENUnit::GenerateAddr));
        if ( inst->isLoad() ) {
            inst_sched->push(new ScheduleEntry(stNum, stPri++, DTLB, TLBUnit::DataLookup));
            inst_sched->push(new ScheduleEntry(stNum, stPri++, DCache, CacheUnit::InitiateReadData));
        }
    } else {
        inst_sched->push(new ScheduleEntry(stNum, stPri++, ExecUnit, ExecutionUnit::ExecuteInst));
    }

    //
    // MEM - Stage 3
    // ---------------------------------------
    stPri = 0; stNum++;
    if ( inst->isStore() ) { // for store, need src reg at this point
        inst_sched->push(new ScheduleEntry(stNum, stPri++, RegManager, UseDefUnit::ReadSrcReg, 1));
    }
    if ( inst->isLoad() ) {
        inst_sched->push(new ScheduleEntry(stNum, stPri++, DCache, CacheUnit::CompleteReadData));
    } else if ( inst->isStore() ) {
        inst_sched->push(new ScheduleEntry(stNum, stPri++, DTLB, TLBUnit::DataLookup));
        inst_sched->push(new ScheduleEntry(stNum, stPri++, DCache, CacheUnit::InitiateWriteData));
    }

    //
    // WB - Stage 4
    // ---------------------------------------
    stPri = 0; stNum++;
    if (inst->isNonSpeculative()) {
        if (inst->isMemRef())
            fatal("Schedule doesnt handle Non-Speculative Memory Instructions.\n");

        if (inst->opClass() == IntMultOp || inst->opClass() == IntDivOp) {
            inst_sched->push(new ScheduleEntry(stNum, stPri++, MDU, MultDivUnit::MultDiv));
        } else {
            inst_sched->push(new ScheduleEntry(stNum, stPri++, ExecUnit, ExecutionUnit::ExecuteInst));
        }
    }

    if ( inst->isStore() )
        inst_sched->push(new ScheduleEntry(stNum, stPri++, DCache, CacheUnit::CompleteWriteData));

    // Write Back to Register File
    for (int idx=0; idx < inst->numDestRegs(); idx++) {
        inst_sched->push(new ScheduleEntry(stNum, stPri++, RegManager, UseDefUnit::WriteDestReg, idx));
    }

    // Graduate Instructions
    inst_sched->push(new ScheduleEntry(stNum, stPri++, Grad, GraduationUnit::GraduateInst));

    return true;
}

};
