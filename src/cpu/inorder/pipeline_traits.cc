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

#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/resources/resource_list.hh"

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
    InstStage *F = inst->addStage();
    InstStage *D = inst->addStage();

    // FETCH
    F->needs(FetchSeq, FetchSeqUnit::AssignNextPC);
    F->needs(ICache, FetchUnit::InitiateFetch);

    // DECODE
    D->needs(ICache, FetchUnit::CompleteFetch);
    D->needs(Decode, DecodeUnit::DecodeInst);
    D->needs(BPred, BranchPredictor::PredictBranch);
    D->needs(FetchSeq, FetchSeqUnit::UpdateTargetPC);

    inst->resSched.init();
}

bool createBackEndSchedule(DynInstPtr &inst)
{
    if (!inst->staticInst) {
        return false;
    }

    InstStage *X = inst->addStage();
    InstStage *M = inst->addStage();
    InstStage *W = inst->addStage();

    // EXECUTE
    for (int idx=0; idx < inst->numSrcRegs(); idx++) {
        if (!idx || !inst->isStore()) {
            X->needs(RegManager, UseDefUnit::ReadSrcReg, idx);
        }
    }

    if ( inst->isNonSpeculative() ) {
        // skip execution of non speculative insts until later
    } else if ( inst->isMemRef() ) {
        if ( inst->isLoad() ) {
            X->needs(AGEN, AGENUnit::GenerateAddr);
        }
    } else if (inst->opClass() == IntMultOp || inst->opClass() == IntDivOp) {
        X->needs(MDU, MultDivUnit::StartMultDiv);
    } else {
        X->needs(ExecUnit, ExecutionUnit::ExecuteInst);
    }

    if (inst->opClass() == IntMultOp || inst->opClass() == IntDivOp) {
        X->needs(MDU, MultDivUnit::EndMultDiv);
    }

    // MEMORY
    if ( inst->isLoad() ) {
        M->needs(DCache, CacheUnit::InitiateReadData);
    } else if ( inst->isStore() ) {
        if ( inst->numSrcRegs() >= 2 ) {            
            M->needs(RegManager, UseDefUnit::ReadSrcReg, 1);
        }        
        M->needs(AGEN, AGENUnit::GenerateAddr);
        M->needs(DCache, CacheUnit::InitiateWriteData);
    }


    // WRITEBACK
    if ( inst->isLoad() ) {
        W->needs(DCache, CacheUnit::CompleteReadData);
    } else if ( inst->isStore() ) {
        W->needs(DCache, CacheUnit::CompleteWriteData);
    }

    if ( inst->isNonSpeculative() ) {
        if ( inst->isMemRef() ) fatal("Non-Speculative Memory Instruction");
        W->needs(ExecUnit, ExecutionUnit::ExecuteInst);
    }

    for (int idx=0; idx < inst->numDestRegs(); idx++) {
        W->needs(RegManager, UseDefUnit::WriteDestReg, idx);
    }

    W->needs(Grad, GraduationUnit::GraduateInst);

    return true;
}

InstStage::InstStage(DynInstPtr inst, int stage_num)
{
    stageNum = stage_num;
    nextTaskPriority = 0;
    instSched = &inst->resSched;
}

void
InstStage::needs(int unit, int request) {
    instSched->push( new ScheduleEntry(
                         stageNum, nextTaskPriority++, unit, request
                         ));
}

void
InstStage::needs(int unit, int request, int param) {
    instSched->push( new ScheduleEntry(
                         stageNum, nextTaskPriority++, unit, request, param
                         ));
}

};
