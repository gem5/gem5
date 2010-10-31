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

#ifndef __CPU_INORDER_COMM_HH__
#define __CPU_INORDER_COMM_HH__

#include <vector>

#include "arch/faults.hh"
#include "arch/isa_traits.hh"
#include "base/types.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inst_seq.hh"

/** Struct that defines the information passed from in between stages */
/** This information mainly goes forward through the pipeline. */
struct InterStageStruct {
    int size;
    ThePipeline::DynInstPtr insts[ThePipeline::StageWidth];
    bool squash;
    bool branchMispredict;
    bool branchTaken;
    uint64_t mispredPC;
    uint64_t nextPC;
    InstSeqNum squashedSeqNum;
    bool includeSquashInst;

    InterStageStruct()
        :size(0),  squash(false),
         branchMispredict(false), branchTaken(false),
         mispredPC(0), nextPC(0),
         squashedSeqNum(0), includeSquashInst(false)
    { }

};

/** Turn This into a Class */
/** Struct that defines all backwards communication. */
struct TimeStruct {
    struct stageComm {
        bool squash;
        bool predIncorrect;
        uint64_t branchAddr;

        // @todo: Might want to package this kind of branch stuff into a single
        // struct as it is used pretty frequently.
        bool branchMispredict;
        bool branchTaken;
        Addr mispredPC;
        TheISA::PCState nextPC;

        unsigned branchCount;

        // Represents the instruction that has either been retired or
        // squashed.  Similar to having a single bus that broadcasts the
        // retired or squashed sequence number.
        InstSeqNum doneSeqNum;
        InstSeqNum bdelayDoneSeqNum;
        bool squashDelaySlot;

        //Just in case we want to do a commit/squash on a cycle
        //(necessary for multiple ROBs?)
        bool commitInsts;
        InstSeqNum squashSeqNum;

        // Communication specifically to the IQ to tell the IQ that it can
        // schedule a non-speculative instruction.
        InstSeqNum nonSpecSeqNum;

        bool uncached;
        ThePipeline::DynInstPtr uncachedLoad;

        bool interruptPending;
        bool clearInterrupt;
    };

    stageComm stageInfo[ThePipeline::NumStages][ThePipeline::MaxThreads];

    bool stageBlock[ThePipeline::NumStages][ThePipeline::MaxThreads];
    bool stageUnblock[ThePipeline::NumStages][ThePipeline::MaxThreads];
};

#endif //__CPU_INORDER_COMM_HH__
