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

#include "arch/isa_traits.hh"
#include "base/types.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inst_seq.hh"

/** Struct that defines the information passed from in between stages */
/** This information mainly goes forward through the pipeline. */
struct InterStageStruct {
    //@todo: probably should make this a list since the amount of
    //       instructions that get passed forward per cycle is
    //       really dependent on issue width, CPI, etc.
    std::vector<ThePipeline::DynInstPtr> insts;

    // Add any information that needs to be passed forward to stages
    // below ...
};

/** Struct that defines all backwards communication. */
struct TimeStruct {
    struct StageComm {
        bool squash;
        InstSeqNum doneSeqNum;

        bool uncached;
        ThePipeline::DynInstPtr uncachedLoad;

        StageComm()
            : squash(false), doneSeqNum(0), uncached(false), uncachedLoad(NULL)
        { }
    };

    StageComm stageInfo[ThePipeline::NumStages][ThePipeline::MaxThreads];
    bool stageBlock[ThePipeline::NumStages][ThePipeline::MaxThreads];
    bool stageUnblock[ThePipeline::NumStages][ThePipeline::MaxThreads];

    TimeStruct()
    {
        for (int i = 0; i < ThePipeline::NumStages; i++) {
            for (int j = 0; j < ThePipeline::MaxThreads; j++) {
                stageBlock[i][j] = false;
                stageUnblock[i][j] = false;
            }
        }
    }

};

#endif //__CPU_INORDER_COMM_HH__
