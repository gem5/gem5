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

#ifndef __CPU_INORDER_PIPELINE_IMPL_HH__
#define __CPU_INORDER_PIPELINE_IMPL_HH__

#include <list>
#include <queue>
#include <vector>

#include "arch/isa_traits.hh"
#include "cpu/base.hh"
#include "params/InOrderCPU.hh"

class InOrderDynInst;
class ScheduleEntry;
class ResourceSked;

/* This Namespace contains constants, typedefs, functions and
 * objects specific to the Pipeline Implementation.
 */
namespace ThePipeline {
    // Pipeline Constants
    const unsigned NumStages = 5;
    const ThreadID MaxThreads = 1;
    const unsigned BackEndStartStage = 2;

    // List of Resources The Pipeline Uses
    enum ResourceId {
       FetchSeq = 0,
       ICache,
       Decode,
       BPred,
       FetchBuff,
       RegManager,
       AGEN,
       ExecUnit,
       MDU,
       DCache,
       Grad,
       FetchBuff2
    };

    typedef InOrderCPUParams Params;
    typedef RefCountingPtr<InOrderDynInst> DynInstPtr;

    //////////////////////////
    // RESOURCE SCHEDULING
    //////////////////////////
    typedef ResourceSked ResSchedule;
    typedef ResourceSked* RSkedPtr;
}




#endif
