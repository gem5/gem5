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
#include <map>
#include <queue>
#include <vector>

#include "arch/isa_traits.hh"
#include "cpu/inorder/params.hh"

class InOrderDynInst;

/* This Namespace contains constants, typedefs, functions and
 * objects specific to the Pipeline Implementation.
 */
namespace ThePipeline {
    // Pipeline Constants
    const unsigned NumStages = 9;
    const unsigned MaxThreads = 2;
    const unsigned StageWidth = 1;
    const unsigned BackEndStartStage = 3;

    // Use this to over-ride default stage widths
    static std::map<unsigned, unsigned> stageBufferSizes;

    //static unsigned interStageBuffSize[NumStages];

    static const unsigned interStageBuffSize[NumStages] = {
        StageWidth, /* Stage 0 - 1 */
        StageWidth, /* Stage 1 - 2 */
        MaxThreads * 4,          /* Stage 2 - 3 */
        StageWidth, /* Stage 3 - 4 */
        MaxThreads * 4, /* Stage 4 - 5 */
        StageWidth, /* Stage 5 - 6 */
        StageWidth, /* Stage 6 - 7 */
        StageWidth, /* Stage 7 - 8 */
        MaxThreads  /* Stage 8 - 9 */
    };


    // Enumerated List of Resources The Pipeline Uses
    enum ResourceList {
       FetchSeq = 0,
       ITLB,
       ICache,
       Decode,
       BPred,
       RegManager,
       AGEN,
       ExecUnit,
       DTLB,
       DCache,
       Grad,
       FetchBuff,
       FetchBuff2
    };

    typedef InOrderCPUParams Params;
    typedef RefCountingPtr<InOrderDynInst> DynInstPtr;

//void initPipelineTraits();

    //////////////////////////
    // RESOURCE SCHEDULING
    //////////////////////////
    struct ScheduleEntry {
        ScheduleEntry(int stage_num, int _priority, int res_num, int _cmd = 0,
                      int _idx = 0) :
            stageNum(stage_num), resNum(res_num), cmd(_cmd),
            idx(_idx), priority(_priority)
        { }
        virtual ~ScheduleEntry(){}

        // Stage number to perform this service.
        int stageNum;

        // Resource ID to access
        int resNum;

        // See specific resource for meaning
        unsigned cmd;

        // See specific resource for meaning
        unsigned idx;

        // Some Resources May Need Priority?
        int priority;
    };

    struct entryCompare {
        bool operator()(const ScheduleEntry* lhs, const ScheduleEntry* rhs) const
        {
            // Prioritize first by stage number that the resource is needed
            if (lhs->stageNum > rhs->stageNum) {
                return true;
            } else if (lhs->stageNum == rhs->stageNum) {
                /*if (lhs->resNum > rhs->resNum) {
                  return true;
                } else {
                  return false;
                  }*/

                if (lhs->priority > rhs->priority) {
                  return true;
                } else {
                  return false;
                }
            } else {
                return false;
            }
        }
    };


    typedef std::priority_queue<ScheduleEntry*, std::vector<ScheduleEntry*>,
                                         entryCompare> ResSchedule;

    void createFrontEndSchedule(DynInstPtr &inst);
    bool createBackEndSchedule(DynInstPtr &inst);
    int getNextPriority(DynInstPtr &inst, int stage_num);
};
#endif
