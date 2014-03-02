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

#ifndef __CPU_INORDER_FIRST_STAGE_HH__
#define __CPU_INORDER_FIRST_STAGE_HH__

#include <queue>
#include <vector>

#include "base/statistics.hh"
#include "cpu/inorder/comm.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_stage.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/timebuf.hh"

class InOrderCPU;

class FirstStage : public PipelineStage {
  public:
    FirstStage(ThePipeline::Params *params, unsigned stage_num);

    /** Set Pointer to CPU */
    void setCPU(InOrderCPU *cpu_ptr);

    /** Evaluate Stage Info. & Execute Stage */
    void processStage(bool &status_change);

    /** Process All Instructions Available */
    void processInsts(ThreadID tid);

    /** Squash Instructions Above a Seq. Num */
    void squash(InstSeqNum squash_seq_num, ThreadID tid);

    void squashDueToMemStall(InstSeqNum seq_num, ThreadID tid);

    /** There are no insts. coming from previous stages, so there is
     *  no need to sort insts here
     */
    void sortInsts() {}

    /** The number of fetching threads in the CPU */
    int numFetchingThreads;

    //@TODO: Add fetch priority information to a resource class...
    /** Fetching Policy, Add new policies here.*/
    enum FetchPriority {
        SingleThread,
        RoundRobin
    };

    /** Fetch policy. */
    FetchPriority fetchPolicy;

    /** List that has the threads organized by priority. */
    std::list<ThreadID> *fetchPriorityList;

    /** Return the next fetching thread */
    ThreadID getFetchingThread(FetchPriority &fetch_priority);

    /** Return next thread given Round Robin Policy for Thread Fetching */
    ThreadID roundRobin();

    /** Takes over from another CPU's thread. */
    void takeOverFrom();
};

#endif // __CPU_INORDER_FIRST_STAGE_HH__
