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

#include "base/str.hh"
#include "cpu/inorder/first_stage.hh"
#include "cpu/inorder/resources/resource_list.hh"
#include "cpu/inorder/resource_pool.hh"
#include "cpu/inorder/cpu.hh"
#include "params/InOrderTrace.hh"

using namespace std;
using namespace ThePipeline;

FirstStage::FirstStage(Params *params, unsigned stage_num)
    : PipelineStage(params, stage_num), numFetchingThreads(1),
      fetchPolicy(FirstStage::RoundRobin)
{
    for(ThreadID tid = 0; tid < this->numThreads; tid++) {
        stageStatus[tid] = Running;
    }
}

void
FirstStage::setCPU(InOrderCPU *cpu_ptr)
{
    cpu = cpu_ptr;

    fetchPriorityList = &cpu->fetchPriorityList;

    DPRINTF(InOrderStage, "Set CPU pointer.\n");
}


void
FirstStage::squash(InstSeqNum squash_seq_num, ThreadID tid)
{
    // Set status to squashing.
    //stageStatus[tid] = Squashing;

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    DPRINTF(InOrderStage, "Removing instructions from stage instruction "
            "list.\n");
    while (!insts[tid].empty()) {
        if (insts[tid].front()->seqNum <= squash_seq_num) {
            DPRINTF(InOrderStage,"[tid:%i]: Cannot remove [sn:%i] because "
                    "it's <= squashing seqNum %i.\n",
                    tid,
                    insts[tid].front()->seqNum,
                    squash_seq_num);

            DPRINTF(InOrderStage, "[tid:%i]: Cannot remove incoming "
                    "instructions before delay slot [sn:%i]. %i insts"
                    "left.\n", tid, squash_seq_num,
                    insts[tid].size());
            break;
        }
        DPRINTF(InOrderStage, "[tid:%i]: Removing instruction, [sn:%i] "
                "PC %s.\n", tid, insts[tid].front()->seqNum,
                insts[tid].front()->pc);
        insts[tid].pop();
    }

    // Now that squash has propagated to the first stage,
    // Alert CPU to remove instructions from the CPU instruction list.
    // @todo: Move this to the CPU object.
    cpu->removeInstsUntil(squash_seq_num, tid);
}

void 
FirstStage::squashDueToMemStall(InstSeqNum seq_num, ThreadID tid)
{
    // Need to preserve the stalling instruction in first-stage
    // since the squash() from first stage also removes
    // the instruction from the CPU (removeInstsUntil). If that
    // functionality gets changed then you can move this offset.
    // (stalling instruction = seq_num + 1)
    squash(seq_num+1, tid);
}


void
FirstStage::processStage(bool &status_change)
{
    list<ThreadID>::iterator threads = activeThreads->begin();

    //Check stall and squash signals.
    while (threads != activeThreads->end()) {
        ThreadID tid = *threads++;
        status_change =  checkSignalsAndUpdate(tid) || status_change;
    }

    for (int insts_fetched = 0; 
         insts_fetched < stageWidth && canSendInstToStage(1); 
         insts_fetched++) {
        ThreadID tid = getFetchingThread(fetchPolicy);

        if (tid >= 0) {
            DPRINTF(InOrderStage, "Processing [tid:%i]\n",tid);
            processThread(status_change, tid);
        } else {
            DPRINTF(InOrderStage, "No more threads to fetch from.\n");
        }
    }

    if (instsProcessed > 0) {
        ++runCycles;
        idle = false;        
    } else {
        ++idleCycles;
        idle = true;        
    }

}

//@TODO: Note in documentation, that when you make a pipeline stage change, 
//then make sure you change the first stage too
void
FirstStage::processInsts(ThreadID tid)
{
    bool all_reqs_completed = true;

    for (int insts_fetched = 0; 
         insts_fetched < stageWidth && canSendInstToStage(1); 
         insts_fetched++) {

        DynInstPtr inst;
        bool new_inst = false;

        if (!insts[tid].empty()) {
            inst = insts[tid].front();
        } else {
            // Get new instruction.
            new_inst = true;

            inst = new InOrderDynInst(cpu,
                                      cpu->thread[tid],
                                      cpu->nextInstSeqNum(tid),
                                      tid,
                                      tid);

#if TRACING_ON
            inst->traceData =
                tracer->getInstRecord(ThePipeline::NumStages,
                                      cpu->stageTracing,
                                      cpu->thread[tid]->getTC());

#else
            inst->traceData = NULL;
#endif      // TRACING_ON

            // Add instruction to the CPU's list of instructions.
            inst->setInstListIt(cpu->addInst(inst));

            // Create Front-End Resource Schedule For Instruction
            ThePipeline::createFrontEndSchedule(inst);
        }

        int reqs_processed = 0;            
        all_reqs_completed = processInstSchedule(inst, reqs_processed);

        // If the instruction isnt squashed & we've completed one request
        // Then we can officially count this instruction toward the stage's 
        // bandwidth count
        if (reqs_processed > 0)
            instsProcessed++;

        if (!all_reqs_completed) {
            if (new_inst) {
                DPRINTF(InOrderStage, "[tid:%u]: [sn:%u] Did not finish all "
                        "requests for this stage. Keep in stage inst. "
                        "list.\n", tid, inst->seqNum);
                insts[tid].push(inst);
            }
            break;
        } else if (!insts[tid].empty()){
            insts[tid].pop();
        }

        sendInstToNextStage(inst);
    }

    // Record that stage has written to the time buffer for activity
    // tracking.
    if (toNextStageIndex) {
        wroteToTimeBuffer = true;
    }
}

ThreadID
FirstStage::getFetchingThread(FetchPriority &fetch_priority)
{
    ThreadID num_active_threads = cpu->numActiveThreads();

    if (num_active_threads > 1) {
        switch (fetch_priority) {
          case SingleThread:
            return cpu->activeThreadId();

          case RoundRobin:
            return roundRobin();

          default:
            return InvalidThreadID;
        }
    } else if (num_active_threads == 1) {
        ThreadID tid = *activeThreads->begin();

        if (stageStatus[tid] == Running ||
            stageStatus[tid] == Idle) {
            return tid;
        } else {
            return InvalidThreadID;
        }
    } else {
        return InvalidThreadID;
    }    
}

ThreadID
FirstStage::roundRobin()
{
    list<ThreadID>::iterator pri_iter = fetchPriorityList->begin();
    list<ThreadID>::iterator end      = fetchPriorityList->end();

    ThreadID high_pri;

    while (pri_iter != end) {
        high_pri = *pri_iter;

        assert(high_pri <= numThreads);

        if (stageStatus[high_pri] == Running ||
            stageStatus[high_pri] == Idle) {

            fetchPriorityList->erase(pri_iter);
            fetchPriorityList->push_back(high_pri);

            return high_pri;
        }

        pri_iter++;
    }

    return InvalidThreadID;
}
