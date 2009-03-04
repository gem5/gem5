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
    : PipelineStage(params, stage_num)
{
    for(int tid=0; tid < this->numThreads; tid++) {
        stageStatus[tid] = Running;
    }

    numFetchingThreads = 1;

    fetchPolicy = RoundRobin;
}

void
FirstStage::setCPU(InOrderCPU *cpu_ptr)
{
    cpu = cpu_ptr;

    fetchPriorityList = &cpu->fetchPriorityList;

    DPRINTF(InOrderStage, "Set CPU pointer.\n");
}


void
FirstStage::squash(InstSeqNum squash_seq_num, unsigned tid)
{
    // Set status to squashing.
    //stageStatus[tid] = Squashing;

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    DPRINTF(InOrderStage, "Removing instructions from stage instruction list.\n");
    while (!insts[tid].empty()) {
        if (insts[tid].front()->seqNum <= squash_seq_num) {
            DPRINTF(InOrderStage,"[tid:%i]: Cannot remove [sn:%i] because it's <= "
                    "squashing seqNum %i.\n",
                    tid,
                    insts[tid].front()->seqNum,
                    squash_seq_num);

            DPRINTF(InOrderStage, "[tid:%i]: Cannot remove incoming "
                    "instructions before delay slot [sn:%i]. %i insts"
                    "left.\n", tid, squash_seq_num,
                    insts[tid].size());
            break;
        }
        DPRINTF(InOrderStage, "[tid:%i]: Removing instruction, [sn:%i] PC %08p.\n",
                tid, insts[tid].front()->seqNum, insts[tid].front()->PC);
        insts[tid].pop();
    }

    // Now that squash has propagated to the first stage,
    // Alert CPU to remove instructions from the CPU instruction list.
    // @todo: Move this to the CPU object.
    cpu->removeInstsUntil(squash_seq_num, tid);
}

void
FirstStage::processStage(bool &status_change)
{
    list<unsigned>::iterator threads = (*activeThreads).begin();

    //Check stall and squash signals.
    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;
        status_change =  checkSignalsAndUpdate(tid) || status_change;
    }

    for (int threadFetched = 0; threadFetched < numFetchingThreads;
         threadFetched++) {
        int tid = getFetchingThread(fetchPolicy);

        if (tid >= 0) {
            DPRINTF(InOrderStage, "Processing [tid:%i]\n",tid);
            processThread(status_change, tid);
        } else {
            DPRINTF(InOrderStage, "No more threads to fetch from.\n");
        }
    }
}

//@TODO: Note in documentation, that when you make a pipeline stage change, then
//make sure you change the first stage too
void
FirstStage::processInsts(unsigned tid)
{
    bool all_reqs_completed = true;

    for (int insts_fetched = 0; insts_fetched < stageWidth && canSendInstToStage(1); insts_fetched++) {
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
                                    tid);

#if TRACING_ON
            inst->traceData =
                tracer->getInstRecord(ThePipeline::NumStages,
                                      cpu->stageTracing,
                                      cpu->thread[tid]->getTC());

#endif      // TRACING_ON

            DPRINTF(RefCount, "creation: [tid:%i]: [sn:%i]: Refcount = %i.\n",
                    inst->readTid(),
                    inst->seqNum,
                    0/*inst->curCount()*/);

            // Add instruction to the CPU's list of instructions.
            inst->setInstListIt(cpu->addInst(inst));

            DPRINTF(RefCount, "after add to CPU List: [tid:%i]: [sn:%i]: Refcount = %i.\n",
                    inst->readTid(),
                    inst->seqNum,
                    0/*inst->curCount()*/);

            // Create Front-End Resource Schedule For Instruction
            ThePipeline::createFrontEndSchedule(inst);
        }

        // Don't let instruction pass to next stage if it hasnt completed
        // all of it's requests for this stage.
        all_reqs_completed = processInstSchedule(inst);

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
        //++stageProcessedInsts;
    }

    // Record that stage has written to the time buffer for activity
    // tracking.
    if (toNextStageIndex) {
        wroteToTimeBuffer = true;
    }
}

int
FirstStage::getFetchingThread(FetchPriority &fetch_priority)
{
    if (numThreads > 1) {
        switch (fetch_priority) {

          case SingleThread:
            return 0;

          case RoundRobin:
            return roundRobin();

          default:
            return -1;
        }
    } else {
        int tid = *((*activeThreads).begin());

        if (stageStatus[tid] == Running ||
            stageStatus[tid] == Idle) {
            return tid;
        } else {
            return -1;
        }
    }

}

int
FirstStage::roundRobin()
{
    list<unsigned>::iterator pri_iter = (*fetchPriorityList).begin();
    list<unsigned>::iterator end      = (*fetchPriorityList).end();

    int high_pri;

    while (pri_iter != end) {
        high_pri = *pri_iter;

        assert(high_pri <= numThreads);

        if (stageStatus[high_pri] == Running ||
            stageStatus[high_pri] == Idle) {

            (*fetchPriorityList).erase(pri_iter);
            (*fetchPriorityList).push_back(high_pri);

            return high_pri;
        }

        pri_iter++;
    }

    return -1;
}
