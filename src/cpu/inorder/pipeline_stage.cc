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
#include "config/the_isa.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/pipeline_stage.hh"
#include "cpu/inorder/resource_pool.hh"
#include "debug/Activity.hh"
#include "debug/InOrderStage.hh"
#include "debug/InOrderStall.hh"
#include "debug/Resource.hh"
#include "debug/ThreadModel.hh"

using namespace std;
using namespace ThePipeline;

PipelineStage::PipelineStage(Params *params, unsigned stage_num)
    : stageNum(stage_num), stageWidth(params->stageWidth),
      numThreads(ThePipeline::MaxThreads), _status(Inactive),
      stageBufferMax(params->stageWidth),
      prevStageValid(false), nextStageValid(false), idle(false)
{
    init(params);
}

PipelineStage::~PipelineStage()
{
   for(ThreadID tid = 0; tid < numThreads; tid++) {
       skidBuffer[tid].clear();
       stalls[tid].resources.clear();
   }
}

void
PipelineStage::init(Params *params)
{
    for(ThreadID tid = 0; tid < numThreads; tid++) {
        stageStatus[tid] = Idle;

        for (int stNum = 0; stNum < NumStages; stNum++) {
            stalls[tid].stage[stNum] = false;
        }
        stalls[tid].resources.clear();

        if (stageNum < BackEndStartStage)
            lastStallingStage[tid] = BackEndStartStage - 1;
        else
            lastStallingStage[tid] = NumStages - 1;
    }

    if ((InOrderCPU::ThreadModel) params->threadModel ==
        InOrderCPU::SwitchOnCacheMiss) {
        switchedOutBuffer.resize(ThePipeline::MaxThreads);
        switchedOutValid.resize(ThePipeline::MaxThreads);
    }
}


std::string
PipelineStage::name() const
{
     return cpu->name() + ".stage" + to_string(stageNum);
}


void
PipelineStage::regStats()
{
   idleCycles
        .name(name() + ".idleCycles")
       .desc("Number of cycles 0 instructions are processed.");
   
    runCycles
        .name(name() + ".runCycles")
        .desc("Number of cycles 1+ instructions are processed.");

    utilization
        .name(name() + ".utilization")
        .desc("Percentage of cycles stage was utilized (processing insts).")
        .precision(6);
    utilization = (runCycles / cpu->numCycles) * 100;
    
}


void
PipelineStage::setCPU(InOrderCPU *cpu_ptr)
{
    cpu = cpu_ptr;

    DPRINTF(InOrderStage, "Set CPU pointer.\n");

    tracer = dynamic_cast<Trace::InOrderTrace *>(cpu->getTracer());
}


void
PipelineStage::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(InOrderStage, "Setting time buffer pointer.\n");
    timeBuffer = tb_ptr;

    // Setup wire to write information back to fetch.
    // @todo: should this be writing to the next stage => -1 and reading from is (0)???
    toPrevStages = timeBuffer->getWire(0);

    // Create wires to get information from proper places in time buffer.
    fromNextStages = timeBuffer->getWire(-1);
}


void
PipelineStage::setPrevStageQueue(TimeBuffer<InterStageStruct> *prev_stage_ptr)
{
    DPRINTF(InOrderStage, "Setting previous stage queue pointer.\n");
    prevStageQueue = prev_stage_ptr;

    // Setup wire to read information from fetch queue.
    prevStage = prevStageQueue->getWire(-1);

    prevStageValid = true;
}



void
PipelineStage::setNextStageQueue(TimeBuffer<InterStageStruct> *next_stage_ptr)
{
    DPRINTF(InOrderStage, "Setting next stage pointer.\n");
    nextStageQueue = next_stage_ptr;

    // Setup wire to write information to proper place in stage queue.
    nextStage = nextStageQueue->getWire(0);
    nextStageValid = true;
}



void
PipelineStage::setActiveThreads(list<ThreadID> *at_ptr)
{
    DPRINTF(InOrderStage, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}

/*inline void
PipelineStage::switchToActive()
{
   if (_status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");

        cpu->activateStage(stageNum);

        _status = Active;
    }
}*/

void
PipelineStage::switchOut()
{
    // Stage can immediately switch out.
    panic("Switching Out of Stages Unimplemented");
}


void
PipelineStage::takeOverFrom()
{
    _status = Inactive;

    // Be sure to reset state and clear out any old instructions.
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        stageStatus[tid] = Idle;

        for (int stNum = 0; stNum < NumStages; stNum++) {
            stalls[tid].stage[stNum] = false;
        }

        stalls[tid].resources.clear();

        skidBuffer[tid].clear();
    }
    wroteToTimeBuffer = false;
}



bool
PipelineStage::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    // Only check pipeline stall from stage directly following this stage
    if (nextStageValid && stalls[tid].stage[stageNum + 1]) {
        DPRINTF(InOrderStage,"[tid:%i]: Stall fom Stage %i detected.\n",
                tid, stageNum + 1);
        ret_val = true;
    }

    if (!stalls[tid].resources.empty()) {
#if TRACING_ON
        string stall_src;

        for (int i=0; i < stalls[tid].resources.size(); i++) {
            stall_src += stalls[tid].resources[i]->res->name() + ":";
        }

        DPRINTF(InOrderStage,"[tid:%i]: Stall fom resources (%s) detected.\n",
                tid, stall_src);
#endif
        ret_val = true;
    }

    return ret_val;
}


void
PipelineStage::removeStalls(ThreadID tid)
{
    for (int st_num = 0; st_num < NumStages; st_num++) {
        if (stalls[tid].stage[st_num]) {
            DPRINTF(InOrderStage, "Removing stall from stage %i.\n",
                    st_num);
            stalls[tid].stage[st_num] = false;
        }

        if (toPrevStages->stageBlock[st_num][tid]) {
            DPRINTF(InOrderStage, "Removing pending block from stage %i.\n",
                    st_num);
            toPrevStages->stageBlock[st_num][tid] = false;
        }

        if (fromNextStages->stageBlock[st_num][tid]) {
            DPRINTF(InOrderStage, "Removing pending block from stage %i.\n",
                    st_num);
            fromNextStages->stageBlock[st_num][tid] = false;
        }
    }
    stalls[tid].resources.clear();
}

inline bool
PipelineStage::prevStageInstsValid()
{
    return prevStage->insts.size() > 0;
}

bool
PipelineStage::isBlocked(ThreadID tid)
{
    return stageStatus[tid] == Blocked;
}

bool
PipelineStage::block(ThreadID tid)
{
    DPRINTF(InOrderStage, "[tid:%d]: Blocking, sending block signal back to "
            "previous stages.\n", tid);

    // If the stage status is blocked or unblocking then stage has not yet
    // signalled fetch to unblock. In that case, there is no need to tell
    // fetch to block.
    if (stageStatus[tid] != Blocked) {
        if (stageStatus[tid] != Unblocking) {
            wroteToTimeBuffer = true;
        }

        stageStatus[tid] = Blocked;

        if (prevStageValid) {
            DPRINTF(InOrderStage, "[tid:%d]: Stage %i setting block signal.\n",
                    tid, stageNum);
            toPrevStages->stageBlock[stageNum][tid] = true;
        }

        return true;
    }


    return false;
}

void
PipelineStage::blockDueToBuffer(ThreadID tid)
{
    DPRINTF(InOrderStage, "[tid:%d]: Blocking instructions from passing to "
            "next stage.\n", tid);

    if (stageStatus[tid] != Blocked) {
        if (stageStatus[tid] != Unblocking) {
            wroteToTimeBuffer = true;
        }

        // Set the status to Blocked.
        stageStatus[tid] = Blocked;
    }
}

bool
PipelineStage::unblock(ThreadID tid)
{
    // @todo: Shouldnt this be if any available slots are open???
    // Stage is done unblocking only if the skid buffer is empty.
    if (skidBuffer[tid].empty()) {
        DPRINTF(InOrderStage, "[tid:%u]: Done unblocking.\n", tid);

        if (prevStageValid)
            toPrevStages->stageUnblock[stageNum][tid] = true;

        wroteToTimeBuffer = true;

        stageStatus[tid] = Running;

        return true;
    }

    DPRINTF(InOrderStage, "[tid:%u]: Currently unblocking.\n", tid);
    return false;
}

void
PipelineStage::setupSquash(DynInstPtr inst, ThreadID tid)
{
    if (cpu->lastSquashCycle[tid] == curTick() &&
        cpu->squashSeqNum[tid] < inst->seqNum){
        DPRINTF(Resource, "Ignoring [sn:%i] branch squash signal due to "
                "another stage's squash signal for after [sn:%i].\n", 
                inst->seqNum, cpu->squashSeqNum[tid]);
    } else {
        InstSeqNum squash_seq_num = inst->squashSeqNum;
        unsigned squash_stage = (nextStageValid) ? stageNum + 1
            : stageNum;

        toPrevStages->stageInfo[squash_stage][tid].squash = true;
        toPrevStages->stageInfo[squash_stage][tid].doneSeqNum =
            squash_seq_num;

        DPRINTF(InOrderStage, "[tid:%i]: Setting up squashing after "
                "[sn:%i], due to [sn:%i] %s. Squash-Start-Stage:%i\n",
                tid, squash_seq_num, inst->seqNum, inst->instName(),
                squash_stage);

        // Save squash num for later stage use
        cpu->lastSquashCycle[tid] = curTick();
        cpu->squashSeqNum[tid] = squash_seq_num;
    }
}

void
PipelineStage::squashDueToMemStall(InstSeqNum seq_num, ThreadID tid)
{
    squash(seq_num, tid);    
}

void
PipelineStage::squashPrevStageInsts(InstSeqNum squash_seq_num, ThreadID tid)
{
    DPRINTF(InOrderStage, "[tid:%i]: Removing instructions from "
            "incoming stage queue.\n", tid);

    int insts_from_prev_stage = prevStage->insts.size();
    for (int i=0; i < insts_from_prev_stage; i++) {
        if (prevStage->insts[i]->threadNumber == tid &&
            prevStage->insts[i]->seqNum > squash_seq_num) {
            DPRINTF(InOrderStage, "[tid:%i]: Squashing instruction, "
                    "[sn:%i] PC %s.\n",
                    tid,
                    prevStage->insts[i]->seqNum,
                    prevStage->insts[i]->pcState());
            prevStage->insts[i]->setSquashed();

            prevStage->insts[i] = cpu->dummyBufferInst;
        }
    }
}

void
PipelineStage::squash(InstSeqNum squash_seq_num, ThreadID tid)
{
    // Set status to squashing.
    stageStatus[tid] = Squashing;

    squashPrevStageInsts(squash_seq_num, tid);

    DPRINTF(InOrderStage, "[tid:%i]: Removing instructions from incoming stage"
            " skidbuffer.\n", tid);
    //@TODO: Walk Through List Using iterator and remove
    //       all instructions over the value
    std::list<DynInstPtr>::iterator cur_it = skidBuffer[tid].begin();
    std::list<DynInstPtr>::iterator end_it = skidBuffer[tid].end();

    while (cur_it != end_it) {
        if ((*cur_it)->seqNum <= squash_seq_num) {
            DPRINTF(InOrderStage, "[tid:%i]: Cannot remove skidBuffer "
                    "instructions (starting w/[sn:%i]) before "
                    "[sn:%i]. %i insts left.\n", tid, 
                    (*cur_it)->seqNum, squash_seq_num,
                    skidBuffer[tid].size());
            cur_it++;
        } else {
            DPRINTF(InOrderStage, "[tid:%i]: Removing instruction, [sn:%i] "
                    " PC %s.\n", tid, (*cur_it)->seqNum, (*cur_it)->pc);
            (*cur_it)->setSquashed();
            cur_it = skidBuffer[tid].erase(cur_it);
        }

    }

}

int
PipelineStage::stageBufferAvail()
{
    unsigned total = 0;

    for (int i=0; i < ThePipeline::MaxThreads; i++) {
        total += skidBuffer[i].size();
    }

    int avail = stageBufferMax - total;
    assert(avail >= 0);

    return avail;
}

bool
PipelineStage::canSendInstToStage(unsigned stage_num)
{
    bool buffer_avail = false;

    if (cpu->pipelineStage[stage_num]->prevStageValid) {
        buffer_avail = cpu->pipelineStage[stage_num]->stageBufferAvail() -
            cpu->pipelineStage[stage_num-1]->nextStage->insts.size() >= 1;
    }

    if (!buffer_avail && nextStageQueueValid(stage_num)) {
        DPRINTF(InOrderStall, "STALL: No room in stage %i buffer.\n", 
                stageNum + 1);
    }

    return buffer_avail;
}

int
PipelineStage::skidSize()
{
    int total = 0;

    for (int i=0; i < ThePipeline::MaxThreads; i++) {
        total += skidBuffer[i].size();
    }

    return total;
}

bool
PipelineStage::skidsEmpty()
{
    list<ThreadID>::iterator threads = activeThreads->begin();

    while (threads != activeThreads->end()) {
        if (!skidBuffer[*threads++].empty())
            return false;
    }

    return true;
}



void
PipelineStage::updateStatus()
{
    bool any_unblocking = false;

    list<ThreadID>::iterator threads = activeThreads->begin();

    while (threads != activeThreads->end()) {
        ThreadID tid = *threads++;

        if (stageStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // Stage will have activity if it's unblocking.
    if (any_unblocking) {
        if (_status == Inactive) {
            _status = Active;

            DPRINTF(Activity, "Activating stage.\n");

            cpu->activateStage(stageNum);
        }
    } else {
        // If it's not unblocking, then stage will not have any internal
        // activity.  Switch it to inactive.
        if (_status == Active) {
            _status = Inactive;
            DPRINTF(Activity, "Deactivating stage.\n");

            cpu->deactivateStage(stageNum);
        }
    }
}

void 
PipelineStage::activateThread(ThreadID tid)
{    
    if (cpu->threadModel == InOrderCPU::SwitchOnCacheMiss) {
        if (!switchedOutValid[tid]) {
            DPRINTF(InOrderStage, "[tid:%i] No instruction available in "
                    "switch out buffer.\n", tid);        
        } else {
            DynInstPtr inst = switchedOutBuffer[tid];

            DPRINTF(InOrderStage,"[tid:%i]: Re-Inserting [sn:%lli] PC:%s into"
                    " stage skidBuffer %i\n", tid, inst->seqNum,
                    inst->pcState(), inst->threadNumber);

            // Make instruction available for pipeline processing
            skidBuffer[tid].push_back(inst);

            // Update PC so that we start fetching after this instruction to
            // prevent "double"-execution of instructions
            cpu->resPool->scheduleEvent((InOrderCPU::CPUEventType)
                                        ResourcePool::UpdateAfterContextSwitch, 
                                        inst, Cycles(0), 0, tid);

            // Clear switchout buffer
            switchedOutBuffer[tid] = NULL;
            switchedOutValid[tid] = false;            

            // Update any CPU stats based off context switches
            cpu->updateContextSwitchStats();            
        }        
    }
    
}


void
PipelineStage::sortInsts()
{
    if (prevStageValid) {
        assert(prevStage->insts.size() <= stageWidth);

        int insts_from_prev_stage = prevStage->insts.size();
        int insts_from_cur_stage = skidSize();
        DPRINTF(InOrderStage, "%i insts available from stage buffer %i. Stage "
                "currently has %i insts from last cycle.\n",
                insts_from_prev_stage, prevStageQueue->id(),
                insts_from_cur_stage);

        int inserted_insts = 0;

        for (int i = 0; i < insts_from_prev_stage; i++) {
            if (prevStage->insts[i]->isSquashed()) {
                DPRINTF(InOrderStage, "[tid:%i]: Ignoring squashed [sn:%i], "
                        "not inserting into stage buffer.\n",
                    prevStage->insts[i]->readTid(),
                    prevStage->insts[i]->seqNum);
                continue;
            }

            ThreadID tid = prevStage->insts[i]->threadNumber;

            if (inserted_insts + insts_from_cur_stage == stageWidth) {
               DPRINTF(InOrderStage, "Stage %i has accepted all insts "
                       "possible for this tick. Placing [sn:%i] in stage %i skidBuffer\n",
                       stageNum, prevStage->insts[i]->seqNum, stageNum - 1);
                cpu->pipelineStage[stageNum - 1]->
                    skidBuffer[tid].push_front(prevStage->insts[i]);

                int prev_stage = stageNum - 1;
                if (cpu->pipelineStage[prev_stage]->stageStatus[tid] == Running ||
                    cpu->pipelineStage[prev_stage]->stageStatus[tid] == Idle) {
                    cpu->pipelineStage[prev_stage]->stageStatus[tid] = Unblocking;
                }
            } else {
                DPRINTF(InOrderStage, "[tid:%i]: Inserting [sn:%i] into stage "
                        "buffer.\n", prevStage->insts[i]->readTid(),
                        prevStage->insts[i]->seqNum);

                skidBuffer[tid].push_back(prevStage->insts[i]);
            }

            prevStage->insts[i] = cpu->dummyBufferInst;

            inserted_insts++;
        }
    }
}



void
PipelineStage::readStallSignals(ThreadID tid)
{
    for (int stage_idx = stageNum+1; stage_idx <= lastStallingStage[tid];
         stage_idx++) {

        DPRINTF(InOrderStage, "[tid:%i] Reading stall signals from Stage "
                "%i. Block:%i Unblock:%i.\n",
                tid,
                stage_idx,
                fromNextStages->stageBlock[stage_idx][tid],
                fromNextStages->stageUnblock[stage_idx][tid]);

        // Check for Stage Blocking Signal
        if (fromNextStages->stageBlock[stage_idx][tid]) {
            DPRINTF(InOrderStage, "[tid:%i] Stall from stage %i set.\n", tid,
                    stage_idx);
            stalls[tid].stage[stage_idx] = true;
        }

        // Check for Stage Unblocking Signal
        if (fromNextStages->stageUnblock[stage_idx][tid]) {
            DPRINTF(InOrderStage, "[tid:%i] Stall from stage %i unset.\n", tid,
                    stage_idx);
            stalls[tid].stage[stage_idx] = false;
        }
    }
}



bool
PipelineStage::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is.
    // Check stall signals, block if necessary.
    // If status was blocked
    //     Check if stall conditions have passed
    //         if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.

    // Update the per thread stall statuses.
    readStallSignals(tid);

    // Check for squash from later pipeline stages
    for (int stage_idx=stageNum; stage_idx < NumStages; stage_idx++) {
        if (fromNextStages->stageInfo[stage_idx][tid].squash) {
            DPRINTF(InOrderStage, "[tid:%u]: Squashing instructions due to "
                    "squash from stage %u.\n", tid, stage_idx);
            InstSeqNum squash_seq_num = fromNextStages->
                stageInfo[stage_idx][tid].doneSeqNum;
            squash(squash_seq_num, tid);
            break; //return true;
        }
    }

    if (checkStall(tid)) {
        return block(tid);
    }

    if (stageStatus[tid] == Blocked) {
        DPRINTF(InOrderStage, "[tid:%u]: Done blocking, switching to "
                "unblocking.\n", tid);

        stageStatus[tid] = Unblocking;

        unblock(tid);

        return true;
    }

    if (stageStatus[tid] == Squashing) {
        if (!skidBuffer[tid].empty()) {
            DPRINTF(InOrderStage, "[tid:%u]: Done squashing, switching to "
                    "unblocking.\n", tid);

            stageStatus[tid] = Unblocking;
        } else {
            // Switch status to running if stage isn't being told to block or
            // squash this cycle.
            DPRINTF(InOrderStage, "[tid:%u]: Done squashing, switching to "
                    "running.\n", tid);

            stageStatus[tid] = Running;
        }

        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause stage to change its status.  Stage remains the same as before.*/
    return false;
}



void
PipelineStage::tick()
{
    idle = false;
    
    wroteToTimeBuffer = false;

    bool status_change = false;
    
    sortInsts();

    instsProcessed = 0;

    processStage(status_change);

    if (status_change) {
        updateStatus();
    }

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }

    DPRINTF(InOrderStage, "\n\n");
}

void
PipelineStage::setResStall(ResReqPtr res_req, ThreadID tid)
{
    DPRINTF(InOrderStage, "Inserting stall from %s.\n", res_req->res->name());
    stalls[tid].resources.push_back(res_req);
}

void
PipelineStage::unsetResStall(ResReqPtr res_req, ThreadID tid)
{
    // Search through stalls to find stalling request and then
    // remove it
    vector<ResReqPtr>::iterator req_it = stalls[tid].resources.begin();
    vector<ResReqPtr>::iterator req_end = stalls[tid].resources.end();

    while (req_it != req_end) {
        if( (*req_it)->res ==  res_req->res && // Same Resource
            (*req_it)->inst ==  res_req->inst && // Same Instruction
            (*req_it)->getSlot() ==  res_req->getSlot()) {
            DPRINTF(InOrderStage, "[tid:%u]: Clearing stall by %s.\n",
                    tid, res_req->res->name());
            stalls[tid].resources.erase(req_it);
            break;
        }

        req_it++;
    }

    if (stalls[tid].resources.size() == 0) {
        DPRINTF(InOrderStage, "[tid:%u]: There are no remaining resource"
                "stalls.\n", tid);
    }
}

// @TODO: Update How we handled threads in CPU. Maybe threads shouldnt be 
// handled one at a time, but instead first come first serve by instruction?
// Questions are how should a pipeline stage handle thread-specific stalls &
// pipeline squashes
void
PipelineStage::processStage(bool &status_change)
{
    list<ThreadID>::iterator threads = activeThreads->begin();

    //Check stall and squash signals.
    while (threads != activeThreads->end()) {
        ThreadID tid = *threads++;

        DPRINTF(InOrderStage,"Processing [tid:%i]\n",tid);
        status_change =  checkSignalsAndUpdate(tid) || status_change;

        processThread(status_change, tid);
    }

    if (nextStageValid) {
        DPRINTF(InOrderStage, "%i insts now available for stage %i.\n",
                nextStage->insts.size(), stageNum + 1);
    }

    if (instsProcessed > 0) {
        ++runCycles;
        idle = false;        
    } else {
        ++idleCycles;        
        idle = true;        
    }
    
    DPRINTF(InOrderStage, "%i left in stage %i incoming buffer.\n", skidSize(),
            stageNum);

    DPRINTF(InOrderStage, "%i available in stage %i incoming buffer.\n", 
            stageBufferAvail(), stageNum);
}

void
PipelineStage::processThread(bool &status_change, ThreadID tid)
{
    // If status is Running or idle,
    //     call processInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from fetch
   //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    // Stage should try to process as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (stageStatus[tid] == Running ||
        stageStatus[tid] == Idle) {
        DPRINTF(InOrderStage, "[tid:%u]: Not blocked, so attempting to run "
                "stage.\n",tid);

        processInsts(tid);
    } else if (stageStatus[tid] == Unblocking) {
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(!skidsEmpty());

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        processInsts(tid);

        status_change = unblock(tid) || status_change;
    }
}


void
PipelineStage::processInsts(ThreadID tid)
{
    // Instructions can come either from the skid buffer or the list of
    // instructions coming from fetch, depending on stage's status.
    int insts_available = skidBuffer[tid].size();

    std::list<DynInstPtr> &insts_to_stage = skidBuffer[tid];

    if (insts_available == 0) {
        DPRINTF(InOrderStage, "[tid:%u]: Nothing to do, breaking out"
                " early.\n",tid);
        return;
    }

    DynInstPtr inst;
    bool last_req_completed = true;

    while (insts_available > 0 &&
           instsProcessed < stageWidth &&
           last_req_completed) {
        assert(!insts_to_stage.empty());

        inst = insts_to_stage.front();

        DPRINTF(InOrderStage, "[tid:%u]: Processing instruction [sn:%lli] "
                "%s with PC %s\n", tid, inst->seqNum,
                inst->instName(),
                inst->pcState());

        if (inst->isSquashed()) {
            DPRINTF(InOrderStage, "[tid:%u]: Instruction %i with PC %s is "
                    "squashed, skipping.\n",
                    tid, inst->seqNum, inst->pcState());

            insts_to_stage.pop_front();

            --insts_available;

            continue;
        }

        int reqs_processed = 0;        
        last_req_completed = processInstSchedule(inst, reqs_processed);

        // If the instruction isnt squashed & we've completed one request
        // Then we can officially count this instruction toward the stage's 
        // bandwidth count
        if (reqs_processed > 0)
            instsProcessed++;

        // Don't let instruction pass to next stage if it hasnt completed
        // all of it's requests for this stage.
        if (!last_req_completed)
            continue;

        // Send to Next Stage or Break Loop
        if (nextStageValid && !sendInstToNextStage(inst)) {
            DPRINTF(InOrderStage, "[tid:%i] [sn:%i] unable to proceed to stage"
                    " %i.\n", tid, inst->seqNum,inst->nextStage);
            break;
        }

        insts_to_stage.pop_front();

        --insts_available;
    }

    // If we didn't process all instructions, then we will need to block
    // and put all those instructions into the skid buffer.
    if (!insts_to_stage.empty()) {
        blockDueToBuffer(tid);
    }

    // Record that stage has written to the time buffer for activity
    // tracking.
    if (instsProcessed) {
        wroteToTimeBuffer = true;
    }
}

bool
PipelineStage::processInstSchedule(DynInstPtr inst,int &reqs_processed)
{
    bool last_req_completed = true;
    ThreadID tid = inst->readTid();

    if (inst->nextResStage() == stageNum) {
        int res_stage_num = inst->nextResStage();

        while (res_stage_num == stageNum) {
            int res_num = inst->nextResource();


            DPRINTF(InOrderStage, "[tid:%i]: [sn:%i]: sending request to %s."
                    "\n", tid, inst->seqNum, cpu->resPool->name(res_num));

            ResReqPtr req = cpu->resPool->request(res_num, inst);
            assert(req->valid);

            bool req_completed = req->isCompleted();
            bool done_in_pipeline = false;
            if (req_completed) {
                DPRINTF(InOrderStage, "[tid:%i]: [sn:%i] request to %s "
                        "completed.\n", tid, inst->seqNum, 
                        cpu->resPool->name(res_num));

                reqs_processed++;                

                req->stagePasses++;                

                done_in_pipeline = inst->finishSkedEntry();
                if (done_in_pipeline) {
                    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] finished "
                            "in pipeline.\n", tid, inst->seqNum);
                }
            } else {
                DPRINTF(InOrderStage, "[tid:%i]: [sn:%i] request to %s failed."
                        "\n", tid, inst->seqNum, cpu->resPool->name(res_num));

                last_req_completed = false;

                if (req->isMemStall() && 
                    cpu->threadModel == InOrderCPU::SwitchOnCacheMiss) {
                    // Save Stalling Instruction
                    DPRINTF(ThreadModel, "[tid:%i] [sn:%i] Detected cache "
                            "miss.\n", tid, inst->seqNum);

                    DPRINTF(InOrderStage, "Inserting [tid:%i][sn:%i] into "
                            "switch out buffer.\n", tid, inst->seqNum);

                    switchedOutBuffer[tid] = inst;
                    switchedOutValid[tid] = true;
                    
                    // Remove Thread From Pipeline & Resource Pool
                    inst->squashingStage = stageNum;
                    inst->squashSeqNum = inst->seqNum;
                    cpu->squashFromMemStall(inst, tid);  

                    // Switch On Cache Miss
                    //=====================
                    // Suspend Thread at end of cycle
                    DPRINTF(ThreadModel, "Suspending [tid:%i] due to cache "
                            "miss.\n", tid);
                    cpu->suspendContext(tid);                    

                    // Activate Next Ready Thread at end of cycle
                    DPRINTF(ThreadModel, "Attempting to activate next ready "
                            "thread due to cache miss.\n");
                    cpu->activateNextReadyContext();
                }
            }

            // If this request is no longer needs to take up bandwidth in the
            // resource, go ahead and free that bandwidth up
            if (req->doneInResource) {
                req->freeSlot();
            }

            // No longer need to process this instruction if the last
            // request it had wasn't completed or if there is nothing
            // else for it to do in the pipeline
            if (done_in_pipeline || !req_completed) {
                break;
            }

            res_stage_num = inst->nextResStage();
        }
    } else {
        DPRINTF(InOrderStage, "[tid:%u]: Instruction [sn:%i] with PC %s "
                " needed no resources in stage %i.\n",
                tid, inst->seqNum, inst->pcState(), stageNum);
    }

    return last_req_completed;
}

bool
PipelineStage::nextStageQueueValid(int stage_num)
{
    return cpu->pipelineStage[stage_num]->nextStageValid;
}


bool
PipelineStage::sendInstToNextStage(DynInstPtr inst)
{
    // Update Next Stage Variable in Instruction
    // NOTE: Some Resources will update this nextStage var. to
    // for bypassing, so can't always assume nextStage=stageNum+1
    if (inst->nextStage == stageNum)
        inst->nextStage++;

    bool success = false;
    ThreadID tid = inst->readTid();
    int next_stage = inst->nextStage;
    int prev_stage = next_stage - 1;

    assert(next_stage >= 1);
    assert(prev_stage >= 0);

    DPRINTF(InOrderStage, "[tid:%u]: Attempting to send instructions to "
            "stage %u.\n", tid, stageNum+1);

    if (!canSendInstToStage(inst->nextStage)) {
        DPRINTF(InOrderStage, "[tid:%u]: Could not send instruction to "
                "stage %u.\n", tid, stageNum+1);
        return false;
    }


    if (nextStageQueueValid(inst->nextStage - 1)) {
        if (inst->seqNum > cpu->squashSeqNum[tid] &&
            curTick() == cpu->lastSquashCycle[tid]) {
            DPRINTF(InOrderStage, "[tid:%u]: [sn:%i]: squashed, skipping "
                    "insertion into stage %i queue.\n", tid, inst->seqNum, 
                    inst->nextStage);
        } else {
            if (nextStageValid) {
                DPRINTF(InOrderStage, "[tid:%u] %i slots available in next "
                        "stage buffer.\n", tid, 
                        cpu->pipelineStage[next_stage]->stageBufferAvail());
            }

            DPRINTF(InOrderStage, "[tid:%u]: [sn:%i]: being placed into  "
                    "index %i of stage buffer %i queue.\n",
                    tid, inst->seqNum,
                    cpu->pipelineStage[prev_stage]->nextStage->insts.size(),
                    cpu->pipelineStage[prev_stage]->nextStageQueue->id());

            // Place instructions in inter-stage communication struct for next
            // pipeline stage to read next cycle
            cpu->pipelineStage[prev_stage]->nextStage->insts.push_back(inst);

            success = true;

            // Take note of trace data for this inst & stage
            if (inst->traceData) {
                //@todo: exec traces are broke. fix them
                inst->traceData->setStageCycle(stageNum, curTick());
            }

        }
    }

    return success;
}

void
PipelineStage::dumpInsts()
{
    cprintf("Insts in Stage %i skidbuffers\n",stageNum);

    for (ThreadID tid = 0; tid < ThePipeline::MaxThreads; tid++) {
        std::list<DynInstPtr>::iterator cur_it =  skidBuffer[tid].begin();
        std::list<DynInstPtr>::iterator end_it =  skidBuffer[tid].end();

        while (cur_it != end_it) {
            DynInstPtr inst = (*cur_it);

            cprintf("Inst. PC:%s\n[tid:%i]\n[sn:%i]\n\n",
                    inst->pcState(), inst->threadNumber, inst->seqNum);

            cur_it++;
        }
    }

}
