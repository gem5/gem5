/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#include <list>
#include <vector>

#include "cpu/inorder/resources/resource_list.hh"
#include "cpu/inorder/resource_pool.hh"

using namespace std;
using namespace ThePipeline;

ResourcePool::ResourcePool(InOrderCPU *_cpu, InOrderCPUParams *params)
    : cpu(_cpu), instUnit(NULL), dataUnit(NULL)
{
    //@todo: use this function to instantiate the resources in resource pool. This will help in the
    //auto-generation of this pipeline model.
    //ThePipeline::addResources(resources, memObjects);

    // Declare Resource Objects
    // name - id - bandwidth - latency - CPU - Parameters
    // --------------------------------------------------
    resources.push_back(new FetchSeqUnit("fetch_seq_unit", FetchSeq,
            StageWidth * 2, 0, _cpu, params));

    resources.push_back(new TLBUnit("itlb", ITLB, StageWidth, 0, _cpu, params));


    // Keep track of the instruction fetch unit so we can easily
    // provide a pointer to it in the CPU.
    instUnit = new FetchUnit("icache_port", ICache,
                             StageWidth * MaxThreads, 0, _cpu,
                             params);
    resources.push_back(instUnit);

    resources.push_back(new DecodeUnit("decode_unit", Decode, StageWidth, 0,
            _cpu, params));

    resources.push_back(new BranchPredictor("branch_predictor", BPred,
            StageWidth, 0, _cpu, params));

    for (int i = 0; i < params->numberOfThreads; i++) {
        char fbuff_name[20];
        sprintf(fbuff_name, "fetch_buffer_t%i", i);
        resources.push_back(new InstBuffer(fbuff_name, FetchBuff + i, 4, 0,
                _cpu, params));
    }

    resources.push_back(new UseDefUnit("regfile_manager", RegManager,
            StageWidth * MaxThreads, 0, _cpu, params));

    resources.push_back(new AGENUnit("agen_unit", AGEN, StageWidth, 0, _cpu,
            params));

    resources.push_back(new ExecutionUnit("execution_unit", ExecUnit,
            StageWidth, 0, _cpu, params));

    resources.push_back(new MultDivUnit("mult_div_unit", MDU, 5, 0, _cpu,
            params));

    resources.push_back(new TLBUnit("dtlb", DTLB, StageWidth, 0, _cpu, params));

    // Keep track of the data load/store unit so we can easily provide
    // a pointer to it in the CPU.
    dataUnit = new CacheUnit("dcache_port", DCache,
                             StageWidth * MaxThreads, 0, _cpu,
                             params);
    resources.push_back(dataUnit);

    resources.push_back(new GraduationUnit("graduation_unit", Grad,
            StageWidth * MaxThreads, 0, _cpu, params));
}

void
ResourcePool::init()
{
    for (int i=0; i < resources.size(); i++) {
        resources[i]->init();
    }
}

string
ResourcePool::name()
{
    return cpu->name() + ".ResourcePool";
}


void
ResourcePool::regStats()
{
    DPRINTF(Resource, "Registering Stats Throughout Resource Pool.\n");

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->regStats();
    }
}

ResReqPtr
ResourcePool::request(int res_idx, DynInstPtr inst)
{
    //Make Sure This is a valid resource ID
    assert(res_idx >= 0 && res_idx < resources.size());

    return resources[res_idx]->request(inst);
}

void
ResourcePool::squash(DynInstPtr inst, int res_idx, InstSeqNum done_seq_num,
                     ThreadID tid)
{
    resources[res_idx]->squash(inst, ThePipeline::NumStages-1, done_seq_num, tid);
}

int
ResourcePool::slotsAvail(int res_idx)
{
    return resources[res_idx]->slotsAvail();
}

int
ResourcePool::slotsInUse(int res_idx)
{
    return resources[res_idx]->slotsInUse();
}

void
ResourcePool::scheduleEvent(InOrderCPU::CPUEventType e_type, DynInstPtr inst,
                            int delay,  int res_idx, ThreadID tid)
{
    assert(delay >= 0);

    ResPoolEvent *res_pool_event = new ResPoolEvent(this);

    switch (e_type)
    {
      case InOrderCPU::ActivateThread:
        {
            DPRINTF(Resource, "Scheduling Activate Thread Resource Pool Event for tick %i.\n",
                    curTick() + delay);
            res_pool_event->setEvent(e_type,
                                     inst,
                                     inst->squashingStage,
                                     inst->squashSeqNum,
                                     inst->readTid());
            res_pool_event->schedule(curTick() + cpu->cycles(delay));

        }
        break;

      case InOrderCPU::SuspendThread:
      case InOrderCPU::DeallocateThread:
        {
            DPRINTF(Resource, "Scheduling Deactivate Thread Resource Pool Event for tick %i.\n",
                    curTick() + delay);

            res_pool_event->setEvent(e_type,
                                     inst,
                                     inst->squashingStage,
                                     inst->squashSeqNum,
                                     tid);

            res_pool_event->schedule(curTick() + cpu->cycles(delay));

        }
        break;

      case ResourcePool::InstGraduated:
        {
            DPRINTF(Resource, "Scheduling Inst-Graduated Resource Pool Event for tick %i.\n",
                    curTick() + delay);

            res_pool_event->setEvent(e_type,
                                     inst,
                                     inst->squashingStage,
                                     inst->seqNum,
                                     inst->readTid());
            res_pool_event->schedule(curTick() + cpu->cycles(delay));

        }
        break;

      case ResourcePool::SquashAll:
        {
            DPRINTF(Resource, "Scheduling Squash Resource Pool Event for tick %i.\n",
                    curTick() + delay);
            res_pool_event->setEvent(e_type,
                                     inst,
                                     inst->squashingStage,
                                     inst->squashSeqNum,
                                     inst->readTid());
            res_pool_event->schedule(curTick() + cpu->cycles(delay));

        }
        break;

      default:
        DPRINTF(Resource, "Ignoring Unrecognized CPU Event Type #%i.\n", e_type);
        ; // If Resource Pool doesnt recognize event, we ignore it.
    }
}

void
ResourcePool::unscheduleEvent(int res_idx, DynInstPtr inst)
{
    resources[res_idx]->unscheduleEvent(inst);
}

void
ResourcePool::squashAll(DynInstPtr inst, int stage_num,
                        InstSeqNum done_seq_num, ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Stage %i squashing all instructions above [sn:%i].\n",
            stage_num, tid, done_seq_num);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->squash(inst, stage_num, done_seq_num, tid);
    }
}

void
ResourcePool::activateAll(ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting Thread Activation to all resources.\n",
            tid);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->activateThread(tid);
    }
}

void
ResourcePool::deactivateAll(ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting Thread Deactivation to all resources.\n",
            tid);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->deactivateThread(tid);
    }
}

void
ResourcePool::instGraduated(InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting [sn:%i] graduation to all resources.\n",
            tid, seq_num);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->instGraduated(seq_num, tid);
    }
}

ResourcePool::ResPoolEvent::ResPoolEvent(ResourcePool *_resPool)
    : Event(&mainEventQueue, CPU_Tick_Pri),
      resPool(_resPool)
{ eventType = (InOrderCPU::CPUEventType) Default; }

void
ResourcePool::ResPoolEvent::process()
{
    switch (eventType)
    {
      case InOrderCPU::ActivateThread:
        resPool->activateAll(tid);
        break;

      case InOrderCPU::SuspendThread:
      case InOrderCPU::DeallocateThread:
        resPool->deactivateAll(tid);
        break;

      case ResourcePool::InstGraduated:
        resPool->instGraduated(seqNum, tid);
        break;

      case ResourcePool::SquashAll:
        resPool->squashAll(inst, stageNum, seqNum, tid);
        break;

      default:
        fatal("Unrecognized Event Type");
    }

    resPool->cpu->cpuEventRemoveList.push(this);
}


const char *
ResourcePool::ResPoolEvent::description()
{
    return "Resource Pool event";
}

/** Schedule resource event, regardless of its current state. */
void
ResourcePool::ResPoolEvent::scheduleEvent(int delay)
{
    if (squashed())
        reschedule(curTick() + resPool->cpu->cycles(delay));
    else if (!scheduled())
        schedule(curTick() + resPool->cpu->cycles(delay));
}

/** Unschedule resource event, regardless of its current state. */
void
ResourcePool::ResPoolEvent::unscheduleEvent()
{
    if (scheduled())
        squash();
}
