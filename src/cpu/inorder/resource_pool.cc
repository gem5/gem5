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

#include "cpu/inorder/resource_pool.hh"
#include "cpu/inorder/resources/resource_list.hh"

#include <vector>
#include <list>

using namespace std;
using namespace ThePipeline;

ResourcePool::ResourcePool(InOrderCPU *_cpu, ThePipeline::Params *params)
    : cpu(_cpu)
{
    //@todo: use this function to instantiate the resources in resource pool. 
    //This will help in the auto-generation of this pipeline model.
    //ThePipeline::addResources(resources, memObjects);

    // Declare Resource Objects
    // name - id - bandwidth - latency - CPU - Parameters
    // --------------------------------------------------
    resources.push_back(new FetchSeqUnit("Fetch-Seq-Unit", FetchSeq, 
                                         StageWidth * 2, 0, _cpu, params));

    memObjects.push_back(ICache);
    resources.push_back(new CacheUnit("icache_port", ICache, 
                                      StageWidth * MaxThreads, 0, _cpu, 
                                      params));

    resources.push_back(new DecodeUnit("Decode-Unit", Decode, 
                                       StageWidth, 0, _cpu, params));

    resources.push_back(new BranchPredictor("Branch-Predictor", BPred, 
                                            StageWidth, 0, _cpu, params));

    resources.push_back(new InstBuffer("Fetch-Buffer-T0", FetchBuff, 4, 
                                       0, _cpu, params));

    resources.push_back(new UseDefUnit("RegFile-Manager", RegManager, 
                                       StageWidth * MaxThreads, 0, _cpu, 
                                       params));

    resources.push_back(new AGENUnit("AGEN-Unit", AGEN, 
                                     StageWidth, 0, _cpu, params));

    resources.push_back(new ExecutionUnit("Execution-Unit", ExecUnit, 
                                          StageWidth, 0, _cpu, params));

    resources.push_back(new MultDivUnit("Mult-Div-Unit", MDU, 5, 0, _cpu, 
                                        params));

    memObjects.push_back(DCache);
    resources.push_back(new CacheUnit("dcache_port", DCache, 
                                      StageWidth * MaxThreads, 0, _cpu, 
                                      params));

    resources.push_back(new GraduationUnit("Graduation-Unit", Grad, 
                                           StageWidth * MaxThreads, 0, _cpu, 
                                           params));

    resources.push_back(new InstBuffer("Fetch-Buffer-T1", FetchBuff2, 4, 
                                       0, _cpu, params));
}

void
ResourcePool::init()
{
    for (int i=0; i < resources.size(); i++) {
        DPRINTF(Resource, "Initializing resource: %s.\n", 
                resources[i]->name());
        
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

Port *
ResourcePool::getPort(const std::string &if_name, int idx)
{
    DPRINTF(Resource, "Binding %s in Resource Pool.\n", if_name);

    for (int i = 0; i < memObjects.size(); i++) {
        int obj_idx = memObjects[i];
        Port *port = resources[obj_idx]->getPort(if_name, idx);
        if (port != NULL) {
            DPRINTF(Resource, "%s set to resource %s(#%i) in Resource Pool.\n", 
                    if_name, resources[obj_idx]->name(), obj_idx);
            return port;
        }
    }

    return NULL;
}

unsigned
ResourcePool::getPortIdx(const std::string &port_name)
{
    DPRINTF(Resource, "Finding Port Idx for %s.\n", port_name);

    for (int i = 0; i < memObjects.size(); i++) {
        unsigned obj_idx = memObjects[i];
        Port *port = resources[obj_idx]->getPort(port_name, obj_idx);
        if (port != NULL) {
            DPRINTF(Resource, "Returning Port Idx %i for %s.\n", obj_idx, 
                    port_name);
            return obj_idx;
        }
    }

    return 0;
}

unsigned
ResourcePool::getResIdx(const std::string &res_name)
{
    DPRINTF(Resource, "Finding Resource Idx for %s.\n", res_name);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        if (resources[idx]->name() == res_name)
            return idx;
    }

    return 0;
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
    resources[res_idx]->squash(inst, ThePipeline::NumStages-1, done_seq_num, 
                               tid);
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

    switch (e_type)
    {
      case InOrderCPU::ActivateThread:
        {
            DPRINTF(Resource, "Scheduling Activate Thread Resource Pool Event "
                    "for tick %i, [tid:%i].\n", curTick + delay, 
                    inst->readTid());
            ResPoolEvent *res_pool_event = 
                new ResPoolEvent(this,
                                 e_type,
                                 inst,
                                 inst->squashingStage,
                                 inst->bdelaySeqNum,
                                 inst->readTid());
            mainEventQueue.schedule(res_pool_event, 
                                    curTick + cpu->ticks(delay));
        }
        break;

      case InOrderCPU::SuspendThread:
      case InOrderCPU::DeallocateThread:
        {

            DPRINTF(Resource, "Scheduling Deactivate Thread Resource Pool "
                    "Event for tick %i.\n", curTick + delay);
            ResPoolEvent *res_pool_event = 
                new ResPoolEvent(this,
                                 e_type,
                                 inst,
                                 inst->squashingStage,
                                 inst->bdelaySeqNum,
                                 tid);

            mainEventQueue.schedule(res_pool_event, 
                                    curTick + cpu->ticks(delay));

        }
        break;

      case ResourcePool::InstGraduated:
        {
            DPRINTF(Resource, "Scheduling Inst-Graduated Resource Pool "
                    "Event for tick %i.\n", curTick + delay);
            ResPoolEvent *res_pool_event = 
                new ResPoolEvent(this,e_type,
                                 inst,
                                 inst->squashingStage,
                                 inst->seqNum,
                                 inst->readTid());
            mainEventQueue.schedule(res_pool_event, 
                                    curTick + cpu->ticks(delay));

        }
        break;

      case ResourcePool::SquashAll:
        {
            DPRINTF(Resource, "Scheduling Squash Resource Pool Event for "
                    "tick %i.\n", curTick + delay);
            ResPoolEvent *res_pool_event = 
                new ResPoolEvent(this,e_type,
                                 inst,
                                 inst->squashingStage,
                                 inst->bdelaySeqNum,
                                 inst->readTid());
            mainEventQueue.schedule(res_pool_event, 
                                    curTick + cpu->ticks(delay));
        }
        break;

      case InOrderCPU::SquashFromMemStall:
        {
            DPRINTF(Resource, "Scheduling Squash Due to Memory Stall Resource "
                    "Pool Event for tick %i.\n",
                    curTick + delay);
            ResPoolEvent *res_pool_event = 
                new ResPoolEvent(this,e_type,
                                 inst,
                                 inst->squashingStage,
                                 inst->seqNum - 1,
                                 inst->readTid());
            mainEventQueue.schedule(res_pool_event, 
                                    curTick + cpu->ticks(delay));
        }
        break;

      default:
        DPRINTF(Resource, "Ignoring Unrecognized CPU Event (%s).\n", 
                InOrderCPU::eventNames[e_type]);
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
    DPRINTF(Resource, "[tid:%i] Stage %i squashing all instructions above "
            "[sn:%i].\n", tid, stage_num, done_seq_num);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->squash(inst, stage_num, done_seq_num, tid);
    }
}

void
ResourcePool::squashDueToMemStall(DynInstPtr inst, int stage_num,
                             InstSeqNum done_seq_num, ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Stage %i squashing all instructions above "
            "[sn:%i].\n", stage_num, tid, done_seq_num);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->squashDueToMemStall(inst, stage_num, done_seq_num, 
                                            tid);
    }
}

void
ResourcePool::activateAll(ThreadID tid)
{
    bool do_activate = cpu->threadModel != InOrderCPU::SwitchOnCacheMiss ||
        cpu->numActiveThreads() < 1 ||
        cpu->activeThreadId() == tid;
    
        
    if (do_activate) {
        DPRINTF(Resource, "[tid:%i] Broadcasting Thread Activation to all "
                    "resources.\n", tid);
 
        int num_resources = resources.size();
 
        for (int idx = 0; idx < num_resources; idx++) {
            resources[idx]->activateThread(tid);
        }
    } else {
        DPRINTF(Resource, "[tid:%i] Ignoring Thread Activation to all "
                    "resources.\n", tid);
     }
}

void
ResourcePool::deactivateAll(ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting Thread Deactivation to all "
            "resources.\n", tid);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->deactivateThread(tid);
    }
}

void
ResourcePool::instGraduated(InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting [sn:%i] graduation to all "
            "resources.\n", tid, seq_num);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->instGraduated(seq_num, tid);
    }
}

ResourcePool::ResPoolEvent::ResPoolEvent(ResourcePool *_resPool)
    : Event((Event::Priority)((unsigned)CPU_Tick_Pri+5)), resPool(_resPool),
      eventType((InOrderCPU::CPUEventType) Default)
{ }

ResourcePool::ResPoolEvent::ResPoolEvent(ResourcePool *_resPool,
                                         InOrderCPU::CPUEventType e_type,
                                         DynInstPtr _inst,
                                         int stage_num,
                                         InstSeqNum seq_num,
                                         ThreadID _tid)
    : Event(CPU_Tick_Pri), resPool(_resPool),
      eventType(e_type), inst(_inst), seqNum(seq_num),
      stageNum(stage_num), tid(_tid)
{ }


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

      case InOrderCPU::SquashFromMemStall:
        resPool->squashDueToMemStall(inst, stageNum, seqNum, tid);
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
      mainEventQueue.reschedule(this,curTick + resPool->cpu->ticks(delay));
    else if (!scheduled())
      mainEventQueue.schedule(this,curTick + resPool->cpu->ticks(delay));
}

/** Unschedule resource event, regardless of its current state. */
void
ResourcePool::ResPoolEvent::unscheduleEvent()
{
    if (scheduled())
        squash();
}
