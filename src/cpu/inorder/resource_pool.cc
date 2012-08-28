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
#include "debug/Resource.hh"

using namespace std;
using namespace ThePipeline;

ResourcePool::ResourcePool(InOrderCPU *_cpu, ThePipeline::Params *params)
    : cpu(_cpu), instUnit(NULL), dataUnit(NULL)
{
    //@todo: use this function to instantiate the resources in resource pool. 
    //This will help in the auto-generation of this pipeline model.
    //ThePipeline::addResources(resources, memObjects);

    int stage_width = cpu->stageWidth;

    // Declare Resource Objects
    // name - id - bandwidth - latency - CPU - Parameters
    // --------------------------------------------------
    resources.push_back(new FetchSeqUnit("fetch_seq_unit", FetchSeq,
                                         stage_width * 2, Cycles(0),
                                         _cpu, params));

    // Keep track of the instruction fetch unit so we can easily
    // provide a pointer to it in the CPU.
    instUnit = new FetchUnit("icache_port", ICache,
                             stage_width * 2 + MaxThreads, Cycles(0), _cpu,
                             params);
    resources.push_back(instUnit);

    resources.push_back(new DecodeUnit("decode_unit", Decode,
                                       stage_width, Cycles(0), _cpu,
                                       params));

    resources.push_back(new BranchPredictor("branch_predictor", BPred,
                                            stage_width, Cycles(0),
                                            _cpu, params));

    resources.push_back(new InstBuffer("fetch_buffer_t0", FetchBuff, 4,
                                       Cycles(0), _cpu, params));

    resources.push_back(new UseDefUnit("regfile_manager", RegManager,
                                       stage_width * 3, Cycles(0), _cpu,
                                       params));

    resources.push_back(new AGENUnit("agen_unit", AGEN,
                                     stage_width, Cycles(0), _cpu,
                                     params));

    resources.push_back(new ExecutionUnit("execution_unit", ExecUnit,
                                          stage_width, Cycles(0), _cpu,
                                          params));

    resources.push_back(new MultDivUnit("mult_div_unit", MDU,
                                        stage_width * 2, Cycles(0),
                                        _cpu, params));

    // Keep track of the data load/store unit so we can easily provide
    // a pointer to it in the CPU.
    dataUnit = new CacheUnit("dcache_port", DCache,
                             stage_width * 2 + MaxThreads, Cycles(0), _cpu,
                             params);
    resources.push_back(dataUnit);

    gradObjects.push_back(BPred);
    resources.push_back(new GraduationUnit("graduation_unit", Grad,
                                           stage_width, Cycles(0), _cpu,
                                           params));

    resources.push_back(new InstBuffer("fetch_buffer_t1", FetchBuff2, 4,
                                       Cycles(0), _cpu, params));

}

ResourcePool::~ResourcePool()
{
    cout << "Deleting resources ..." << endl;
    
    for (int i=0; i < resources.size(); i++) {
        DPRINTF(Resource, "Deleting resource: %s.\n", resources[i]->name());
        
        delete resources[i];
    }    
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
ResourcePool::print()
{
    for (int i=0; i < resources.size(); i++) {
        DPRINTF(InOrderDynInst, "Res:%i %s\n",
                i, resources[i]->name());
    }

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

unsigned
ResourcePool::getResIdx(const ThePipeline::ResourceId &res_id)
{
    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        if (resources[idx]->getId() == res_id)
            return idx;
    }

    // todo: change return value to int and return a -1 here
    //       maybe even have enumerated type
    //       panic for now...
    panic("Can't find resource idx for: %i\n", res_id);

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

void
ResourcePool::trap(Fault fault, ThreadID tid, DynInstPtr inst)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting Trap to all "
            "resources.\n", tid);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++)
        resources[idx]->trap(fault, tid, inst);
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

//@todo: split this function and call this version schedulePoolEvent
//       and use this scheduleEvent for scheduling a specific event on 
//       a resource
//@todo: For arguments that arent being used in a ResPoolEvent, a dummyParam
//       or some typedef can be used to signify what's important info
//       to the event construction
void
ResourcePool::scheduleEvent(InOrderCPU::CPUEventType e_type, DynInstPtr inst,
                            Cycles delay,  int res_idx, ThreadID tid)
{
    assert(delay >= 0);

    Tick when = cpu->clockEdge(delay);

    switch ((int)e_type)
    {
      case ResourcePool::InstGraduated:
        {
            DPRINTF(Resource, "Scheduling Inst-Graduated Resource Pool "
                    "Event for tick %i.\n", curTick() + delay);
            ResPoolEventPri grad_pri = ResGrad_Pri;
            ResPoolEvent *res_pool_event = 
                new ResPoolEvent(this,
                                 e_type,
                                 inst,
                                 inst->squashingStage,
                                 inst->seqNum,
                                 inst->readTid(),
                                 grad_pri);
            cpu->schedule(res_pool_event, when);
        }
        break;

      case ResourcePool::SquashAll:
        {
            DPRINTF(Resource, "Scheduling Squash Resource Pool Event for "
                    "tick %i.\n", curTick() + delay);
            ResPoolEventPri squash_pri = ResSquash_Pri;
            ResPoolEvent *res_pool_event = 
                new ResPoolEvent(this,
                                 e_type,
                                 inst,
                                 inst->squashingStage,
                                 inst->squashSeqNum,
                                 inst->readTid(),
                                 squash_pri);
            cpu->schedule(res_pool_event, when);
        }
        break;

      case ResourcePool::UpdateAfterContextSwitch:
        {
            DPRINTF(Resource, "Scheduling UpdatePC Resource Pool Event "
                    "for tick %i.\n",
                    curTick() + delay);
            ResPoolEvent *res_pool_event = new ResPoolEvent(this,
                                                            e_type,
                                                            inst,
                                                            inst->squashingStage,
                                                            inst->seqNum,
                                                            inst->readTid());
            cpu->schedule(res_pool_event, when);
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
    DPRINTF(Resource, "[tid:%i] Broadcasting Squash All Event "
            " starting w/stage %i for all instructions above [sn:%i].\n",
             tid, stage_num, done_seq_num);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->squash(inst, stage_num, done_seq_num, tid);
    }
}

void
ResourcePool::squashDueToMemStall(DynInstPtr inst, int stage_num,
                             InstSeqNum done_seq_num, ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting SquashDueToMemStall Event"
            " starting w/stage %i for all instructions above [sn:%i].\n",
            tid, stage_num, done_seq_num);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->squashDueToMemStall(inst, stage_num, done_seq_num, 
                                            tid);
    }
}

void
ResourcePool::activateThread(ThreadID tid)
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
ResourcePool::deactivateThread(ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting Thread Deactivation to all "
            "resources.\n", tid);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->deactivateThread(tid);
    }
}

void
ResourcePool::suspendThread(ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting Thread Suspension to all "
            "resources.\n", tid);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->suspendThread(tid);
    }
}

void
ResourcePool::instGraduated(InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting [sn:%i] graduation to "
            "appropriate resources.\n", tid, seq_num);

    int num_resources = gradObjects.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[gradObjects[idx]]->instGraduated(seq_num, tid);
    }
}

void
ResourcePool::updateAfterContextSwitch(DynInstPtr inst, ThreadID tid)
{
    DPRINTF(Resource, "[tid:%i] Broadcasting Update PC to all resources.\n",
            tid);

    int num_resources = resources.size();

    for (int idx = 0; idx < num_resources; idx++) {
        resources[idx]->updateAfterContextSwitch(inst, tid);
    }
}

ResourcePool::ResPoolEvent::ResPoolEvent(ResourcePool *_resPool,
                                         InOrderCPU::CPUEventType e_type,
                                         DynInstPtr _inst,
                                         int stage_num,
                                         InstSeqNum seq_num,
                                         ThreadID _tid,
                                         ResPoolEventPri res_pri)
    : Event(res_pri), resPool(_resPool),
      eventType(e_type), inst(_inst), seqNum(seq_num),
      stageNum(stage_num), tid(_tid)
{ }


void
ResourcePool::ResPoolEvent::process()
{
    switch ((int)eventType)
    {

      case ResourcePool::InstGraduated:
        resPool->instGraduated(seqNum, tid);
        break;

      case ResourcePool::SquashAll:
        resPool->squashAll(inst, stageNum, seqNum, tid);
        break;

      case ResourcePool::UpdateAfterContextSwitch:
        resPool->updateAfterContextSwitch(inst, tid);
        break;

      default:
        fatal("Unrecognized Event Type");
    }

    resPool->cpu->cpuEventRemoveList.push(this);
}


const char *
ResourcePool::ResPoolEvent::description() const
{
    return "Resource Pool event";
}

/** Schedule resource event, regardless of its current state. */
void
ResourcePool::ResPoolEvent::scheduleEvent(Cycles delay)
{
    InOrderCPU *cpu = resPool->cpu;
    assert(!scheduled() || squashed());
    cpu->reschedule(this, cpu->clockEdge(delay), true);
}

/** Unschedule resource event, regardless of its current state. */
void
ResourcePool::ResPoolEvent::unscheduleEvent()
{
    if (scheduled())
        squash();
}
