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

#include <list>
#include <vector>

#include "base/str.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/resource.hh"
#include "cpu/inorder/resource_pool.hh"
#include "debug/ExecFaulting.hh"
#include "debug/RefCount.hh"
#include "debug/ResReqCount.hh"
#include "debug/Resource.hh"

using namespace std;

Resource::Resource(string res_name, int res_id, int res_width,
                   Cycles res_latency, InOrderCPU *_cpu)
    : resName(res_name), id(res_id),
      width(res_width), latency(res_latency), cpu(_cpu),
      resourceEvent(NULL)
{
    reqs.resize(width);

    // Use to deny a instruction a resource.
    deniedReq = new ResourceRequest(this);
    deniedReq->valid = true;
}

Resource::~Resource()
{
    if (resourceEvent) {
        delete [] resourceEvent;
    }

    delete deniedReq;

    for (int i = 0; i < width; i++) {
        delete reqs[i];
    }
}


void
Resource::init()
{
    // If the resource has a zero-cycle (no latency)
    // function, then no reason to have events
    // that will process them for the right tick
    if (latency > Cycles(0))
      resourceEvent = new ResourceEvent[width];


    for (int i = 0; i < width; i++)
      reqs[i] = new ResourceRequest(this);


    initSlots();
}

void
Resource::initSlots()
{
    // Add available slot numbers for resource
    for (int slot_idx = 0; slot_idx < width; slot_idx++) {
        availSlots.push_back(slot_idx);

        if (resourceEvent) {
            resourceEvent[slot_idx].init(this, slot_idx);
        }
    }
}

std::string
Resource::name()
{
    return cpu->name() + "."  + resName;
}

int
Resource::slotsAvail()
{
    return availSlots.size();
}

int
Resource::slotsInUse()
{
    return width - availSlots.size();
}

void
Resource::freeSlot(int slot_idx)
{
    DPRINTF(Resource, "Deallocating [slot:%i].\n",
            slot_idx);

    // Put slot number on this resource's free list
    availSlots.push_back(slot_idx);

    // Invalidate Request & Reset it's flags
    reqs[slot_idx]->clearRequest();
}

int
Resource::findSlot(DynInstPtr inst)
{
    int slot_num = -1;

    for (int i = 0; i < width; i++) {
        if (reqs[i]->valid &&
            reqs[i]->getInst()->seqNum == inst->seqNum) {
            slot_num = reqs[i]->getSlot();
        }
    }
    return slot_num;
}

int
Resource::getSlot(DynInstPtr inst)
{
    int slot_num = -1;

    if (slotsAvail() != 0) {
        slot_num = availSlots[0];

        vector<int>::iterator vect_it = availSlots.begin();

        assert(slot_num == *vect_it);

        availSlots.erase(vect_it);
    }

    return slot_num;
}

ResReqPtr
Resource::request(DynInstPtr inst)
{
    // See if the resource is already serving this instruction.
    // If so, use that request;
    bool try_request = false;
    int slot_num = -1;
    int stage_num;
    ResReqPtr inst_req = findRequest(inst);

    if (inst_req) {
        // If some preprocessing has to be done on instruction
        // that has already requested once, then handle it here.
        // update the 'try_request' variable if we should
        // re-execute the request.
        requestAgain(inst, try_request);

        slot_num = inst_req->getSlot();
        stage_num = inst_req->getStageNum();
    } else {
        // Get new slot # for instruction
        slot_num = getSlot(inst);

        if (slot_num != -1) {
            DPRINTF(Resource, "Allocating [slot:%i] for [tid:%i]: [sn:%i]\n",
                    slot_num, inst->readTid(), inst->seqNum);

            // Get Stage # from Schedule Entry
            stage_num = inst->curSkedEntry->stageNum;
            unsigned cmd = inst->curSkedEntry->cmd;

            // Generate Resource Request
            inst_req = getRequest(inst, stage_num, id, slot_num, cmd);

            if (inst->staticInst) {
                DPRINTF(Resource, "[tid:%i]: [sn:%i] requesting this "
                        "resource.\n",
                        inst->readTid(), inst->seqNum);
            } else {
                DPRINTF(Resource, "[tid:%i]: instruction requesting this "
                        "resource.\n",
                        inst->readTid());
            }

            try_request = true;
        } else {
            DPRINTF(Resource, "No slot available for [tid:%i]: [sn:%i]\n",
                    inst->readTid(), inst->seqNum);
        }

    }

    if (try_request) {
        // Schedule execution of resource
        scheduleExecution(slot_num);
    } else {
        inst_req = deniedReq;
        rejectRequest(inst);
    }

    return inst_req;
}

void
Resource::requestAgain(DynInstPtr inst, bool &do_request)
{
    do_request = true;

    if (inst->staticInst) {
        DPRINTF(Resource, "[tid:%i]: [sn:%i] requesting this resource "
                "again.\n",
                inst->readTid(), inst->seqNum);
    } else {
        DPRINTF(Resource, "[tid:%i]: requesting this resource again.\n",
                inst->readTid());
    }
}

ResReqPtr
Resource::getRequest(DynInstPtr inst, int stage_num, int res_idx,
                     int slot_num, unsigned cmd)
{
    reqs[slot_num]->setRequest(inst, stage_num, id, slot_num, cmd);
    return reqs[slot_num];
}

ResReqPtr
Resource::findRequest(DynInstPtr inst)
{
    for (int i = 0; i < width; i++) {
        if (reqs[i]->valid &&
            reqs[i]->getInst() == inst) {
            return reqs[i];
        }
    }

    return NULL;
}

void
Resource::rejectRequest(DynInstPtr inst)
{
    DPRINTF(RefCount, "[tid:%i]: Unable to grant request for [sn:%i].\n",
            inst->readTid(), inst->seqNum);
}

void
Resource::execute(int slot_idx)
{
    //@todo: have each resource print out command their executing
    DPRINTF(Resource, "[tid:%i]: Executing %s resource.\n",
            reqs[slot_idx]->getTid(), name());
    reqs[slot_idx]->setCompleted(true);
    reqs[slot_idx]->done();
}

void
Resource::deactivateThread(ThreadID tid)
{
    // In the most basic case, deactivation means squashing everything
    // from a particular thread
    DynInstPtr dummy_inst = new InOrderDynInst(cpu, NULL, 0, tid, tid);
    squash(dummy_inst, 0, 0, tid);
}

void
Resource::setupSquash(DynInstPtr inst, int stage_num, ThreadID tid)
{
    // Squash In Pipeline Stage
    cpu->pipelineStage[stage_num]->setupSquash(inst, tid);

    // Schedule Squash Through-out Resource Pool
    cpu->resPool->scheduleEvent(
        (InOrderCPU::CPUEventType)ResourcePool::SquashAll, inst,
        Cycles(0));
}

void
Resource::squash(DynInstPtr inst, int stage_num, InstSeqNum squash_seq_num,
                 ThreadID tid)
{
    //@todo: check squash seq num before squashing. can save time going
    //       through this function.
    for (int i = 0; i < width; i++) {
        ResReqPtr req_ptr = reqs[i];
        DynInstPtr inst = req_ptr->getInst();

        if (req_ptr->valid &&
            inst->readTid() == tid &&
            inst->seqNum > squash_seq_num) {

            DPRINTF(Resource, "[tid:%i]: Squashing [sn:%i].\n",
                    req_ptr->getInst()->readTid(),
                    req_ptr->getInst()->seqNum);

            req_ptr->setSquashed();

            int req_slot_num = req_ptr->getSlot();

            if (latency > Cycles(0)) {
                if (resourceEvent[req_slot_num].scheduled())
                    unscheduleEvent(req_slot_num);
            }

            freeSlot(req_slot_num);
        }
    }
}

void
Resource::squashDueToMemStall(DynInstPtr inst, int stage_num,
                              InstSeqNum squash_seq_num,
                              ThreadID tid)
{
    squash(inst, stage_num, squash_seq_num, tid);    
}

void
Resource::squashThenTrap(int stage_num, DynInstPtr inst)
{
    ThreadID tid = inst->readTid();

    inst->setSquashInfo(stage_num);
    setupSquash(inst, stage_num, tid);

    if (inst->traceData) {
        if (inst->staticInst &&
            inst->fault != NoFault && DTRACE(ExecFaulting)) {
            inst->traceData->setStageCycle(stage_num, curTick());
            inst->traceData->setFetchSeq(inst->seqNum);
            inst->traceData->dump();
        }

        delete inst->traceData;
        inst->traceData = NULL;
    }

    cpu->trapContext(inst->fault, tid, inst);
}

void
Resource::scheduleExecution(int slot_num)
{
    if (latency > Cycles(0)) {
        scheduleEvent(slot_num, latency);
    } else {
        execute(slot_num);
    }
}

void
Resource::scheduleEvent(int slot_idx, Cycles delay)
{
    DPRINTF(Resource, "[tid:%i]: Scheduling event for [sn:%i] on tick %i.\n",
            reqs[slot_idx]->inst->readTid(),
            reqs[slot_idx]->inst->seqNum,
            cpu->clockEdge(delay));
    resourceEvent[slot_idx].scheduleEvent(delay);
}

bool
Resource::scheduleEvent(DynInstPtr inst, Cycles delay)
{
    int slot_idx = findSlot(inst);

    if(slot_idx != -1)
        resourceEvent[slot_idx].scheduleEvent(delay);

    return slot_idx;
}

void
Resource::unscheduleEvent(int slot_idx)
{
    resourceEvent[slot_idx].unscheduleEvent();
}

bool
Resource::unscheduleEvent(DynInstPtr inst)
{
    int slot_idx = findSlot(inst);

    if(slot_idx != -1)
        resourceEvent[slot_idx].unscheduleEvent();

    return slot_idx;
}

int ResourceRequest::resReqID = 0;

int ResourceRequest::maxReqCount = 0;

ResourceRequest::ResourceRequest(Resource *_res)
    : res(_res), inst(NULL), stagePasses(0), valid(false), doneInResource(false),
      completed(false), squashed(false), processing(false),
      memStall(false)
{
}

ResourceRequest::~ResourceRequest()
{
#ifdef DEBUG
        res->cpu->resReqCount--;
        DPRINTF(ResReqCount, "Res. Req %i deleted. resReqCount=%i.\n", reqID, 
                res->cpu->resReqCount);
#endif
        inst = NULL;
}

std::string
ResourceRequest::name()
{
    return csprintf("%s[slot:%i]:", res->name(), slotNum);
}

void
ResourceRequest::setRequest(DynInstPtr _inst, int stage_num,
                            int res_idx, int slot_num, unsigned _cmd)
{
    valid = true;
    inst = _inst;
    stageNum = stage_num;
    resIdx = res_idx;
    slotNum = slot_num;
    cmd = _cmd;
}

void
ResourceRequest::clearRequest()
{
    valid = false;
    inst = NULL;
    stagePasses = 0;
    completed = false;
    doneInResource = false;
    squashed = false;
    memStall = false;
}

void
ResourceRequest::freeSlot()
{
    assert(res);

    // Free Slot So Another Instruction Can Use This Resource
    res->freeSlot(slotNum);
}

void
ResourceRequest::done(bool completed)
{
    DPRINTF(Resource, "done with request from "
            "[sn:%i] [tid:%i].\n",
            inst->seqNum, inst->readTid());

    setCompleted(completed);

    doneInResource = true;
}

ResourceEvent::ResourceEvent()
    : Event((Event::Priority)Resource_Event_Pri)
{ }

ResourceEvent::ResourceEvent(Resource *res, int slot_idx)
  : Event((Event::Priority)Resource_Event_Pri), resource(res),
      slotIdx(slot_idx)
{ }

void
ResourceEvent::init(Resource *res, int slot_idx)
{
    resource = res;
    slotIdx = slot_idx;
}

void
ResourceEvent::process()
{
    resource->execute(slotIdx);
}

const char *
ResourceEvent::description() const
{
    string desc = resource->name() + "-event:slot[" + to_string(slotIdx)
        + "]";

    return desc.c_str();
}

void
ResourceEvent::scheduleEvent(Cycles delay)
{
    assert(!scheduled() || squashed());
    resource->cpu->reschedule(this,
                              resource->cpu->clockEdge(delay), true);
}
