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

#include <vector>
#include <list>
#include "cpu/inorder/resource.hh"
#include "cpu/inorder/cpu.hh"
using namespace std;

Resource::Resource(string res_name, int res_id, int res_width,
                   int res_latency, InOrderCPU *_cpu)
    : resName(res_name), id(res_id),
      width(res_width), latency(res_latency), cpu(_cpu)
{
    // Use to deny a instruction a resource.
    deniedReq = new ResourceRequest(this, NULL, 0, 0, 0, 0);
}

Resource::~Resource()
{
    delete [] resourceEvent;
    delete deniedReq;    
}


void
Resource::init()
{
    // Set Up Resource Events to Appropriate Resource BandWidth
    resourceEvent = new ResourceEvent[width];

    initSlots();
}

void
Resource::initSlots()
{
    // Add available slot numbers for resource
    for (int slot_idx = 0; slot_idx < width; slot_idx++) {
        availSlots.push_back(slot_idx);
        resourceEvent[slot_idx].init(this, slot_idx);
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
    // Put slot number on this resource's free list
    availSlots.push_back(slot_idx);

    // Erase Request Pointer From Request Map
    std::map<int, ResReqPtr>::iterator req_it = reqMap.find(slot_idx);

    assert(req_it != reqMap.end());
    reqMap.erase(req_it);

}

// TODO: More efficiently search for instruction's slot within
// resource.
int
Resource::findSlot(DynInstPtr inst)
{
    map<int, ResReqPtr>::iterator map_it = reqMap.begin();
    map<int, ResReqPtr>::iterator map_end = reqMap.end();

    int slot_num = -1;

    while (map_it != map_end) {
        if ((*map_it).second->getInst()->seqNum ==
            inst->seqNum) {
            slot_num = (*map_it).second->getSlot();
        }
        map_it++;
    }

    return slot_num;
}

int
Resource::getSlot(DynInstPtr inst)
{
    int slot_num;

    if (slotsAvail() != 0) {
        slot_num = availSlots[0];

        vector<int>::iterator vect_it = availSlots.begin();

        assert(slot_num == *vect_it);

        availSlots.erase(vect_it);
    } else {
        DPRINTF(Resource, "[tid:%i]: No slots in resource "
                "available to service [sn:%i].\n", inst->readTid(),
                inst->seqNum);
        slot_num = -1;

        map<int, ResReqPtr>::iterator map_it = reqMap.begin();
        map<int, ResReqPtr>::iterator map_end = reqMap.end();

        while (map_it != map_end) {
            if ((*map_it).second) {
                DPRINTF(Resource, "Currently Serving request from: "
                        "[tid:%i] [sn:%i].\n",
                        (*map_it).second->getInst()->readTid(),
                        (*map_it).second->getInst()->seqNum);
            }
            map_it++;
        }
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
            // Get Stage # from Schedule Entry
            stage_num = inst->resSched.top()->stageNum;
            unsigned cmd = inst->resSched.top()->cmd;

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

            reqMap[slot_num] = inst_req;

            try_request = true;
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
    return new ResourceRequest(this, inst, stage_num, id, slot_num,
                               cmd);
}

ResReqPtr
Resource::findRequest(DynInstPtr inst)
{
    map<int, ResReqPtr>::iterator map_it = reqMap.begin();
    map<int, ResReqPtr>::iterator map_end = reqMap.end();

    bool found = false;
    ResReqPtr req = NULL;
    
    while (map_it != map_end) {
        if ((*map_it).second &&
            (*map_it).second->getInst() == inst) {            
            req = (*map_it).second;
            //return (*map_it).second;
            assert(found == false);
            found = true;            
        }
        map_it++;
    }

    return req;    
    //return NULL;
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
    DPRINTF(Resource, "[tid:%i]: Executing %s resource.\n",
            reqMap[slot_idx]->getTid(), name());
    reqMap[slot_idx]->setCompleted(true);
    reqMap[slot_idx]->fault = NoFault;
    reqMap[slot_idx]->done();
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
Resource::squash(DynInstPtr inst, int stage_num, InstSeqNum squash_seq_num,
                 ThreadID tid)
{
    std::vector<int> slot_remove_list;

    map<int, ResReqPtr>::iterator map_it = reqMap.begin();
    map<int, ResReqPtr>::iterator map_end = reqMap.end();

    while (map_it != map_end) {
        ResReqPtr req_ptr = (*map_it).second;

        if (req_ptr &&
            req_ptr->getInst()->readTid() == tid &&
            req_ptr->getInst()->seqNum > squash_seq_num) {

            DPRINTF(Resource, "[tid:%i]: Squashing [sn:%i].\n",
                    req_ptr->getInst()->readTid(),
                    req_ptr->getInst()->seqNum);

            req_ptr->setSquashed();

            int req_slot_num = req_ptr->getSlot();

            if (resourceEvent[req_slot_num].scheduled())
                unscheduleEvent(req_slot_num);

            // Mark request for later removal
            cpu->reqRemoveList.push(req_ptr);

            // Mark slot for removal from resource
            slot_remove_list.push_back(req_ptr->getSlot());
        }

        map_it++;
    }

    // Now Delete Slot Entry from Req. Map
    for (int i = 0; i < slot_remove_list.size(); i++) {
        freeSlot(slot_remove_list[i]);
    }
}

void
Resource::squashDueToMemStall(DynInstPtr inst, int stage_num,
                              InstSeqNum squash_seq_num,
                              ThreadID tid)
{
    squash(inst, stage_num, squash_seq_num, tid);    
}

Tick
Resource::ticks(int num_cycles)
{
    return cpu->ticks(num_cycles);
}


void
Resource::scheduleExecution(int slot_num)
{
    int res_latency = getLatency(slot_num);

    if (res_latency >= 1) {
        scheduleEvent(slot_num, res_latency);
    } else {
        execute(slot_num);
    }
}

void
Resource::scheduleEvent(int slot_idx, int delay)
{
    DPRINTF(Resource, "[tid:%i]: Scheduling event for [sn:%i] on tick %i.\n",
            reqMap[slot_idx]->inst->readTid(),
            reqMap[slot_idx]->inst->seqNum,
            cpu->ticks(delay) + curTick);
    resourceEvent[slot_idx].scheduleEvent(delay);
}

bool
Resource::scheduleEvent(DynInstPtr inst, int delay)
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

ResourceRequest::ResourceRequest(Resource *_res, DynInstPtr _inst, 
                                 int stage_num, int res_idx, int slot_num, 
                                 unsigned _cmd)
    : res(_res), inst(_inst), cmd(_cmd),  stageNum(stage_num),
      resIdx(res_idx), slotNum(slot_num), completed(false),
      squashed(false), processing(false), memStall(false)
{
#ifdef DEBUG
        reqID = resReqID++;
        res->cpu->resReqCount++;
        DPRINTF(ResReqCount, "Res. Req %i created. resReqCount=%i.\n", reqID, 
                res->cpu->resReqCount);

        if (res->cpu->resReqCount > 100) {
            fatal("Too many undeleted resource requests. Memory leak?\n");
        }

        if (res->cpu->resReqCount > maxReqCount) {            
            maxReqCount = res->cpu->resReqCount;
            res->cpu->maxResReqCount = maxReqCount;            
        }
        
#endif

        stagePasses = 0;
        complSlotNum = -1;
        
}

ResourceRequest::~ResourceRequest()
{
#ifdef DEBUG
        res->cpu->resReqCount--;
        DPRINTF(ResReqCount, "Res. Req %i deleted. resReqCount=%i.\n", reqID, 
                res->cpu->resReqCount);
#endif
}

void
ResourceRequest::done(bool completed)
{
    DPRINTF(Resource, "%s [slot:%i] done with request from "
            "[sn:%i] [tid:%i].\n", res->name(), slotNum,
            inst->seqNum, inst->readTid());

    setCompleted(completed);

    // Used for debugging purposes
    if (completed) {
        complSlotNum = slotNum;
    
        // Would like to start a convention such as all requests deleted in
        // resources/pipeline
        // but a little more complex then it seems...
        // For now, all COMPLETED requests deleted in resource..
        //          all FAILED requests deleted in pipeline stage
        //          *all SQUASHED requests deleted in resource
        res->cpu->reqRemoveList.push(res->reqMap[slotNum]);
    }
    
    // Free Slot So Another Instruction Can Use This Resource
    res->freeSlot(slotNum);

    // change slot # to -1, since we check slotNum to see if request is
    // still valid
    slotNum = -1;
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
ResourceEvent::description()
{
    string desc = resource->name() + " event";

    return desc.c_str();
}
