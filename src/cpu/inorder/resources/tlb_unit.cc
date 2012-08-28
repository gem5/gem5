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

#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/resources/tlb_unit.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/first_stage.hh"
#include "cpu/inorder/pipeline_traits.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

TLBUnit::TLBUnit(string res_name, int res_id, int res_width,
                 Cycles res_latency, InOrderCPU *_cpu,
                 ThePipeline::Params *params)
: Resource(res_name, res_id, res_width, res_latency, _cpu)
{
    // Hard-Code Selection For Now
    if (res_name == "I-TLB")
        _tlb = params->itb;
    else if (res_name == "D-TLB")
        _tlb = params->dtb;
    else
        fatal("Unrecognized TLB name passed by user");

    for (int i=0; i < MaxThreads; i++) {
        tlbBlocked[i] = false;
    }
}

TheISA::TLB*
TLBUnit::tlb()
{
    return _tlb;

}

void
TLBUnit::init()
{
    resourceEvent = new TLBUnitEvent[width];

    for (int i = 0; i < width; i++) {
        reqs[i] = new TLBUnitRequest(this);
    }

    initSlots();
}

int
TLBUnit::getSlot(DynInstPtr inst)
{
    if (tlbBlocked[inst->threadNumber]) {
        return -1;
    } else {
        return Resource::getSlot(inst);
    }
}

ResourceRequest*
TLBUnit::getRequest(DynInstPtr _inst, int stage_num,
                            int res_idx, int slot_num,
                            unsigned cmd)
{
    TLBUnitRequest *tlb_req = dynamic_cast<TLBUnitRequest*>(reqs[slot_num]);
    tlb_req->setRequest(inst, stage_num, id, slot_num, cmd);
    return ud_req;
}

void
TLBUnit::execute(int slot_idx)
{
    // After this is working, change this to a reinterpret cast
    // for performance considerations
    TLBUnitRequest* tlb_req = dynamic_cast<TLBUnitRequest*>(reqs[slot_idx]);
    assert(tlb_req != 0x0);

    DynInstPtr inst = tlb_req->inst;
    ThreadID tid = inst->readTid();
    InstSeqNum seq_num = inst->seqNum;
    int stage_num = tlb_req->getStageNum();

    tlb_req->fault = NoFault;

    assert(cpu->thread[tid]->getTC() != 0x0);
    assert(cpu->pipelineStage[stage_num] != 0x0);

    switch (tlb_req->cmd)
    {
      case FetchLookup:
        {
            tlb_req->fault =
                _tlb->translateAtomic(tlb_req->memReq,
                                      cpu->thread[tid]->getTC(), TheISA::TLB::Execute);

            if (tlb_req->fault != NoFault) {
                DPRINTF(InOrderTLB, "[tid:%i]: %s encountered while translating "
                        "addr:%08p for [sn:%i].\n", tid, tlb_req->fault->name(),
                        tlb_req->memReq->getVaddr(), seq_num);

                DPRINTF(InOrderTLB, "slot:%i sn:%i schedule event.\n", slot_idx, seq_num);

                cpu->pipelineStage[stage_num]->setResStall(tlb_req, tid);
                tlbBlocked[tid] = true;
                scheduleEvent(slot_idx, 1);

                // @TODO: SHOULDNT BREAK EXECUTION at misspeculated PC Fault
                // Let CPU handle the fault
                cpu->trap(tlb_req->fault, tid);
            } else {
                DPRINTF(InOrderTLB, "[tid:%i]: [sn:%i] virt. addr %08p translated "
                        "to phys. addr:%08p.\n", tid, seq_num,
                        tlb_req->memReq->getVaddr(),
                        tlb_req->memReq->getPaddr());
                tlb_req->done();
            }
        }
        break;

      case DataReadLookup:
      case DataWriteLookup:
        {
            DPRINTF(InOrderTLB, "[tid:%i]: [sn:%i]: Attempting to translate %08p.\n",
                    tid, seq_num, tlb_req->memReq->getVaddr());


            TheISA::TLB::Mode tlb_mode = (tlb_req->cmd == DataReadLookup) ?
                TheISA::TLB::Read : TheISA::TLB::Write;

            tlb_req->fault =
                _tlb->translateAtomic(tlb_req->memReq,
                                      cpu->thread[tid]->getTC(), tlb_mode);

            if (tlb_req->fault != NoFault) {
                DPRINTF(InOrderTLB, "[tid:%i]: %s encountered while translating "
                        "addr:%08p for [sn:%i] %s.\n", tid, tlb_req->fault->name(),
                        tlb_req->memReq->getVaddr(), seq_num, inst->instName());

                if (inst->isDataPrefetch()) {
                    DPRINTF(InOrderTLB, "Ignoring %s fault for data prefetch\n",
                            tlb_req->fault->name());

                    tlb_req->fault = NoFault;

                    tlb_req->done();
                } else {
                    cpu->pipelineStage[stage_num]->setResStall(tlb_req, tid);
                    tlbBlocked[tid] = true;
                    scheduleEvent(slot_idx, 1);

                    // Let CPU handle the fault
                    cpu->trap(tlb_req->fault, tid, inst);
                }
            } else {
                DPRINTF(InOrderTLB, "[tid:%i]: [sn:%i] virt. addr %08p translated "
                        "to phys. addr:%08p.\n", tid, seq_num,
                        tlb_req->memReq->getVaddr(),
                        tlb_req->memReq->getPaddr());
                tlb_req->done();
            }
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}

TLBUnitEvent::TLBUnitEvent()
    : ResourceEvent()
{ }

void
TLBUnitEvent::process()
{
    DynInstPtr inst = resource->reqs[slotIdx]->inst;
    int stage_num = resource->reqs[slotIdx]->getStageNum();
    ThreadID tid = inst->threadNumber;

    DPRINTF(InOrderTLB, "Waking up from TLB Miss caused by [sn:%i].\n",
            inst->seqNum);

    TLBUnit* tlb_res = dynamic_cast<TLBUnit*>(resource);
    assert(tlb_res);

    tlb_res->tlbBlocked[tid] = false;

    tlb_res->cpu->pipelineStage[stage_num]->
        unsetResStall(tlb_res->reqs[slotIdx], tid);
}

void
TLBUnit::squash(DynInstPtr inst, int stage_num,
                   InstSeqNum squash_seq_num, ThreadID tid)
{
    for (int i = 0; i < width; i++) {
        ResReqPtr req_ptr = reqs[i];

         if (req_ptr->valid &&
             req_ptr->getInst()->readTid() == tid &&
             req_ptr->getInst()->seqNum > squash_seq_num) {

             DPRINTF(Resource, "[tid:%i]: Squashing [sn:%i].\n",
                     req_ptr->getInst()->readTid(),
                     req_ptr->getInst()->seqNum);

             req_ptr->setSquashed();

             int req_slot_num = req_ptr->getSlot();

             tlbBlocked[tid] = false;

             int stall_stage = reqs[req_slot_num]->getStageNum();

             cpu->pipelineStage[stall_stage]->
                 unsetResStall(reqs[req_slot_num], tid);

             if (resourceEvent[req_slot_num].scheduled())
                 unscheduleEvent(req_slot_num);

             freeSlot(req_slot_num);
         }
     }
}


