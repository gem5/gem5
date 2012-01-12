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

#include "cpu/inorder/resources/mult_div_unit.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/resource_pool.hh"
#include "cpu/op_class.hh"
#include "debug/InOrderMDU.hh"
#include "debug/Resource.hh"

using namespace std;
using namespace ThePipeline;

MultDivUnit::MultDivUnit(string res_name, int res_id, int res_width,
                         int res_latency, InOrderCPU *_cpu,
                         ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      multRepeatRate(params->multRepeatRate),
      multLatency(params->multLatency),
      div8RepeatRate(params->div8RepeatRate),
      div8Latency(params->div8Latency),
      div16RepeatRate(params->div16RepeatRate),
      div16Latency(params->div16Latency),
      div24RepeatRate(params->div24RepeatRate),
      div24Latency(params->div24Latency),
      div32RepeatRate(params->div32RepeatRate),
      div32Latency(params->div32Latency),
      lastMDUCycle(0), lastOpType(No_OpClass)
{ }

void
MultDivUnit::regStats()
{
    multiplies
        .name(name() + ".multiplies")
        .desc("Number of Multipy Operations Executed");

    divides
        .name(name() + ".divides")
        .desc("Number of Divide Operations Executed");

    Resource::regStats();
}

void
MultDivUnit::init()
{
    // Set Up Resource Events to Appropriate Resource BandWidth
    resourceEvent = new MDUEvent[width];

    for (int i = 0; i < width; i++) {
        reqs[i] = new ResourceRequest(this);
    }

    initSlots();
}

//@TODO: Should we push this behavior into base-class to generically
//       accomodate all multicyle resources?
void
MultDivUnit::requestAgain(DynInstPtr inst, bool &service_request)
{
    ResReqPtr mult_div_req = findRequest(inst);
    assert(mult_div_req);

    service_request = true;

    // Check to see if this instruction is requesting the same command
    // or a different one
    if (mult_div_req->cmd != inst->curSkedEntry->cmd) {
        // If different, then update command in the request
        mult_div_req->cmd = inst->curSkedEntry->cmd;
        DPRINTF(InOrderMDU,
                "[tid:%i]: [sn:%i]: Updating the command for this "
                "instruction\n", inst->readTid(), inst->seqNum);
    } else {
        // If same command, just check to see if access was completed
        // but dont try to re-execute
        DPRINTF(InOrderMDU,
                "[tid:%i]: [sn:%i]: requesting this resource again\n",
                inst->readTid(), inst->seqNum);
    }
}
int
MultDivUnit::getSlot(DynInstPtr inst)
{
    // If MDU already has instruction, return current slot.
    int slot_num = findSlot(inst);

    // If we have this instruction's request already then return
    if (slot_num != -1 &&         
        inst->curSkedEntry->cmd == reqs[slot_num]->cmd)
        return slot_num;
    
    unsigned repeat_rate = 0;

    /** Enforce MDU dependencies after a multiply is seen last */
    if (lastOpType == IntMultOp) {
        repeat_rate = multRepeatRate;
    }

    /** Enforce dependencies after a divide is seen last */
    if (lastOpType == IntDivOp) {
        switch (lastDivSize) {
          case 8:
            repeat_rate = div8RepeatRate;
            break;

          case 16:
            repeat_rate = div16RepeatRate;
            break;

          case 24:
            repeat_rate = div24RepeatRate;
            break;

          case 32:
            repeat_rate = div32RepeatRate;
            break;
        }
    }

    if (lastMDUCycle + repeat_rate > curTick()) {
        DPRINTF(InOrderMDU, "MDU not ready to process another inst. until %i, "
                "denying request.\n", lastMDUCycle + repeat_rate);
        return -1;
    } else {
        int rval = Resource::getSlot(inst);
        DPRINTF(InOrderMDU, "MDU request should pass: %i.\n",
                rval);

        if (rval != -1) {            
            lastMDUCycle = curTick();
            lastOpType = inst->opClass();
            lastInstName = inst->staticInst->getName();
        }
      
        return rval;
    }
}

int
MultDivUnit::getDivOpSize(DynInstPtr inst)
{
    // Get RT Register from instruction (index #1)
    uint32_t div_op = inst->readIntSrc(1);

    if (div_op <= 0xFF) {
        return 8;
    } else if (div_op <= 0xFFFF) {
        return 16;
    } else if (div_op <= 0xFFFFFF) {
        return 24;
    } else {
        return 32;
    }
}

void 
MultDivUnit::execute(int slot_num)
{
    ResourceRequest* mult_div_req = reqs[slot_num];
    DynInstPtr inst = reqs[slot_num]->inst;
    if (inst->fault != NoFault) {
        DPRINTF(InOrderMDU,
                "[tid:%i]: [sn:%i]: Detected %s fault @ %x. Forwarding to "
                "next stage.\n", inst->readTid(), inst->seqNum, inst->fault->name(),
                inst->pcState());
        mult_div_req->done();
        return;
    }

    DPRINTF(InOrderMDU, "Executing [sn:%i] ...\n", slot_num);

    switch (mult_div_req->cmd)
    {
      case StartMultDiv:
        {
            DPRINTF(InOrderMDU, "Start MDU called ...\n");

            OpClass op_class = inst->opClass();
            if (op_class == IntMultOp) {
                scheduleEvent(slot_num, multLatency);
            } else if (op_class == IntDivOp) {
                int op_size = getDivOpSize(inst);

                switch (op_size)
                {
                  case 8:
                    scheduleEvent(slot_num, div8Latency);
                    break;

                  case 16:
                    scheduleEvent(slot_num, div16Latency);
                    break;

                  case 24:
                    scheduleEvent(slot_num, div24Latency);
                    break;

                  case 32:
                    scheduleEvent(slot_num, div32Latency);
                    break;
                }

                lastDivSize = op_size;
            }

            // Allow to pass through to next stage while
            // event processes
            mult_div_req->setProcessing();
            mult_div_req->setCompleted();
        }
        break;
        
      case MultDiv:
        DPRINTF(InOrderMDU, "Execute MDU called ...\n");        
        exeMulDiv(slot_num);        
        mult_div_req->done();
        break;


      case EndMultDiv:
        //@TODO: Why not allow high-latency requests to sleep
        //      within stage until event wakes up????
        //      Seems wasteful to continually check to see if
        //      this is done when we have a event in parallel
        //      counting down the time
        {
            DPRINTF(InOrderMDU, "End MDU called ...\n");    
            if (!mult_div_req->isProcessing()) {
                DPRINTF(InOrderMDU, "Mult/Div finished.\n");                    
                mult_div_req->done();            
            } else {                
                mult_div_req->setCompleted(false);
            }
            
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}

void 
MultDivUnit::exeMulDiv(int slot_num)
{
    ResourceRequest* mult_div_req = reqs[slot_num];
    DynInstPtr inst = reqs[slot_num]->inst;

    inst->fault = inst->execute();

    if (inst->opClass() == IntMultOp) {
        multiplies++;
    } else if (inst->opClass() == IntDivOp) {
        divides++;
    }

    if (inst->fault == NoFault) {
        inst->setExecuted();

        DPRINTF(InOrderMDU, "[tid:%i]: The result of execution is 0x%x.\n",
                inst->readTid(), inst->readIntResult(0));
    } else {
        DPRINTF(InOrderMDU, "[tid:%i]: [sn:%i]: had a %s "
                "fault.\n", inst->readTid(), inst->seqNum, inst->fault->name());
    }    

    mult_div_req->setProcessing(false);
    cpu->wakeCPU();
}

void
MultDivUnit::squash(DynInstPtr inst, int stage_num, InstSeqNum squash_seq_num,
                 ThreadID tid)
{
    for (int i = 0; i < width; i++) {
        ResReqPtr req_ptr = reqs[i];
        DynInstPtr inst = req_ptr->getInst();

        if (req_ptr->valid &&
            inst->readTid() == tid &&
            inst->seqNum > squash_seq_num) {

            DPRINTF(InOrderMDU, "[tid:%i]: Squashing [sn:%i].\n",
                    req_ptr->getInst()->readTid(),
                    req_ptr->getInst()->seqNum);

            req_ptr->setSquashed();

            int req_slot_num = req_ptr->getSlot();

            if (req_ptr->isProcessing())
                DPRINTF(InOrderMDU, "[tid:%i]: Squashed [sn:%i], but "
                        "waiting for MDU operation to complete.\n",
                        req_ptr->getInst()->readTid(),
                        req_ptr->getInst()->seqNum);
            else
                freeSlot(req_slot_num);
        }
    }
}

MDUEvent::MDUEvent()
    : ResourceEvent()
{ }

void
MDUEvent::process()
{
    MultDivUnit* mdu_res = reinterpret_cast<MultDivUnit*>(resource);

    mdu_res->exeMulDiv(slotIdx);

    ResourceRequest* mult_div_req = resource->reqs[slotIdx];

    if (mult_div_req->isSquashed())
        mdu_res->freeSlot(slotIdx);
}


