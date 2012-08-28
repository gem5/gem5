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
#include "cpu/inorder/resources/inst_buffer.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "debug/InOrderInstBuffer.hh"
#include "debug/Resource.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

InstBuffer::InstBuffer(string res_name, int res_id, int res_width,
                       Cycles res_latency, InOrderCPU *_cpu,
                       ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu)
{ }

void
InstBuffer::regStats()
{
    instsBypassed
        .name(name() + ".instsBypassed")
        .desc("Number of Instructions Bypassed.")
        .prereq(instsBypassed);    

    Resource::regStats();
}

void
InstBuffer::execute(int slot_idx)
{
    ResReqPtr ib_req = reqs[slot_idx];
    DynInstPtr inst = ib_req->inst;
    ThreadID tid = inst->readTid();
    int stage_num = ib_req->getStageNum();

    switch (ib_req->cmd)
    {
      case ScheduleOrBypass:
        {
            int next_stage = stage_num + 1;
            int bypass_stage = stage_num + 2;
            bool do_bypass = true;

            if (!instList.empty()) {
                DPRINTF(InOrderInstBuffer, "[sn:%i] cannot bypass stage %i "
                        "because buffer isn't empty.\n",
                        inst->seqNum, next_stage);
                do_bypass = false;
            } else if(cpu->pipelineStage[bypass_stage]->isBlocked(tid)) {
                DPRINTF(InOrderInstBuffer, "[sn:%i] cannot bypass stage %i "
                        "because stage %i is blocking.\n",
                        inst->seqNum, next_stage);
                do_bypass = false;
            } else if(cpu->pipelineStage[bypass_stage]->
                      stageBufferAvail() <= 0) {
                DPRINTF(InOrderInstBuffer, "[sn:%i] cannot bypass stage %i "
                        "because there is no room in stage %i incoming stage "
                        "buffer.\n", inst->seqNum, next_stage);
                do_bypass = false;
            }

            if (!do_bypass) { // SCHEDULE USAGE OF BUFFER
                DPRINTF(InOrderInstBuffer, "Scheduling [sn:%i] for buffer "
                        "insertion in stage %i\n",
                        inst->seqNum, next_stage);

                // Add to schedule: Insert into buffer in next stage
                int stage_pri = 20;
                RSkedPtr insert_sked = (stage_num >= ThePipeline::BackEndStartStage) ?
                    inst->backSked : inst->frontSked;

                insert_sked->push(new ScheduleEntry(next_stage,
                                                      stage_pri,
                                                      id,
                                                      InstBuffer::InsertInst));

                // Add to schedule: Remove from buffer in next next (bypass)
                // stage
                stage_pri = 20;
                RSkedPtr bypass_sked = (stage_num >= ThePipeline::BackEndStartStage) ?
                    inst->backSked : inst->frontSked;

               bypass_sked->push(new ScheduleEntry(bypass_stage,
                                                      stage_pri,
                                                      id,
                                                      InstBuffer::RemoveInst));
            } else {         // BYPASS BUFFER & NEXT STAGE
                DPRINTF(InOrderInstBuffer, "Setting [sn:%i] to bypass stage "
                        "%i and enter stage %i.\n", inst->seqNum, next_stage,
                        bypass_stage);
                inst->setNextStage(bypass_stage);
                instsBypassed++;
            }

            ib_req->done();
        }
        break;

      case InsertInst:
        {
            bool inserted = false;

            if (instList.size() < width) {
                DPRINTF(InOrderInstBuffer, "[tid:%i]: Inserting [sn:%i] into "
                        "buffer.\n", tid, inst->seqNum);
                insert(inst);
                inserted = true;
            } else {
                DPRINTF(InOrderInstBuffer, "[tid:%i]: Denying [sn:%i] request "
                        "because buffer is full.\n", tid, inst->seqNum);


                std::list<DynInstPtr>::iterator list_it = instList.begin();
                std::list<DynInstPtr>::iterator list_end = instList.end();

                while (list_it != list_end) {
                    DPRINTF(Resource,"Serving [tid:%i] [sn:%i].\n",
                            (*list_it)->readTid(), (*list_it)->seqNum);
                    list_it++;
                }
            }

            ib_req->done(inserted);
        }
        break;

      case RemoveInst:
        {
            DPRINTF(InOrderInstBuffer, "[tid:%i]: Removing [sn:%i] from "
                    "buffer.\n", tid, inst->seqNum);
            remove(inst);
            ib_req->done();
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }

    DPRINTF(InOrderInstBuffer, "Buffer now contains %i insts.\n",
            instList.size());
}

void
InstBuffer::insert(DynInstPtr inst)
{
    instList.push_back(inst);
}

void
InstBuffer::remove(DynInstPtr inst)
{
    std::list<DynInstPtr>::iterator list_it = instList.begin();
    std::list<DynInstPtr>::iterator list_end = instList.end();

    while (list_it != list_end) {
        if((*list_it) == inst) {
            instList.erase(list_it);
            break;
        }
        list_it++;
    }
}

void
InstBuffer::pop(ThreadID tid)
{
    instList.pop_front();
}

ThePipeline::DynInstPtr
InstBuffer::top(ThreadID tid)
{
    return instList.front();
}

void
InstBuffer::squash(DynInstPtr inst, int stage_num,
                   InstSeqNum squash_seq_num, ThreadID tid)
{
    queue<list<DynInstPtr>::iterator> remove_list;
    list<DynInstPtr>::iterator list_it = instList.begin();
    list<DynInstPtr>::iterator list_end = instList.end();

    // Collect All Instructions to be Removed in Remove List
    while (list_it != list_end) {
        if((*list_it)->readTid() == tid &&
           (*list_it)->seqNum > squash_seq_num) {
            (*list_it)->setSquashed();
            remove_list.push(list_it);
        }

        list_it++;
    }

    // Removed Instructions from InstList & Clear Remove List
    while (!remove_list.empty()) {
        DPRINTF(InOrderInstBuffer, "[tid:%i]: Removing squashed [sn:%i] from "
                "buffer.\n", tid, (*remove_list.front())->seqNum);
        instList.erase(remove_list.front());
        remove_list.pop();
    }

    Resource::squash(inst, stage_num, squash_seq_num, tid);
}
