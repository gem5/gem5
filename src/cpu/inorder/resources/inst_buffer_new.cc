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
#include "arch/isa_traits.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resources/inst_buffer.hh"
#include "cpu/inorder/cpu.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

InstBuffer::InstBuffer(string res_name, int res_id, int res_width,
                 int res_latency, InOrderCPU *_cpu)
    : Resource(res_name, res_id, res_width, res_latency, _cpu)
{ }

ResReqPtr
InstBuffer::getRequest(DynInstPtr inst, int stage_num, int res_idx,
                     int slot_num)
{
    // After this is working, change this to a reinterpret cast
    // for performance considerations
    InstBufferEntry* ib_entry = dynamic_cast<InstBufferEntry*>(inst->resSched.top());
    assert(ib_entry);

    return new InstBufferRequest(this, inst, stage_num, id, slot_num,
                             ib_entry->cmd);
}

void
InstBuffer::execute(int slot_idx)
{
    // After this is working, change this to a reinterpret cast
    // for performance considerations
    InstBufferRequest* ib_req = dynamic_cast<InstBufferRequest*>(reqMap[slot_idx]);
    assert(ib_req);

    DynInstPtr inst = ib_req->inst;
    ThreadID tid = inst->readTid();
    int seq_num = inst->seqNum;
    ib_req->fault = NoFault;

    switch (ib_req->cmd)
    {
      case InsertInst:
        {
            DPRINTF(Resource, "[tid:%i]: Inserting [sn:%i] into buffer.\n",
                tid, seq_num);
            insert(inst);
            ib_req->done();
        }
        break;

      case RemoveInst:
        {
            DPRINTF(Resource, "[tid:%i]: Removing [sn:%i] from buffer.\n",
                    tid, seq_num);
            remove(inst);
            ib_req->done();
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }

    DPRINTF(Resource, "Buffer now contains %i insts.\n", instList.size());
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
InstBuffer::pop()
{ instList.pop_front(); }

ThePipeline::DynInstPtr
InstBuffer::top()
{ return instList.front(); }

void
InstBuffer::squash(InstSeqNum squash_seq_num, ThreadID tid)
{
    list<DynInstPtr>::iterator list_it = instList.begin();
    list<DynInstPtr>::iterator list_end = instList.end();
    queue<list<DynInstPtr>::iterator> remove_list;

    // Collect All Instructions to be Removed in Remove List
    while (list_it != list_end) {
        if((*list_it)->seqNum > squash_seq_num) {
            DPRINTF(Resource, "[tid:%i]: Squashing [sn:%i] in resource.\n",
                    tid, (*list_it)->seqNum);
            (*list_it)->setSquashed();
            remove_list.push(list_it);
        }

        list_it++;
    }

    // Removed Instructions from InstList & Clear Remove List
    while (!remove_list.empty()) {
        instList.erase(remove_list.front());
        remove_list.pop();
    }

    Resource::squash(squash_seq_num, tid);
}
