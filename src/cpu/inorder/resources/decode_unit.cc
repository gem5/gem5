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

#include "config/the_isa.hh"
#include "cpu/inorder/resources/decode_unit.hh"
#include "debug/InOrderDecode.hh"
#include "debug/InOrderStall.hh"
#include "debug/Resource.hh"

using namespace TheISA;
using namespace ThePipeline;
using namespace std;

DecodeUnit::DecodeUnit(std::string res_name, int res_id, int res_width,
                       int res_latency, InOrderCPU *_cpu,
                       ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu)
{
    for (ThreadID tid = 0; tid < MaxThreads; tid++) {
        regDepMap[tid] = &cpu->archRegDepMap[tid];
    }
}

void
DecodeUnit::execute(int slot_num)
{
    ResourceRequest* decode_req = reqs[slot_num];
    DynInstPtr inst = reqs[slot_num]->inst;

    switch (decode_req->cmd)
    {
      case DecodeInst:
        {

            if (inst->fault != NoFault) {
                inst->setBackSked(cpu->faultSked);
                DPRINTF(InOrderDecode,"[tid:%i]: Fault found for instruction [sn:%i]\n",
                        inst->readTid(), inst->seqNum);
            } else {
                assert(!inst->staticInst->isMacroop());
                inst->setBackSked(cpu->createBackEndSked(inst));
                DPRINTF(InOrderDecode,"Decoded instruction [sn:%i]: %s : 0x%x\n",
                        inst->seqNum, inst->instName(),
                        inst->staticInst->machInst);
            }

            if (inst->backSked != NULL) {
                DPRINTF(InOrderDecode,
                    "[tid:%i]: Back End Schedule created for %s  [sn:%i].\n",
                        inst->readTid(), inst->instName(), inst->seqNum);
                decode_req->done();
            } else {
                DPRINTF(Resource,
                    "[tid:%i] Static Inst not available to decode.\n",
                        inst->readTid());
                DPRINTF(Resource,
                    "Unable to create schedule for instruction [sn:%i] \n",
                    inst->seqNum);
                DPRINTF(InOrderStall, "STALL: \n");
                decode_req->done(false);
            }
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}

