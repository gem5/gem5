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

#ifndef __CPU_INORDER_INST_BUFF_UNIT_HH__
#define __CPU_INORDER_INST_BUFF_UNIT_HH__

#include <vector>
#include <list>
#include <string>

#include "cpu/inorder/resource.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/cpu.hh"

class InstBuffer : public Resource {
  public:
    typedef InOrderDynInst::DynInstPtr DynInstPtr;

  public:
    enum Command {
        InsertInst,
        InsertAddr,
        RemoveInst,
        RemoveAddr
    };

  public:
    InstBuffer(std::string res_name, int res_id, int res_width,
              int res_latency, InOrderCPU *_cpu);
    virtual ~InstBuffer() {}

    virtual ResourceRequest* getRequest(DynInstPtr _inst, int stage_num,
                                        int res_idx, int slot_num);

    virtual void execute(int slot_num);

    virtual void insert(DynInstPtr inst);

    virtual void remove(DynInstPtr inst);

    virtual void pop();

    virtual DynInstPtr top();

    virtual void squash(InstSeqNum squash_seq_num, unsigned tid);

  protected:
    /** List of instructions this resource is currently
     *  processing.
     */
    std::list<DynInstPtr> instList;

    /** @todo: Add Resource Stats Here */

};

struct InstBufferEntry : public ThePipeline::ScheduleEntry {
    InstBufferEntry(int stage_num, int res_num, InstBuffer::Command _cmd) :
        ScheduleEntry(stage_num, res_num), cmd(_cmd)
    { }

    InstBuffer::Command cmd;
};

class InstBufferRequest : public ResourceRequest {
  public:
    typedef InOrderDynInst::DynInstPtr DynInstPtr;

  public:
    InstBufferRequest(InstBuffer *res, DynInstPtr inst, int stage_num, int res_idx, int slot_num,
                  InstBuffer::Command _cmd)
        : ResourceRequest(res, inst, stage_num, res_idx, slot_num),
          cmd(_cmd)
    { }

    InstBuffer::Command cmd;
};


#endif //__CPU_INORDER_INST_BUFF_UNIT_HH__
