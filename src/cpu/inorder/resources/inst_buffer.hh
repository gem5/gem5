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

#include <list>
#include <string>
#include <vector>

#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource.hh"

class InstBuffer : public Resource {
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;

  public:
    enum Command {
        InsertInst,
        InsertAddr,
        RemoveInst,
        RemoveAddr,
        ScheduleOrBypass
    };

  public:
    InstBuffer(std::string res_name, int res_id, int res_width,
               int res_latency, InOrderCPU *_cpu, ThePipeline::Params *params);

    void regStats();

    void execute(int slot_num);

    void insert(DynInstPtr inst);

    void remove(DynInstPtr inst);

    void pop(ThreadID tid);

    DynInstPtr top(ThreadID tid);

    void squash(DynInstPtr inst, int stage_num,
                        InstSeqNum squash_seq_num, ThreadID tid);
  protected:
    /** List of instructions this resource is currently
     *  processing.
     */
    std::list<DynInstPtr> instList;

  public:
    /////////////////////////////////////////////////////////////////
    //
    // RESOURCE STATISTICS
    //
    /////////////////////////////////////////////////////////////////
    /** Number of Instruction Requests the Resource Processes */
    Stats::Scalar instsBypassed;

};

#endif //__CPU_INORDER_INST_BUFF_UNIT_HH__
