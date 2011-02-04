/*
 * Copyright (c) 2011 The Regents of The University of Michigan
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

#ifndef __CPU_INORDER_FETCH_UNIT_HH__
#define __CPU_INORDER_FETCH_UNIT_HH__

#include <vector>
#include <list>
#include <string>

#include "arch/predecoder.hh"
#include "arch/tlb.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource.hh"
#include "cpu/inorder/resources/cache_unit.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "params/InOrderCPU.hh"
#include "sim/sim_object.hh"

class FetchUnit : public CacheUnit
{
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;

  public:
    FetchUnit(std::string res_name, int res_id, int res_width,
              int res_latency, InOrderCPU *_cpu, ThePipeline::Params *params);

    /** Actions that this resources can take on an instruction */
    enum Command {
        InitiateFetch,
        CompleteFetch
    };

  public:
    ResourceRequest* getRequest(DynInstPtr _inst, int stage_num,
                                int res_idx, int slot_num,
                                unsigned cmd);

    int getSlot(DynInstPtr inst);

    /** Executes one of the commands from the "Command" enum */
    void execute(int slot_num);

    void squash(DynInstPtr inst, int stage_num,
                InstSeqNum squash_seq_num, ThreadID tid);

    /** After memory request is completed, then turn the fetched data
        into an instruction.
    */
    void processCacheCompletion(PacketPtr pkt);

    /** Create request that will interface w/TLB and Memory objects */
    virtual void setupMemRequest(DynInstPtr inst, CacheReqPtr cache_req,
                                 int acc_size, int flags);

    /** Align a PC to the start of an I-cache block. */
    Addr cacheBlockAlignPC(Addr addr)
    {
        return (addr & ~(cacheBlkMask));
    }

    void removeAddrDependency(DynInstPtr inst);

  public:
    /** The mem line being fetched. */
    uint8_t *fetchData[ThePipeline::MaxThreads];


    /** The Addr of the cacheline that has been loaded. */
    //Addr cacheBlockAddr[ThePipeline::MaxThreads];
    //unsigned fetchOffset[ThePipeline::MaxThreads];
};

#endif //__CPU_FETCH_UNIT_HH__
