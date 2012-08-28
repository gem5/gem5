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

#ifndef __CPU_INORDER_USE_DEF_UNIT_HH__
#define __CPU_INORDER_USE_DEF_UNIT_HH__

#include <list>
#include <string>
#include <vector>

#include "cpu/inorder/first_stage.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/reg_dep_map.hh"
#include "cpu/inorder/resource.hh"
#include "cpu/func_unit.hh"

class UseDefUnit : public Resource {
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;
    typedef TheISA::RegIndex RegIndex;

    enum Command {
        ReadSrcReg,
        WriteDestReg,
        MarkDestRegs
    };

  public:
    UseDefUnit(std::string res_name, int res_id, int res_width,
               Cycles res_latency, InOrderCPU *_cpu,
               ThePipeline::Params *params);

    void init();

    ResourceRequest* getRequest(DynInstPtr _inst, int stage_num,
                                        int res_idx, int slot_num,
                                        unsigned cmd);

    ResReqPtr findRequest(DynInstPtr inst);

    void execute(int slot_num);

    void updateAfterContextSwitch(DynInstPtr inst, ThreadID tid);    

    void regStats();
    
  protected:
    RegDepMap *regDepMap[ThePipeline::MaxThreads];

    bool *nonSpecInstActive[ThePipeline::MaxThreads];
    InstSeqNum *nonSpecSeqNum[ThePipeline::MaxThreads];

    bool serializeOnNextInst[ThePipeline::MaxThreads];
    InstSeqNum serializeAfterSeqNum[ThePipeline::MaxThreads];

    Stats::Average uniqueRegsPerSwitch;
    std::map<RegIndex, bool> uniqueIntRegMap;
    std::map<RegIndex, bool> uniqueFloatRegMap;
    std::map<RegIndex, bool> uniqueMiscRegMap;

  public:
    class UseDefRequest : public ResourceRequest {
      public:
        typedef ThePipeline::DynInstPtr DynInstPtr;

      public:
        UseDefRequest(UseDefUnit *res)
            : ResourceRequest(res)
        { }

        int useDefIdx;

        void setRequest(DynInstPtr _inst, int stage_num, int res_idx,
                        int slot_num, unsigned _cmd, int idx)
        {
            useDefIdx = idx;

            ResourceRequest::setRequest(_inst, stage_num, res_idx, slot_num,
                                        _cmd);
        }
    };

  protected:
    /** Int. Register File Reads */
    Stats::Scalar intRegFileReads;

    /** Int. Register File Writes */
    Stats::Scalar intRegFileWrites;

    /** Int. Register File Total Accesses (Read+Write) */
    Stats::Formula intRegFileAccs;

    /** Float Register File Reads */
    Stats::Scalar floatRegFileReads;

    /** Float Register File Writes */
    Stats::Scalar floatRegFileWrites;

    /** Float Register File Total Accesses (Read+Write) */
    Stats::Formula floatRegFileAccs;

    /** Source Register Forwarding */
    Stats::Scalar regForwards;
};

#endif //__CPU_INORDER_USE_DEF_UNIT_HH__
