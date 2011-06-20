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
#include "cpu/inorder/resources/use_def.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "debug/InOrderStall.hh"
#include "debug/InOrderUseDef.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

UseDefUnit::UseDefUnit(string res_name, int res_id, int res_width,
                       int res_latency, InOrderCPU *_cpu,
                       ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu)
{
    for (ThreadID tid = 0; tid < ThePipeline::MaxThreads; tid++) {
        nonSpecInstActive[tid] = &cpu->nonSpecInstActive[tid];
        nonSpecSeqNum[tid] = &cpu->nonSpecSeqNum[tid];

        regDepMap[tid] = &cpu->archRegDepMap[tid];
    }

}

void
UseDefUnit::regStats()
{
    uniqueRegsPerSwitch
        .name(name() + ".uniqueRegsPerSwitch")
        .desc("Number of Unique Registers Needed Per Context Switch")
        .prereq(uniqueRegsPerSwitch);

    regFileReads
        .name(name() + ".regFileReads")
        .desc("Number of Reads from Register File");

    regForwards
        .name(name() + ".regForwards")
        .desc("Number of Registers Read Through Forwarding Logic");

    regFileWrites
        .name(name() + ".regFileWrites")
        .desc("Number of Writes to Register File");

    regFileAccs
        .name(name() + ".regFileAccesses")
        .desc("Number of Total Accesses (Read+Write) to the Register File");
    regFileAccs = regFileReads + regFileWrites;
    
    Resource::regStats();
}

void
UseDefUnit::init()
{
    // Set Up Resource Events to Appropriate Resource BandWidth
    if (latency > 0) {
        resourceEvent = new ResourceEvent[width];
    } else {
        resourceEvent = NULL;
    }

    for (int i = 0; i < width; i++) {
        reqs[i] = new UseDefRequest(this);
    }

    initSlots();
}

ResReqPtr
UseDefUnit::getRequest(DynInstPtr inst, int stage_num, int res_idx,
                     int slot_num, unsigned cmd)
{
    UseDefRequest *ud_req = dynamic_cast<UseDefRequest*>(reqs[slot_num]);
    ud_req->setRequest(inst, stage_num, id, slot_num, cmd,
                       inst->curSkedEntry->idx);
    return ud_req;
}


ResReqPtr
UseDefUnit::findRequest(DynInstPtr inst)
{
    for (int i = 0; i < width; i++) {
        UseDefRequest* ud_req =
            dynamic_cast<UseDefRequest*>(reqs[i]);
        assert(ud_req);

        if (ud_req->valid &&
            ud_req->getInst() == inst &&
            ud_req->cmd == inst->curSkedEntry->cmd &&
            ud_req->useDefIdx == inst->curSkedEntry->idx) {
            return ud_req;
        }
    }

    return NULL;
}

void
UseDefUnit::execute(int slot_idx)
{
    // After this is working, change this to a reinterpret cast
    // for performance considerations
    UseDefRequest* ud_req = dynamic_cast<UseDefRequest*>(reqs[slot_idx]);
    assert(ud_req);

    DynInstPtr inst = ud_req->inst;
    ThreadID tid = inst->readTid();
    InstSeqNum seq_num = inst->seqNum;
    int ud_idx = ud_req->useDefIdx;

    // If there is a non-speculative instruction
    // in the pipeline then stall instructions here
    if (*nonSpecInstActive[tid] == true && seq_num > *nonSpecSeqNum[tid]) {
        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i] cannot execute because"
                "there is non-speculative instruction [sn:%i] has not "
                "graduated.\n", tid, seq_num, *nonSpecSeqNum[tid]);
        ud_req->done(false);
        return;
    } else if (inst->isNonSpeculative()) {
        *nonSpecInstActive[tid] = true;
        *nonSpecSeqNum[tid] = seq_num;
    }

    //@todo: may want to make a separate schedule entry for setting
    //       destination register dependencies
    if (!inst->isRegDepEntry()) {
        regDepMap[tid]->insert(inst);
    }

    switch (ud_req->cmd)
    {
      case ReadSrcReg:
        {
            InOrderCPU::RegType reg_type;
            RegIndex reg_idx = inst->_srcRegIdx[ud_idx];
            RegIndex flat_idx = cpu->flattenRegIdx(reg_idx, reg_type, tid);
            
            DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Attempting to read source "
                    "register idx %i (reg #%i, flat#%i).\n",
                    tid, seq_num, ud_idx, reg_idx, flat_idx);

            if (regDepMap[tid]->canRead(reg_type, flat_idx, inst)) {
                switch (reg_type)
                {
                  case InOrderCPU::IntType:
                    {
                        uniqueIntRegMap[flat_idx] = true;

                        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Reading Int Reg %i"
                                " (%i) from Register File:%i.\n",
                                tid, seq_num,
                                reg_idx, flat_idx,
                                cpu->readIntReg(flat_idx,inst->readTid()));
                        inst->setIntSrc(ud_idx,
                                        cpu->readIntReg(flat_idx,
                                                        inst->readTid()));
                    }
                    break;

                  case InOrderCPU::FloatType:
                    {
                        uniqueFloatRegMap[flat_idx] = true;
                        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Reading Float Reg %i"
                                " (%i) from Register File:%x (%08f).\n",
                                tid, seq_num,
                                reg_idx - FP_Base_DepTag, flat_idx,
                                cpu->readFloatRegBits(flat_idx,
                                                      inst->readTid()),
                                cpu->readFloatReg(flat_idx,
                                                  inst->readTid()));

                        inst->setFloatSrc(ud_idx,
                                          cpu->readFloatReg(flat_idx,
                                                            inst->readTid()));
                    }
                    break;

                  case InOrderCPU::MiscType:
                    {
                        uniqueMiscRegMap[flat_idx] = true;
                        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Reading Misc Reg %i "
                                " (%i) from Register File:%i.\n",
                                tid, seq_num,
                                reg_idx - Ctrl_Base_DepTag, flat_idx,
                                cpu->readMiscReg(flat_idx,
                                                 inst->readTid()));
                        inst->setIntSrc(ud_idx,
                                        cpu->readMiscReg(flat_idx,
                                                         inst->readTid()));
                    }
                    break;

                  default:
                    panic("Invalid Register Type: %i", reg_type);
                }

                regFileReads++;
                ud_req->done();
            } else {
                // Look for forwarding opportunities
                DynInstPtr forward_inst = regDepMap[tid]->canForward(reg_type,
                                                                     flat_idx,
                                                                     inst,
                                                                     reg_idx);

                if (forward_inst) {
                    int dest_reg_idx =
                        forward_inst->getDestIdxNum(reg_idx);

                    switch (reg_type)
                    {
                      case InOrderCPU::IntType:
                        {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg %i (%i), value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%i.\n",
                                    tid, reg_idx, flat_idx,
                                    forward_inst->readIntResult(dest_reg_idx),
                                    forward_inst->seqNum, 
                                    inst->seqNum, ud_idx);
                            inst->setIntSrc(ud_idx, 
                                            forward_inst->
                                            readIntResult(dest_reg_idx));
                        }
                        break;

                      case InOrderCPU::FloatType:
                        {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg %i (%i) value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%i.\n",
                                    tid, reg_idx - FP_Base_DepTag, flat_idx,
                                    forward_inst->readFloatResult(dest_reg_idx),
                                    forward_inst->seqNum, inst->seqNum, ud_idx);
                            inst->setFloatSrc(ud_idx,
                                              forward_inst->
                                              readFloatResult(dest_reg_idx));
                        }
                        break;

                      case InOrderCPU::MiscType:
                        {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg %i (%i) value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%i.\n",
                                    tid, reg_idx - Ctrl_Base_DepTag, flat_idx,
                                    forward_inst->readIntResult(dest_reg_idx),
                                    forward_inst->seqNum, 
                                    inst->seqNum, ud_idx);
                            inst->setIntSrc(ud_idx, 
                                            forward_inst->
                                            readIntResult(dest_reg_idx));
                        }
                        break;

                      default:
                        panic("Invalid Register Type: %i", reg_type);
                    }

                    regForwards++;
                    ud_req->done();
                } else {
                    DPRINTF(InOrderUseDef, "[tid:%i]: Source register idx: %i "
                            "is not ready to read.\n",
                            tid, reg_idx);
                    DPRINTF(InOrderStall, "STALL: [tid:%i]: waiting to read "
                            "register (idx=%i)\n",
                            tid, reg_idx);
                    ud_req->done(false);
                }
            }
        }
        break;

      case WriteDestReg:
        {
            InOrderCPU::RegType reg_type;
            RegIndex reg_idx = inst->_destRegIdx[ud_idx];
            RegIndex flat_idx = cpu->flattenRegIdx(reg_idx, reg_type, tid);

            if (regDepMap[tid]->canWrite(reg_type, flat_idx, inst)) {
                DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Flattening register idx %i "
                        "(%i) and Attempting to write to Register File.\n",
                        tid, seq_num, reg_idx, flat_idx);

                switch (reg_type)
                {
                  case InOrderCPU::IntType:
                    {
                        uniqueIntRegMap[flat_idx] = true;

                        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Writing Int. Result "
                                "0x%x to register idx %i (%i).\n",
                                tid, seq_num, inst->readIntResult(ud_idx),
                                reg_idx, flat_idx);

                        // Remove Dependencies
                        regDepMap[tid]->removeFront(reg_type, flat_idx, inst);

                        cpu->setIntReg(flat_idx,
                                       inst->readIntResult(ud_idx),
                                       inst->readTid());
                    }
                    break;

                  case InOrderCPU::FloatType:
                    {
                        uniqueFloatRegMap[flat_idx] = true;

                        // Remove Reg. Dependecny Block on this Register
                        regDepMap[tid]->removeFront(reg_type, flat_idx, inst);

                        if (inst->resultType(ud_idx) == 
                            InOrderDynInst::Integer) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Writing FP-Bits "
                                    "Result 0x%x (bits:0x%x) to register "
                                    "idx %i (%i).\n",
                                    tid, seq_num,
                                    inst->readFloatResult(ud_idx), 
                                    inst->readIntResult(ud_idx), 
                                    reg_idx - FP_Base_DepTag, flat_idx);

                            // Check for FloatRegBits Here
                            cpu->setFloatRegBits(flat_idx,
                                                 inst->readIntResult(ud_idx),
                                                 inst->readTid());
                        } else if (inst->resultType(ud_idx) == 
                                   InOrderDynInst::Float) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Writing Float "
                                    "Result 0x%x (bits:0x%x) to register "
                                    "idx %i (%i).\n",
                                    tid, seq_num, inst->readFloatResult(ud_idx),
                                    inst->readIntResult(ud_idx), 
                                    reg_idx - FP_Base_DepTag, flat_idx);

                            cpu->setFloatReg(flat_idx,
                                             inst->readFloatResult(ud_idx),
                                             inst->readTid());
                        } else if (inst->resultType(ud_idx) == 
                                   InOrderDynInst::Double) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Writing Double "
                                    "Result 0x%x (bits:0x%x) to register "
                                    "idx %i (%i).\n",
                                    tid, seq_num,
                                    inst->readFloatResult(ud_idx), 
                                    inst->readIntResult(ud_idx), 
                                    reg_idx - FP_Base_DepTag, flat_idx);

                            // Check for FloatRegBits Here
                            cpu->setFloatReg(flat_idx,
                                             inst->readFloatResult(ud_idx),
                                             inst->readTid());
                        } else {
                            panic("Result Type Not Set For [sn:%i] %s.\n", 
                                  inst->seqNum, inst->instName());
                        }

                    }
                    break;

                  case InOrderCPU::MiscType:
                    {
                        uniqueMiscRegMap[flat_idx] = true;

                        DPRINTF(InOrderUseDef, "[tid:%i]: Writing Misc. 0x%x "
                                "to register idx %i.\n",
                                tid, inst->readIntResult(ud_idx), reg_idx - Ctrl_Base_DepTag);

                        // Remove Dependencies
                        regDepMap[tid]->removeFront(reg_type, flat_idx, inst);

                        cpu->setMiscReg(flat_idx,
                                    inst->readIntResult(ud_idx),
                                        inst->readTid());
                    }
                    break;

                  default:
                    panic("Invalid Register Type: %i", reg_type);
                }

                regFileWrites++;
                ud_req->done();
            } else {
                DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Dest. register idx: %i is "
                        "not ready to write.\n",
                        tid, seq_num, reg_idx);
                DPRINTF(InOrderStall, "STALL: [tid:%i]: waiting to write "
                        "register (idx=%i)\n",
                        tid, reg_idx);
                ud_req->done(false);
            }
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }

}

void
UseDefUnit::updateAfterContextSwitch(DynInstPtr inst, ThreadID tid)
{
    uniqueRegsPerSwitch = uniqueIntRegMap.size() + uniqueFloatRegMap.size()
        + uniqueMiscRegMap.size();
    uniqueIntRegMap.clear();
    uniqueFloatRegMap.clear();
    uniqueMiscRegMap.clear();
}
