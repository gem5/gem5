/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
                       Cycles res_latency, InOrderCPU *_cpu,
                       ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu)
{
    for (ThreadID tid = 0; tid < ThePipeline::MaxThreads; tid++) {
        nonSpecInstActive[tid] = &cpu->nonSpecInstActive[tid];
        nonSpecSeqNum[tid] = &cpu->nonSpecSeqNum[tid];
        serializeOnNextInst[tid] =  false;
        serializeAfterSeqNum[tid] = 0;
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

    intRegFileReads
        .name(name() + ".intRegFileReads")
        .desc("Number of Reads from Int. Register File");

    intRegFileWrites
        .name(name() + ".intRegFileWrites")
        .desc("Number of Writes to Int. Register File");

    intRegFileAccs
        .name(name() + ".intRegFileAccesses")
        .desc("Total Accesses (Read+Write) to the Int. Register File");
    intRegFileAccs = intRegFileReads + intRegFileWrites;

    floatRegFileReads
        .name(name() + ".floatRegFileReads")
        .desc("Number of Reads from FP Register File");

    floatRegFileWrites
        .name(name() + ".floatRegFileWrites")
        .desc("Number of Writes to FP Register File");

    floatRegFileAccs
        .name(name() + ".floatRegFileAccesses")
        .desc("Total Accesses (Read+Write) to the FP Register File");
    floatRegFileAccs = floatRegFileReads + floatRegFileWrites;

    //@todo: add miscreg reads/writes
    //       add forwarding by type???

    regForwards
        .name(name() + ".regForwards")
        .desc("Number of Registers Read Through Forwarding Logic");
    
    Resource::regStats();
}

void
UseDefUnit::init()
{
    // Set Up Resource Events to Appropriate Resource BandWidth
    if (latency > Cycles(0)) {
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
    UseDefRequest* ud_req = dynamic_cast<UseDefRequest*>(reqs[slot_idx]);
    DynInstPtr inst = ud_req->inst;
    ThreadID tid = inst->readTid();
    InstSeqNum seq_num = inst->seqNum;
    int ud_idx = ud_req->useDefIdx;

    if (serializeOnNextInst[tid] &&
        seq_num > serializeAfterSeqNum[tid]) {
        inst->setSerializeBefore();
        serializeOnNextInst[tid] = false;
    }

    if ((inst->isIprAccess() || inst->isSerializeBefore()) &&
        cpu->instList[tid].front() != inst) {
        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i] Serialize before instruction encountered."
                " Blocking until pipeline is clear.\n", tid, seq_num);
        ud_req->done(false);
        return;
    } else if (inst->isStoreConditional() || inst->isSerializeAfter()) {
        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i] Serialize after instruction encountered."
                " Blocking until pipeline is clear.\n", tid, seq_num);
        serializeOnNextInst[tid] = true;
        serializeAfterSeqNum[tid] = seq_num;
    }

    if (inst->fault != NoFault) {
        DPRINTF(InOrderUseDef,
                "[tid:%i]: [sn:%i]: Detected %s fault @ %x. Forwarding to "
                "next stage.\n", inst->readTid(), inst->seqNum, inst->fault->name(),
                inst->pcState());
        ud_req->done();
        return;
    }

    // If there is a non-speculative instruction
    // in the pipeline then stall instructions here
    // ---
    if (*nonSpecInstActive[tid] && seq_num > *nonSpecSeqNum[tid]) {
        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i] cannot execute because"
                "there is non-speculative instruction [sn:%i] has not "
                "graduated.\n", tid, seq_num, *nonSpecSeqNum[tid]);
        ud_req->done(false);
        return;
    } else if (inst->isNonSpeculative()) {
        *nonSpecInstActive[tid] = true;
        *nonSpecSeqNum[tid] = seq_num;
    }

    switch (ud_req->cmd)
    {
      case ReadSrcReg:
        {
            RegClass reg_type;
            RegIndex reg_idx = inst->_srcRegIdx[ud_idx];
            RegIndex flat_idx = cpu->flattenRegIdx(reg_idx, reg_type, tid);
            inst->flattenSrcReg(ud_idx, flat_idx);
            
            if (flat_idx == TheISA::ZeroReg && reg_type == IntRegClass) {
                DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Ignoring Reading of ISA-ZeroReg "
                        "(Int. Reg %i).\n", tid, inst->seqNum, flat_idx);
                ud_req->done();
                return;
            } else {
                DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Attempting to read source "
                        "register idx %i (reg #%i, flat#%i).\n",
                        tid, seq_num, ud_idx, reg_idx, flat_idx);
            }

            if (regDepMap[tid]->canRead(reg_type, flat_idx, inst)) {
                switch (reg_type)
                {
                  case IntRegClass:
                    {
                        uniqueIntRegMap[flat_idx] = true;

                        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Reading Int Reg %i"
                                " (%i) from Register File:0x%x.\n",
                                tid, seq_num,
                                reg_idx, flat_idx,
                                cpu->readIntReg(flat_idx,inst->readTid()));
                        inst->setIntSrc(ud_idx,
                                        cpu->readIntReg(flat_idx,
                                                        inst->readTid()));
                        intRegFileReads++;
                    }
                    break;

                  case FloatRegClass:
                    {
                        uniqueFloatRegMap[flat_idx] = true;
                        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Reading Float Reg %i"
                                " (%i) from Register File:%x (%08f).\n",
                                tid, seq_num,
                                reg_idx - FP_Reg_Base, flat_idx,
                                cpu->readFloatRegBits(flat_idx,
                                                      inst->readTid()),
                                cpu->readFloatReg(flat_idx,
                                                  inst->readTid()));

                        inst->setFloatSrc(ud_idx,
                                          cpu->readFloatReg(flat_idx,
                                                            inst->readTid()));
                        inst->setFloatRegBitsSrc(ud_idx,
                                                 cpu->readFloatRegBits(flat_idx,
                                                                       inst->readTid()));
                        floatRegFileReads++;
                    }
                    break;

                  case MiscRegClass:
                    {
                        uniqueMiscRegMap[flat_idx] = true;
                        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Reading Misc Reg %i "
                                " (%i) from Register File:0x%x.\n",
                                tid, seq_num,
                                reg_idx - Misc_Reg_Base, flat_idx,
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

                ud_req->done();
            } else {
                // Look for forwarding opportunities
                DynInstPtr forward_inst = regDepMap[tid]->canForward(reg_type,
                                                                     flat_idx,
                                                                     inst);

                if (forward_inst) {
                    int dest_reg_idx =
                        forward_inst->getDestIdxNum(flat_idx);

                    switch (reg_type)
                    {
                      case IntRegClass:
                        {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg %i (%i), value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%x.\n",
                                    tid, reg_idx, flat_idx,
                                    forward_inst->readIntResult(dest_reg_idx),
                                    forward_inst->seqNum, 
                                    inst->seqNum, ud_idx);
                            inst->setIntSrc(ud_idx, 
                                            forward_inst->
                                            readIntResult(dest_reg_idx));
                        }
                        break;

                      case FloatRegClass:
                        {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg %i (%i) value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%i.\n",
                                    tid, reg_idx - FP_Reg_Base, flat_idx,
                                    forward_inst->readFloatResult(dest_reg_idx),
                                    forward_inst->seqNum, inst->seqNum, ud_idx);
                            inst->setFloatSrc(ud_idx,
                                              forward_inst->
                                              readFloatResult(dest_reg_idx));
                        }
                        break;

                      case MiscRegClass:
                        {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg %i (%i) value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%i.\n",
                                    tid, reg_idx - Misc_Reg_Base, flat_idx,
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
            RegClass reg_type;
            RegIndex reg_idx = inst->_destRegIdx[ud_idx];
            RegIndex flat_idx = cpu->flattenRegIdx(reg_idx, reg_type, tid);

            if (flat_idx == TheISA::ZeroReg && reg_type == IntRegClass) {
                DPRINTF(IntRegs, "[tid:%i]: Ignoring Writing of ISA-ZeroReg "
                        "(Int. Reg %i)\n", tid, flat_idx);
                ud_req->done();
                return;
            }

            if (regDepMap[tid]->canWrite(reg_type, flat_idx, inst)) {
                DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Flattening register idx %i "
                        "(%i) and Attempting to write to Register File.\n",
                        tid, seq_num, reg_idx, flat_idx);

                switch (reg_type)
                {
                  case IntRegClass:
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
                        intRegFileWrites++;
                    }
                    break;

                  case FloatRegClass:
                    {
                        uniqueFloatRegMap[flat_idx] = true;

                        // Remove Reg. Dependecny Block on this Register
                        regDepMap[tid]->removeFront(reg_type, flat_idx, inst);

                        if (inst->resultType(ud_idx) == 
                            InOrderDynInst::FloatBits) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Writing FP-Bits "
                                    "Result %08f (bits:0x%x) to register "
                                    "idx %i (%i).\n",
                                    tid, seq_num,
                                    inst->readFloatResult(ud_idx), 
                                    inst->readFloatBitsResult(ud_idx),
                                    reg_idx - FP_Reg_Base, flat_idx);

                            // Check for FloatRegBits Here
                            cpu->setFloatRegBits(flat_idx,
                                                 inst->readFloatBitsResult(ud_idx),
                                                 inst->readTid());
                        } else if (inst->resultType(ud_idx) == 
                                   InOrderDynInst::Float) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Writing Float "
                                    "Result %08f (bits:0x%x) to register "
                                    "idx %i (%i).\n",
                                    tid, seq_num, inst->readFloatResult(ud_idx),
                                    inst->readIntResult(ud_idx), 
                                    reg_idx - FP_Reg_Base, flat_idx);

                            cpu->setFloatReg(flat_idx,
                                             inst->readFloatResult(ud_idx),
                                             inst->readTid());
                        } else if (inst->resultType(ud_idx) == 
                                   InOrderDynInst::Double) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i]: Writing Double "
                                    "Result %08f (bits:0x%x) to register "
                                    "idx %i (%i).\n",
                                    tid, seq_num,
                                    inst->readFloatResult(ud_idx), 
                                    inst->readIntResult(ud_idx), 
                                    reg_idx - FP_Reg_Base, flat_idx);

                            cpu->setFloatReg(flat_idx,
                                             inst->readFloatResult(ud_idx),
                                             inst->readTid());
                        } else {
                            panic("Result Type Not Set For [sn:%i] %s.\n", 
                                  inst->seqNum, inst->instName());
                        }

                        floatRegFileWrites++;
                    }
                    break;

                  case MiscRegClass:
                    {
                        uniqueMiscRegMap[flat_idx] = true;

                        DPRINTF(InOrderUseDef, "[tid:%i]: Writing Misc. 0x%x "
                                "to register idx %i.\n",
                                tid, inst->readIntResult(ud_idx), reg_idx - Misc_Reg_Base);

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

      case MarkDestRegs:
        {
            regDepMap[tid]->insert(inst);
            ud_req->done();
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
