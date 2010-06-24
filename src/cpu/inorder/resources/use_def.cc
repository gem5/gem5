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
#include "config/the_isa.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resources/use_def.hh"
#include "cpu/inorder/cpu.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

UseDefUnit::UseDefUnit(string res_name, int res_id, int res_width,
                       int res_latency, InOrderCPU *_cpu,
                       ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      maxSeqNum((InstSeqNum)-1)
{
    for (ThreadID tid = 0; tid < ThePipeline::MaxThreads; tid++) {
        nonSpecInstActive[tid] = &cpu->nonSpecInstActive[tid];
        nonSpecSeqNum[tid] = &cpu->nonSpecSeqNum[tid];

        outReadSeqNum[tid] = maxSeqNum;
        outWriteSeqNum[tid] = maxSeqNum;

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

ResReqPtr
UseDefUnit::getRequest(DynInstPtr inst, int stage_num, int res_idx,
                     int slot_num, unsigned cmd)
{
    return new UseDefRequest(this, inst, stage_num, id, slot_num, cmd,
                             inst->resSched.top()->idx);
}


ResReqPtr
UseDefUnit::findRequest(DynInstPtr inst)
{
    map<int, ResReqPtr>::iterator map_it = reqMap.begin();
    map<int, ResReqPtr>::iterator map_end = reqMap.end();

    while (map_it != map_end) {
        UseDefRequest* ud_req = 
            dynamic_cast<UseDefRequest*>((*map_it).second);
        assert(ud_req);

        if (ud_req &&
            ud_req->getInst() == inst &&
            ud_req->cmd == inst->resSched.top()->cmd &&
            ud_req->useDefIdx == inst->resSched.top()->idx) {
            return ud_req;
        }
        map_it++;
    }

    return NULL;
}

void
UseDefUnit::execute(int slot_idx)
{
    // After this is working, change this to a reinterpret cast
    // for performance considerations
    UseDefRequest* ud_req = dynamic_cast<UseDefRequest*>(reqMap[slot_idx]);
    assert(ud_req);

    DynInstPtr inst = ud_req->inst;
    ThreadID tid = inst->readTid();
    int seq_num = inst->seqNum;
    int ud_idx = ud_req->useDefIdx;

    // If there is a non-speculative instruction
    // in the pipeline then stall instructions here
    if (*nonSpecInstActive[tid] == true &&
        seq_num > *nonSpecSeqNum[tid]) {
        DPRINTF(InOrderUseDef, "[tid:%i]: [sn:%i] cannot execute because"
                "there is non-speculative instruction [sn:%i] has not "
                "graduated.\n", tid, seq_num, *nonSpecSeqNum[tid]);
        return;
    } else if (inst->isNonSpeculative()) {
        *nonSpecInstActive[tid] = true;
        *nonSpecSeqNum[tid] = seq_num;
    }

    switch (ud_req->cmd)
    {
      case ReadSrcReg:
        {
            int reg_idx = inst->_srcRegIdx[ud_idx];
            
            DPRINTF(InOrderUseDef, "[tid:%i]: Attempting to read source "
                    "register idx %i (reg #%i).\n",
                    tid, ud_idx, reg_idx);

            // Ask register dependency map if it is OK to read from Arch. 
            // Reg. File
            if (regDepMap[tid]->canRead(reg_idx, inst)) {
                
                uniqueRegMap[reg_idx] = true;

                if (inst->seqNum <= outReadSeqNum[tid]) {
                    if (reg_idx < FP_Base_DepTag) {
                        DPRINTF(InOrderUseDef, "[tid:%i]: Reading Int Reg %i"
                                "from Register File:%i.\n",
                                tid, 
                                reg_idx, 
                                cpu->readIntReg(reg_idx,inst->readTid()));
                        inst->setIntSrc(ud_idx,
                                        cpu->readIntReg(reg_idx,
                                                        inst->readTid()));
                    } else if (reg_idx < Ctrl_Base_DepTag) {
                        reg_idx -= FP_Base_DepTag;
                        DPRINTF(InOrderUseDef, "[tid:%i]: Reading Float Reg %i"
                                "from Register File:%x (%08f).\n",
                                tid,
                                reg_idx,
                                cpu->readFloatRegBits(reg_idx, 
                                                      inst->readTid()),
                                cpu->readFloatReg(reg_idx, 
                                                  inst->readTid()));

                        inst->setFloatSrc(ud_idx,
                                          cpu->readFloatReg(reg_idx, 
                                                            inst->readTid()));
                    } else {
                        reg_idx -= Ctrl_Base_DepTag;
                        DPRINTF(InOrderUseDef, "[tid:%i]: Reading Misc Reg %i "
                                "from Register File:%i.\n",
                                tid, 
                                reg_idx, 
                                cpu->readMiscReg(reg_idx, 
                                                 inst->readTid()));
                        inst->setIntSrc(ud_idx,
                                        cpu->readMiscReg(reg_idx, 
                                                         inst->readTid()));
                    }

                    outReadSeqNum[tid] = maxSeqNum;
                    regFileReads++;
                    ud_req->done();
                } else {
                    DPRINTF(InOrderUseDef, "[tid:%i]: Unable to read because "
                            "of [sn:%i] hasnt read it's registers yet.\n", 
                            tid, outReadSeqNum[tid]);
                    DPRINTF(InOrderStall, "STALL: [tid:%i]: waiting for "
                            "[sn:%i] to write\n",
                            tid, outReadSeqNum[tid]);
                    ud_req->done(false);
                }

            } else {
                // Look for forwarding opportunities
                DynInstPtr forward_inst = regDepMap[tid]->canForward(reg_idx,
                                                                     inst);

                if (forward_inst) {

                    if (inst->seqNum <= outReadSeqNum[tid]) {
                        int dest_reg_idx = 
                            forward_inst->getDestIdxNum(reg_idx);

                        if (reg_idx < FP_Base_DepTag) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%i.\n",
                                    tid, 
                                    forward_inst->readIntResult(dest_reg_idx),
                                    forward_inst->seqNum, 
                                    inst->seqNum, ud_idx);
                            inst->setIntSrc(ud_idx, 
                                            forward_inst->
                                            readIntResult(dest_reg_idx));
                        } else if (reg_idx < Ctrl_Base_DepTag) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%i.\n",
                                    tid, 
                                    forward_inst->readFloatResult(dest_reg_idx),
                                    forward_inst->seqNum, inst->seqNum, ud_idx);
                            inst->setFloatSrc(ud_idx,
                                              forward_inst->
                                              readFloatResult(dest_reg_idx));
                        } else {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Forwarding dest."
                                    " reg value 0x%x from "
                                    "[sn:%i] to [sn:%i] source #%i.\n",
                                    tid, 
                                    forward_inst->readIntResult(dest_reg_idx),
                                    forward_inst->seqNum, 
                                    inst->seqNum, ud_idx);
                            inst->setIntSrc(ud_idx, 
                                            forward_inst->
                                            readIntResult(dest_reg_idx));
                        }

                        outReadSeqNum[tid] = maxSeqNum;
                        regForwards++;
                        ud_req->done();
                    } else {
                        DPRINTF(InOrderUseDef, "[tid:%i]: Unable to read "
                                "because of [sn:%i] hasnt read it's"
                                " registers yet.\n", tid, outReadSeqNum[tid]);
                        DPRINTF(InOrderStall, "STALL: [tid:%i]: waiting for "
                                "[sn:%i] to forward\n",
                                tid, outReadSeqNum[tid]);
                        ud_req->done(false);
                    }
                } else {
                    DPRINTF(InOrderUseDef, "[tid:%i]: Source register idx: %i"
                            "is not ready to read.\n",
                            tid, reg_idx);
                    DPRINTF(InOrderStall, "STALL: [tid:%i]: waiting to read "
                            "register (idx=%i)\n",
                            tid, reg_idx);
                    outReadSeqNum[tid] = inst->seqNum;
                    ud_req->done(false);
                }
            }
        }
        break;

      case WriteDestReg:
        {
            int reg_idx = inst->_destRegIdx[ud_idx];

            if (regDepMap[tid]->canWrite(reg_idx, inst)) {
                DPRINTF(InOrderUseDef, "[tid:%i]: Flattening register idx %i &"
                        "Attempting to write to Register File.\n",
                        tid, reg_idx);
                uniqueRegMap[reg_idx] = true;
                if (inst->seqNum <= outReadSeqNum[tid]) {
                    if (reg_idx < FP_Base_DepTag) {
                        DPRINTF(InOrderUseDef, "[tid:%i]: Writing Int. Result "
                                "0x%x to register idx %i.\n",
                                tid, inst->readIntResult(ud_idx), reg_idx);

                        // Remove Dependencies
                        regDepMap[tid]->removeFront(reg_idx, inst);

                        cpu->setIntReg(reg_idx,
                                       inst->readIntResult(ud_idx),
                                       inst->readTid());
                    } else if(reg_idx < Ctrl_Base_DepTag) {
                        // Remove Dependencies
                        regDepMap[tid]->removeFront(reg_idx, inst);

                        reg_idx -= FP_Base_DepTag;

                        if (inst->resultType(ud_idx) == 
                            InOrderDynInst::Integer) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Writing FP-Bits "
                                    "Result 0x%x (bits:0x%x) to register "
                                    "idx %i.\n",
                                    tid, 
                                    inst->readFloatResult(ud_idx), 
                                    inst->readIntResult(ud_idx), 
                                    reg_idx);

                            // Check for FloatRegBits Here
                            cpu->setFloatRegBits(reg_idx, 
                                             inst->readIntResult(ud_idx),
                                             inst->readTid());
                        } else if (inst->resultType(ud_idx) == 
                                   InOrderDynInst::Float) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Writing Float "
                                    "Result 0x%x (bits:0x%x) to register "
                                    "idx %i.\n",
                                    tid, inst->readFloatResult(ud_idx), 
                                    inst->readIntResult(ud_idx), 
                                    reg_idx);

                            cpu->setFloatReg(reg_idx,
                                             inst->readFloatResult(ud_idx),
                                             inst->readTid());
                        } else if (inst->resultType(ud_idx) == 
                                   InOrderDynInst::Double) {
                            DPRINTF(InOrderUseDef, "[tid:%i]: Writing Double "
                                    "Result 0x%x (bits:0x%x) to register "
                                    "idx %i.\n",
                                    tid, 
                                    inst->readFloatResult(ud_idx), 
                                    inst->readIntResult(ud_idx), 
                                    reg_idx);

                            // Check for FloatRegBits Here
                            cpu->setFloatReg(reg_idx, 
                                             inst->readFloatResult(ud_idx),
                                             inst->readTid());
                        } else {
                            panic("Result Type Not Set For [sn:%i] %s.\n", 
                                  inst->seqNum, inst->instName());
                        }

                    } else {
                        DPRINTF(InOrderUseDef, "[tid:%i]: Writing Misc. 0x%x "
                                "to register idx %i.\n",
                                tid, inst->readIntResult(ud_idx), reg_idx);

                        // Remove Dependencies
                        regDepMap[tid]->removeFront(reg_idx, inst);

                        reg_idx -= Ctrl_Base_DepTag;

                        cpu->setMiscReg(reg_idx,
                                        inst->readIntResult(ud_idx),
                                        inst->readTid());
                    }

                    outWriteSeqNum[tid] = maxSeqNum;
                    regFileWrites++;
                    ud_req->done();
                } else {
                    DPRINTF(InOrderUseDef, "[tid:%i]: Unable to write because "
                            "of [sn:%i] hasnt read it's"
                            " registers yet.\n", tid, outReadSeqNum);
                    DPRINTF(InOrderStall, "STALL: [tid:%i]: waiting for "
                            "[sn:%i] to read\n",
                            tid, outReadSeqNum);
                    ud_req->done(false);
                }
            } else {
                DPRINTF(InOrderUseDef, "[tid:%i]: Dest. register idx: %i is "
                        "not ready to write.\n",
                        tid, reg_idx);
                DPRINTF(InOrderStall, "STALL: [tid:%i]: waiting to write "
                        "register (idx=%i)\n",
                        tid, reg_idx);
                outWriteSeqNum[tid] = inst->seqNum;
                ud_req->done(false);
            }
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }

}

void
UseDefUnit::squash(DynInstPtr inst, int stage_num, InstSeqNum squash_seq_num,
                   ThreadID tid)
{
    DPRINTF(InOrderUseDef, "[tid:%i]: Updating Due To Squash After [sn:%i].\n",
            tid, squash_seq_num);

    std::vector<int> slot_remove_list;

    map<int, ResReqPtr>::iterator map_it = reqMap.begin();
    map<int, ResReqPtr>::iterator map_end = reqMap.end();

    while (map_it != map_end) {
        ResReqPtr req_ptr = (*map_it).second;

        if (req_ptr &&
            req_ptr->getInst()->readTid() == tid &&
            req_ptr->getInst()->seqNum > squash_seq_num) {

            DPRINTF(InOrderUseDef, "[tid:%i]: Squashing [sn:%i].\n",
                    req_ptr->getInst()->readTid(),
                    req_ptr->getInst()->seqNum);

            int req_slot_num = req_ptr->getSlot();

            if (latency > 0) {                
                assert(0);
                
                unscheduleEvent(req_slot_num);
            }
            
            // Mark request for later removal
            cpu->reqRemoveList.push(req_ptr);

            // Mark slot for removal from resource
            slot_remove_list.push_back(req_ptr->getSlot());
        }

        map_it++;
    }

    // Now Delete Slot Entry from Req. Map
    for (int i = 0; i < slot_remove_list.size(); i++) {
        freeSlot(slot_remove_list[i]);
    }

    if (outReadSeqNum[tid] >= squash_seq_num) {
        DPRINTF(InOrderUseDef, "[tid:%i]: Outstanding Read Seq Num Reset.\n", 
                tid);
        outReadSeqNum[tid] = maxSeqNum;
    } else if (outReadSeqNum[tid] != maxSeqNum) {
        DPRINTF(InOrderUseDef, "[tid:%i]: No need to reset Outstanding Read "
                "Seq Num %i\n",
                tid, outReadSeqNum[tid]);
    }

    if (outWriteSeqNum[tid] >= squash_seq_num) {
        DPRINTF(InOrderUseDef, "[tid:%i]: Outstanding Write Seq Num Reset.\n", 
                tid);
        outWriteSeqNum[tid] = maxSeqNum;
    } else if (outWriteSeqNum[tid] != maxSeqNum) {
        DPRINTF(InOrderUseDef, "[tid:%i]: No need to reset Outstanding Write "
                "Seq Num %i\n",
                tid, outWriteSeqNum[tid]);
    }
}

void
UseDefUnit::updateAfterContextSwitch(DynInstPtr inst, ThreadID tid)
{
    uniqueRegsPerSwitch = uniqueRegMap.size();
    uniqueRegMap.clear();    
}
