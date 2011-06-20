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

#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/reg_dep_map.hh"
#include "debug/RegDepMap.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

RegDepMap::RegDepMap(int size)
{
    regMap.resize(InOrderCPU::NumRegTypes);
    regMap[InOrderCPU::IntType].resize(NumIntRegs);
    regMap[InOrderCPU::FloatType].resize(NumFloatRegs);
    regMap[InOrderCPU::MiscType].resize(NumMiscRegs);
}

RegDepMap::~RegDepMap()
{
    clear();
}

string
RegDepMap::name()
{
    return cpu->name() + ".RegDepMap";
}

std::string RegDepMap::mapNames[InOrderCPU::NumRegTypes] =
{"IntReg", "FloatReg", "MiscReg"};

void
RegDepMap::setCPU(InOrderCPU *_cpu)
{
    cpu = _cpu;

}

void
RegDepMap::clear()
{
    for (int i = 0; i < regMap.size(); i++) {
        for (int j = 0; j < regMap[j].size(); j++)
            regMap[i][j].clear();
        regMap[i].clear();
    }
    regMap.clear();
}

void
RegDepMap::insert(DynInstPtr inst)
{
    int dest_regs = inst->numDestRegs();

    DPRINTF(RegDepMap, "Setting Output Dependencies for [sn:%i] "
            ", %s (dest. regs = %i).\n",
            inst->seqNum,
            inst->instName(),
            dest_regs);

    for (int i = 0; i < dest_regs; i++) {
        InOrderCPU::RegType reg_type;
        TheISA::RegIndex raw_idx = inst->destRegIdx(i);
        TheISA::RegIndex flat_idx = cpu->flattenRegIdx(raw_idx,
                                                       reg_type,
                                                       inst->threadNumber);

        DPRINTF(RegDepMap, "[sn:%i] #%i flattened %i to %i.\n",
                inst->seqNum, i, raw_idx, flat_idx);

        inst->flattenDestReg(i, flat_idx);

        if (flat_idx == TheISA::ZeroReg && reg_type == InOrderCPU::IntType) {
            DPRINTF(RegDepMap, "[sn:%i]: Ignoring Insert-Dependency tracking for "
                    "ISA-ZeroReg (Int. Reg %i).\n", inst->seqNum,
                    flat_idx);
            continue;
        }

        insert(reg_type, flat_idx, inst);
    }
}


void
RegDepMap::insert(uint8_t reg_type, RegIndex idx, DynInstPtr inst)
{
    DPRINTF(RegDepMap, "Inserting [sn:%i] onto %s dep. list for "
            "reg. idx %i.\n", inst->seqNum, mapNames[reg_type],
            idx);

    regMap[reg_type][idx].push_back(inst);

    inst->setRegDepEntry();
}

void
RegDepMap::remove(DynInstPtr inst)
{
    if (inst->isRegDepEntry()) {
        int dest_regs = inst->numDestRegs();

        DPRINTF(RegDepMap, "Removing [sn:%i]'s entries from reg. dep. map. for "
                ", %s (dest. regs = %i).\n",
                inst->seqNum,
                inst->instName(),
                dest_regs);


        for (int i = 0; i < dest_regs; i++) {
            RegIndex flat_idx = inst->flattenedDestRegIdx(i);
            InOrderCPU::RegType reg_type = cpu->getRegType(inst->destRegIdx(i));

            // Merge Dyn Inst & CPU Result Types
            if (flat_idx == TheISA::ZeroReg &&
                reg_type == InOrderCPU::IntType) {
                DPRINTF(RegDepMap, "[sn:%i]: Ignoring Remove-Dependency tracking for "
                        "ISA-ZeroReg (Int. Reg %i).\n", inst->seqNum,
                        flat_idx);
                continue;
            }


            remove(reg_type, flat_idx, inst);
        }

        inst->clearRegDepEntry();
    }
}

void
RegDepMap::remove(uint8_t reg_type, RegIndex idx, DynInstPtr inst)
{
    std::list<DynInstPtr>::iterator list_it = regMap[reg_type][idx].begin();
    std::list<DynInstPtr>::iterator list_end = regMap[reg_type][idx].end();


    while (list_it != list_end) {
        if((*list_it) == inst) {
            DPRINTF(RegDepMap, "Removing [sn:%i] from %s dep. list for "
                    "reg. idx %i.\n", inst->seqNum, mapNames[reg_type],
                    idx);
            regMap[reg_type][idx].erase(list_it);
            return;
        }
        list_it++;
    }
    panic("[sn:%i] Did not find entry for %i, type:%i\n", inst->seqNum, idx, reg_type);
}

void
RegDepMap::removeFront(uint8_t reg_type, RegIndex idx, DynInstPtr inst)
{
   std::list<DynInstPtr>::iterator list_it = regMap[reg_type][idx].begin();

   DPRINTF(RegDepMap, "[tid:%u]: Removing dependency entry on reg. idx "
           "%i for [sn:%i].\n", inst->readTid(), idx, inst->seqNum);

   assert(list_it != regMap[reg_type][idx].end());

   assert(inst == (*list_it));

   regMap[reg_type][idx].erase(list_it);
}

bool
RegDepMap::canRead(uint8_t reg_type, RegIndex idx, DynInstPtr inst)
{
    if (regMap[reg_type][idx].size() == 0)
        return true;

    std::list<DynInstPtr>::iterator list_it = regMap[reg_type][idx].begin();

    if (inst->seqNum <= (*list_it)->seqNum) {
        return true;
    } else {
        DPRINTF(RegDepMap, "[sn:%i] Can't read from RegFile, [sn:%i] has "
                "not written it's value back yet.\n",
                inst->seqNum, (*list_it)->seqNum);
        return false;
    }
}

ThePipeline::DynInstPtr
RegDepMap::canForward(uint8_t reg_type, unsigned reg_idx, DynInstPtr inst)
{
    std::list<DynInstPtr>::iterator list_it = regMap[reg_type][reg_idx].begin();
    std::list<DynInstPtr>::iterator list_end = regMap[reg_type][reg_idx].end();

    DynInstPtr forward_inst = NULL;

    // Look for instruction immediately in front of requestor to supply
    // data
    while (list_it != list_end &&
           (*list_it)->seqNum < inst->seqNum) {
        forward_inst = (*list_it);
        list_it++;
    }

    if (forward_inst) {
        int dest_reg_idx = forward_inst->getDestIdxNum(reg_idx);
        assert(dest_reg_idx != -1);

        DPRINTF(RegDepMap, "[sn:%i] Found potential forwarding value for reg %i "
                " w/ [sn:%i] dest. reg. #%i\n",
                inst->seqNum, reg_idx, forward_inst->seqNum, dest_reg_idx);

        if (forward_inst->isExecuted() &&
            forward_inst->readResultTime(dest_reg_idx) < curTick()) {
            return forward_inst;
        } else {
            if (!forward_inst->isExecuted()) {
                DPRINTF(RegDepMap, "[sn:%i] Can't get value through "
                        "forwarding, [sn:%i] %s has not been executed yet.\n",
                        inst->seqNum, forward_inst->seqNum, forward_inst->instName());
            } else if (forward_inst->readResultTime(dest_reg_idx) >= curTick()) {
                DPRINTF(RegDepMap, "[sn:%i] Can't get value through "
                        "forwarding, [sn:%i] executed on tick:%i.\n",
                        inst->seqNum, forward_inst->seqNum,
                        forward_inst->readResultTime(dest_reg_idx));
            }

            return NULL;
        }
    } else {
        DPRINTF(RegDepMap, "[sn:%i] No instruction found to forward from.\n",
                inst->seqNum);
        return NULL;
    }
}

bool
RegDepMap::canWrite(uint8_t reg_type, RegIndex idx, DynInstPtr inst)
{
    if (regMap[reg_type][idx].size() == 0)
        return true;

    std::list<DynInstPtr>::iterator list_it = regMap[reg_type][idx].begin();

    if (inst->seqNum <= (*list_it)->seqNum) {
        return true;
    } else {
        DPRINTF(RegDepMap, "[sn:%i] Can't write from RegFile: [sn:%i] "
                "has not written it's value back yet.\n", inst->seqNum,
                (*list_it)->seqNum);
    }

    return false;
}

void
RegDepMap::dump()
{
    for (int reg_type = 0; reg_type < InOrderCPU::NumRegTypes; reg_type++) {
        for (int idx=0; idx < regMap.size(); idx++) {
            if (regMap[idx].size() > 0) {
                cprintf("Reg #%i (size:%i): ", idx, regMap[reg_type][idx].size());

                std::list<DynInstPtr>::iterator list_it =
                    regMap[reg_type][idx].begin();
                std::list<DynInstPtr>::iterator list_end =
                    regMap[reg_type][idx].end();

                while (list_it != list_end) {
                    cprintf("[sn:%i] ", (*list_it)->seqNum);
                    list_it++;
                }
                cprintf("\n");
            }
        }
    }    
}
