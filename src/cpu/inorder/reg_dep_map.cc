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
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/reg_dep_map.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/cpu.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

RegDepMap::RegDepMap(int size)
{
    regMap.resize(size);
}

RegDepMap::~RegDepMap()
{
    for (int i = 0; i < regMap.size(); i++) {
        regMap[i].clear();
    }
    regMap.clear();
}

string
RegDepMap::name()
{
    return cpu->name() + ".RegDepMap";
}

void
RegDepMap::setCPU(InOrderCPU *_cpu)
{
    cpu = _cpu;
}

void
RegDepMap::clear()
{
    regMap.clear();
}

void
RegDepMap::insert(DynInstPtr inst)
{
    int dest_regs = inst->numDestRegs();

    DPRINTF(RegDepMap, "Setting Output Dependencies for [sn:%i] "
            ", %s (dest. regs = %i).\n",
            inst->seqNum,
            inst->staticInst->getName(),
            dest_regs);

    for (int i = 0; i < dest_regs; i++) {
        int idx = inst->destRegIdx(i);

        //if (inst->numFPDestRegs())
        //  idx += TheISA::FP_Base_DepTag;

        insert(idx, inst);
    }
}


void
RegDepMap::insert(unsigned idx, DynInstPtr inst)
{
    DPRINTF(RegDepMap, "Inserting [sn:%i] onto dep. list for reg. idx %i.\n",
            inst->seqNum, idx);

    regMap[idx].push_back(inst);

    inst->setRegDepEntry();
}

void
RegDepMap::remove(DynInstPtr inst)
{
    if (inst->isRegDepEntry()) {
        DPRINTF(RegDepMap, "Removing [sn:%i]'s entries from reg. dep. map.\n",
                inst->seqNum);

        int dest_regs = inst->numDestRegs();

        for (int i = 0; i < dest_regs; i++) {
            int idx = inst->destRegIdx(i);
            remove(idx, inst);
        }
    }
}

void
RegDepMap::remove(unsigned idx, DynInstPtr inst)
{
    std::list<DynInstPtr>::iterator list_it = regMap[idx].begin();
    std::list<DynInstPtr>::iterator list_end = regMap[idx].end();

    while (list_it != list_end) {
        if((*list_it) == inst) {
            regMap[idx].erase(list_it);
            break;
        }

        list_it++;
    }
}

void
RegDepMap::removeFront(unsigned idx, DynInstPtr inst)
{
   std::list<DynInstPtr>::iterator list_it = regMap[idx].begin();

   DPRINTF(RegDepMap, "[tid:%u]: Removing dependency entry on phys. reg."
           "%i for [sn:%i].\n", inst->readTid(), idx, inst->seqNum);

   assert(list_it != regMap[idx].end());

   assert(inst == (*list_it));

   regMap[idx].erase(list_it);
}

bool
RegDepMap::canRead(unsigned idx, DynInstPtr inst)
{
    if (regMap[idx].size() == 0)
        return true;

    std::list<DynInstPtr>::iterator list_it = regMap[idx].begin();

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
RegDepMap::canForward(unsigned reg_idx, DynInstPtr inst)
{
    std::list<DynInstPtr>::iterator list_it = regMap[reg_idx].begin();
    std::list<DynInstPtr>::iterator list_end = regMap[reg_idx].end();

    DynInstPtr forward_inst = NULL;

    // Look for first, oldest instruction
    while (list_it != list_end &&
           (*list_it)->seqNum < inst->seqNum) {
        forward_inst = (*list_it);
        list_it++;
    }

    if (forward_inst) {
        int dest_reg_idx = forward_inst->getDestIdxNum(reg_idx);
        assert(dest_reg_idx != -1);

        if (forward_inst->isExecuted() &&
            forward_inst->readResultTime(dest_reg_idx) < curTick()) {
            return forward_inst;
        } else {
            if (!forward_inst->isExecuted()) {
                DPRINTF(RegDepMap, "[sn:%i] Can't get value through "
                        "forwarding, [sn:%i] has not been executed yet.\n",
                        inst->seqNum, forward_inst->seqNum);
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
RegDepMap::canWrite(unsigned idx, DynInstPtr inst)
{
    if (regMap[idx].size() == 0)
        return true;

    std::list<DynInstPtr>::iterator list_it = regMap[idx].begin();

    if (inst->seqNum <= (*list_it)->seqNum) {
        return true;
    } else {
        DPRINTF(RegDepMap, "[sn:%i] Can't write from RegFile: [sn:%i] "
                "has not written it's value back yet.\n", inst->seqNum,
                (*list_it)->seqNum);
    }

    return false;
}

int
RegDepMap::depSize(unsigned idx)
{
    return regMap[idx].size();
}

ThePipeline::DynInstPtr
RegDepMap::findBypassInst(unsigned idx)
{
    std::list<DynInstPtr>::iterator list_it = regMap[idx].begin();

    if (depSize(idx) == 1)
        return NULL;

    list_it++;

    while (list_it != regMap[idx].end()) {
        if((*list_it)->isExecuted()) {
            return *list_it;
            break;
        }
    }

    return NULL;
}

void
RegDepMap::dump()
{
    
    for (int idx=0; idx < regMap.size(); idx++) {
        
        if (regMap[idx].size() > 0) {
            cprintf("Reg #%i (size:%i): ", idx, regMap[idx].size());

            std::list<DynInstPtr>::iterator list_it = regMap[idx].begin();
            std::list<DynInstPtr>::iterator list_end = regMap[idx].end();
        
            while (list_it != list_end) {
                cprintf("[sn:%i] ", (*list_it)->seqNum);

                list_it++;            
            }        

            cprintf("\n");
        }
        
    }    
}
