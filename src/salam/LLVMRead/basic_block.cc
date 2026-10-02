/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 */

#include "basic_block.hh"
#include "llvm/IR/CFG.h"
#include "sim/sim_object.hh"

using namespace SALAM;

SALAM::BasicBlock::BasicBlock(uint64_t id, gem5::SimObject *owner, bool dbg)
    : SALAM::Value(id, owner, dbg)
{}

SALAM::BasicBlock::~BasicBlock()
{}

SALAM::BasicBlock::BasicBlock_Debugger::BasicBlock_Debugger()
{}

void
SALAM::BasicBlock::BasicBlock_Debugger::dumper(SALAM::BasicBlock *bb)
{}

void
SALAM::BasicBlock::initialize(llvm::Value *irval, irvmap *vmap,
                              SALAM::valueListTy *valueList)
{
    if (dbg) {
        DPRINTFS(LLVMParse, owner,
                 "Initialize Values - BasicBlock::initialize\n");
    }

    Value::initialize(irval, vmap);
    // Parse irval for BasicBlock params
    llvm::BasicBlock *bb = llvm::dyn_cast<llvm::BasicBlock>(irval);
    assert(bb);

    for (auto it = llvm::pred_begin(bb); it != pred_end(bb); ++it) {
        llvm::BasicBlock *predecessor = *it;
        std::shared_ptr<SALAM::BasicBlock> pred =
            std::dynamic_pointer_cast<SALAM::BasicBlock>(
                vmap->find(predecessor)->second);
        predecessors.push_back(pred);
    }

    if (dbg) {
        DPRINTFS(LLVMParse, owner, "Initialize BasicBlocks\n");
    }

    for (auto inst_iter = bb->begin(); inst_iter != bb->end(); inst_iter++) {
        llvm::Instruction &inst = *inst_iter;
        std::shared_ptr<SALAM::Value> instval = vmap->find(&inst)->second;
        assert(instval);
        std::shared_ptr<SALAM::Instruction> instruct =
            std::dynamic_pointer_cast<SALAM::Instruction>(instval);
        assert(instruct);
        instructions.push_back(instruct);
        instruct->initialize(&inst, vmap, valueList);
        if (dbg) {
            DPRINTFS(LLVMParse, owner,
                     "Instruction (UID: %d) Initialization Complete\n",
                     instruct->getUID());
        }
    }
}
