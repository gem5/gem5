/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Reinhardt
 */

#include "gpu-compute/kernel_cfg.hh"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

#include "gpu-compute/gpu_static_inst.hh"

void
ControlFlowInfo::assignImmediatePostDominators(
        const std::vector<GPUStaticInst*>& instructions)
{
    ControlFlowInfo cfg(instructions);
    cfg.findImmediatePostDominators();
}


ControlFlowInfo::ControlFlowInfo(const std::vector<GPUStaticInst*>& insts) :
        instructions(insts)
{
    createBasicBlocks();
    connectBasicBlocks();
}

BasicBlock*
ControlFlowInfo::basicBlock(int inst_num) const {
    for (auto& block: basicBlocks) {
       int first_block_id = block->firstInstruction->instNum();
       if (inst_num >= first_block_id &&
               inst_num < first_block_id + block->size) {
           return block.get();
       }
    }
    return nullptr;
}


GPUStaticInst*
ControlFlowInfo::lastInstruction(const BasicBlock* block) const
{
    if (block->isExit()) {
        return nullptr;
    }

    return instructions.at(block->firstInstruction->instNum() +
                           block->size - 1);
}

BasicBlock*
ControlFlowInfo::postDominator(const BasicBlock* block) const
{
    if (block->isExit()) {
        return nullptr;
    }
    return basicBlock(lastInstruction(block)->ipdInstNum());
}

void
ControlFlowInfo::createBasicBlocks()
{
    assert(!instructions.empty());
    std::set<int> leaders;
    // first instruction is a leader
    leaders.insert(0);
    for (int i = 1; i < instructions.size(); i++) {
        GPUStaticInst* instruction = instructions[i];
        if (instruction->o_type == Enums::OT_BRANCH) {
            const int target_pc = instruction->getTargetPc();
            leaders.insert(target_pc);
            leaders.insert(i + 1);
        }
    }

    size_t block_size = 0;
    for (int i = 0; i < instructions.size(); i++) {
        if (leaders.find(i) != leaders.end()) {
            uint32_t id = basicBlocks.size();
            if (id > 0) {
                basicBlocks.back()->size = block_size;
            }
            block_size = 0;
            basicBlocks.emplace_back(new BasicBlock(id, instructions[i]));
        }
        block_size++;
    }
    basicBlocks.back()->size = block_size;
    // exit basic block
    basicBlocks.emplace_back(new BasicBlock(basicBlocks.size(), nullptr));
}

void
ControlFlowInfo::connectBasicBlocks()
{
    BasicBlock* exit_bb = basicBlocks.back().get();
    for (auto& bb : basicBlocks) {
        if (bb->isExit()) {
            break;
        }
        GPUStaticInst* last = lastInstruction(bb.get());
        if (last->o_type == Enums::OT_RET) {
            bb->successorIds.insert(exit_bb->id);
            break;
        }
        if (last->o_type == Enums::OT_BRANCH) {
            const uint32_t target_pc = last->getTargetPc();
            BasicBlock* target_bb = basicBlock(target_pc);
            bb->successorIds.insert(target_bb->id);
        }

        // Unconditional jump instructions have a unique successor
        if (!last->unconditionalJumpInstruction()) {
            BasicBlock* next_bb = basicBlock(last->instNum() + 1);
            bb->successorIds.insert(next_bb->id);
        }
    }
}


// In-place set intersection
static void
intersect(std::set<uint32_t>& a, const std::set<uint32_t>& b)
{
    std::set<uint32_t>::iterator it = a.begin();
    while (it != a.end()) {
        it = b.find(*it) != b.end() ? ++it : a.erase(it);
    }
}


void
ControlFlowInfo::findPostDominators()
{
    // the only postdominator of the exit block is itself
    basicBlocks.back()->postDominatorIds.insert(basicBlocks.back()->id);
    //copy all basic blocks to all postdominator lists except for exit block
    for (auto& block : basicBlocks) {
        if (!block->isExit()) {
            for (uint32_t i = 0; i < basicBlocks.size(); i++) {
                block->postDominatorIds.insert(i);
            }
        }
    }

    bool change = true;
    while (change) {
        change = false;
        for (int h = basicBlocks.size() - 2; h >= 0; --h) {
            size_t num_postdominators =
                    basicBlocks[h]->postDominatorIds.size();
            for (int s : basicBlocks[h]->successorIds) {
                intersect(basicBlocks[h]->postDominatorIds,
                          basicBlocks[s]->postDominatorIds);
            }
            basicBlocks[h]->postDominatorIds.insert(h);
            change |= (num_postdominators
                    != basicBlocks[h]->postDominatorIds.size());
        }
    }
}


// In-place set difference
static void
setDifference(std::set<uint32_t>&a,
           const std::set<uint32_t>& b, uint32_t exception)
{
    for (uint32_t b_elem : b) {
        if (b_elem != exception) {
            a.erase(b_elem);
        }
    }
}

void
ControlFlowInfo::findImmediatePostDominators()
{
    assert(basicBlocks.size() > 1); // Entry and exit blocks must be present

    findPostDominators();

    for (auto& basicBlock : basicBlocks) {
        if (basicBlock->isExit()) {
            continue;
        }
        std::set<uint32_t> candidates = basicBlock->postDominatorIds;
        candidates.erase(basicBlock->id);
        for (uint32_t postDominatorId : basicBlock->postDominatorIds) {
            if (postDominatorId != basicBlock->id) {
                setDifference(candidates,
                           basicBlocks[postDominatorId]->postDominatorIds,
                           postDominatorId);
            }
        }
        assert(candidates.size() == 1);
        GPUStaticInst* last_instruction = lastInstruction(basicBlock.get());
        BasicBlock* ipd_block = basicBlocks[*(candidates.begin())].get();
        if (!ipd_block->isExit()) {
            GPUStaticInst* ipd_first_inst = ipd_block->firstInstruction;
            last_instruction->ipdInstNum(ipd_first_inst->instNum());
        } else {
            last_instruction->ipdInstNum(last_instruction->instNum() + 1);
        }
    }
}

void
ControlFlowInfo::printPostDominators() const
{
    for (auto& block : basicBlocks) {
        std::cout << "PD(" << block->id << ") = {";
        std::copy(block->postDominatorIds.begin(),
                  block->postDominatorIds.end(),
                  std::ostream_iterator<uint32_t>(std::cout, ", "));
        std::cout << "}" << std::endl;
    }
}

void
ControlFlowInfo::printImmediatePostDominators() const
{
    for (const auto& block : basicBlocks) {
        if (block->isExit()) {
            continue;
        }
        std::cout << "IPD(" << block->id << ") = ";
        std::cout << postDominator(block.get())->id << ", ";
    }
    std::cout << std::endl;
}
void
ControlFlowInfo::printBasicBlocks() const
{
    for (GPUStaticInst* inst : instructions) {
        int inst_num = inst->instNum();
        std::cout << inst_num << " [" << basicBlock(inst_num)->id
                << "]: " << inst->disassemble();
        if (inst->o_type == Enums::OT_BRANCH) {
            std::cout << ", PC = " << inst->getTargetPc();
        }
        std::cout << std::endl;
    }
}

void
ControlFlowInfo::printBasicBlockDot() const
{
    printf("digraph {\n");
    for (const auto& basic_block : basicBlocks) {
        printf("\t");
        for (uint32_t successorId : basic_block->successorIds) {
            printf("%d -> %d; ", basic_block->id, successorId);
        }
        printf("\n");
    }
    printf("}\n");
}
