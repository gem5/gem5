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

#ifndef __KERNEL_CFG_HH__
#define __KERNEL_CFG_HH__

#include <cstddef>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>


class GPUStaticInst;
class HsailCode;

struct BasicBlock
{
    BasicBlock(uint32_t num, GPUStaticInst* begin) :
            id(num), size(0), firstInstruction(begin)
    {
    }

    bool
    isEntry() const
    {
        return !id;
    }

    bool
    isExit() const
    {
        return !size;
    }

    /**
     * Unique identifier for the block within a given kernel.
     */
    const uint32_t id;

    /**
     * Number of instructions contained in the block
     */
    size_t size;

    /**
     * Pointer to first instruction of the block.
     */
    GPUStaticInst* firstInstruction;

    /**
     * Identifiers of the blocks that follow (are reachable from) this block.
     */
    std::set<uint32_t> successorIds;

    /**
     * Identifiers of the blocks that will be visited from this block.
     */
    std::set<uint32_t> postDominatorIds;
};

class ControlFlowInfo
{
public:

    /**
     * Compute immediate post-dominator instruction for kernel instructions.
     */
    static void assignImmediatePostDominators(
            const std::vector<GPUStaticInst*>& instructions);

private:
    ControlFlowInfo(const std::vector<GPUStaticInst*>& instructions);

    GPUStaticInst* lastInstruction(const BasicBlock* block) const;

    BasicBlock* basicBlock(int inst_addr) const;

    BasicBlock* postDominator(const BasicBlock* block) const;

    void createBasicBlocks();

    void connectBasicBlocks();

    void findPostDominators();

    void findImmediatePostDominators();

    void printBasicBlocks() const;

    void printBasicBlockDot() const;

    void printPostDominators() const;

    void printImmediatePostDominators() const;

    std::vector<std::unique_ptr<BasicBlock>> basicBlocks;
    std::vector<GPUStaticInst*> instructions;
};

#endif // __KERNEL_CFG_HH__
