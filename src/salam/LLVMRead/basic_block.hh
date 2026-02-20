/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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
 */

#ifndef __SALAM_BASIC_BLOCK_HH__
#define __SALAM_BASIC_BLOCK_HH__

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>

#include "debug_flags.hh"
#include "instruction.hh"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instruction.h"
#include "value.hh"

namespace SALAM
{
class Instruction; // Do not remove

class BasicBlock : public Value
{
  private:
    std::vector<std::shared_ptr<SALAM::BasicBlock>> predecessors;
    std::vector<std::shared_ptr<SALAM::Instruction>> instructions;

  protected:
    class BasicBlock_Debugger : public Debugger
    {
      public:
        BasicBlock_Debugger();
        ~BasicBlock_Debugger() = default;
        using SALAM::Debugger::dumper;
        virtual void dumper(SALAM::BasicBlock *bb);
    };
    BasicBlock_Debugger *bb_dbg;

  public:
    BasicBlock(uint64_t id, gem5::SimObject *owner, bool dbg);
    ~BasicBlock();
    virtual bool
    isBasicBlock()
    {
        return true;
    }
    using SALAM::Value::initialize;
    void initialize(llvm::Value *irval, irvmap *vmap,
                    SALAM::valueListTy *valueList);
    std::vector<std::shared_ptr<SALAM::Instruction>> *
    Instructions()
    {
        return &instructions;
    }
    void
    dump()
    {
        if (dbg) {
            bb_dbg->dumper(this);
        }
    }
    bool
    validPredecessor(std::shared_ptr<SALAM::BasicBlock> bb)
    {
        auto it = std::find(predecessors.begin(), predecessors.end(), bb);
        if (it == predecessors.end()) {
            return false;
        }
        return true;
    }
};
} // namespace SALAM

#endif //__SALAM_BASIC_BLOCK_HH__
