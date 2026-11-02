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

#include "salam/HWModeling/instruction_config.hh"

#include <map>

#include "base/logging.hh"
#include "llvm/IR/Instruction.h"
#include "params/InstConfig.hh"
#include "params/InstructionConfig.hh"

namespace gem5
{

InstructionConfig::InstructionConfig(const Params &params) : SimObject(params)
{
    const char *expected = nullptr;
    // Use the linked LLVM version's opcode definitions, including opcodes
    // the execution engine may later reject as unsupported. Do not infer
    // instruction identity from an arbitrary SimObject child name.
    switch (params.opcode_num) {
#define HANDLE_INST(number, opcode, instclass)                                \
    case number:                                                              \
        expected = llvm::Instruction::getOpcodeName(number);                  \
        break;
#include "llvm/IR/Instruction.def"
        default:
            fatal("%s: invalid LLVM opcode_num %u for opcode_name '%s'",
                  name(), params.opcode_num, params.opcode_name);
    }
    fatal_if(params.opcode_name != expected,
             "%s: opcode_name '%s' does not match LLVM opcode_num %u (%s)",
             name(), params.opcode_name, params.opcode_num, expected);
}

uint32_t
InstructionConfig::getFunctionalUnit() const
{
    return params().functional_unit;
}

uint32_t
InstructionConfig::getOpcodeNum() const
{
    return params().opcode_num;
}

InstConfig::InstConfig(const Params &params)
    : SimObject(params), instList(params.inst_list)
{
    std::map<uint32_t, InstructionConfig *> opcodes;
    for (auto *instruction : instList) {
        const auto opcode = instruction->getOpcodeNum();
        auto inserted = opcodes.emplace(opcode, instruction);
        fatal_if(!inserted.second,
                 "%s: duplicate LLVM opcode_num %u in %s and %s", name(),
                 opcode, inserted.first->second->name(), instruction->name());
    }
}

} // namespace gem5
