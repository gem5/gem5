/*
 * Copyright (c) 2024 Google LLC
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
 */

#include "arch/riscv/insts/zcmp.hh"

#include <string>

#include "arch/riscv/regs/int.hh"
#include "arch/riscv/utility.hh"

namespace gem5
{

namespace RiscvISA
{

CmMacroInst::CmMacroInst(
    const char* mnem, ExtMachInst machInst, OpClass opClass)
    : RiscvMacroInst(mnem, machInst, opClass), rlist(machInst.rlist)
{
}

// Ref: https://github.com/riscv-software-src/riscv-isa-sim/blob/f7d0dba60/
//      riscv/decode.h#L168
uint64_t
CmMacroInst::stackAdj() const
{
    uint64_t stack_adj_base = 0;
    switch (machInst.rlist) {
      case 15:
        stack_adj_base += 16;
        [[fallthrough]];
      case 14:
        if (machInst.rv_type == RV64) {
            stack_adj_base += 16;
        }
        [[fallthrough]];
      case 13:
      case 12:
        stack_adj_base += 16;
        [[fallthrough]];
      case 11:
      case 10:
        if (machInst.rv_type == RV64) {
            stack_adj_base += 16;
        }
        [[fallthrough]];
      case 9:
      case 8:
        stack_adj_base += 16;
        [[fallthrough]];
      case 7:
      case 6:
        if (machInst.rv_type == RV64) {
            stack_adj_base += 16;
        }
        [[fallthrough]];
      case 5:
      case 4:
        stack_adj_base += 16;
        break;
    }

    return stack_adj_base + machInst.spimm * 16;
}

std::string
CmMacroInst::getRlistStr() const
{
    std::string s = "";
    switch (machInst.rlist) {
      case 15:
        s = csprintf("{%s, %s-%s}", registerName(ReturnAddrReg),
                     registerName(int_reg::S0),
                     registerName(PushPopRegList[0]));
        break;
      case 14:
      case 13:
      case 12:
      case 11:
      case 10:
      case 9:
      case 8:
      case 7:
      case 6:
        s = csprintf("{%s, %s-%s}", registerName(ReturnAddrReg),
                     registerName(int_reg::S0),
                     registerName(PushPopRegList[16-machInst.rlist]));
        break;
      case 5:
        s = csprintf("{%s, %s}", registerName(ReturnAddrReg),
                     registerName(int_reg::S0));
        break;
      case 4:
        s = csprintf("{%s}", registerName(ReturnAddrReg));
        break;
      default:
        break;
    }

    return s;
}

} // namespace RiscvISA
} // namespace gem5
