/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2007-2008 The Florida State University
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

#include "arch/arm/insts/pred_inst.hh"

namespace gem5
{

namespace ArmISA
{
std::string
PredIntOp::generateDisassembly(Addr pc,
                               const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    unsigned rotate = machInst.rotate * 2;
    uint32_t imm = machInst.imm;
    imm = (imm << (32 - rotate)) | (imm >> rotate);
    printDataInst(ss, false, machInst.opcode4 == 0, machInst.sField,
                  machInst.rd, machInst.rn, machInst.rm, machInst.rs,
                  machInst.shiftSize, (ArmShiftType)(uint32_t)machInst.shift,
                  imm);
    return ss.str();
}

std::string
PredImmOp::generateDisassembly(Addr pc,
                               const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printDataInst(ss, true, machInst.opcode4 == 0, machInst.sField,
                  machInst.rd, machInst.rn, machInst.rm, machInst.rs,
                  machInst.shiftSize, (ArmShiftType)(uint32_t)machInst.shift,
                  imm);
    return ss.str();
}

std::string
DataImmOp::generateDisassembly(Addr pc,
                               const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printDataInst(ss, true, false, /*XXX not really s*/ false, dest, op1,
                  int_reg::Zero, int_reg::Zero, 0, LSL, imm);
    return ss.str();
}

std::string
DataRegOp::generateDisassembly(Addr pc,
                               const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printDataInst(ss, false, true, /*XXX not really s*/ false, dest, op1, op2,
                  int_reg::Zero, shiftAmt, shiftType, 0);
    return ss.str();
}

std::string
DataRegRegOp::generateDisassembly(Addr pc,
                                  const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printDataInst(ss, false, false, /*XXX not really s*/ false, dest, op1, op2,
                  shift, 0, shiftType, 0);
    return ss.str();
}

std::string
PredMacroOp::generateDisassembly(Addr pc,
                                 const loader::SymbolTable *symtab) const
{
    std::stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    return ss.str();
}

} // namespace ArmISA
} // namespace gem5
