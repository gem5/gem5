/*
 * Copyright (c) 2022 ARM Limited
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

#include "arch/arm/insts/sme.hh"

namespace gem5
{

namespace ArmISA
{

std::string
SmeAddOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#%d", imm);
    ss << ", ";
    printVecReg(ss, op1, true);
    ss << ", ";
    printVecPredReg(ss, gp1);
    ss << ", ";
    printVecPredReg(ss, gp2);
    return ss.str();
}

std::string
SmeAddVlOp::generateDisassembly(Addr pc,
                                const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << ", ";
    printVecReg(ss, dest);
    ss << ", ";
    printVecReg(ss, op1);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    return ss.str();
}

std::string
SmeLd1xSt1xOp::generateDisassembly(Addr pc,
                                   const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#%d", imm);
    ss << ", ";
    printIntReg(ss, op1);
    ss << ", ";
    printVecPredReg(ss, gp);
    ss << ", ";
    printIntReg(ss, op2);
    ss << ", ";
    printIntReg(ss, op3);
    return ss.str();
}

std::string
SmeLdrStrOp::generateDisassembly(Addr pc,
                                 const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#%d", imm);
    ss << ", ";
    printIntReg(ss, op1, true);
    ss << ", ";
    printIntReg(ss, op2, true);
    return ss.str();
}

std::string
SmeMovExtractOp::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printVecReg(ss, op1, true);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << ", ";
    printVecPredReg(ss, gp);
    ss << ", ";
    printIntReg(ss, op2);
    return ss.str();
}

std::string
SmeMovInsertOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#%d", imm);
    ss << ", ";
    printVecReg(ss, op1, true);
    ss << ", ";
    printVecPredReg(ss, gp);
    ss << ", ";
    printIntReg(ss, op2);
    return ss.str();
}

std::string
SmeOPOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#%d", imm);
    ss << ", ";
    printVecPredReg(ss, gp1);
    ss << ", ";
    printVecPredReg(ss, gp2);
    ss << ", ";
    printVecReg(ss, op1, true);
    ss << ", ";
    printVecReg(ss, op2, true);
    return ss.str();
}

std::string
SmeRdsvlOp::generateDisassembly(Addr pc,
                                const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << ", ";
    printVecReg(ss, dest);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    return ss.str();
}

std::string
SmeZeroOp::generateDisassembly(Addr pc,
                               const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ArmStaticInst::printMnemonic(ss, "", false);
    ccprintf(ss, "#%d", imm);
    return ss.str();
}

} // namespace ArmISA
} // namespace gem5
