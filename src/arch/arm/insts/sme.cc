/*
 * Copyright (c) 2022, 2025-2026 ARM Limited
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
SmeAddOp::generateDisassembly(Addr pc,
                              const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "ZA%d", imm);
    ss << ", ";
    printVecPredReg(ss, gp1);
    ss << ", ";
    printVecPredReg(ss, gp2);
    ss << ", ";
    printVecReg(ss, op1, true);
    return ss.str();
}

std::string
SmeAddVlOp::generateDisassembly(Addr pc,
                                const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
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
    ccprintf(ss, "ZA%d.%s", zad, (V ? "V" : "H"));
    ss << "[";
    printIntReg(ss, op2);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "], ";
    printVecPredReg(ss, gp);
    ss << ", [";
    printIntReg(ss, op1);
    ss << ", ";
    printIntReg(ss, op3);
    ss << "]";
    return ss.str();
}

std::string
SmeLdrStrOp::generateDisassembly(Addr pc,
                                 const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "ZA");
    ss << "[";
    printIntReg(ss, op2, true);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "], [";
    printIntReg(ss, op1, true);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << ", MUL VL]";
    return ss.str();
}

std::string
SmeLdrStrTableOp::generateDisassembly(Addr pc,
                                      const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZT0";
    ss << ", ";
    printIntReg(ss, op1, true);
    return ss.str();
}

std::string
SmeZeroArrayOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "]";
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
    printVecPredReg(ss, gp);
    ss << ", ";
    ccprintf(ss, "ZA%d.%s", zan, (v ? "V" : "H"));
    ss << "[";
    printIntReg(ss, op2);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
SmeMovInsertOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "ZA%d.%s", zad, (v ? "V" : "H"));
    ss << "[";
    printIntReg(ss, op2);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "], ";
    printVecPredReg(ss, gp);
    ss << ", ";
    printVecReg(ss, op1, true);
    return ss.str();
}

std::string
SmeMovExtract1RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printVecReg(ss, dest1, true);
    ss << ", ";
    ccprintf(ss, "ZA%d.%s", zan, (v ? "V" : "H"));
    ss << "[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
SmeMovExtract2RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest2, true);
    ss << "}, ";
    ccprintf(ss, "ZA%d.%s", zan, (v ? "V" : "H"));
    ss << "[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
SmeMovInsert2RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "ZA%d.%s", zad, (v ? "V" : "H"));
    ss << "[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}";
    return ss.str();
}

std::string
SmeMovExtract4RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, ";
    ccprintf(ss, "ZA%d.%s", zan, (v ? "V" : "H"));
    ss << "[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
SmeMovInsert4RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "ZA%d.%s", zad, (v ? "V" : "H"));
    ss << "[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}";
    return ss.str();
}

std::string
SmeMovArrayExtract2RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest2, true);
    ss << "}, ";
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
SmeMovArrayExtract4RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, ";
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
SmeMovArrayInsert2RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}, ";
    return ss.str();
}

std::string
SmeMovArrayInsert4RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}, ";
    return ss.str();
}

std::string
SmeOPOp::generateDisassembly(Addr pc,
                             const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "ZA%d", imm);
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
    printIntReg(ss, dest);
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
    ccprintf(ss, "ZA%d", imm);
    return ss.str();
}

std::string
Sme2Vector1x2Op::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    if (destr) {
        printVecReg(ss, dest, true);
        ss << ", ";
    }
    printVecReg(ss, dest, true);
    ss << ", {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2Vector1x4Op::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    if (destr) {
        printVecReg(ss, dest, true);
        ss << ", ";
    }
    printVecReg(ss, dest, true);
    ss << ", {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2Vector2x1Op::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    if (destr) {
        ss << "{";
        printVecReg(ss, dest1, true);
        ss << "-";
        printVecReg(ss, dest2, true);
        ss << "}, ";
    }
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest2, true);
    ss << "}, ";
    printVecReg(ss, op1, true);
    return ss.str();
}

std::string
Sme2Vector4x1Op::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    if (destr) {
        ss << "{";
        printVecReg(ss, dest1, true);
        ss << "-";
        printVecReg(ss, dest4, true);
        ss << "}, ";
    }
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, ";
    printVecReg(ss, op1, true);
    return ss.str();
}

std::string
Sme2Vector2x2Op::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    if (destr) {
        ss << "{";
        printVecReg(ss, dest1, true);
        ss << "-";
        printVecReg(ss, dest2, true);
        ss << "}, ";
    }
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest2, true);
    ss << "}, {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2Vector4x2Op::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    if (destr) {
        ss << "{";
        printVecReg(ss, dest1, true);
        ss << "-";
        printVecReg(ss, dest4, true);
        ss << "}, ";
    }
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2Vector4x4Op::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    if (destr) {
        ss << "{";
        printVecReg(ss, dest1, true);
        ss << "-";
        printVecReg(ss, dest4, true);
        ss << "}, ";
    }
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2MultiSgl1RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], ";
    printVecReg(ss, op1, true);
    ss << ", ";
    printVecReg(ss, op5, true);
    return ss.str();
}

std::string
Sme2MultiSgl2RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}, ";
    printVecReg(ss, op5, true);
    return ss.str();
}

std::string
Sme2MultiSgl4RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}, ";
    printVecReg(ss, op5, true);
    return ss.str();
}

std::string
Sme2MultiVec2RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}, {";
    printVecReg(ss, op5, true);
    ss << "-";
    printVecReg(ss, op6, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2MultiVec4RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}, {";
    printVecReg(ss, op5, true);
    ss << "-";
    printVecReg(ss, op8, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2MultiIdx1RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], ";
    printVecReg(ss, op1, true);
    ss << ", ";
    printVecReg(ss, op5, true);
    ss << "[";
    ccprintf(ss, "#%d", idx);
    ss << "]";
    return ss.str();
}

std::string
Sme2MultiIdx2RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}, ";
    printVecReg(ss, op5, true);
    ss << "[";
    ccprintf(ss, "#%d", idx);
    ss << "]";
    return ss.str();
}

std::string
Sme2MultiIdx4RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}, ";
    printVecReg(ss, op5, true);
    ss << "[";
    ccprintf(ss, "#%d", idx);
    ss << "]";
    return ss.str();
}

std::string
Sme2ZlutOp::generateDisassembly(Addr pc,
                                const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZT0";
    return ss.str();
}

std::string
Sme2ZaZ2RegOp::generateDisassembly(Addr pc,
                                   const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2ZaZ4RegOp::generateDisassembly(Addr pc,
                                   const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "ZA[";
    printIntReg(ss, index);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "], {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2Clamp2RegOp::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest2, true);
    ss << "}, ";
    printVecReg(ss, op1, true);
    ss << ", ";
    printVecReg(ss, op2, true);
    return ss.str();
}

std::string
Sme2Clamp4RegOp::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, ";
    printVecReg(ss, op1, true);
    ss << ", ";
    printVecReg(ss, op2, true);
    return ss.str();
}

std::string
Sme2Rshr2RegOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printVecReg(ss, dest, true);
    ss << ", {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}, ";
    ccprintf(ss, "#%d", imm);
    return ss.str();
}

std::string
Sme2Rshr4RegOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printVecReg(ss, dest, true);
    ss << ", {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}, ";
    ccprintf(ss, "#%d", imm);
    return ss.str();
}

std::string
Sme2SSel2RegOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest2, true);
    ss << "}, ";
    printVecPredReg(ss, gp, true);
    ss << ", {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}, {";
    printVecReg(ss, op5, true);
    ss << "-";
    printVecReg(ss, op6, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2SSel4RegOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, ";
    printVecPredReg(ss, gp, true);
    ss << ", {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op4, true);
    ss << "}, {";
    printVecReg(ss, op5, true);
    ss << "-";
    printVecReg(ss, op8, true);
    ss << "}";
    return ss.str();
}

std::string
Sme2Luti1RegOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printVecReg(ss, dest1, true);
    ss << ", ";
    ss << "ZT0";
    ss << ", ";
    printVecReg(ss, op1, true);
    ss << "[";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
Sme2Luti2RegOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest2, true);
    ss << "}, ";
    ss << "ZT0";
    ss << ", ";
    printVecReg(ss, op1, true);
    ss << "[";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
Sme2Luti4RegOp::generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, ";
    ss << "ZT0";
    ss << ", ";
    printVecReg(ss, op1, true);
    ss << "[";
    ccprintf(ss, "%d", imm);
    ss << "]";
    return ss.str();
}

std::string
Sme2Luti4to8b4RegOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ss << "{";
    printVecReg(ss, dest1, true);
    ss << "-";
    printVecReg(ss, dest4, true);
    ss << "}, ";
    ss << "ZT0";
    ss << ", {";
    printVecReg(ss, op1, true);
    ss << "-";
    printVecReg(ss, op2, true);
    ss << "}";
    return ss.str();
}

} // namespace ArmISA
} // namespace gem5
