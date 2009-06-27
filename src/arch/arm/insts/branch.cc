/* Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Stephen Hines
 */

#include "arch/arm/insts/branch.hh"
#include "base/loader/symtab.hh"

namespace ArmISA
{
Addr
Branch::branchTarget(Addr branchPC) const
{
    return branchPC + 8 + disp;
}

Addr
Jump::branchTarget(ThreadContext *tc) const
{
    Addr NPC = tc->readPC() + 8;
    uint64_t Rb = tc->readIntReg(_srcRegIdx[0]);
    return (Rb & ~3) | (NPC & 1);
}

const std::string &
PCDependentDisassembly::disassemble(Addr pc,
                                    const SymbolTable *symtab) const
{
    if (!cachedDisassembly ||
        pc != cachedPC || symtab != cachedSymtab)
    {
        if (cachedDisassembly)
            delete cachedDisassembly;

        cachedDisassembly =
            new std::string(generateDisassembly(pc, symtab));
        cachedPC = pc;
        cachedSymtab = symtab;
    }

    return *cachedDisassembly;
}

std::string
Branch::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;

    printMnemonic(ss);
    ss << "\t";

    Addr target = pc + 8 + disp;
    ccprintf(ss, "%#x", target);
    printMemSymbol(ss, symtab, " <", target, ">");

    return ss.str();
}

std::string
BranchExchange::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    if (_numSrcRegs > 0) {
        printReg(ss, _srcRegIdx[0]);
    }
    return ss.str();
}

std::string
Jump::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    return ss.str();
}
}
