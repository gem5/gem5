/*
 * Copyright (c) 2009 The University of Edinburgh
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
 * Authors: Timothy M. Jones
 */

#include "arch/power/insts/branch.hh"
#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"

using namespace PowerISA;

const std::string &
PCDependentDisassembly::disassemble(Addr pc, const SymbolTable *symtab) const
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

PowerISA::PCState
BranchPCRel::branchTarget(const PowerISA::PCState &pc) const
{
    return (uint32_t)(pc.pc() + disp);
}

std::string
BranchPCRel::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    Addr target = pc + disp;

    std::string str;
    if (symtab && symtab->findSymbol(target, str))
        ss << str;
    else
        ccprintf(ss, "0x%x", target);

    return ss.str();
}

PowerISA::PCState
BranchNonPCRel::branchTarget(const PowerISA::PCState &pc) const
{
    return targetAddr;
}

std::string
BranchNonPCRel::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    std::string str;
    if (symtab && symtab->findSymbol(targetAddr, str))
        ss << str;
    else
        ccprintf(ss, "0x%x", targetAddr);

    return ss.str();
}

PowerISA::PCState
BranchPCRelCond::branchTarget(const PowerISA::PCState &pc) const
{
    return (uint32_t)(pc.pc() + disp);
}

std::string
BranchPCRelCond::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    ss << bo << ", " << bi << ", ";

    Addr target = pc + disp;

    std::string str;
    if (symtab && symtab->findSymbol(target, str))
        ss << str;
    else
        ccprintf(ss, "0x%x", target);

    return ss.str();
}

PowerISA::PCState
BranchNonPCRelCond::branchTarget(const PowerISA::PCState &pc) const
{
    return targetAddr;
}

std::string
BranchNonPCRelCond::generateDisassembly(Addr pc,
                                        const SymbolTable *symtab) const
{
    std::stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    ss << bo << ", " << bi << ", ";

    std::string str;
    if (symtab && symtab->findSymbol(targetAddr, str))
        ss << str;
    else
        ccprintf(ss, "0x%x", targetAddr);

    return ss.str();
}

PowerISA::PCState
BranchRegCond::branchTarget(ThreadContext *tc) const
{
    uint32_t regVal = tc->readIntReg(_srcRegIdx[_numSrcRegs - 1]);
    return regVal & 0xfffffffc;
}

std::string
BranchRegCond::generateDisassembly(Addr pc,
                                   const SymbolTable *symtab) const
{
    std::stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    ss << bo << ", " << bi << ", ";

    return ss.str();
}
