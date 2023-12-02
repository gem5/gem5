/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
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

#include "arch/sparc/insts/branch.hh"

////////////////////////////////////////////////////////////////////
//
// Branch instructions
//

namespace gem5
{

namespace SparcISA
{

template class BranchNBits<19>;
template class BranchNBits<22>;
template class BranchNBits<30>;

std::string
Branch::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream response;

    printMnemonic(response, mnemonic);
    printRegArray(response, &srcRegIdx(0), _numSrcRegs);
    if (_numDestRegs && _numSrcRegs)
            response << ", ";
    printDestReg(response, 0);

    return response.str();
}

std::string
BranchImm13::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream response;

    printMnemonic(response, mnemonic);
    printRegArray(response, &srcRegIdx(0), _numSrcRegs);
    if (_numSrcRegs > 0)
        response << ", ";
    ccprintf(response, "0x%x", imm);
    if (_numDestRegs > 0)
        response << ", ";
    printDestReg(response, 0);

    return response.str();
}

std::string
BranchDisp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream response;

    Addr target = disp + pc;

    printMnemonic(response, mnemonic);
    ccprintf(response, "%#x", target);

    loader::SymbolTable::const_iterator it;
    if (symtab && (it = symtab->findNearest(target)) != symtab->end()) {
        ccprintf(response, " <%s", it->name());
        if (it->address() != target)
            ccprintf(response, "+%d>", target - it->address());
        else
            ccprintf(response, ">");
    }

    return response.str();
}

} // namespace SparcISA
} // namespace gem5
