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

#include "arch/sparc/insts/integer.hh"

namespace SparcISA
{

////////////////////////////////////////////////////////////////////
//
// Integer operate instructions
//

bool
IntOp::printPseudoOps(std::ostream &os, Addr pc,
                      const Loader::SymbolTable *symbab) const
{
    if (!std::strcmp(mnemonic, "or") && _srcRegIdx[0].index() == 0) {
        printMnemonic(os, "mov");
        printSrcReg(os, 1);
        ccprintf(os, ", ");
        printDestReg(os, 0);
        return true;
    }
    return false;
}

bool
IntOpImm::printPseudoOps(std::ostream &os, Addr pc,
                         const Loader::SymbolTable *symbab) const
{
    if (!std::strcmp(mnemonic, "or")) {
        if (_numSrcRegs > 0 && _srcRegIdx[0].index() == 0) {
            if (imm == 0) {
                printMnemonic(os, "clr");
            } else {
                printMnemonic(os, "mov");
                ccprintf(os, " %#x, ", imm);
            }
            printDestReg(os, 0);
            return true;
        } else if (imm == 0) {
            printMnemonic(os, "mov");
            printSrcReg(os, 0);
            ccprintf(os, ", ");
            printDestReg(os, 0);
            return true;
        }
    }
    return false;
}

std::string
IntOp::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream response;

    if (printPseudoOps(response, pc, symtab))
        return response.str();
    printMnemonic(response, mnemonic);
    printRegArray(response, _srcRegIdx, _numSrcRegs);
    if (_numDestRegs && _numSrcRegs)
        response << ", ";
    printDestReg(response, 0);
    return response.str();
}

std::string
IntOpImm::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream response;

    if (printPseudoOps(response, pc, symtab))
        return response.str();
    printMnemonic(response, mnemonic);
    printRegArray(response, _srcRegIdx, _numSrcRegs);
    if (_numSrcRegs > 0)
        response << ", ";
    ccprintf(response, "%#x", imm);
    if (_numDestRegs > 0)
        response << ", ";
    printDestReg(response, 0);
    return response.str();
}

std::string
SetHi::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream response;

    printMnemonic(response, mnemonic);
    ccprintf(response, "%%hi(%#x), ", imm);
    printDestReg(response, 0);
    return response.str();
}

}
