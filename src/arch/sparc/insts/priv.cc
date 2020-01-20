/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
 * All rights reserved
 * Copyright 2017 Google Inc.
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

#include "arch/sparc/insts/priv.hh"

namespace SparcISA
{

std::string
Priv::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream response;

    printMnemonic(response, mnemonic);

    return response.str();
}

std::string
RdPriv::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream response;

    printMnemonic(response, mnemonic);

    ccprintf(response, " %%%s, ", regName);
    printDestReg(response, 0);

    return response.str();
}

std::string
WrPriv::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream response;

    printMnemonic(response, mnemonic);

    ccprintf(response, " ");
    // If the first reg is %g0, don't print it.
    // This improves readability
    if (_srcRegIdx[0].index() != 0) {
        printSrcReg(response, 0);
        ccprintf(response, ", ");
    }
    printSrcReg(response, 1);
    ccprintf(response, ", %%%s", regName);

    return response.str();
}

std::string
WrPrivImm::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream response;

    printMnemonic(response, mnemonic);

    ccprintf(response, " ");
    // If the first reg is %g0, don't print it.
    // This improves readability
    if (_srcRegIdx[0].index() != 0) {
        printSrcReg(response, 0);
        ccprintf(response, ", ");
    }
    ccprintf(response, "%#x, %%%s", imm, regName);

    return response.str();
}

}
