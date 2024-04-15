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

#include "arch/sparc/insts/blockmem.hh"

namespace gem5
{

namespace SparcISA
{

std::string
BlockMemMicro::generateDisassembly(Addr pc,
                                   const loader::SymbolTable *symtab) const
{
    std::stringstream response;
    bool load = flags[IsLoad];
    bool save = flags[IsStore];

    printMnemonic(response, mnemonic);
    if (save) {
        printReg(response, srcRegIdx(0));
        ccprintf(response, ", ");
    }
    ccprintf(response, "[ ");
    printReg(response, srcRegIdx(!save ? 0 : 1));
    ccprintf(response, " + ");
    printReg(response, srcRegIdx(!save ? 1 : 2));
    ccprintf(response, " ]");
    if (load) {
        ccprintf(response, ", ");
        printReg(response, destRegIdx(0));
    }

    return response.str();
}

std::string
BlockMemImmMicro::generateDisassembly(Addr pc,
                                      const loader::SymbolTable *symtab) const
{
    std::stringstream response;
    bool load = flags[IsLoad];
    bool save = flags[IsStore];

    printMnemonic(response, mnemonic);
    if (save) {
        printReg(response, srcRegIdx(1));
        ccprintf(response, ", ");
    }
    ccprintf(response, "[ ");
    printReg(response, srcRegIdx(0));
    if (imm >= 0)
        ccprintf(response, " + 0x%x ]", imm);
    else
        ccprintf(response, " + -0x%x ]", -imm);
    if (load) {
        ccprintf(response, ", ");
        printReg(response, destRegIdx(0));
    }

    return response.str();
}

} // namespace SparcISA
} // namespace gem5
