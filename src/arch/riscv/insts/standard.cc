/*
 * Copyright (c) 2015 RISC-V Foundation
 * Copyright (c) 2017 The University of Virginia
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
 * Authors: Alec Roelke
 */

#include "arch/riscv/insts/standard.hh"

#include <sstream>
#include <string>

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/utility.hh"
#include "cpu/static_inst.hh"

using namespace std;

namespace RiscvISA
{

string
RegOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    stringstream ss;
    ss << mnemonic << ' ' << registerName(_destRegIdx[0]) << ", " <<
        registerName(_srcRegIdx[0]) << ", " <<
        registerName(_srcRegIdx[1]);
    return ss.str();
}

string
CSROp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    stringstream ss;
    ss << mnemonic << ' ' << registerName(_destRegIdx[0]) << ", ";
    if (_numSrcRegs > 0)
        ss << registerName(_srcRegIdx[0]) << ", ";
    auto name = MiscRegNames.find(csr);
    if (name != MiscRegNames.end())
        ss << name->second;
    else
        ss << "?? (" << hex << "0x" << csr << ")";
    return ss.str();
}

}