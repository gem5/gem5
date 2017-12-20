/*
 * Copyright (c) 2011-2013,2018 ARM Limited
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
 *
 * Authors: Gabe Black
 */

#include "arch/arm/insts/mem64.hh"

#include "arch/arm/tlb.hh"
#include "base/loader/symtab.hh"
#include "mem/request.hh"

using namespace std;

namespace ArmISA
{

std::string
SysDC64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, ", ");
    printIntReg(ss, base);
    return ss.str();
}



void
Memory64::startDisassembly(std::ostream &os) const
{
    printMnemonic(os, "", false);
    printIntReg(os, dest);
    ccprintf(os, ", [");
    printIntReg(os, base);
}

void
Memory64::setExcAcRel(bool exclusive, bool acrel)
{
    if (exclusive)
        memAccessFlags |= Request::LLSC;
    else
        memAccessFlags |= ArmISA::TLB::AllowUnaligned;
    if (acrel) {
        flags[IsMemBarrier] = true;
        flags[IsWriteBarrier] = true;
        flags[IsReadBarrier] = true;
    }
}

std::string
MemoryImm64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    startDisassembly(ss);
    if (imm)
        ccprintf(ss, ", #%d", imm);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryDImm64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ccprintf(ss, ", ");
    printIntReg(ss, dest2);
    ccprintf(ss, ", [");
    printIntReg(ss, base);
    if (imm)
        ccprintf(ss, ", #%d", imm);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryDImmEx64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, result);
    ccprintf(ss, ", ");
    printIntReg(ss, dest);
    ccprintf(ss, ", ");
    printIntReg(ss, dest2);
    ccprintf(ss, ", [");
    printIntReg(ss, base);
    if (imm)
        ccprintf(ss, ", #%d", imm);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryPreIndex64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    startDisassembly(ss);
    ccprintf(ss, ", #%d]!", imm);
    return ss.str();
}

std::string
MemoryPostIndex64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    startDisassembly(ss);
    if (imm)
        ccprintf(ss, "], #%d", imm);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryReg64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    startDisassembly(ss);
    printExtendOperand(false, ss, offset, type, shiftAmt);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryRaw64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    startDisassembly(ss);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryEx64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ccprintf(ss, ", ");
    printIntReg(ss, result);
    ccprintf(ss, ", [");
    printIntReg(ss, base);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryLiteral64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ccprintf(ss, ", #%d", pc + imm);
    return ss.str();
}
}
