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

////////////////////////////////////////////////////////////////////
//
// Trap instructions
//

#ifndef __ARCH_SPARC_INSTS_TRAP_HH__
#define __ARCH_SPARC_INSTS_TRAP_HH__

#include "arch/sparc/insts/static_inst.hh"

namespace gem5
{

namespace SparcISA
{

/**
 * Base class for trap instructions,
 * or instructions that always fault.
 */
class Trap : public SparcStaticInst
{
  protected:

    // Constructor
    Trap(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        SparcStaticInst(mnem, _machInst, __opClass),
        trapNum(bits(_machInst, 7, 0))
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;

    int trapNum;
};

class FpUnimpl : public SparcStaticInst
{
  protected:
    using SparcStaticInst::SparcStaticInst;

    std::string
    generateDisassembly(
            Addr pc,  const loader::SymbolTable *symtab) const override
    {
        return mnemonic;
    }
};

} // namespace SparcISA
} // namespace gem5

#endif // __ARCH_SPARC_INSTS_TRAP_HH__
