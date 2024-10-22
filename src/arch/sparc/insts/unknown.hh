/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __ARCH_SPARC_INSTS_UNKNOWN_HH__
#define __ARCH_SPARC_INSTS_UNKNOWN_HH__

#include "arch/sparc/insts/static_inst.hh"

namespace gem5
{

namespace SparcISA
{

/**
 * Class for Unknown/Illegal instructions
 */
class Unknown : public SparcStaticInst
{
  public:

    // Constructor
    Unknown(ExtMachInst _machInst) :
            SparcStaticInst("unknown", _machInst, No_OpClass)
    {
        flags[IsInvalid] = true;
    }

    Fault
    execute(ExecContext *, trace::InstRecord *) const override
    {
        return std::make_shared<IllegalInstruction>();
    }

    std::string
    generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override
    {
        return "Unknown instruction";
    }

};

} // namespace SparcISA
} // namespace gem5

#endif // __ARCH_SPARC_INSTS_UNKNOWN_HH__
