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

#ifndef __ARCH_POWER_MEM_HH__
#define __ARCH_POWER_MEM_HH__

#include "arch/power/insts/static_inst.hh"

namespace PowerISA
{

/**
 * Base class for memory operations.
 */
class MemOp : public PowerStaticInst
{
  protected:

    /// Memory request flags.  See mem_req_base.hh.
    unsigned memAccessFlags;

    /// Constructor
    MemOp(const char *mnem, MachInst _machInst, OpClass __opClass)
      : PowerStaticInst(mnem, _machInst, __opClass),
        memAccessFlags(0)
    {
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};


/**
 * Class for memory operations with displacement.
 */
class MemDispOp : public MemOp
{
  protected:

    int16_t disp;

    /// Constructor
    MemDispOp(const char *mnem, MachInst _machInst, OpClass __opClass)
      : MemOp(mnem, _machInst, __opClass), disp(machInst.d)
    {
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

} // namespace PowerISA

#endif //__ARCH_POWER_INSTS_MEM_HH__
