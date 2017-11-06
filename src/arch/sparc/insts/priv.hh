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
 *
 * Authors: Ali Saidi
 *          Gabe Black
 *          Steve Reinhardt
 */

#ifndef __ARCH_SPARC_INSTS_PRIV_HH__
#define __ARCH_SPARC_INSTS_PRIV_HH__

#include "arch/sparc/insts/static_inst.hh"

namespace SparcISA
{

/**
 * Base class for privelege mode operations.
 */
class Priv : public SparcStaticInst
{
  protected:
    using SparcStaticInst::SparcStaticInst;
    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class PrivReg : public Priv
{
  protected:
    PrivReg(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass, char const * _regName) :
        Priv(mnem, _machInst, __opClass), regName(_regName)
    {}

    char const *regName;
};

// This class is for instructions that explicitly read control
// registers. It provides a special generateDisassembly function.
class RdPriv : public PrivReg
{
  protected:
    using PrivReg::PrivReg;
    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

// This class is for instructions that explicitly write control
// registers. It provides a special generateDisassembly function.
class WrPriv : public PrivReg
{
  protected:
    using PrivReg::PrivReg;
    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

/**
 * Base class for privelege mode operations with immediates.
 */
class PrivImm : public Priv
{
  protected:
    // Constructor
    PrivImm(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        Priv(mnem, _machInst, __opClass), imm(bits(_machInst, 12, 0))
    {}

    int32_t imm;
};

// This class is for instructions that explicitly write control
// registers. It provides a special generateDisassembly function.
class WrPrivImm : public PrivImm
{
  protected:
    // Constructor
    WrPrivImm(const char *mnem, ExtMachInst _machInst,
              OpClass __opClass, char const *_regName) :
        PrivImm(mnem, _machInst, __opClass), regName(_regName)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;

    char const *regName;
}
;
}

#endif //__ARCH_SPARC_INSTS_PRIV_HH__
