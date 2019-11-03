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
 *
 * Authors: Ali Saidi
 *          Gabe Black
 *          Steve Reinhardt
 */

#ifndef __ARCH_SPARC_INSTS_INTEGER_HH__
#define __ARCH_SPARC_INSTS_INTEGER_HH__

#include "arch/sparc/insts/static_inst.hh"

namespace SparcISA
{

////////////////////////////////////////////////////////////////////
//
// Integer operate instructions
//

/**
 * Base class for integer operations.
 */
class IntOp : public SparcStaticInst
{
  protected:
    using SparcStaticInst::SparcStaticInst;

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;

    virtual bool printPseudoOps(std::ostream &os, Addr pc,
                                const SymbolTable *symtab) const;
};

/**
 * Base class for immediate integer operations.
 */
class IntOpImm : public IntOp
{
  protected:
    // Constructor
    IntOpImm(const char *mnem, ExtMachInst _machInst,
             OpClass __opClass, int64_t _imm) :
        IntOp(mnem, _machInst, __opClass), imm(_imm)
    {}

    int64_t imm;

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;

    bool printPseudoOps(std::ostream &os, Addr pc,
                        const SymbolTable *symtab) const override;
};

/**
 * Base class for 10 bit immediate integer operations.
 */
class IntOpImm10 : public IntOpImm
{
  protected:
    // Constructor
    IntOpImm10(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        IntOpImm(mnem, _machInst, __opClass, sext<10>(bits(_machInst, 9, 0)))
    {}
};

/**
 * Base class for 11 bit immediate integer operations.
 */
class IntOpImm11 : public IntOpImm
{
  protected:
    IntOpImm11(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        IntOpImm(mnem, _machInst, __opClass, sext<10>(bits(_machInst, 10, 0)))
    {}
};

/**
 * Base class for 13 bit immediate integer operations.
 */
class IntOpImm13 : public IntOpImm
{
  protected:
    IntOpImm13(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        IntOpImm(mnem, _machInst, __opClass, sext<13>(bits(_machInst, 12, 0)))
    {}
};

/**
 * Base class for sethi.
 */
class SetHi : public IntOpImm
{
  protected:
    // Constructor
    SetHi(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        IntOpImm(mnem, _machInst, __opClass, bits(_machInst, 21, 0) << 10)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

}

#endif // __ARCH_SPARCH_INSTS_INTEGER_HH__
