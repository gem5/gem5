/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
 * All rights reserved.
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
 * Authors: Gabe Black
 */
#ifndef __ARCH_SPARC_INSTS_STATIC_INST_HH__
#define __ARCH_SPARC_INSTS_STATIC_INST_HH__

#include <cstdint>

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"

namespace SparcISA
{

enum CondTest
{
    Always=0x8,
    Never=0x0,
    NotEqual=0x9,
    Equal=0x1,
    Greater=0xA,
    LessOrEqual=0x2,
    GreaterOrEqual=0xB,
    Less=0x3,
    GreaterUnsigned=0xC,
    LessOrEqualUnsigned=0x4,
    CarryClear=0xD,
    CarrySet=0x5,
    Positive=0xE,
    Negative=0x6,
    OverflowClear=0xF,
    OverflowSet=0x7
};

extern const char *CondTestAbbrev[];

enum FpCondTest
{
    FAlways=0x8,
    FNever=0x0,
    FUnordered=0x7,
    FGreater=0x6,
    FUnorderedOrGreater=0x5,
    FLess=0x4,
    FUnorderedOrLess=0x3,
    FLessOrGreater=0x2,
    FNotEqual=0x1,
    FEqual=0x9,
    FUnorderedOrEqual=0xA,
    FGreaterOrEqual=0xB,
    FUnorderedOrGreaterOrEqual=0xC,
    FLessOrEqual=0xD,
    FUnorderedOrLessOrEqual=0xE,
    FOrdered=0xF
};

/**
 * Base class for all SPARC static instructions.
 */
class SparcStaticInst : public StaticInst
{
  protected:
    using StaticInst::StaticInst;

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;

    static void printMnemonic(std::ostream &os, const char *mnemonic);
    static void printReg(std::ostream &os, RegId reg);

    void printSrcReg(std::ostream &os, int reg) const;
    void printDestReg(std::ostream &os, int reg) const;

    void printRegArray(std::ostream &os,
        const RegId indexArray[], int num) const;

    void advancePC(PCState &pcState) const override;

    static bool passesFpCondition(uint32_t fcc, uint32_t condition);
    static bool passesCondition(uint32_t codes, uint32_t condition);

    size_t
    asBytes(void *buf, size_t size) override
    {
        return simpleAsBytes(buf, size, machInst);
    }
};

}

#endif //__ARCH_SPARC_INSTS_STATIC_INST_HH__
