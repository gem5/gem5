/*
 * Copyright (c) 2010, 2012-2013, 2017-2018 ARM Limited
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

#ifndef __ARCH_ARM_INSTS_MISC_HH__
#define __ARCH_ARM_INSTS_MISC_HH__

#include "arch/arm/insts/pred_inst.hh"

class MrsOp : public PredOp
{
  protected:
    IntRegIndex dest;

    MrsOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
            IntRegIndex _dest) :
        PredOp(mnem, _machInst, __opClass), dest(_dest)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class MsrBase : public PredOp
{
  protected:
    uint8_t byteMask;

    MsrBase(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
            uint8_t _byteMask) :
        PredOp(mnem, _machInst, __opClass), byteMask(_byteMask)
    {}

    void printMsrBase(std::ostream &os) const;
};

class MsrImmOp : public MsrBase
{
  protected:
    uint32_t imm;

    MsrImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
             uint32_t _imm, uint8_t _byteMask) :
        MsrBase(mnem, _machInst, __opClass, _byteMask), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class MsrRegOp : public MsrBase
{
  protected:
    IntRegIndex op1;

    MsrRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
             IntRegIndex _op1, uint8_t _byteMask) :
        MsrBase(mnem, _machInst, __opClass, _byteMask), op1(_op1)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class MrrcOp : public PredOp
{
  protected:
    MiscRegIndex op1;
    IntRegIndex dest;
    IntRegIndex dest2;
    uint32_t    imm;

    MrrcOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
           MiscRegIndex _op1, IntRegIndex _dest, IntRegIndex _dest2,
           uint32_t _imm) :
        PredOp(mnem, _machInst, __opClass), op1(_op1), dest(_dest),
        dest2(_dest2), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class McrrOp : public PredOp
{
  protected:
    IntRegIndex op1;
    IntRegIndex op2;
    MiscRegIndex dest;
    uint32_t    imm;

    McrrOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
           IntRegIndex _op1, IntRegIndex _op2, MiscRegIndex _dest,
           uint32_t _imm) :
        PredOp(mnem, _machInst, __opClass), op1(_op1), op2(_op2),
        dest(_dest), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class ImmOp : public PredOp
{
  protected:
    uint64_t imm;

    ImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
             uint64_t _imm) :
        PredOp(mnem, _machInst, __opClass), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegImmOp : public PredOp
{
  protected:
    IntRegIndex dest;
    uint64_t imm;

    RegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
             IntRegIndex _dest, uint64_t _imm) :
        PredOp(mnem, _machInst, __opClass), dest(_dest), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegRegOp : public PredOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;

    RegRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
             IntRegIndex _dest, IntRegIndex _op1) :
        PredOp(mnem, _machInst, __opClass), dest(_dest), op1(_op1)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegImmRegOp : public PredOp
{
  protected:
    IntRegIndex dest;
    uint64_t imm;
    IntRegIndex op1;

    RegImmRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, uint64_t _imm, IntRegIndex _op1) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), imm(_imm), op1(_op1)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegRegRegImmOp : public PredOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    IntRegIndex op2;
    uint64_t imm;

    RegRegRegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                   uint64_t _imm) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegRegRegRegOp : public PredOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    IntRegIndex op2;
    IntRegIndex op3;

    RegRegRegRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   IntRegIndex _dest, IntRegIndex _op1,
                   IntRegIndex _op2, IntRegIndex _op3) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), op3(_op3)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegRegRegOp : public PredOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    IntRegIndex op2;

    RegRegRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegRegImmOp : public PredOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    uint64_t imm;

    RegRegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _op1,
                uint64_t _imm) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class MiscRegRegImmOp : public PredOp
{
  protected:
    MiscRegIndex dest;
    IntRegIndex op1;
    uint64_t imm;

    MiscRegRegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    MiscRegIndex _dest, IntRegIndex _op1,
                    uint64_t _imm) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegMiscRegImmOp : public PredOp
{
  protected:
    IntRegIndex dest;
    MiscRegIndex op1;
    uint64_t imm;

    RegMiscRegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    IntRegIndex _dest, MiscRegIndex _op1,
                    uint64_t _imm) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), imm(_imm)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegImmImmOp : public PredOp
{
  protected:
    IntRegIndex dest;
    uint64_t imm1;
    uint64_t imm2;

    RegImmImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, uint64_t _imm1, uint64_t _imm2) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), imm1(_imm1), imm2(_imm2)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegRegImmImmOp : public PredOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    uint64_t imm1;
    uint64_t imm2;

    RegRegImmImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   IntRegIndex _dest, IntRegIndex _op1,
                   uint64_t _imm1, uint64_t _imm2) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), imm1(_imm1), imm2(_imm2)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class RegImmRegShiftOp : public PredOp
{
  protected:
    IntRegIndex dest;
    uint64_t imm;
    IntRegIndex op1;
    int32_t shiftAmt;
    ArmShiftType shiftType;

    RegImmRegShiftOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                     IntRegIndex _dest, uint64_t _imm, IntRegIndex _op1,
                     int32_t _shiftAmt, ArmShiftType _shiftType) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), imm(_imm), op1(_op1),
        shiftAmt(_shiftAmt), shiftType(_shiftType)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class UnknownOp : public PredOp
{
  protected:

    UnknownOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        PredOp(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

#endif
