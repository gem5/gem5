/*
 * Copyright (c) 2011-2013 ARM Limited
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
 */

#ifndef __ARCH_ARM_INSTS_DATA64_HH__
#define __ARCH_ARM_INSTS_DATA64_HH__

#include "arch/arm/insts/static_inst.hh"
#include "base/trace.hh"

namespace ArmISA
{

class DataXImmOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1;
    uint64_t imm;

    DataXImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
               IntRegIndex _dest, IntRegIndex _op1, uint64_t _imm) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataXImmOnlyOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest;
    uint64_t imm;

    DataXImmOnlyOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   IntRegIndex _dest, uint64_t _imm) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataXSRegOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1, op2;
    int32_t shiftAmt;
    ArmShiftType shiftType;

    DataXSRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                int32_t _shiftAmt, ArmShiftType _shiftType) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2),
        shiftAmt(_shiftAmt), shiftType(_shiftType)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataXERegOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1, op2;
    ArmExtendType extendType;
    int32_t shiftAmt;

    DataXERegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                ArmExtendType _extendType, int32_t _shiftAmt) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2),
        extendType(_extendType), shiftAmt(_shiftAmt)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataX1RegOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1;

    DataX1RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _op1) :
        ArmStaticInst(mnem, _machInst, __opClass), dest(_dest), op1(_op1)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataX1RegImmOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1;
    uint64_t imm;

    DataX1RegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   IntRegIndex _dest, IntRegIndex _op1, uint64_t _imm) :
        ArmStaticInst(mnem, _machInst, __opClass), dest(_dest), op1(_op1),
        imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataX1Reg2ImmOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1;
    uint64_t imm1, imm2;

    DataX1Reg2ImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    IntRegIndex _dest, IntRegIndex _op1, uint64_t _imm1,
                    uint64_t _imm2) :
        ArmStaticInst(mnem, _machInst, __opClass), dest(_dest), op1(_op1),
        imm1(_imm1), imm2(_imm2)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataX2RegOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1, op2;

    DataX2RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataX2RegImmOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1, op2;
    uint64_t imm;

    DataX2RegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                   uint64_t _imm) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataX3RegOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1, op2, op3;

    DataX3RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                IntRegIndex _op3) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), op3(_op3)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataXCondCompImmOp : public ArmStaticInst
{
  protected:
    IntRegIndex op1;
    uint64_t imm;
    ConditionCode condCode;
    uint8_t defCc;

    DataXCondCompImmOp(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, IntRegIndex _op1, uint64_t _imm,
                      ConditionCode _condCode, uint8_t _defCc) :
        ArmStaticInst(mnem, _machInst, __opClass),
        op1(_op1), imm(_imm), condCode(_condCode), defCc(_defCc)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataXCondCompRegOp : public ArmStaticInst
{
  protected:
    IntRegIndex op1, op2;
    ConditionCode condCode;
    uint8_t defCc;

    DataXCondCompRegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, IntRegIndex _op1, IntRegIndex _op2,
                       ConditionCode _condCode, uint8_t _defCc) :
        ArmStaticInst(mnem, _machInst, __opClass),
        op1(_op1), op2(_op2), condCode(_condCode), defCc(_defCc)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

class DataXCondSelOp : public ArmStaticInst
{
  protected:
    IntRegIndex dest, op1, op2;
    ConditionCode condCode;

    DataXCondSelOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                   ConditionCode _condCode) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), condCode(_condCode)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

}

#endif //__ARCH_ARM_INSTS_PREDINST_HH__
