/*
 * Copyright (c) 2022 ARM Limited
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

#ifndef __ARCH_ARM_INSTS_SME_HH__
#define __ARCH_ARM_INSTS_SME_HH__

#include "arch/arm/insts/static_inst.hh"

namespace gem5
{

namespace ArmISA
{

// Used for SME ADDHA/ADDVA
class SmeAddOp : public ArmStaticInst
{
  protected:
    uint64_t imm;
    RegIndex op1;
    RegIndex gp1;
    RegIndex gp2;

    SmeAddOp(const char *mnem, ExtMachInst _machInst,
             OpClass __opClass, uint64_t _imm, RegIndex _op1,
             RegIndex _gp1, RegIndex _gp2) :
        ArmStaticInst(mnem, _machInst, __opClass),
        imm(_imm), op1(_op1), gp1(_gp1), gp2(_gp2)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

// Used for the SME ADDSPL/ADDSVL instructions
class SmeAddVlOp : public ArmStaticInst
{
  protected:
    RegIndex dest;
    RegIndex op1;
    int8_t imm;

    SmeAddVlOp(const char *mnem, ExtMachInst _machInst,
               OpClass __opClass, RegIndex _dest, RegIndex _op1,
               int8_t _imm) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

// Used for SME LD1x/ST1x instrucions
class SmeLd1xSt1xOp : public ArmStaticInst
{
  protected:
    uint64_t imm;
    RegIndex op1;
    RegIndex gp;
    RegIndex op2;
    RegIndex op3;
    bool V;

    SmeLd1xSt1xOp(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint64_t _imm, RegIndex _op1,
                    RegIndex _gp, RegIndex _op2,
                    RegIndex _op3, bool _V) :
        ArmStaticInst(mnem, _machInst, __opClass),
        imm(_imm), op1(_op1), gp(_gp), op2(_op2), op3(_op3), V(_V)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

// Used for SME LDR/STR instructions
class SmeLdrStrOp : public ArmStaticInst
{
  protected:
    uint64_t imm;
    RegIndex op1;
    RegIndex op2;

    SmeLdrStrOp(const char *mnem, ExtMachInst _machInst,
                OpClass __opClass, uint64_t _imm, RegIndex _op1,
                RegIndex _op2) :
        ArmStaticInst(mnem, _machInst, __opClass),
        imm(_imm), op1(_op1), op2(_op2)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Tile to Vector)
class SmeMovExtractOp : public ArmStaticInst
{
  protected:
    RegIndex op1;
    uint8_t imm;
    RegIndex gp;
    RegIndex op2;
    bool v;

    SmeMovExtractOp(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, RegIndex _op1, uint8_t _imm,
                    RegIndex _gp, RegIndex _op2, bool _v) :
        ArmStaticInst(mnem, _machInst, __opClass),
        op1(_op1), imm(_imm), gp(_gp), op2(_op2), v(_v)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Vector to Tile)
class SmeMovInsertOp : public ArmStaticInst
{
  protected:
    uint8_t imm;
    RegIndex op1;
    RegIndex gp;
    RegIndex op2;
    bool v;

    SmeMovInsertOp(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint8_t _imm, RegIndex _op1,
                    RegIndex _gp, RegIndex _op2, bool _v) :
        ArmStaticInst(mnem, _machInst, __opClass),
        imm(_imm), op1(_op1), gp(_gp), op2(_op2), v(_v)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

// Used for SME output product instructions
class SmeOPOp : public ArmStaticInst
{
  protected:
    uint64_t imm;
    RegIndex op1;
    RegIndex gp1;
    RegIndex gp2;
    RegIndex op2;

    SmeOPOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
            uint64_t _imm, RegIndex _op1, RegIndex _gp1,
            RegIndex _gp2, RegIndex _op2) :
        ArmStaticInst(mnem, _machInst, __opClass),
        imm(_imm), op1(_op1), gp1(_gp1), gp2(_gp2), op2(_op2)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

// Used for the SME RDSVL instruction
class SmeRdsvlOp : public ArmStaticInst
{
  protected:
    RegIndex dest;
    int8_t imm;

    SmeRdsvlOp(const char *mnem, ExtMachInst _machInst,
               OpClass __opClass, RegIndex _dest, int8_t _imm) :
        ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

// Used for SME ZERO
class SmeZeroOp : public ArmStaticInst
{
  protected:
    uint8_t imm;

    SmeZeroOp(const char *mnem, ExtMachInst _machInst,
                OpClass __opClass, uint8_t _imm) :
        ArmStaticInst(mnem, _machInst, __opClass),
        imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

} // namespace ArmISA
} // namespace gem5

#endif  // __ARCH_ARM_INSTS_SME_HH__
