/*
 * Copyright (c) 2022, 2025-2026 ARM Limited
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
class SmeAddOp : public ArmSmeStaticInst
{
  protected:
    uint64_t imm;
    RegIndex op1;
    RegIndex gp1;
    RegIndex gp2;

    SmeAddOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
             uint64_t _imm, RegIndex _op1, RegIndex _gp1, RegIndex _gp2)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          imm(_imm),
          op1(_op1),
          gp1(_gp1),
          gp2(_gp2)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

// Used for the SME ADDSPL/ADDSVL instructions
class SmeAddVlOp : public ArmSmeStaticInst
{
  protected:
    RegIndex dest;
    RegIndex op1;
    uint64_t imm;

    SmeAddVlOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
               RegIndex _dest, RegIndex _op1, uint64_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

// Used for SME LD1x/ST1x instrucions
class SmeLd1xSt1xOp : public ArmSmeStaticInst
{
  protected:
    uint64_t zad;
    uint64_t imm;
    RegIndex op1;
    RegIndex gp;
    RegIndex op2;
    RegIndex op3;
    bool V;

    SmeLd1xSt1xOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                  uint64_t _zad, uint64_t _imm, RegIndex _op1, RegIndex _gp,
                  RegIndex _op2, RegIndex _op3, bool _V)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          zad(_zad),
          imm(_imm),
          op1(_op1),
          gp(_gp),
          op2(_op2),
          op3(_op3),
          V(_V)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

// Used for SME LDR/STR instructions
class SmeLdrStrOp : public ArmSmeStaticInst
{
  protected:
    uint64_t imm;
    RegIndex op1;
    RegIndex op2;

    SmeLdrStrOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                uint64_t _imm, RegIndex _op1, RegIndex _op2)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          imm(_imm),
          op1(_op1),
          op2(_op2)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

// Used for SME LDR ZT0 instructions
class SmeLdrStrTableOp : public ArmSmeStaticInst
{
  protected:
    RegIndex op1;

    SmeLdrStrTableOp(const char *mnem, ExtMachInst _machInst,
                     OpClass __opClass, RegIndex _op1)
        : ArmSmeStaticInst(mnem, _machInst, __opClass), op1(_op1)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME ZERO instructions
class SmeZeroArrayOp : public ArmSmeStaticInst
{
  protected:
    RegIndex index;
    uint8_t imm;

    SmeZeroArrayOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _index, uint8_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          index(_index),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Tile to Vector)
class SmeMovExtractOp : public ArmSmeStaticInst
{
  protected:
    RegIndex op1;
    uint8_t zan;
    uint8_t imm;
    RegIndex gp;
    RegIndex op2;
    bool v;

    SmeMovExtractOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _op1, uint8_t _zan, uint8_t _imm, RegIndex _gp,
                    RegIndex _op2, bool _v)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          op1(_op1),
          zan(_zan),
          imm(_imm),
          gp(_gp),
          op2(_op2),
          v(_v)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Vector to Tile)
class SmeMovInsertOp : public ArmSmeStaticInst
{
  protected:
    uint8_t zad;
    uint8_t imm;
    RegIndex op1;
    RegIndex gp;
    RegIndex op2;
    bool v;

    SmeMovInsertOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   uint8_t _zad, uint8_t _imm, RegIndex _op1, RegIndex _gp,
                   RegIndex _op2, bool _v)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          zad(_zad),
          imm(_imm),
          op1(_op1),
          gp(_gp),
          op2(_op2),
          v(_v)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Tile to Vector)
class SmeMovExtract1RegOp : public ArmSmeStaticInst
{
  protected:
    uint8_t zan;
    RegIndex index;
    uint8_t imm;
    RegIndex dest1;
    bool v;

    SmeMovExtract1RegOp(const char *mnem, ExtMachInst _machInst,
                        OpClass __opClass, uint8_t _zan, RegIndex _index,
                        uint8_t _imm, RegIndex _dest1, bool _v)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          zan(_zan),
          index(_index),
          imm(_imm),
          dest1(_dest1),
          v(_v)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Tile to Vector)
class SmeMovExtract2RegOp : public ArmSmeStaticInst
{
  protected:
    uint8_t zan;
    RegIndex index;
    uint8_t imm;
    RegIndex dest1;
    RegIndex dest2;
    bool v;

    SmeMovExtract2RegOp(const char *mnem, ExtMachInst _machInst,
                        OpClass __opClass, uint8_t _zan, RegIndex _index,
                        uint8_t _imm, RegIndex _dest1, RegIndex _dest2,
                        bool _v)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          zan(_zan),
          index(_index),
          imm(_imm),
          dest1(_dest1),
          dest2(_dest2),
          v(_v)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Vector to Tile)
class SmeMovInsert2RegOp : public ArmSmeStaticInst
{
  protected:
    uint8_t zad;
    RegIndex index;
    uint8_t imm;
    RegIndex op1;
    RegIndex op2;
    bool v;

    SmeMovInsert2RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, uint8_t _zad, RegIndex _index,
                       uint8_t _imm, RegIndex _op1, RegIndex _op2, bool _v)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          zad(_zad),
          index(_index),
          imm(_imm),
          op1(_op1),
          op2(_op2),
          v(_v)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Tile to Vector)
class SmeMovExtract4RegOp : public ArmSmeStaticInst
{
  protected:
    uint8_t zan;
    RegIndex index;
    uint8_t imm;
    RegIndex dest1;
    RegIndex dest2;
    RegIndex dest3;
    RegIndex dest4;
    bool v;

    SmeMovExtract4RegOp(const char *mnem, ExtMachInst _machInst,
                        OpClass __opClass, uint8_t _zan, RegIndex _index,
                        uint8_t _imm, RegIndex _dest1, RegIndex _dest2,
                        RegIndex _dest3, RegIndex _dest4, bool _v)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          zan(_zan),
          index(_index),
          imm(_imm),
          dest1(_dest1),
          dest2(_dest2),
          dest3(_dest3),
          dest4(_dest4),
          v(_v)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Vector to Tile)
class SmeMovInsert4RegOp : public ArmSmeStaticInst
{
  protected:
    uint8_t zad;
    RegIndex index;
    uint8_t imm;
    RegIndex op1;
    RegIndex op2;
    RegIndex op3;
    RegIndex op4;
    bool v;

    SmeMovInsert4RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, uint8_t _zad, RegIndex _index,
                       uint8_t _imm, RegIndex _op1, RegIndex _op2,
                       RegIndex _op3, RegIndex _op4, bool _v)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          zad(_zad),
          index(_index),
          imm(_imm),
          op1(_op1),
          op2(_op2),
          op3(_op3),
          op4(_op4),
          v(_v)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Array to Vector)
class SmeMovArrayExtract2RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex index;
    uint8_t imm;
    RegIndex dest1;
    RegIndex dest2;

    SmeMovArrayExtract2RegOp(const char *mnem, ExtMachInst _machInst,
                             OpClass __opClass, RegIndex _index, uint8_t _imm,
                             RegIndex _dest1, RegIndex _dest2)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          index(_index),
          imm(_imm),
          dest1(_dest1),
          dest2(_dest2)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Array to Vector)
class SmeMovArrayExtract4RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex index;
    uint8_t imm;
    RegIndex dest1;
    RegIndex dest2;
    RegIndex dest3;
    RegIndex dest4;

    SmeMovArrayExtract4RegOp(const char *mnem, ExtMachInst _machInst,
                             OpClass __opClass, RegIndex _index, uint8_t _imm,
                             RegIndex _dest1, RegIndex _dest2, RegIndex _dest3,
                             RegIndex _dest4)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          index(_index),
          imm(_imm),
          dest1(_dest1),
          dest2(_dest2),
          dest3(_dest3),
          dest4(_dest4)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Vector to Array)
class SmeMovArrayInsert2RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex index;
    uint8_t imm;
    RegIndex op1;
    RegIndex op2;

    SmeMovArrayInsert2RegOp(const char *mnem, ExtMachInst _machInst,
                            OpClass __opClass, RegIndex _index, uint8_t _imm,
                            RegIndex _op1, RegIndex _op2)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          index(_index),
          imm(_imm),
          op1(_op1),
          op2(_op2)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME MOVA (Vector to Array)
class SmeMovArrayInsert4RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex index;
    uint8_t imm;
    RegIndex op1;
    RegIndex op2;
    RegIndex op3;
    RegIndex op4;

    SmeMovArrayInsert4RegOp(const char *mnem, ExtMachInst _machInst,
                            OpClass __opClass, RegIndex _index, uint8_t _imm,
                            RegIndex _op1, RegIndex _op2, RegIndex _op3,
                            RegIndex _op4)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          index(_index),
          imm(_imm),
          op1(_op1),
          op2(_op2),
          op3(_op3),
          op4(_op4)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

// Used for SME output product instructions
class SmeOPOp : public ArmSmeStaticInst
{
  protected:
    uint64_t imm;
    RegIndex op1;
    RegIndex gp1;
    RegIndex gp2;
    RegIndex op2;

    SmeOPOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
            uint64_t _imm, RegIndex _op1, RegIndex _gp1, RegIndex _gp2,
            RegIndex _op2)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          imm(_imm),
          op1(_op1),
          gp1(_gp1),
          gp2(_gp2),
          op2(_op2)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

// Used for the SME RDSVL instruction
class SmeRdsvlOp : public ArmSmeStaticInst
{
  protected:
    RegIndex dest;
    uint64_t imm;

    SmeRdsvlOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
               RegIndex _dest, uint64_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass), dest(_dest), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

// Used for SME ZERO
class SmeZeroOp : public ArmSmeStaticInst
{
  protected:
    uint8_t imm;

    SmeZeroOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              uint8_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class Sme2Vector1x2Op : public ArmSmeStaticInst
{
  protected:
    RegIndex dest;
    RegIndex op1;
    RegIndex op2;
    bool destr;

    Sme2Vector1x2Op(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest, RegIndex _op1, RegIndex _op2, bool _destr)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          op2(_op2),
          destr(_destr)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class Sme2Vector1x4Op : public ArmSmeStaticInst
{
  protected:
    RegIndex dest;
    RegIndex op1;
    RegIndex op2;
    RegIndex op3;
    RegIndex op4;
    bool destr;

    Sme2Vector1x4Op(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest, RegIndex _op1, RegIndex _op2,
                    RegIndex _op3, RegIndex _op4, bool _destr)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          op2(_op2),
          op3(_op3),
          op4(_op4),
          destr(_destr)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class Sme2Vector2x1Op : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex dest2;
    RegIndex op1;
    bool destr;

    Sme2Vector2x1Op(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest1, RegIndex _dest2, RegIndex _op1,
                    bool _destr)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          dest2(_dest2),
          op1(_op1),
          destr(_destr)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class Sme2Vector4x1Op : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex dest2;
    RegIndex dest3;
    RegIndex dest4;
    RegIndex op1;
    bool destr;

    Sme2Vector4x1Op(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest1, RegIndex _dest2, RegIndex _dest3,
                    RegIndex _dest4, RegIndex _op1, bool _destr)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          dest2(_dest2),
          dest3(_dest3),
          dest4(_dest4),
          op1(_op1),
          destr(_destr)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 ZIP, UZP, SCVFT, UCVTF instructions with 2 registers
class Sme2Vector2x2Op : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex dest2;
    RegIndex op1;
    RegIndex op2;
    bool destr;

    Sme2Vector2x2Op(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest1, RegIndex _dest2, RegIndex _op1,
                    RegIndex _op2, bool _destr)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          dest2(_dest2),
          op1(_op1),
          op2(_op2),
          destr(_destr)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 SUNPK/UUNPK with 4 registers.
class Sme2Vector4x2Op : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex dest2;
    RegIndex dest3;
    RegIndex dest4;
    RegIndex op1;
    RegIndex op2;
    bool destr;

    Sme2Vector4x2Op(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest1, RegIndex _dest2, RegIndex _dest3,
                    RegIndex _dest4, RegIndex _op1, RegIndex _op2, bool _destr)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          dest2(_dest2),
          dest3(_dest3),
          dest4(_dest4),
          op1(_op1),
          op2(_op2),
          destr(_destr)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 ZIP, UZP, SCVFT, UCVTF instructions with 4 registers
class Sme2Vector4x4Op : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex dest2;
    RegIndex dest3;
    RegIndex dest4;
    RegIndex op1;
    RegIndex op2;
    RegIndex op3;
    RegIndex op4;
    bool destr;

    Sme2Vector4x4Op(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest1, RegIndex _dest2, RegIndex _dest3,
                    RegIndex _dest4, RegIndex _op1, RegIndex _op2,
                    RegIndex _op3, RegIndex _op4, bool _destr)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          dest2(_dest2),
          dest3(_dest3),
          dest4(_dest4),
          op1(_op1),
          op2(_op2),
          op3(_op3),
          op4(_op4),
          destr(_destr)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 1 reg instructions
class Sme2MultiSgl1RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex index;
    uint8_t imm;
    RegIndex op1;
    // Skipping to op5 to make this more easily compatible with the derrived
    // class.
    RegIndex op5;

    Sme2MultiSgl1RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _index, uint8_t _imm,
                       RegIndex _op1, RegIndex _op5)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          index(_index),
          imm(_imm),
          op1(_op1),
          op5(_op5)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 single 2 reg instructions
class Sme2MultiSgl2RegOp : public Sme2MultiSgl1RegOp
{
  protected:
    RegIndex op2;

    Sme2MultiSgl2RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _index, uint8_t _imm,
                       RegIndex _op1, RegIndex _op2, RegIndex _op5)
        : Sme2MultiSgl1RegOp(mnem, _machInst, __opClass, _index, _imm, _op1,
                             _op5),
          op2(_op2)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 single 4 reg instructions
class Sme2MultiSgl4RegOp : public Sme2MultiSgl2RegOp
{
  protected:
    RegIndex op3;
    RegIndex op4;

    Sme2MultiSgl4RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _index, uint8_t _imm,
                       RegIndex _op1, RegIndex _op2, RegIndex _op3,
                       RegIndex _op4, RegIndex _op5)
        : Sme2MultiSgl2RegOp(mnem, _machInst, __opClass, _index, _imm, _op1,
                             _op2, _op5),
          op3(_op3),
          op4(_op4)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 multi-vector 2 reg instructions
class Sme2MultiVec2RegOp : public Sme2MultiSgl2RegOp
{
  protected:
    RegIndex op6;

    Sme2MultiVec2RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _index, uint8_t _imm,
                       RegIndex _op1, RegIndex _op2, RegIndex _op5,
                       RegIndex _op6)
        : Sme2MultiSgl2RegOp(mnem, _machInst, __opClass, _index, _imm, _op1,
                             _op2, _op5),
          op6(_op6)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 multi-vector 4 reg instructions
class Sme2MultiVec4RegOp : public Sme2MultiVec2RegOp
{
  protected:
    RegIndex op3;
    RegIndex op4;
    RegIndex op7;
    RegIndex op8;

    Sme2MultiVec4RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _index, uint8_t _imm,
                       RegIndex _op1, RegIndex _op2, RegIndex _op3,
                       RegIndex _op4, RegIndex _op5, RegIndex _op6,
                       RegIndex _op7, RegIndex _op8)
        : Sme2MultiVec2RegOp(mnem, _machInst, __opClass, _index, _imm, _op1,
                             _op2, _op5, _op6),
          op3(_op3),
          op4(_op4),
          op7(_op7),
          op8(_op8)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 multi-index 1 reg instructions
class Sme2MultiIdx1RegOp : public Sme2MultiSgl1RegOp
{
  protected:
    uint8_t idx;

    Sme2MultiIdx1RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _index, uint8_t _imm,
                       RegIndex _op1, RegIndex _op5, uint8_t _idx)
        : Sme2MultiSgl1RegOp(mnem, _machInst, __opClass, _index, _imm, _op1,
                             _op5),
          idx(_idx)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 multi-index 2 reg instructions
class Sme2MultiIdx2RegOp : public Sme2MultiIdx1RegOp
{
  protected:
    RegIndex op2;

    Sme2MultiIdx2RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _index, uint8_t _imm,
                       RegIndex _op1, RegIndex _op2, RegIndex _op5,
                       uint8_t _idx)
        : Sme2MultiIdx1RegOp(mnem, _machInst, __opClass, _index, _imm, _op1,
                             _op5, _idx),
          op2(_op2)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 multi-index 4 reg instructions
class Sme2MultiIdx4RegOp : public Sme2MultiIdx2RegOp
{
  protected:
    RegIndex op3;
    RegIndex op4;

    Sme2MultiIdx4RegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _index, uint8_t _imm,
                       RegIndex _op1, RegIndex _op2, RegIndex _op3,
                       RegIndex _op4, RegIndex _op5, uint8_t _idx)
        : Sme2MultiIdx2RegOp(mnem, _machInst, __opClass, _index, _imm, _op1,
                             _op2, _op5, _idx),
          op3(_op3),
          op4(_op4)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for the SME2 ZLUT instruction
class Sme2ZlutOp : public ArmSmeStaticInst
{
  protected:
    Sme2ZlutOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : ArmSmeStaticInst(mnem, _machInst, __opClass)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// TODO
class Sme2ZaZ2RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex index;
    uint8_t imm;
    RegIndex op1;
    RegIndex op2;

    Sme2ZaZ2RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                  RegIndex _index, uint8_t _imm, RegIndex _op1, RegIndex _op2)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          index(_index),
          imm(_imm),
          op1(_op1),
          op2(_op2)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// TODO
class Sme2ZaZ4RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex index;
    uint8_t imm;
    RegIndex op1;
    RegIndex op2;
    RegIndex op3;
    RegIndex op4;

    Sme2ZaZ4RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                  RegIndex _index, uint8_t _imm, RegIndex _op1, RegIndex _op2,
                  RegIndex _op3, RegIndex _op4)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          index(_index),
          imm(_imm),
          op1(_op1),
          op2(_op2),
          op3(_op3),
          op4(_op4)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 clamp instructions
class Sme2Clamp2RegOp : public Sme2Vector2x2Op
{
  protected:
    Sme2Clamp2RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest1, RegIndex _dest2, RegIndex _op1,
                    RegIndex _op2)
        : Sme2Vector2x2Op(mnem, _machInst, __opClass, _dest1, _dest2, _op1,
                          _op2, false)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 clamp instructions
class Sme2Clamp4RegOp : public Sme2Vector4x2Op
{
  protected:
    Sme2Clamp4RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    RegIndex _dest1, RegIndex _dest2, RegIndex _dest3,
                    RegIndex _dest4, RegIndex _op1, RegIndex _op2)
        : Sme2Vector4x2Op(mnem, _machInst, __opClass, _dest1, _dest2, _dest3,
                          _dest4, _op1, _op2, false)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 2 reg saturating right shift
class Sme2Rshr2RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex dest;
    RegIndex op1;
    RegIndex op2;
    uint8_t imm;

    Sme2Rshr2RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest, RegIndex _op1, RegIndex _op2, uint8_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          op2(_op2),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 4 reg saturating right shift
class Sme2Rshr4RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex dest;
    RegIndex op1;
    RegIndex op2;
    RegIndex op3;
    RegIndex op4;
    uint8_t imm;

    Sme2Rshr4RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest, RegIndex _op1, RegIndex _op2, RegIndex _op3,
                   RegIndex _op4, uint8_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          op2(_op2),
          op3(_op3),
          op4(_op4),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 SEL 2 register
class Sme2SSel2RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex dest2;
    RegIndex gp;
    RegIndex op1;
    RegIndex op2;
    RegIndex op5;
    RegIndex op6;

    Sme2SSel2RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest1, RegIndex _dest2, RegIndex _gp,
                   RegIndex _op1, RegIndex _op2, RegIndex _op5, RegIndex _op6)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          dest2(_dest2),
          gp(_gp),
          op1(_op1),
          op2(_op2),
          op5(_op5),
          op6(_op6)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 SEL 4 register
class Sme2SSel4RegOp : public Sme2SSel2RegOp
{
  protected:
    RegIndex dest3;
    RegIndex dest4;
    RegIndex op3;
    RegIndex op4;
    RegIndex op7;
    RegIndex op8;

    Sme2SSel4RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest1, RegIndex _dest2, RegIndex _dest3,
                   RegIndex _dest4, RegIndex _gp, RegIndex _op1, RegIndex _op2,
                   RegIndex _op3, RegIndex _op4, RegIndex _op5, RegIndex _op6,
                   RegIndex _op7, RegIndex _op8)
        : Sme2SSel2RegOp(mnem, _machInst, __opClass, _dest1, _dest2, _gp, _op1,
                         _op2, _op5, _op6),
          dest3(_dest3),
          dest4(_dest4),
          op3(_op3),
          op4(_op4),
          op7(_op7),
          op8(_op8)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 LUTI 1 register
class Sme2Luti1RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex op1;
    uint64_t imm;

    Sme2Luti1RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest1, RegIndex _op1, uint64_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          op1(_op1),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 LUTI 2 register
class Sme2Luti2RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex dest2;
    RegIndex op1;
    uint64_t imm;

    Sme2Luti2RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest1, RegIndex _dest2, RegIndex _op1,
                   uint64_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          dest2(_dest2),
          op1(_op1),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 LUTI 4 register
class Sme2Luti4RegOp : public ArmSmeStaticInst
{
  protected:
    RegIndex dest1;
    RegIndex dest2;
    RegIndex dest3;
    RegIndex dest4;
    RegIndex op1;
    uint64_t imm;

    Sme2Luti4RegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest1, RegIndex _dest2, RegIndex _dest3,
                   RegIndex _dest4, RegIndex _op1, uint64_t _imm)
        : ArmSmeStaticInst(mnem, _machInst, __opClass),
          dest1(_dest1),
          dest2(_dest2),
          dest3(_dest3),
          dest4(_dest4),
          op1(_op1),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

// Used for SME2 LUTI4 (four registers, 8-bit)
class Sme2Luti4to8b4RegOp : public Sme2Vector4x2Op
{
  protected:
    Sme2Luti4to8b4RegOp(const char *mnem, ExtMachInst _machInst,
                        OpClass __opClass, RegIndex _dest1, RegIndex _dest2,
                        RegIndex _dest3, RegIndex _dest4, RegIndex _op1,
                        RegIndex _op2)
        : Sme2Vector4x2Op(mnem, _machInst, __opClass, _dest1, _dest2, _dest3,
                          _dest4, _op1, _op2, false)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

} // namespace ArmISA
} // namespace gem5

#endif  // __ARCH_ARM_INSTS_SME_HH__
