/*
 * Copyright (c) 2010, 2012-2013, 2017-2018, 2021 Arm Limited
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

#ifndef __ARCH_ARM_INSTS_MISC_HH__
#define __ARCH_ARM_INSTS_MISC_HH__

#include "arch/arm/insts/pred_inst.hh"

namespace gem5
{

class MrsOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;

    MrsOp(const char *mnem, ArmISA::ExtMachInst _machInst, OpClass __opClass,
          RegIndex _dest)
        : ArmISA::PredOp(mnem, _machInst, __opClass), dest(_dest)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class MsrBase : public ArmISA::PredOp
{
  protected:
    uint8_t byteMask;

    MsrBase(const char *mnem, ArmISA::ExtMachInst _machInst, OpClass __opClass,
            uint8_t _byteMask)
        : ArmISA::PredOp(mnem, _machInst, __opClass), byteMask(_byteMask)
    {}

    void printMsrBase(std::ostream &os) const;
};

class MsrImmOp : public MsrBase
{
  protected:
    uint32_t imm;

    MsrImmOp(const char *mnem, ArmISA::ExtMachInst _machInst,
             OpClass __opClass, uint32_t _imm, uint8_t _byteMask)
        : MsrBase(mnem, _machInst, __opClass, _byteMask), imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class MsrRegOp : public MsrBase
{
  protected:
    RegIndex op1;

    MsrRegOp(const char *mnem, ArmISA::ExtMachInst _machInst,
             OpClass __opClass, RegIndex _op1, uint8_t _byteMask)
        : MsrBase(mnem, _machInst, __opClass, _byteMask), op1(_op1)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class MrrcOp : public ArmISA::PredOp
{
  protected:
    ArmISA::MiscRegIndex op1;
    RegIndex dest;
    RegIndex dest2;
    uint32_t imm;

    MrrcOp(const char *mnem, ArmISA::ExtMachInst _machInst, OpClass __opClass,
           ArmISA::MiscRegIndex _op1, RegIndex _dest, RegIndex _dest2,
           uint32_t _imm)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          op1(_op1),
          dest(_dest),
          dest2(_dest2),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class McrrOp : public ArmISA::PredOp
{
  protected:
    RegIndex op1;
    RegIndex op2;
    ArmISA::MiscRegIndex dest;
    uint32_t imm;

    McrrOp(const char *mnem, ArmISA::ExtMachInst _machInst, OpClass __opClass,
           RegIndex _op1, RegIndex _op2, ArmISA::MiscRegIndex _dest,
           uint32_t _imm)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          op1(_op1),
          op2(_op2),
          dest(_dest),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class ImmOp : public ArmISA::PredOp
{
  protected:
    uint64_t imm;

    ImmOp(const char *mnem, ArmISA::ExtMachInst _machInst, OpClass __opClass,
          uint64_t _imm)
        : ArmISA::PredOp(mnem, _machInst, __opClass), imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegImmOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    uint64_t imm;

    RegImmOp(const char *mnem, ArmISA::ExtMachInst _machInst,
             OpClass __opClass, RegIndex _dest, uint64_t _imm)
        : ArmISA::PredOp(mnem, _machInst, __opClass), dest(_dest), imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegRegOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    RegIndex op1;

    RegRegOp(const char *mnem, ArmISA::ExtMachInst _machInst,
             OpClass __opClass, RegIndex _dest, RegIndex _op1)
        : ArmISA::PredOp(mnem, _machInst, __opClass), dest(_dest), op1(_op1)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;

    RegOp(const char *mnem, ArmISA::ExtMachInst _machInst, OpClass __opClass,
          RegIndex _dest)
        : ArmISA::PredOp(mnem, _machInst, __opClass), dest(_dest)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegImmRegOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    uint64_t imm;
    RegIndex op1;

    RegImmRegOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                OpClass __opClass, RegIndex _dest, uint64_t _imm,
                RegIndex _op1)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          imm(_imm),
          op1(_op1)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegRegRegImmOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    RegIndex op1;
    RegIndex op2;
    uint64_t imm;

    RegRegRegImmOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                   OpClass __opClass, RegIndex _dest, RegIndex _op1,
                   RegIndex _op2, uint64_t _imm)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          op2(_op2),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegRegRegRegOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    RegIndex op1;
    RegIndex op2;
    RegIndex op3;

    RegRegRegRegOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                   OpClass __opClass, RegIndex _dest, RegIndex _op1,
                   RegIndex _op2, RegIndex _op3)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          op2(_op2),
          op3(_op3)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegRegRegOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    RegIndex op1;
    RegIndex op2;

    RegRegRegOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                OpClass __opClass, RegIndex _dest, RegIndex _op1,
                RegIndex _op2)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          op2(_op2)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegRegImmOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    RegIndex op1;
    uint64_t imm;

    RegRegImmOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                OpClass __opClass, RegIndex _dest, RegIndex _op1,
                uint64_t _imm)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class MiscRegRegImmOp : public ArmISA::PredOp
{
  protected:
    ArmISA::MiscRegIndex dest;
    RegIndex op1;
    uint64_t imm;

    MiscRegRegImmOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                    OpClass __opClass, ArmISA::MiscRegIndex _dest,
                    RegIndex _op1, uint64_t _imm)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegMiscRegImmOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    ArmISA::MiscRegIndex op1;
    uint64_t imm;

    RegMiscRegImmOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                    OpClass __opClass, RegIndex _dest,
                    ArmISA::MiscRegIndex _op1, uint64_t _imm)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          imm(_imm)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegImmImmOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    uint64_t imm1;
    uint64_t imm2;

    RegImmImmOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                OpClass __opClass, RegIndex _dest, uint64_t _imm1,
                uint64_t _imm2)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          imm1(_imm1),
          imm2(_imm2)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegRegImmImmOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    RegIndex op1;
    uint64_t imm1;
    uint64_t imm2;

    RegRegImmImmOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                   OpClass __opClass, RegIndex _dest, RegIndex _op1,
                   uint64_t _imm1, uint64_t _imm2)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          op1(_op1),
          imm1(_imm1),
          imm2(_imm2)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class RegImmRegShiftOp : public ArmISA::PredOp
{
  protected:
    RegIndex dest;
    uint64_t imm;
    RegIndex op1;
    int32_t shiftAmt;
    ArmISA::ArmShiftType shiftType;

    RegImmRegShiftOp(const char *mnem, ArmISA::ExtMachInst _machInst,
                     OpClass __opClass, RegIndex _dest, uint64_t _imm,
                     RegIndex _op1, int32_t _shiftAmt,
                     ArmISA::ArmShiftType _shiftType)
        : ArmISA::PredOp(mnem, _machInst, __opClass),
          dest(_dest),
          imm(_imm),
          op1(_op1),
          shiftAmt(_shiftAmt),
          shiftType(_shiftType)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class UnknownOp : public ArmISA::PredOp
{
  protected:
    UnknownOp(const char *mnem, ArmISA::ExtMachInst _machInst,
              OpClass __opClass)
        : ArmISA::PredOp(mnem, _machInst, __opClass)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

/**
 * Certain mrc/mcr instructions act as nops or flush the pipe based on what
 * register the instruction is trying to access. This inst/class exists so that
 * we can still check for hyp traps, as the normal nop instruction
 * does not.
 */
class McrMrcMiscInst : public ArmISA::ArmStaticInst
{
  protected:
    uint64_t iss;
    ArmISA::MiscRegIndex miscReg;

  public:
    McrMrcMiscInst(const char *_mnemonic, ArmISA::ExtMachInst _machInst,
                   uint64_t _iss, ArmISA::MiscRegIndex _miscReg);

    Fault execute(ExecContext *xc,
                  trace::InstRecord *traceData) const override;

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

/**
 * This class is also used for IMPLEMENTATION DEFINED registers, whose mcr/mrc
 * behaviour is trappable even for unimplemented registers.
 */
class McrMrcImplDefined : public McrMrcMiscInst
{
  public:
    McrMrcImplDefined(const char *_mnemonic, ArmISA::ExtMachInst _machInst,
                      uint64_t _iss, ArmISA::MiscRegIndex _miscReg);

    Fault execute(ExecContext *xc,
                  trace::InstRecord *traceData) const override;

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

class TlbiOp : public MiscRegRegImmOp
{
  protected:
    TlbiOp(const char *mnem, ArmISA::ExtMachInst _machInst, OpClass __opClass,
           ArmISA::MiscRegIndex _dest, RegIndex _op1, uint64_t _imm)
        : MiscRegRegImmOp(mnem, _machInst, __opClass, _dest, _op1, _imm)
    {}

    void performTlbi(ExecContext *xc, ArmISA::MiscRegIndex dest_idx,
                     RegVal value) const;
};

} // namespace gem5

#endif
