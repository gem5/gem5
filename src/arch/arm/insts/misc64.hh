/*
 * Copyright (c) 2011-2013,2017-2019, 2021-2022, 2024 Arm Limited
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

#ifndef __ARCH_ARM_INSTS_MISC64_HH__
#define __ARCH_ARM_INSTS_MISC64_HH__

#include "arch/arm/insts/static_inst.hh"
#include "arch/arm/types.hh"

namespace gem5
{

class ImmOp64 : public ArmISA::ArmStaticInst
{
  protected:
    uint64_t imm;

    ImmOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
            OpClass __opClass, uint64_t _imm) :
        ArmISA::ArmStaticInst(mnem, _machInst, __opClass), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class RegOp64 : public ArmISA::ArmStaticInst
{
  protected:
    RegIndex op1;

    RegOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
            OpClass __opClass, RegIndex _op1) :
        ArmISA::ArmStaticInst(mnem, _machInst, __opClass), op1(_op1)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class RegImmImmOp64 : public ArmISA::ArmStaticInst
{
  protected:
    RegIndex op1;
    uint64_t imm1;
    uint64_t imm2;

    RegImmImmOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
                  OpClass __opClass, RegIndex _op1,
                  uint64_t _imm1, uint64_t _imm2) :
        ArmISA::ArmStaticInst(mnem, _machInst, __opClass),
        op1(_op1), imm1(_imm1), imm2(_imm2)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class RegRegImmImmOp64 : public ArmISA::ArmStaticInst
{
  protected:
    RegIndex dest;
    RegIndex op1;
    uint64_t imm1;
    uint64_t imm2;

    RegRegImmImmOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
                     OpClass __opClass, RegIndex _dest,
                     RegIndex _op1, uint64_t _imm1,
                     int64_t _imm2) :
        ArmISA::ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), imm1(_imm1), imm2(_imm2)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class RegRegRegImmOp64 : public ArmISA::ArmStaticInst
{
  protected:
    RegIndex dest;
    RegIndex op1;
    RegIndex op2;
    uint64_t imm;

    RegRegRegImmOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
                     OpClass __opClass, RegIndex _dest,
                     RegIndex _op1, RegIndex _op2,
                     uint64_t _imm) :
        ArmISA::ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class UnknownOp64 : public ArmISA::ArmStaticInst
{
  protected:

    UnknownOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
            OpClass __opClass) :
        ArmISA::ArmStaticInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

/**
 * This class is implementing the Base class for a generic AArch64
 * instruction which is making use of system registers (MiscReg), like
 * MSR,MRS,SYS.  The common denominator or those instruction is the
 * chance that the system register access is trapped to an upper
 * Exception level. MiscRegOp64 is providing that feature.  Other
 * "pseudo" instructions, like access to implementation defined
 * registers can inherit from this class to make use of the trapping
 * functionalities even if there is no data movement between GPRs and
 * system register.
 */
class MiscRegOp64 : public ArmISA::ArmStaticInst
{
  protected:
    bool _miscRead;

    MiscRegOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
                OpClass __opClass, bool misc_read) :
        ArmISA::ArmStaticInst(mnem, _machInst, __opClass),
        _miscRead(misc_read)
    {}

    uint32_t _iss(const ArmISA::MiscRegNum64 &misc_reg,
            RegIndex int_index) const;

  public:
    virtual uint32_t iss() const { return 0; }

    bool miscRead() const { return _miscRead; }

    Fault generateTrap(ArmISA::ExceptionLevel el) const;
    Fault generateTrap(ArmISA::ExceptionLevel el,
            ArmISA::ExceptionClass ec, uint32_t iss) const;
};

class MiscRegImmOp64 : public MiscRegOp64
{
  protected:
    ArmISA::MiscRegIndex dest;
    uint32_t imm;

    MiscRegImmOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
                   OpClass __opClass, ArmISA::MiscRegIndex _dest,
                   uint32_t _imm) :
        MiscRegOp64(mnem, _machInst, __opClass, false),
        dest(_dest), imm(_imm)
    {}

    /** Returns the "register view" of the immediate field.
     * as if it was a MSR PSTATE REG instruction.
     * This means basically shifting and masking depending on
     * which PSTATE field is being set/cleared.
     */
    RegVal miscRegImm() const;

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class MiscRegRegImmOp64 : public MiscRegOp64
{
  protected:
    ArmISA::MiscRegIndex dest;
    RegIndex op1;

    MiscRegRegImmOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
                      OpClass __opClass, ArmISA::MiscRegIndex _dest,
                      RegIndex _op1) :
        MiscRegOp64(mnem, _machInst, __opClass, false),
        dest(_dest), op1(_op1)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;

    uint32_t iss() const override;
};

class RegMiscRegImmOp64 : public MiscRegOp64
{
  protected:
    RegIndex dest;
    ArmISA::MiscRegIndex op1;

    RegMiscRegImmOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
                      OpClass __opClass, RegIndex _dest,
                      ArmISA::MiscRegIndex _op1) :
        MiscRegOp64(mnem, _machInst, __opClass, true),
        dest(_dest), op1(_op1)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;

    uint32_t iss() const override;
};

class MiscRegImplDefined64 : public MiscRegOp64
{
  protected:
    const std::string fullMnemonic;
    const ArmISA::MiscRegNum64 miscReg;
    const RegIndex intReg;

  public:
    MiscRegImplDefined64(const char *mnem, ArmISA::ExtMachInst _machInst,
                         ArmISA::MiscRegNum64 &&misc_reg, RegIndex int_reg,
                         bool misc_read, const std::string full_mnem) :
        MiscRegOp64(mnem, _machInst, No_OpClass, misc_read),
        fullMnemonic(full_mnem), miscReg(misc_reg), intReg(int_reg)
    {
        assert(decodeAArch64SysReg(miscReg) == ArmISA::MISCREG_IMPDEF_UNIMPL);
    }

  protected:
    Fault execute(ExecContext *xc,
                  trace::InstRecord *traceData) const override;

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;

    uint32_t iss() const override;
};

class RegNone : public ArmISA::ArmStaticInst
{
  protected:
    RegIndex dest;

    RegNone(const char *mnem, ArmISA::ExtMachInst _machInst,
            OpClass __opClass, RegIndex _dest) :
        ArmISA::ArmStaticInst(mnem, _machInst, __opClass),
        dest(_dest)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const;
};

class TlbiOp64 : public MiscRegRegImmOp64
{
  protected:
    using TlbiFunc = std::function<void(ThreadContext*,RegVal)>;

    static std::unordered_map<ArmISA::MiscRegIndex, TlbiFunc> tlbiOps;

    static void tlbiAll(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable);

    static void tlbiVmall(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable,
        bool stage2=false);

    static void tlbiVa(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable,
        bool last_level);

    static void tlbiVaa(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable,
        bool last_level);

    static void tlbiAsid(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable);

    static void tlbiIpaS2(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable,
        bool last_level);

    static void tlbiRvaa(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable,
        bool last_level);

    static void tlbiRva(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable,
        bool last_level);

    static void tlbiRipaS2(ThreadContext *tc, RegVal value,
        bool secure, ArmISA::TranslationRegime regime, bool shareable,
        bool last_level);

  protected:
    TlbiOp64(const char *mnem, ArmISA::ExtMachInst _machInst,
             OpClass __opClass, ArmISA::MiscRegIndex _dest,
             RegIndex _op1) :
        MiscRegRegImmOp64(mnem, _machInst, __opClass, _dest, _op1)
    {}

    void performTlbi(ExecContext *xc,
                     ArmISA::MiscRegIndex idx, RegVal value) const;
};

} // namespace gem5

#endif
