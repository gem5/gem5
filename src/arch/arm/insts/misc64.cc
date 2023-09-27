/*
 * Copyright (c) 2011-2013,2017-2024 Arm Limited
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

#include "arch/arm/insts/misc64.hh"
#include "arch/arm/isa.hh"

#include "arch/arm/tlbi_op.hh"

namespace gem5
{

using namespace ArmISA;

std::string
ImmOp64::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#0x%x", imm);
    return ss.str();
}

std::string
RegOp64::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, op1);
    return ss.str();
}

std::string
RegImmImmOp64::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, op1);
    ccprintf(ss, "#0x%x", imm1);
    ss << ", ";
    ccprintf(ss, "#0x%x", imm2);
    return ss.str();
}

std::string
RegRegImmImmOp64::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ccprintf(ss, ", #%d, #%d", imm1, imm2);
    return ss.str();
}

std::string
RegRegRegImmOp64::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ss << ", ";
    printIntReg(ss, op2);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
UnknownOp64::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    return csprintf("%-10s (inst %#08x)", "unknown", encoding());
}

uint32_t
MiscRegOp64::_iss(const MiscRegNum64 &misc_reg, RegIndex int_index) const
{
    return _miscRead |
        (misc_reg.crm << 1) |
        (int_index << 5) |
        (misc_reg.crn << 10) |
        (misc_reg.op1 << 14) |
        (misc_reg.op2 << 17) |
        (misc_reg.op0 << 20);
}

Fault
MiscRegOp64::generateTrap(ExceptionLevel el) const
{
    return generateTrap(el, ExceptionClass::TRAPPED_MSR_MRS_64, iss());
}

RegVal
MiscRegImmOp64::miscRegImm() const
{
    switch (dest) {
      case MISCREG_SPSEL:
        return imm & 0x1;
      case MISCREG_PAN:
        return (imm & 0x1) << 22;
      case MISCREG_UAO:
        return (imm & 0x1) << 23;
      default:
        panic("Not a valid PSTATE field register\n");
    }
}

std::string
MiscRegImmOp64::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printMiscReg(ss, dest);
    ss << ", ";
    ccprintf(ss, "#0x%x", imm);
    return ss.str();
}

std::string
MiscRegRegImmOp64::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printMiscReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    return ss.str();
}

uint32_t
MiscRegRegImmOp64::iss() const
{
    const auto misc_reg = encodeAArch64SysReg(dest);
    assert(misc_reg.has_value());
    return _iss(misc_reg.value(), op1);
}

std::string
RegMiscRegImmOp64::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printMiscReg(ss, op1);
    return ss.str();
}

uint32_t
RegMiscRegImmOp64::iss() const
{
    const auto misc_reg = encodeAArch64SysReg(op1);
    assert(misc_reg.has_value());
    return _iss(misc_reg.value(), dest);
}

Fault
MiscRegImplDefined64::execute(ExecContext *xc,
                              trace::InstRecord *traceData) const
{
    auto tc = xc->tcBase();
    const CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);

    return checkFaultAccessAArch64SysReg(
        MISCREG_IMPDEF_UNIMPL, cpsr, tc, *this);
}

std::string
MiscRegImplDefined64::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    return csprintf("%-10s (implementation defined)", fullMnemonic.c_str());
}

uint32_t
MiscRegImplDefined64::iss() const
{
    return _iss(miscReg, intReg);
}

std::string
RegNone::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    return ss.str();
}

void
TlbiOp64::tlbiAll(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable,
    TlbiAttr attrs)
{
    TLBIALLEL tlbi_op(regime, ss, attrs);
    if (shareable) {
        tlbi_op.broadcast(tc);
    } else {
        tlbi_op(tc);
    }
}

void
TlbiOp64::tlbiVmall(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable,
    bool stage2, TlbiAttr attrs)
{
    TLBIVMALL tlbi_op(regime, ss, stage2, attrs);
    if (shareable) {
        tlbi_op.broadcast(tc);
    } else {
        tlbi_op(tc);
    }
}

void
TlbiOp64::tlbiVa(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable,
    bool last_level, TlbiAttr attrs)
{
    if (MMU::hasUnprivRegime(regime)) {
        // The asid will only be used when e2h == 1
        bool asid_16bits = ArmSystem::haveLargeAsid64(tc);
        auto asid = asid_16bits ? bits(value, 63, 48) :
                                  bits(value, 55, 48);

        TLBIMVA tlbi_op(regime, ss, static_cast<Addr>(bits(value, 43, 0)) << 12,
                        asid, last_level, attrs);
        if (shareable) {
            tlbi_op.broadcast(tc);
        } else {
            tlbi_op(tc);
        }
    } else {
        TLBIMVAA tlbi_op(regime, ss,
            static_cast<Addr>(bits(value, 43, 0)) << 12, last_level,
            attrs);
        if (shareable) {
            tlbi_op.broadcast(tc);
        } else {
            tlbi_op(tc);
        }
    }
}

void
TlbiOp64::tlbiVaa(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable,
    bool last_level, TlbiAttr attrs)
{
    TLBIMVAA tlbi_op(regime, ss,
        static_cast<Addr>(bits(value, 43, 0)) << 12, last_level,
        attrs);
    if (shareable) {
        tlbi_op.broadcast(tc);
    } else {
        tlbi_op(tc);
    }
}

void
TlbiOp64::tlbiAsid(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable,
    TlbiAttr attrs)
{
    bool asid_16bits = ArmSystem::haveLargeAsid64(tc);
    auto asid = asid_16bits ? bits(value, 63, 48) :
                              bits(value, 55, 48);

    TLBIASID tlbi_op(regime, ss, asid, attrs);
    if (shareable) {
        tlbi_op.broadcast(tc);
    } else {
        tlbi_op(tc);
    }
}

void
TlbiOp64::tlbiIpaS2(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable,
    bool last_level, TlbiAttr attrs)
{
    if (EL2Enabled(tc)) {
        TLBIIPA tlbi_op(tc, TranslationRegime::EL10, ss, value,
            last_level, attrs);

        if (shareable) {
            tlbi_op.broadcast(tc);
        } else {
            tlbi_op(tc);
        }
    }
}

void
TlbiOp64::tlbiRvaa(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable,
    bool last_level, TlbiAttr attrs)
{
    TLBIRMVAA tlbi_op(regime, ss, value, last_level, attrs);
    if (shareable) {
        tlbi_op.broadcast(tc);
    } else {
        tlbi_op(tc);
    }
}

void
TlbiOp64::tlbiRva(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable, bool last_level,
    TlbiAttr attrs)
{
    if (MMU::hasUnprivRegime(regime)) {
        // The asid will only be used when e2h == 1
        bool asid_16bits = ArmSystem::haveLargeAsid64(tc);
        auto asid = asid_16bits ? bits(value, 63, 48) :
                                  bits(value, 55, 48);

        TLBIRMVA tlbi_op(regime, ss, value, asid, last_level, attrs);
        if (shareable) {
            tlbi_op.broadcast(tc);
        } else {
            tlbi_op(tc);
        }
    } else {
        tlbiRvaa(tc, value, ss, regime, shareable, last_level, attrs);
    }
}

void
TlbiOp64::tlbiRipaS2(ThreadContext *tc, RegVal value,
    SecurityState ss, TranslationRegime regime, bool shareable,
    bool last_level, TlbiAttr attrs)
{
    if (EL2Enabled(tc)) {
        TLBIRIPA tlbi_op(tc, TranslationRegime::EL10, ss, value,
            last_level, attrs);

        if (shareable) {
            tlbi_op.broadcast(tc);
        } else {
            tlbi_op(tc);
        }
    }
}

bool
TlbiOp64::fnxsAttrs(ThreadContext *tc)
{
    HCRX hcrx = tc->readMiscRegNoEffect(MISCREG_HCRX_EL2);
    return currEL(tc) == EL1 &&
        HaveExt(tc, ArmExtension::FEAT_XS) &&
        HaveExt(tc, ArmExtension::FEAT_HCX) &&
        isHcrxEL2Enabled(tc) && hcrx.fnxs;
}

std::unordered_map<MiscRegIndex, TlbiOp64::TlbiFunc> TlbiOp64::tlbiOps = {
    { MISCREG_TLBI_ALLE3, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                false); // shareable
        }
    },

    { MISCREG_TLBI_ALLE3NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                false, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ALLE3IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true); // shareable
        }
    },

    { MISCREG_TLBI_ALLE3ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ALLE3OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true); // shareable
        }
    },

    { MISCREG_TLBI_ALLE3OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_ALLE2, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                false); // shareable
        }
    },

    { MISCREG_TLBI_ALLE2NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                false, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ALLE2IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true); // shareable
        }
    },

    { MISCREG_TLBI_ALLE2ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ALLE2OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true); // shareable
        }
    },

    { MISCREG_TLBI_ALLE2OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ALLE1, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false); // shareable
        }
    },

    { MISCREG_TLBI_ALLE1NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ALLE1IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true); // shareable
        }
    },

    { MISCREG_TLBI_ALLE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ALLE1OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true); // shareable
        }
    },

    { MISCREG_TLBI_ALLE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiAll(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_VMALLE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // stage2
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VMALLE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // stage2
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VMALLE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // stage2
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VMALLE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // stage2
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VMALLE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // stage2
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VMALLE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // stage2
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VMALLS12E1, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                true); // stage2
        }
    },

    { MISCREG_TLBI_VMALLS12E1NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                true, // stage2
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VMALLS12E1IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true); // stage2
        }
    },

    { MISCREG_TLBI_VMALLS12E1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true, // stage2
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VMALLS12E1OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true); // stage2
        }
    },

    { MISCREG_TLBI_VMALLS12E1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVmall(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true, // stage2
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_VAE3, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                false, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_VAE3NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                false, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_VAE3IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_VAE3ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAE3OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_VAE3OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3,            // regime
                true,           // shareable
                false,         // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_VALE3, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                false, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_VALE3NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3,            // regime
                false,          // shareable
                true,          // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_VALE3IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_VALE3ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3,            // regime
                true,           // shareable
                true,          // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_VALE3OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure, // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_VALE3OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                SecurityState::Secure,           // secure
                TranslationRegime::EL3,            // regime
                true,           // shareable
                true,          // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAE2, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                false, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_VAE2NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime,               // regime
                false,             // shareable
                false,            // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAE2IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                TranslationRegime::EL2, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_VAE2ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                TranslationRegime::EL2,               // regime
                true,              // shareable
                false,            // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_VAE2OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc,EL2), // secure
                TranslationRegime::EL2, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_VAE2OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc,EL2), // secure
                TranslationRegime::EL2,               // regime
                true,              // shareable
                false,            // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_VALE2, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                false, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_VALE2NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime,               // regime
                false,             // shareable
                true,            // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VALE2IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_VALE2ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime,               // regime
                true,             // shareable
                true,            // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VALE2OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_VALE2OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime,               // regime
                true,             // shareable
                true,            // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VALE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                false, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VALE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VALE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VALE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VALE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VALE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ASIDE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiAsid(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_ASIDE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiAsid(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ASIDE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiAsid(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_ASIDE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiAsid(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_ASIDE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiAsid(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_ASIDE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiAsid(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAAE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAAE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAAE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAAE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAAE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAAE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAALE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAALE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                true, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAALE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAALE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_VAALE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_VAALE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiVaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_IPAS2E1, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                false); // last level
        }
    },

    { MISCREG_TLBI_IPAS2E1NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                false, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_IPAS2E1IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false); // last level
        }
    },

    { MISCREG_TLBI_IPAS2E1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_IPAS2E1OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false); // last level
        }
    },

    { MISCREG_TLBI_IPAS2E1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_IPAS2LE1, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                true); // last level
        }
    },

    { MISCREG_TLBI_IPAS2LE1NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                true, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_IPAS2LE1IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true); // last level
        }
    },

    { MISCREG_TLBI_IPAS2LE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_IPAS2LE1OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true); // last level
        }
    },

    { MISCREG_TLBI_IPAS2LE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiIpaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true, // last level
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_RVAE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAAE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAAE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAAE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAAE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_RVAAE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAAE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVALE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVALE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVALE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAALE1, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAALE1NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                shareable, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAALE1IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAALE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAALE1OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            const TlbiAttr attrs = fnxsAttrs(tc) ?
                TlbiAttr::ExcludeXS : TlbiAttr::None;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                attrs); // attrs
        }
    },

    { MISCREG_TLBI_RVAALE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TlbiOp64::tlbiRvaa(tc, value,
                securityStateAtEL(tc, translationEl(regime)), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RIPAS2E1, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RIPAS2E1NXS, [](ThreadContext *tc, RegVal value)
        {
            tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },


    { MISCREG_TLBI_RIPAS2E1IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RIPAS2E1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RIPAS2E1OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RIPAS2E1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RIPAS2E1OS, [](ThreadContext *tc, RegVal value)
        {
            tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RIPAS2E1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RIPAS2LE1, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RIPAS2LE1NXS, [](ThreadContext *tc, RegVal value)
        {
            tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                false, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RIPAS2LE1IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RIPAS2LE1ISNXS, [](ThreadContext *tc, RegVal value)
        {
            tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RIPAS2LE1OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RIPAS2LE1OSNXS, [](ThreadContext *tc, RegVal value)
        {
            tlbiRipaS2(tc, value,
                securityStateAtEL(tc, EL1), // secure
                TranslationRegime::EL10, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAE2, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                false, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RVAE2NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                false, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAE2IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RVAE2ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAE2OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RVAE2OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE2, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                false, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RVALE2NXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                false, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE2IS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RVALE2ISNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE2OS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RVALE2OSNXS, [](ThreadContext *tc, RegVal value)
        {
            const TranslationRegime regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL2), // secure
                regime, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAE3, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                false, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RVAE3NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                false, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAE3IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RVAE3ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVAE3OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                false); // last level only
        }
    },

    { MISCREG_TLBI_RVAE3OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                false, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE3, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                false, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RVALE3NXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                false, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE3IS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RVALE3ISNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },

    { MISCREG_TLBI_RVALE3OS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                true); // last level only
        }
    },

    { MISCREG_TLBI_RVALE3OSNXS, [](ThreadContext *tc, RegVal value)
        {
            TlbiOp64::tlbiRva(tc, value,
                securityStateAtEL(tc, EL3), // secure
                TranslationRegime::EL3, // regime
                true, // shareable
                true, // last level only
                TlbiAttr::ExcludeXS); // attrs
        }
    },
};

void
TlbiOp64::performTlbi(ExecContext *xc, MiscRegIndex dest_idx, RegVal value) const
{
    ThreadContext* tc = xc->tcBase();

    if (auto it = tlbiOps.find(dest_idx); it != tlbiOps.end()) {
        it->second(tc, value);
    } else {
        panic("Invalid TLBI\n");
    }
}

} // namespace gem5
