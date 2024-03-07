/*
 * Copyright (c) 2011-2013,2017-2023 Arm Limited
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

Fault
MiscRegOp64::generateTrap(ExceptionLevel el, ExceptionClass ec,
        uint32_t iss) const
{
    switch (el) {
      case EL1:
        return std::make_shared<SupervisorTrap>(getEMI(), iss, ec);
      case EL2:
        return std::make_shared<HypervisorTrap>(getEMI(), iss, ec);
      case EL3:
        return std::make_shared<SecureMonitorTrap>(getEMI(), iss, ec);
      default:
        panic("Invalid EL: %d\n", el);
    }
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
TlbiOp64::performTlbi(ExecContext *xc, MiscRegIndex dest_idx, RegVal value) const
{
    ThreadContext* tc = xc->tcBase();
    auto isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
    auto release = isa->getRelease();

    bool asid_16bits = ArmSystem::haveLargeAsid64(tc);

    switch (dest_idx) {
      // AArch64 TLB Invalidate All, EL3
      case MISCREG_TLBI_ALLE3:
        {
            TLBIALLEL tlbiOp(TranslationRegime::EL3, true);
            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate All, EL3, Inner Shareable
      case MISCREG_TLBI_ALLE3IS:
      // AArch64 TLB Invalidate All, EL3, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_ALLE3OS:
        {
            TLBIALLEL tlbiOp(TranslationRegime::EL3, true);
            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate All, EL2
      case MISCREG_TLBI_ALLE2:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TLBIALLEL tlbiOp(regime, secure);
            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate All, EL2, Inner Shareable
      case MISCREG_TLBI_ALLE2IS:
      // AArch64 TLB Invalidate All, EL2, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_ALLE2OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL2) ?
                TranslationRegime::EL20 : TranslationRegime::EL2;

            TLBIALLEL tlbiOp(regime, secure);
            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate All, EL1
      case MISCREG_TLBI_ALLE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            TLBIALLEL tlbiOp(TranslationRegime::EL10, secure);
            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate All, EL1, Inner Shareable
      case MISCREG_TLBI_ALLE1IS:
      // AArch64 TLB Invalidate All, EL1, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_ALLE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            TLBIALLEL tlbiOp(TranslationRegime::EL10, secure);
            tlbiOp.broadcast(tc);
            return;
        }
      case MISCREG_TLBI_VMALLS12E1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            TLBIVMALL tlbiOp(TranslationRegime::EL10, secure, true);
            tlbiOp(tc);
            return;
        }
      case MISCREG_TLBI_VMALLE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIVMALL tlbiOp(regime, secure, false);
            tlbiOp(tc);
            return;
        }
      case MISCREG_TLBI_VMALLS12E1IS:
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VMALLS12E1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            TLBIVMALL tlbiOp(TranslationRegime::EL10, secure, true);
            tlbiOp.broadcast(tc);
            return;
        }
      case MISCREG_TLBI_VMALLE1IS:
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VMALLE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIVMALL tlbiOp(regime, secure, false);
            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, EL3
      case MISCREG_TLBI_VAE3:
        {

            TLBIMVAA tlbiOp(TranslationRegime::EL3, true,
                            static_cast<Addr>(bits(value, 43, 0)) << 12,
                            false);
            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, Last Level, EL3
      case MISCREG_TLBI_VALE3:
        {

            TLBIMVAA tlbiOp(TranslationRegime::EL3, true,
                            static_cast<Addr>(bits(value, 43, 0)) << 12,
                            true);
            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, EL3, Inner Shareable
      case MISCREG_TLBI_VAE3IS:
      // AArch64 TLB Invalidate by VA, EL3, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VAE3OS:
        {
            TLBIMVAA tlbiOp(TranslationRegime::EL3, true,
                            static_cast<Addr>(bits(value, 43, 0)) << 12,
                            false);

            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, Last Level, EL3, Inner Shareable
      case MISCREG_TLBI_VALE3IS:
      // AArch64 TLB Invalidate by VA, Last Level, EL3, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VALE3OS:
        {
            TLBIMVAA tlbiOp(TranslationRegime::EL3, true,
                            static_cast<Addr>(bits(value, 43, 0)) << 12,
                            true);

            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, EL2
      case MISCREG_TLBI_VAE2:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;

            if (ELIsInHost(tc, EL2)) {
                // The asid will only be used when e2h == 1
                auto asid = asid_16bits ? bits(value, 63, 48) :
                                          bits(value, 55, 48);

                TLBIMVA tlbiOp(TranslationRegime::EL20, secure,
                               static_cast<Addr>(bits(value, 43, 0)) << 12,
                               asid, false);
                tlbiOp(tc);
            } else {
                TLBIMVAA tlbiOp(TranslationRegime::EL2, secure,
                                static_cast<Addr>(bits(value, 43, 0)) << 12,
                                false);
                tlbiOp(tc);
            }
            return;
        }
      // AArch64 TLB Invalidate by VA, Last Level, EL2
      case MISCREG_TLBI_VALE2:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;

            if (ELIsInHost(tc, EL2)) {
                // The asid will only be used when e2h == 1
                auto asid = asid_16bits ? bits(value, 63, 48) :
                                          bits(value, 55, 48);

                TLBIMVA tlbiOp(TranslationRegime::EL20, secure,
                               static_cast<Addr>(bits(value, 43, 0)) << 12,
                               asid, true);
                tlbiOp(tc);
            } else {
                TLBIMVAA tlbiOp(TranslationRegime::EL2, secure,
                                static_cast<Addr>(bits(value, 43, 0)) << 12,
                                true);
                tlbiOp(tc);
            }
            return;
        }
      // AArch64 TLB Invalidate by VA, EL2, Inner Shareable
      case MISCREG_TLBI_VAE2IS:
      // AArch64 TLB Invalidate by VA, EL2, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VAE2OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;

            if (ELIsInHost(tc, EL2)) {
                // The asid will only be used when e2h == 1
                auto asid = asid_16bits ? bits(value, 63, 48) :
                                          bits(value, 55, 48);

                TLBIMVA tlbiOp(TranslationRegime::EL20, secure,
                               static_cast<Addr>(bits(value, 43, 0)) << 12,
                               asid, false);
                tlbiOp.broadcast(tc);
            } else {
                TLBIMVAA tlbiOp(TranslationRegime::EL2, secure,
                                static_cast<Addr>(bits(value, 43, 0)) << 12,
                                false);
                tlbiOp.broadcast(tc);
            }
            return;
        }
      // AArch64 TLB Invalidate by VA, Last Level, EL2, Inner Shareable
      case MISCREG_TLBI_VALE2IS:
      // AArch64 TLB Invalidate by VA, Last Level, EL2, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VALE2OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;

            if (ELIsInHost(tc, EL2)) {
                // The asid will only be used when e2h == 1
                auto asid = asid_16bits ? bits(value, 63, 48) :
                                          bits(value, 55, 48);

                TLBIMVA tlbiOp(TranslationRegime::EL20, secure,
                               static_cast<Addr>(bits(value, 43, 0)) << 12,
                               asid, true);
                tlbiOp.broadcast(tc);
            } else {
                TLBIMVAA tlbiOp(TranslationRegime::EL2, secure,
                                static_cast<Addr>(bits(value, 43, 0)) << 12,
                                true);
                tlbiOp.broadcast(tc);
            }
            return;
        }
      // AArch64 TLB Invalidate by VA, EL1
      case MISCREG_TLBI_VAE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIMVA tlbiOp(regime, secure,
                           static_cast<Addr>(bits(value, 43, 0)) << 12,
                           asid, false);

            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, Last Level, EL1
      case MISCREG_TLBI_VALE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIMVA tlbiOp(regime, secure,
                           static_cast<Addr>(bits(value, 43, 0)) << 12,
                           asid, true);

            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, EL1, Inner Shareable
      case MISCREG_TLBI_VAE1IS:
      // AArch64 TLB Invalidate by VA, EL1, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VAE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIMVA tlbiOp(regime, secure,
                           static_cast<Addr>(bits(value, 43, 0)) << 12,
                           asid, false);

            tlbiOp.broadcast(tc);
            return;
        }
      case MISCREG_TLBI_VALE1IS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIMVA tlbiOp(regime, secure,
                           static_cast<Addr>(bits(value, 43, 0)) << 12,
                           asid, true);

            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate by ASID, EL1
      case MISCREG_TLBI_ASIDE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIASID tlbiOp(regime, secure, asid);
            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate by ASID, EL1, Inner Shareable
      case MISCREG_TLBI_ASIDE1IS:
      // AArch64 TLB Invalidate by ASID, EL1, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_ASIDE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIASID tlbiOp(regime, secure, asid);
            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, All ASID, EL1
      case MISCREG_TLBI_VAAE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIMVAA tlbiOp(regime, secure,
                static_cast<Addr>(bits(value, 43, 0)) << 12,
                false);

            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, Last Level, All ASID, EL1
      case MISCREG_TLBI_VAALE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIMVAA tlbiOp(regime, secure,
                static_cast<Addr>(bits(value, 43, 0)) << 12,
                true);

            tlbiOp(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, All ASID, EL1, Inner Shareable
      case MISCREG_TLBI_VAAE1IS:
      // AArch64 TLB Invalidate by VA, All ASID, EL1, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VAAE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIMVAA tlbiOp(regime, secure,
                static_cast<Addr>(bits(value, 43, 0)) << 12,
                false);

            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate by VA, All ASID,
      // Last Level, EL1, Inner Shareable
      case MISCREG_TLBI_VAALE1IS:
      // AArch64 TLB Invalidate by VA, All ASID,
      // Last Level, EL1, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_VAALE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIMVAA tlbiOp(regime, secure,
                static_cast<Addr>(bits(value, 43, 0)) << 12,
                true);

            tlbiOp.broadcast(tc);
            return;
        }
      // AArch64 TLB Invalidate by Intermediate Physical Address,
      // Stage 2, EL1
      case MISCREG_TLBI_IPAS2E1:
        {
            if (EL2Enabled(tc)) {
                SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

                bool secure = release->has(ArmExtension::SECURITY) &&
                    !scr.ns && !bits(value, 63);

                const int top_bit = ArmSystem::physAddrRange(tc) == 52 ?
                    39 : 35;
                TLBIIPA tlbiOp(TranslationRegime::EL10, secure,
                    static_cast<Addr>(bits(value, top_bit, 0)) << 12,
                    false);

                tlbiOp(tc);
            }
            return;
        }
      // AArch64 TLB Invalidate by Intermediate Physical Address,
      // Stage 2, Last Level EL1
      case MISCREG_TLBI_IPAS2LE1:
        {
            if (EL2Enabled(tc)) {
                SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

                bool secure = release->has(ArmExtension::SECURITY) &&
                    !scr.ns && !bits(value, 63);

                TLBIIPA tlbiOp(TranslationRegime::EL10, secure,
                    static_cast<Addr>(bits(value, 35, 0)) << 12,
                    true);

                tlbiOp(tc);
            }
            return;
        }
      // AArch64 TLB Invalidate by Intermediate Physical Address,
      // Stage 2, EL1, Inner Shareable
      case MISCREG_TLBI_IPAS2E1IS:
      // AArch64 TLB Invalidate by Intermediate Physical Address,
      // Stage 2, EL1, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_IPAS2E1OS:
        {
            if (EL2Enabled(tc)) {
                SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

                bool secure = release->has(ArmExtension::SECURITY) &&
                    !scr.ns && !bits(value, 63);

                const int top_bit = ArmSystem::physAddrRange(tc) == 52 ?
                    39 : 35;
                TLBIIPA tlbiOp(TranslationRegime::EL10, secure,
                    static_cast<Addr>(bits(value, top_bit, 0)) << 12,
                    false);

                tlbiOp.broadcast(tc);
            }
            return;
        }
      // AArch64 TLB Invalidate by Intermediate Physical Address,
      // Stage 2, Last Level, EL1, Inner Shareable
      case MISCREG_TLBI_IPAS2LE1IS:
      // AArch64 TLB Invalidate by Intermediate Physical Address,
      // Stage 2, Last Level, EL1, Outer Shareable
      // We are currently not distinguishing Inner and Outer domains.
      // We therefore implement TLBIOS instructions as TLBIIS
      case MISCREG_TLBI_IPAS2LE1OS:
        {
            if (EL2Enabled(tc)) {
                SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

                bool secure = release->has(ArmExtension::SECURITY) &&
                    !scr.ns && !bits(value, 63);

                TLBIIPA tlbiOp(TranslationRegime::EL10, secure,
                    static_cast<Addr>(bits(value, 35, 0)) << 12,
                    true);

                tlbiOp.broadcast(tc);
            }
            return;
        }
      case MISCREG_TLBI_RVAE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIRMVA tlbiOp(regime, secure, value, asid, false);

            if (tlbiOp.valid())
                tlbiOp(tc);
            return;
        }
      case MISCREG_TLBI_RVAE1IS:
      case MISCREG_TLBI_RVAE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIRMVA tlbiOp(regime, secure, value, asid, false);

            if (tlbiOp.valid())
                tlbiOp.broadcast(tc);
            return;
        }
      case MISCREG_TLBI_RVAAE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIRMVAA tlbiOp(regime, secure, value, false);

            if (tlbiOp.valid())
                tlbiOp(tc);
            return;
        }
      case MISCREG_TLBI_RVAAE1IS:
      case MISCREG_TLBI_RVAAE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIRMVAA tlbiOp(regime, secure, value, false);

            if (tlbiOp.valid())
                tlbiOp.broadcast(tc);
            return;
        }
      case MISCREG_TLBI_RVALE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIRMVA tlbiOp(regime, secure, value, asid, true);

            if (tlbiOp.valid())
                tlbiOp(tc);
            return;
        }
      case MISCREG_TLBI_RVALE1IS:
      case MISCREG_TLBI_RVALE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
            auto asid = asid_16bits ? bits(value, 63, 48) :
                                      bits(value, 55, 48);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIRMVA tlbiOp(regime, secure, value, asid, true);

            if (tlbiOp.valid())
                tlbiOp.broadcast(tc);
            return;
        }
      case MISCREG_TLBI_RVAALE1:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIRMVAA tlbiOp(regime, secure, value, true);

            if (tlbiOp.valid())
                tlbiOp(tc);
            return;
        }
      case MISCREG_TLBI_RVAALE1IS:
      case MISCREG_TLBI_RVAALE1OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;
            auto regime = ELIsInHost(tc, EL0) ?
                TranslationRegime::EL20 : TranslationRegime::EL10;

            TLBIRMVAA tlbiOp(regime, secure, value, true);

            if (tlbiOp.valid())
                tlbiOp.broadcast(tc);
            return;
        }
      case MISCREG_TLBI_RIPAS2E1:
        {
            if (EL2Enabled(tc)) {
                SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

                bool secure = release->has(ArmExtension::SECURITY) &&
                    !scr.ns && !bits(value, 63);

                TLBIRIPA tlbiOp(TranslationRegime::EL10, secure, value, false);

                tlbiOp(tc);
            }
            return;
        }
      case MISCREG_TLBI_RIPAS2E1IS:
        {
            if (EL2Enabled(tc)) {
                SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

                bool secure = release->has(ArmExtension::SECURITY) &&
                    !scr.ns && !bits(value, 63);

                TLBIRIPA tlbiOp(TranslationRegime::EL10, secure, value, false);

                tlbiOp.broadcast(tc);
            }
            return;
        }
      case MISCREG_TLBI_RIPAS2LE1:
        {
            if (EL2Enabled(tc)) {
                SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

                bool secure = release->has(ArmExtension::SECURITY) &&
                    !scr.ns && !bits(value, 63);

                TLBIRIPA tlbiOp(TranslationRegime::EL10, secure, value, true);

                tlbiOp(tc);
            }
            return;
        }
      case MISCREG_TLBI_RIPAS2LE1IS:
        {
            if (EL2Enabled(tc)) {
                SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

                bool secure = release->has(ArmExtension::SECURITY) &&
                    !scr.ns && !bits(value, 63);

                TLBIRIPA tlbiOp(TranslationRegime::EL10, secure, value, true);

                tlbiOp.broadcast(tc);
            }
            return;
        }
      case MISCREG_TLBI_RVAE2:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;

            if (ELIsInHost(tc, EL2)) {
                // The asid will only be used when e2h == 1
                auto asid = asid_16bits ? bits(value, 63, 48) :
                                          bits(value, 55, 48);

                TLBIRMVA tlbiOp(TranslationRegime::EL20, secure, value, asid, false);

                if (tlbiOp.valid())
                    tlbiOp(tc);
            } else {
                TLBIRMVAA tlbiOp(TranslationRegime::EL2, secure, value, false);

                if (tlbiOp.valid())
                    tlbiOp(tc);
            }
            return;
        }
      case MISCREG_TLBI_RVAE2IS:
      case MISCREG_TLBI_RVAE2OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;

            if (ELIsInHost(tc, EL2)) {
                // The asid will only be used when e2h == 1
                auto asid = asid_16bits ? bits(value, 63, 48) :
                                          bits(value, 55, 48);

                TLBIRMVA tlbiOp(TranslationRegime::EL20, secure, value, asid, false);

                if (tlbiOp.valid())
                    tlbiOp.broadcast(tc);
            } else {
                TLBIRMVAA tlbiOp(TranslationRegime::EL2, secure, value, false);

                if (tlbiOp.valid())
                    tlbiOp.broadcast(tc);
            }
            return;
        }
      case MISCREG_TLBI_RVALE2:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;

            if (ELIsInHost(tc, EL2)) {
                // The asid will only be used when e2h == 1
                auto asid = asid_16bits ? bits(value, 63, 48) :
                                          bits(value, 55, 48);

                TLBIRMVA tlbiOp(TranslationRegime::EL20, secure, value, asid, true);

                if (tlbiOp.valid())
                    tlbiOp(tc);
            } else {
                TLBIRMVAA tlbiOp(TranslationRegime::EL2, secure, value, true);

                if (tlbiOp.valid())
                    tlbiOp(tc);
            }
            return;
        }
      case MISCREG_TLBI_RVALE2IS:
      case MISCREG_TLBI_RVALE2OS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            bool secure = release->has(ArmExtension::SECURITY) && !scr.ns;

            if (ELIsInHost(tc, EL2)) {
                // The asid will only be used when e2h == 1
                auto asid = asid_16bits ? bits(value, 63, 48) :
                                          bits(value, 55, 48);

                TLBIRMVA tlbiOp(TranslationRegime::EL20, secure, value, asid, true);

                if (tlbiOp.valid())
                    tlbiOp.broadcast(tc);
            } else {
                TLBIRMVAA tlbiOp(TranslationRegime::EL2, secure, value, true);

                if (tlbiOp.valid())
                    tlbiOp.broadcast(tc);
            }
            return;
        }
      case MISCREG_TLBI_RVAE3:
        {
            TLBIRMVAA tlbiOp(TranslationRegime::EL3, true, value, false);
            if (tlbiOp.valid())
                tlbiOp(tc);
            return;
        }
      case MISCREG_TLBI_RVAE3IS:
      case MISCREG_TLBI_RVAE3OS:
        {
            TLBIRMVAA tlbiOp(TranslationRegime::EL3, true, value, false);
            if (tlbiOp.valid())
                tlbiOp.broadcast(tc);
            return;
        }
      case MISCREG_TLBI_RVALE3:
        {
            TLBIRMVAA tlbiOp(TranslationRegime::EL3, true, value, true);
            if (tlbiOp.valid())
                tlbiOp(tc);
            return;
        }
      case MISCREG_TLBI_RVALE3IS:
      case MISCREG_TLBI_RVALE3OS:
        {
            TLBIRMVAA tlbiOp(TranslationRegime::EL3, true, value, true);
            if (tlbiOp.valid())
                tlbiOp.broadcast(tc);
            return;
        }
      default:
        panic("Invalid TLBI\n");
    }
}

} // namespace gem5
