/*
 * Copyright (c) 2010, 2012-2013, 2017-2018, 2021, 2023-2024 Arm Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#include "arch/arm/insts/misc.hh"
#include "arch/arm/tlbi_op.hh"

#include "cpu/reg_class.hh"

namespace gem5
{

using namespace ArmISA;

std::string
MrsOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    bool foundPsr = false;
    for (unsigned i = 0; i < numSrcRegs(); i++) {
        const RegId& reg = srcRegIdx(i);
        if (!reg.is(MiscRegClass)) {
            continue;
        }
        if (reg.index() == MISCREG_CPSR) {
            ss << "cpsr";
            foundPsr = true;
            break;
        }
        if (reg.index() == MISCREG_SPSR) {
            ss << "spsr";
            foundPsr = true;
            break;
        }
    }
    if (!foundPsr) {
        ss << "????";
    }
    return ss.str();
}

void
MsrBase::printMsrBase(std::ostream &os) const
{
    printMnemonic(os);
    bool apsr = false;
    bool foundPsr = false;
    for (unsigned i = 0; i < numDestRegs(); i++) {
        const RegId& reg = destRegIdx(i);
        if (!reg.is(MiscRegClass)) {
            continue;
        }
        if (reg.index() == MISCREG_CPSR) {
            os << "cpsr_";
            foundPsr = true;
            break;
        }
        if (reg.index() == MISCREG_SPSR) {
            if (bits(byteMask, 1, 0)) {
                os << "spsr_";
            } else {
                os << "apsr_";
                apsr = true;
            }
            foundPsr = true;
            break;
        }
    }
    if (!foundPsr) {
        os << "????";
        return;
    }
    if (bits(byteMask, 3)) {
        if (apsr) {
            os << "nzcvq";
        } else {
            os << "f";
        }
    }
    if (bits(byteMask, 2)) {
        if (apsr) {
            os << "g";
        } else {
            os << "s";
        }
    }
    if (bits(byteMask, 1)) {
        os << "x";
    }
    if (bits(byteMask, 0)) {
        os << "c";
    }
}

std::string
MsrImmOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMsrBase(ss);
    ccprintf(ss, ", #%#x", imm);
    return ss.str();
}

std::string
MsrRegOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMsrBase(ss);
    ss << ", ";
    printIntReg(ss, op1);
    return ss.str();
}

std::string
MrrcOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, dest2);
    ss << ", ";
    printMiscReg(ss, op1);
    return ss.str();
}

std::string
McrrOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printMiscReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ss << ", ";
    printIntReg(ss, op2);
    return ss.str();
}

std::string
ImmOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    ccprintf(ss, "#%d", imm);
    return ss.str();
}

std::string
RegImmOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
RegRegOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    return ss.str();
}

std::string
RegOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    return ss.str();
}

std::string
RegRegRegImmOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ss << ", ";
    printIntReg(ss, op2);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
RegRegRegRegOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ss << ", ";
    printIntReg(ss, op2);
    ss << ", ";
    printIntReg(ss, op3);
    return ss.str();
}

std::string
RegRegRegOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ss << ", ";
    printIntReg(ss, op2);
    return ss.str();
}

std::string
RegRegImmOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
MiscRegRegImmOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printMiscReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    return ss.str();
}

std::string
RegMiscRegImmOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printMiscReg(ss, op1);
    return ss.str();
}

std::string
RegImmImmOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ccprintf(ss, ", #%d, #%d", imm1, imm2);
    return ss.str();
}

std::string
RegRegImmImmOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ccprintf(ss, ", #%d, #%d", imm1, imm2);
    return ss.str();
}

std::string
RegImmRegOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ccprintf(ss, ", #%d, ", imm);
    printIntReg(ss, op1);
    return ss.str();
}

std::string
RegImmRegShiftOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ccprintf(ss, ", #%d, ", imm);
    printShiftOperand(ss, op1, true, shiftAmt, int_reg::Zero, shiftType);
    printIntReg(ss, op1);
    return ss.str();
}

std::string
UnknownOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    return csprintf("%-10s (inst %#08x)", "unknown", encoding());
}

McrMrcMiscInst::McrMrcMiscInst(const char *_mnemonic, ExtMachInst _machInst,
                               uint64_t _iss, MiscRegIndex _miscReg)
    : ArmStaticInst(_mnemonic, _machInst, No_OpClass)
{
    flags[IsNonSpeculative] = true;
    iss = _iss;
    miscReg = _miscReg;
}

Fault
McrMrcMiscInst::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    return mcrMrc15Trap(miscReg, machInst, xc->tcBase(), iss);
}

std::string
McrMrcMiscInst::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    return csprintf("%-10s (pipe flush)", mnemonic);
}

McrMrcImplDefined::McrMrcImplDefined(const char *_mnemonic,
                                     ExtMachInst _machInst, uint64_t _iss,
                                     MiscRegIndex _miscReg)
    : McrMrcMiscInst(_mnemonic, _machInst, _iss, _miscReg)
{}

Fault
McrMrcImplDefined::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    Fault fault = mcrMrc15Trap(miscReg, machInst, xc->tcBase(), iss);
    if (fault != NoFault) {
        return fault;
    } else {
        return std::make_shared<UndefinedInstruction>(machInst, false,
                                                      mnemonic);
    }
}

std::string
McrMrcImplDefined::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    return csprintf("%-10s (implementation defined)", mnemonic);
}

void
TlbiOp::performTlbi(ExecContext *xc, MiscRegIndex dest_idx, RegVal value) const
{
    ThreadContext* tc = xc->tcBase();
    auto isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
    auto release = isa->getRelease();

    switch (dest_idx) {
      case MISCREG_TLBIALL: // TLBI all entries, EL0&1,
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TLBIALL tlbiOp(TranslationRegime::EL10, ss);
            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // TLB Invalidate All, Inner Shareable
      case MISCREG_TLBIALLIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIALL tlbiOp(TranslationRegime::EL10, ss);
            tlbiOp.broadcast(tc);
            return;
        }
      // Instruction TLB Invalidate All
      case MISCREG_ITLBIALL:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            ITLBIALL tlbiOp(TranslationRegime::EL10, ss);
            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // Data TLB Invalidate All
      case MISCREG_DTLBIALL:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            DTLBIALL tlbiOp(TranslationRegime::EL10, ss);
            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // TLB Invalidate by VA
      case MISCREG_TLBIMVA:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TLBIMVA tlbiOp(TranslationRegime::EL10,
                           ss,
                           mbits(value, 31, 12),
                           bits(value, 7, 0),
                           false);

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // TLB Invalidate by VA, Last Level
      case MISCREG_TLBIMVAL:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TLBIMVA tlbiOp(TranslationRegime::EL10,
                           ss,
                           mbits(value, 31, 12),
                           bits(value, 7, 0),
                           true);

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // TLB Invalidate by VA, Inner Shareable
      case MISCREG_TLBIMVAIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIMVA tlbiOp(TranslationRegime::EL10,
                           ss,
                           mbits(value, 31, 12),
                           bits(value, 7, 0),
                           false);

            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate by VA, Last Level, Inner Shareable
      case MISCREG_TLBIMVALIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIMVA tlbiOp(TranslationRegime::EL10,
                           ss,
                           mbits(value, 31, 12),
                           bits(value, 7, 0),
                           true);

            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate by ASID match
      case MISCREG_TLBIASID:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TLBIASID tlbiOp(TranslationRegime::EL10,
                            ss,
                            bits(value, 7, 0));

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // TLB Invalidate by ASID match, Inner Shareable
      case MISCREG_TLBIASIDIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIASID tlbiOp(TranslationRegime::EL10,
                            ss,
                            bits(value, 7, 0));

            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate by VA, All ASID
      case MISCREG_TLBIMVAA:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;
            TLBIMVAA tlbiOp(TranslationRegime::EL10, ss,
                            mbits(value, 31, 12), false);

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // TLB Invalidate by VA, Last Level, All ASID
      case MISCREG_TLBIMVAAL:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            TLBIMVAA tlbiOp(TranslationRegime::EL10, ss,
                            mbits(value, 31, 12), true);

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // TLB Invalidate by VA, All ASID, Inner Shareable
      case MISCREG_TLBIMVAAIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIMVAA tlbiOp(TranslationRegime::EL10, ss,
                            mbits(value, 31, 12), false);

            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate by VA, All ASID, Last Level, Inner Shareable
      case MISCREG_TLBIMVAALIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIMVAA tlbiOp(TranslationRegime::EL10, ss,
                            mbits(value, 31, 12), true);

            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate by VA, Hyp mode
      case MISCREG_TLBIMVAH:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIMVAA tlbiOp(TranslationRegime::EL2, ss,
                            mbits(value, 31, 12), false);

            tlbiOp(tc);
            return;
        }
      // TLB Invalidate by VA, Last Level, Hyp mode
      case MISCREG_TLBIMVALH:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIMVAA tlbiOp(TranslationRegime::EL2, ss,
                            mbits(value, 31, 12), true);

            tlbiOp(tc);
            return;
        }
      // TLB Invalidate by VA, Hyp mode, Inner Shareable
      case MISCREG_TLBIMVAHIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIMVAA tlbiOp(TranslationRegime::EL2, ss,
                            mbits(value, 31, 12), false);

            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate by VA, Hyp mode, Last Level, Inner Shareable
      case MISCREG_TLBIMVALHIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIMVAA tlbiOp(TranslationRegime::EL2, ss,
                            mbits(value, 31, 12), true);

            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate by Intermediate Physical Address, Stage 2
      case MISCREG_TLBIIPAS2:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIIPA tlbiOp(TranslationRegime::EL10,
                           ss,
                           static_cast<Addr>(bits(value, 35, 0)) << 12,
                           false);

            tlbiOp(tc);
            return;
        }
      // TLB Invalidate by Intermediate Physical Address, Stage 2,
      // Last Level
      case MISCREG_TLBIIPAS2L:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIIPA tlbiOp(TranslationRegime::EL10,
                           ss,
                           static_cast<Addr>(bits(value, 35, 0)) << 12,
                           true);

            tlbiOp(tc);
            return;
        }
      // TLB Invalidate by Intermediate Physical Address, Stage 2,
      // Inner Shareable
      case MISCREG_TLBIIPAS2IS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIIPA tlbiOp(TranslationRegime::EL10,
                           ss,
                           static_cast<Addr>(bits(value, 35, 0)) << 12,
                           false);

            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate by Intermediate Physical Address, Stage 2,
      // Last Level, Inner Shareable
      case MISCREG_TLBIIPAS2LIS:
        {
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            TLBIIPA tlbiOp(TranslationRegime::EL10,
                           ss,
                           static_cast<Addr>(bits(value, 35, 0)) << 12,
                           true);

            tlbiOp.broadcast(tc);
            return;
        }
      // Instruction TLB Invalidate by VA
      case MISCREG_ITLBIMVA:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            ITLBIMVA tlbiOp(TranslationRegime::EL10,
                            ss,
                            mbits(value, 31, 12),
                            bits(value, 7, 0));

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // Data TLB Invalidate by VA
      case MISCREG_DTLBIMVA:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            DTLBIMVA tlbiOp(TranslationRegime::EL10,
                            ss,
                            mbits(value, 31, 12),
                            bits(value, 7, 0));

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // Instruction TLB Invalidate by ASID match
      case MISCREG_ITLBIASID:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            ITLBIASID tlbiOp(TranslationRegime::EL10,
                             ss,
                             bits(value, 7, 0));

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // Data TLB Invalidate by ASID match
      case MISCREG_DTLBIASID:
        {
            HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
            SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);

            auto ss = release->has(ArmExtension::SECURITY) && !scr.ns ?
                SecurityState::Secure : SecurityState::NonSecure;
            // Check for Force Broadcast. Ignored if HCR_EL2.TGE == 1
            bool shareable = currEL(tc) == EL1 && EL2Enabled(tc) &&
                hcr.fb && !hcr.tge;

            DTLBIASID tlbiOp(TranslationRegime::EL10,
                             ss,
                             bits(value, 7, 0));

            if (shareable) {
                tlbiOp.broadcast(tc);
            } else {
                tlbiOp(tc);
            }
            return;
        }
      // TLB Invalidate All, Non-Secure Non-Hyp
      case MISCREG_TLBIALLNSNH:
        {
            TLBIALLN tlbiOp(TranslationRegime::EL10);
            tlbiOp(tc);
            return;
        }
      // TLB Invalidate All, Non-Secure Non-Hyp, Inner Shareable
      case MISCREG_TLBIALLNSNHIS:
        {
            TLBIALLN tlbiOp(TranslationRegime::EL10);
            tlbiOp.broadcast(tc);
            return;
        }
      // TLB Invalidate All, Hyp mode
      case MISCREG_TLBIALLH:
        {
            TLBIALLN tlbiOp(TranslationRegime::EL2);
            tlbiOp(tc);
            return;
        }
      // TLB Invalidate All, Hyp mode, Inner Shareable
      case MISCREG_TLBIALLHIS:
        {
            TLBIALLN tlbiOp(TranslationRegime::EL2);
            tlbiOp.broadcast(tc);
            return;
        }
      default:
        panic("Unrecognized TLBIOp\n");
    }
}

} // namespace gem5
