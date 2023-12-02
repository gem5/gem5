/*
 * Copyright (c) 2010-2014, 2016-2020,2022 Arm Limited
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
 * Copyright (c) 2007-2008 The Florida State University
 * All rights reserved.
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

#include "arch/arm/insts/static_inst.hh"

#include "arch/arm/faults.hh"
#include "arch/arm/isa.hh"
#include "arch/arm/self_debug.hh"
#include "arch/arm/utility.hh"
#include "base/condcodes.hh"
#include "base/cprintf.hh"
#include "base/loader/symtab.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

namespace ArmISA
{
// Shift Rm by an immediate value
int32_t
ArmStaticInst::shift_rm_imm(uint32_t base, uint32_t shamt,
                                uint32_t type, uint32_t cfval) const
{
    assert(shamt < 32);
    ArmShiftType shiftType;
    shiftType = (ArmShiftType)type;

    switch (shiftType)
    {
      case LSL:
        return base << shamt;
      case LSR:
        if (shamt == 0)
            return 0;
        else
            return base >> shamt;
      case ASR:
        if (shamt == 0)
            return (base >> 31) | -((base & (1 << 31)) >> 31);
        else
            return (base >> shamt) | -((base & (1 << 31)) >> shamt);
      case ROR:
        if (shamt == 0)
            return (cfval << 31) | (base >> 1); // RRX
        else
            return (base << (32 - shamt)) | (base >> shamt);
      default:
        ccprintf(std::cerr, "Unhandled shift type\n");
        exit(1);
        break;
    }
    return 0;
}

int64_t
ArmStaticInst::shiftReg64(uint64_t base, uint64_t shiftAmt,
                          ArmShiftType type, uint8_t width) const
{
    shiftAmt = shiftAmt % width;
    ArmShiftType shiftType;
    shiftType = (ArmShiftType)type;

    switch (shiftType)
    {
      case LSL:
        return base << shiftAmt;
      case LSR:
        if (shiftAmt == 0)
            return base;
        else
            return (base & mask(width)) >> shiftAmt;
      case ASR:
        if (shiftAmt == 0) {
            return base;
        } else {
            int sign_bit = bits(base, intWidth - 1);
            base >>= shiftAmt;
            base = sign_bit ? (base | ~mask(intWidth - shiftAmt)) : base;
            return base & mask(intWidth);
        }
      case ROR:
        if (shiftAmt == 0)
            return base;
        else
            return (base << (width - shiftAmt)) | (base >> shiftAmt);
      default:
        ccprintf(std::cerr, "Unhandled shift type\n");
        exit(1);
        break;
    }
    return 0;
}

int64_t
ArmStaticInst::extendReg64(uint64_t base, ArmExtendType type,
                           uint64_t shiftAmt, uint8_t width) const
{
    bool sign_extend = false;
    int len = 0;
    switch (type) {
      case UXTB:
        len = 8;
        break;
      case UXTH:
        len = 16;
        break;
      case UXTW:
        len = 32;
        break;
      case UXTX:
        len = 64;
        break;
      case SXTB:
        len = 8;
        sign_extend = true;
        break;
      case SXTH:
        len = 16;
        sign_extend = true;
        break;
      case SXTW:
        len = 32;
        sign_extend = true;
        break;
      case SXTX:
        len = 64;
        sign_extend = true;
        break;
    }
    len = len <= width - shiftAmt ? len : width - shiftAmt;
    uint64_t tmp = (uint64_t) bits(base, len - 1, 0) << shiftAmt;
    if (sign_extend) {
        int sign_bit = bits(tmp, len + shiftAmt - 1);
        tmp = sign_bit ? (tmp | ~mask(len + shiftAmt)) : tmp;
    }
    return tmp & mask(width);
}

// Shift Rm by Rs
int32_t
ArmStaticInst::shift_rm_rs(uint32_t base, uint32_t shamt,
                               uint32_t type, uint32_t cfval) const
{
    enum ArmShiftType shiftType;
    shiftType = (enum ArmShiftType) type;

    switch (shiftType)
    {
      case LSL:
        if (shamt >= 32)
            return 0;
        else
            return base << shamt;
      case LSR:
        if (shamt >= 32)
            return 0;
        else
            return base >> shamt;
      case ASR:
        if (shamt >= 32)
            return (base >> 31) | -((base & (1 << 31)) >> 31);
        else
            return (base >> shamt) | -((base & (1 << 31)) >> shamt);
      case ROR:
        shamt = shamt & 0x1f;
        if (shamt == 0)
            return base;
        else
            return (base << (32 - shamt)) | (base >> shamt);
      default:
        ccprintf(std::cerr, "Unhandled shift type\n");
        exit(1);
        break;
    }
    return 0;
}


// Generate C for a shift by immediate
bool
ArmStaticInst::shift_carry_imm(uint32_t base, uint32_t shamt,
                                   uint32_t type, uint32_t cfval) const
{
    enum ArmShiftType shiftType;
    shiftType = (enum ArmShiftType) type;

    switch (shiftType)
    {
      case LSL:
        if (shamt == 0)
            return cfval;
        else
            return (base >> (32 - shamt)) & 1;
      case LSR:
        if (shamt == 0)
            return (base >> 31);
        else
            return (base >> (shamt - 1)) & 1;
      case ASR:
        if (shamt == 0)
            return (base >> 31);
        else
            return (base >> (shamt - 1)) & 1;
      case ROR:
        shamt = shamt & 0x1f;
        if (shamt == 0)
            return (base & 1); // RRX
        else
            return (base >> (shamt - 1)) & 1;
      default:
        ccprintf(std::cerr, "Unhandled shift type\n");
        exit(1);
        break;
    }
    return 0;
}


// Generate C for a shift by Rs
bool
ArmStaticInst::shift_carry_rs(uint32_t base, uint32_t shamt,
                                  uint32_t type, uint32_t cfval) const
{
    enum ArmShiftType shiftType;
    shiftType = (enum ArmShiftType) type;

    if (shamt == 0)
        return cfval;

    switch (shiftType)
    {
      case LSL:
        if (shamt > 32)
            return 0;
        else
            return (base >> (32 - shamt)) & 1;
      case LSR:
        if (shamt > 32)
            return 0;
        else
            return (base >> (shamt - 1)) & 1;
      case ASR:
        if (shamt > 32)
            shamt = 32;
        return (base >> (shamt - 1)) & 1;
      case ROR:
        shamt = shamt & 0x1f;
        if (shamt == 0)
            shamt = 32;
        return (base >> (shamt - 1)) & 1;
      default:
        ccprintf(std::cerr, "Unhandled shift type\n");
        exit(1);
        break;
    }
    return 0;
}

void
ArmStaticInst::printIntReg(std::ostream &os, RegIndex reg_idx,
                           uint8_t opWidth) const
{
    if (opWidth == 0)
        opWidth = intWidth;
    if (aarch64) {
        if (reg_idx == int_reg::Ureg0)
            ccprintf(os, "ureg0");
        else if (reg_idx == int_reg::Spx)
            ccprintf(os, "%s%s", (opWidth == 32) ? "w" : "", "sp");
        else if (reg_idx == int_reg::X31)
            ccprintf(os, "%szr", (opWidth == 32) ? "w" : "x");
        else
            ccprintf(os, "%s%d", (opWidth == 32) ? "w" : "x", reg_idx);
    } else {
        switch (reg_idx) {
          case int_reg::Pc:
            ccprintf(os, "pc");
            break;
          case StackPointerReg:
            ccprintf(os, "sp");
            break;
          case FramePointerReg:
             ccprintf(os, "fp");
             break;
          case ReturnAddressReg:
             ccprintf(os, "lr");
             break;
          default:
             ccprintf(os, "r%d", reg_idx);
             break;
        }
    }
}

void ArmStaticInst::printPFflags(std::ostream &os, int flag) const
{
    const char *flagtoprfop[]= { "PLD", "PLI", "PST", "Reserved"};
    const char *flagtotarget[] = { "L1", "L2", "L3", "Reserved"};
    const char *flagtopolicy[] = { "KEEP", "STRM"};

    ccprintf(os, "%s%s%s", flagtoprfop[(flag>>3)&3],
             flagtotarget[(flag>>1)&3], flagtopolicy[flag&1]);
}

void
ArmStaticInst::printFloatReg(std::ostream &os, RegIndex reg_idx) const
{
    ccprintf(os, "f%d", reg_idx);
}

void
ArmStaticInst::printVecReg(std::ostream &os, RegIndex reg_idx,
                           bool isSveVecReg) const
{
    ccprintf(os, "%s%d", isSveVecReg ? "z" : "v", reg_idx);
}

void
ArmStaticInst::printVecPredReg(std::ostream &os, RegIndex reg_idx) const
{
    ccprintf(os, "p%d", reg_idx);
}

void
ArmStaticInst::printCCReg(std::ostream &os, RegIndex reg_idx) const
{
    ccprintf(os, "cc_%s", ArmISA::cc_reg::RegName[reg_idx]);
}

void
ArmStaticInst::printMiscReg(std::ostream &os, RegIndex reg_idx) const
{
    assert(reg_idx < NUM_MISCREGS);
    ccprintf(os, "%s", ArmISA::miscRegName[reg_idx]);
}

void
ArmStaticInst::printMnemonic(std::ostream &os,
                             const std::string &suffix,
                             bool withPred,
                             bool withCond64,
                             ConditionCode cond64) const
{
    os << "  " << mnemonic;
    if (withPred && !aarch64) {
        printCondition(os, machInst.condCode);
        os << suffix;
    } else if (withCond64) {
        os << ".";
        printCondition(os, cond64);
        os << suffix;
    }
    if (machInst.bigThumb)
        os << ".w";
    os << "   ";
}

void
ArmStaticInst::printTarget(std::ostream &os, Addr target,
                           const loader::SymbolTable *symtab) const
{
    if (symtab) {
        auto it = symtab->findNearest(target);
        if (it != symtab->end()) {
            ccprintf(os, "<%s", it->name());
            Addr delta = target - it->address();
            if (delta)
                ccprintf(os, "+%d>", delta);
            else
                ccprintf(os, ">");
            return;
        }
    }
    ccprintf(os, "%#x", target);
}

void
ArmStaticInst::printCondition(std::ostream &os,
                              unsigned code,
                              bool noImplicit) const
{
    switch (code) {
      case COND_EQ:
        os << "eq";
        break;
      case COND_NE:
        os << "ne";
        break;
      case COND_CS:
        os << "cs";
        break;
      case COND_CC:
        os << "cc";
        break;
      case COND_MI:
        os << "mi";
        break;
      case COND_PL:
        os << "pl";
        break;
      case COND_VS:
        os << "vs";
        break;
      case COND_VC:
        os << "vc";
        break;
      case COND_HI:
        os << "hi";
        break;
      case COND_LS:
        os << "ls";
        break;
      case COND_GE:
        os << "ge";
        break;
      case COND_LT:
        os << "lt";
        break;
      case COND_GT:
        os << "gt";
        break;
      case COND_LE:
        os << "le";
        break;
      case COND_AL:
        // This one is implicit.
        if (noImplicit)
            os << "al";
        break;
      case COND_UC:
        // Unconditional.
        if (noImplicit)
            os << "uc";
        break;
      default:
        panic("Unrecognized condition code %d.\n", code);
    }
}

void
ArmStaticInst::printMemSymbol(std::ostream &os,
                              const loader::SymbolTable *symtab,
                              const std::string &prefix,
                              const Addr addr,
                              const std::string &suffix) const
{
    if (symtab) {
        auto it = symtab->findNearest(addr);
        if (it != symtab->end()) {
            ccprintf(os, "%s%s", prefix, it->name());
            if (it->address() != addr)
                ccprintf(os, "+%d", addr - it->address());
            ccprintf(os, suffix);
        }
    }
}

void
ArmStaticInst::printShiftOperand(std::ostream &os,
                                     RegIndex rm,
                                     bool immShift,
                                     uint32_t shiftAmt,
                                     RegIndex rs,
                                     ArmShiftType type) const
{
    bool firstOp = false;

    if (rm != int_reg::Zero) {
        printIntReg(os, rm);
    }

    bool done = false;

    if ((type == LSR || type == ASR) && immShift && shiftAmt == 0)
        shiftAmt = 32;

    switch (type) {
      case LSL:
        if (immShift && shiftAmt == 0) {
            done = true;
            break;
        }
        if (!firstOp)
            os << ", ";
        os << "LSL";
        break;
      case LSR:
        if (!firstOp)
            os << ", ";
        os << "LSR";
        break;
      case ASR:
        if (!firstOp)
            os << ", ";
        os << "ASR";
        break;
      case ROR:
        if (immShift && shiftAmt == 0) {
            if (!firstOp)
                os << ", ";
            os << "RRX";
            done = true;
            break;
        }
        if (!firstOp)
            os << ", ";
        os << "ROR";
        break;
      default:
        panic("Tried to disassemble unrecognized shift type.\n");
    }
    if (!done) {
        if (!firstOp)
            os << " ";
        if (immShift)
            os << "#" << shiftAmt;
        else
            printIntReg(os, rs);
    }
}

void
ArmStaticInst::printExtendOperand(bool firstOperand, std::ostream &os,
                                  RegIndex rm, ArmExtendType type,
                                  int64_t shiftAmt) const
{
    if (!firstOperand)
        ccprintf(os, ", ");
    printIntReg(os, rm);
    if (type == UXTX && shiftAmt == 0)
        return;
    switch (type) {
      case UXTB: ccprintf(os, ", UXTB");
        break;
      case UXTH: ccprintf(os, ", UXTH");
        break;
      case UXTW: ccprintf(os, ", UXTW");
        break;
      case UXTX: ccprintf(os, ", LSL");
        break;
      case SXTB: ccprintf(os, ", SXTB");
        break;
      case SXTH: ccprintf(os, ", SXTH");
        break;
      case SXTW: ccprintf(os, ", SXTW");
        break;
      case SXTX: ccprintf(os, ", SXTW");
        break;
    }
    if (type == UXTX || shiftAmt)
        ccprintf(os, " #%d", shiftAmt);
}

void
ArmStaticInst::printDataInst(std::ostream &os, bool withImm,
        bool immShift, bool s, RegIndex rd, RegIndex rn,
        RegIndex rm, RegIndex rs, uint32_t shiftAmt,
        ArmShiftType type, uint64_t imm) const
{
    printMnemonic(os, s ? "s" : "");
    bool firstOp = true;

    // Destination
    if (rd != int_reg::Zero) {
        firstOp = false;
        printIntReg(os, rd);
    }

    // Source 1.
    if (rn != int_reg::Zero) {
        if (!firstOp)
            os << ", ";
        firstOp = false;
        printIntReg(os, rn);
    }

    if (!firstOp)
        os << ", ";
    if (withImm) {
        ccprintf(os, "#%ld", imm);
    } else {
        printShiftOperand(os, rm, immShift, shiftAmt, rs, type);
    }
}

std::string
ArmStaticInst::generateDisassembly(Addr pc,
                                   const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    return ss.str();
}

Fault
ArmStaticInst::softwareBreakpoint32(ExecContext *xc, uint16_t imm) const
{
    const auto tc = xc->tcBase();
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const HDCR mdcr = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);
    if ((EL2Enabled(tc) && !ELIs32(tc, EL2) &&
         (hcr.tge || mdcr.tde)) || !ELIs32(tc, EL1)) {
        // Route to AArch64 Software Breakpoint
        return std::make_shared<SoftwareBreakpoint>(machInst, imm);
    } else {
        // Execute AArch32 Software Breakpoint
        return std::make_shared<PrefetchAbort>(readPC(xc),
                                               ArmFault::DebugEvent,
                                               false,
                                               ArmFault::UnknownTran,
                                               ArmFault::BRKPOINT);
    }
}

Fault
ArmStaticInst::advSIMDFPAccessTrap64(ExceptionLevel el) const
{
    switch (el) {
      case EL1:
        return std::make_shared<SupervisorTrap>(
            machInst, 0x1E00000, ExceptionClass::TRAPPED_SIMD_FP);
      case EL2:
        return std::make_shared<HypervisorTrap>(
            machInst, 0x1E00000, ExceptionClass::TRAPPED_SIMD_FP);
      case EL3:
        return std::make_shared<SecureMonitorTrap>(
            machInst, 0x1E00000, ExceptionClass::TRAPPED_SIMD_FP);

      default:
        panic("Illegal EL in advSIMDFPAccessTrap64\n");
    }
}


Fault
ArmStaticInst::checkFPAdvSIMDTrap64(ThreadContext *tc, CPSR cpsr) const
{
    if (currEL(tc) <= EL2 && EL2Enabled(tc)) {
        bool trap_el2 = false;
        CPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL2);
        HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        if (HaveExt(tc, ArmExtension::FEAT_VHE) && hcr.e2h == 0x1) {
            switch (cptr_en_check.fpen) {
              case 0:
              case 2:
                trap_el2 = !(currEL(tc) == EL1 && hcr.tge == 1);
                break;
              case 1:
                trap_el2 = (currEL(tc) == EL0 && hcr.tge == 1);
                break;
              default:
                trap_el2 = false;
                break;
            }
        } else if (cptr_en_check.tfp) {
            trap_el2 = true;
        }

        if (trap_el2) {
            return advSIMDFPAccessTrap64(EL2);
        }
    }

    if (ArmSystem::haveEL(tc, EL3)) {
        CPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL3);
        if (cptr_en_check.tfp) {
            return advSIMDFPAccessTrap64(EL3);
        }
    }

    return NoFault;
}

Fault
ArmStaticInst::checkFPAdvSIMDEnabled64(ThreadContext *tc,
                                       CPSR cpsr, CPACR cpacr) const
{
    const ExceptionLevel el = currEL(tc);
    if (((el == EL0 && cpacr.fpen != 0x3) ||
        (el == EL1 && !(cpacr.fpen & 0x1))) &&
        !ELIsInHost(tc, el))
        return advSIMDFPAccessTrap64(EL1);

    return checkFPAdvSIMDTrap64(tc, cpsr);
}

Fault
ArmStaticInst::checkAdvSIMDOrFPEnabled32(ThreadContext *tc,
                                         CPSR cpsr, CPACR cpacr,
                                         NSACR nsacr, FPEXC fpexc,
                                         bool fpexc_check, bool advsimd) const
{
    const bool have_virtualization = ArmSystem::haveEL(tc, EL2);
    const bool have_security = ArmSystem::haveEL(tc, EL3);
    const bool is_secure = isSecure(tc);
    const ExceptionLevel cur_el = currEL(tc);

    if (cur_el == EL0 && ELIs64(tc, EL1))
        return checkFPAdvSIMDEnabled64(tc, cpsr, cpacr);

    uint8_t cpacr_cp10 = cpacr.cp10;
    bool cpacr_asedis = cpacr.asedis;

    if (have_security && !ELIs64(tc, EL3) && !is_secure) {
        if (nsacr.nsasedis)
            cpacr_asedis = true;
        if (nsacr.cp10 == 0)
            cpacr_cp10 = 0;
    }

    if (cur_el != EL2) {
        if (advsimd && cpacr_asedis)
            return disabledFault();

        if ((cur_el == EL0 && cpacr_cp10 != 0x3) ||
            (cur_el != EL0 && !(cpacr_cp10 & 0x1)))
            return disabledFault();
    }

    if (fpexc_check && !fpexc.en)
        return disabledFault();

    // -- aarch32/exceptions/traps/AArch32.CheckFPAdvSIMDTrap --

    if (have_virtualization && !is_secure && ELIs64(tc, EL2))
        return checkFPAdvSIMDTrap64(tc, cpsr);

    if (have_virtualization && !is_secure) {
        HCPTR hcptr = tc->readMiscReg(MISCREG_HCPTR);
        bool hcptr_cp10 = hcptr.tcp10;
        bool hcptr_tase = hcptr.tase;

        if (have_security && !ELIs64(tc, EL3) && !is_secure) {
            if (nsacr.nsasedis)
                hcptr_tase = true;
            if (nsacr.cp10)
                hcptr_cp10 = true;
        }

        if ((advsimd && hcptr_tase) || hcptr_cp10) {
            const uint32_t iss = advsimd ? (1 << 5) : 0xA;
            if (cur_el == EL2) {
                return std::make_shared<UndefinedInstruction>(
                    machInst, iss,
                    ExceptionClass::TRAPPED_HCPTR, mnemonic);
            } else {
                return std::make_shared<HypervisorTrap>(
                    machInst, iss,
                    ExceptionClass::TRAPPED_HCPTR);
            }

        }
    }

    if (have_security && ELIs64(tc, EL3)) {
        HCPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL3);
        if (cptr_en_check.tfp)
            return advSIMDFPAccessTrap64(EL3);
    }

    return NoFault;
}

inline bool
ArmStaticInst::isWFxTrapping(ThreadContext *tc,
                             ExceptionLevel tgtEl,
                             bool isWfe) const
{
    bool trap = false;
    SCTLR sctlr = ((SCTLR)tc->readMiscReg(MISCREG_SCTLR_EL1));
    HCR hcr = ((HCR)tc->readMiscReg(MISCREG_HCR_EL2));
    SCR scr = ((SCR)tc->readMiscReg(MISCREG_SCR_EL3));

    switch (tgtEl) {
      case EL1:
        trap = isWfe? !sctlr.ntwe : !sctlr.ntwi;
        break;
      case EL2:
        trap = isWfe? hcr.twe : hcr.twi;
        break;
      case EL3:
        trap = isWfe? scr.twe : scr.twi;
        break;
      default:
        break;
    }

    return trap;
}

Fault
ArmStaticInst::checkForWFxTrap32(ThreadContext *tc,
                                 ExceptionLevel targetEL,
                                 bool isWfe) const
{
    // Check if target exception level is implemented.
    assert(ArmSystem::haveEL(tc, targetEL));

    // Check for routing to AArch64: this happens if the
    // target exception level (where the trap will be handled)
    // is using aarch64
    if (ELIs64(tc, targetEL)) {
        return checkForWFxTrap64(tc, targetEL, isWfe);
    }

    // Check if processor needs to trap at selected exception level
    bool trap = isWFxTrapping(tc, targetEL, isWfe);

    if (trap) {
        uint32_t iss = isWfe? 0x1E00001 : /* WFE Instruction syndrome */
                              0x1E00000;  /* WFI Instruction syndrome */
        switch (targetEL) {
          case EL1:
            return std::make_shared<UndefinedInstruction>(
                machInst, iss,
                ExceptionClass::TRAPPED_WFI_WFE, mnemonic);
          case EL2:
            return std::make_shared<HypervisorTrap>(
                machInst, iss,
                ExceptionClass::TRAPPED_WFI_WFE);
          case EL3:
            return std::make_shared<SecureMonitorTrap>(
                machInst, iss,
                ExceptionClass::TRAPPED_WFI_WFE);
          default:
            panic("Unrecognized Exception Level: %d\n", targetEL);
        }
    }

    return NoFault;
}

Fault
ArmStaticInst::checkForWFxTrap64(ThreadContext *tc,
                                 ExceptionLevel targetEL,
                                 bool isWfe) const
{
    // Check if target exception level is implemented.
    assert(ArmSystem::haveEL(tc, targetEL));

    // Check if processor needs to trap at selected exception level
    bool trap = isWFxTrapping(tc, targetEL, isWfe);

    if (trap) {
        uint32_t iss = isWfe? 0x1E00001 : /* WFE Instruction syndrome */
                              0x1E00000;  /* WFI Instruction syndrome */
        switch (targetEL) {
          case EL1:
            return std::make_shared<SupervisorTrap>(
                machInst, iss,
                ExceptionClass::TRAPPED_WFI_WFE);
          case EL2:
            return std::make_shared<HypervisorTrap>(
                machInst, iss,
                ExceptionClass::TRAPPED_WFI_WFE);
          case EL3:
            return std::make_shared<SecureMonitorTrap>(
                machInst, iss,
                ExceptionClass::TRAPPED_WFI_WFE);
          default:
            panic("Unrecognized Exception Level: %d\n", targetEL);
        }
    }

    return NoFault;
}

Fault
ArmStaticInst::trapWFx(ThreadContext *tc,
                       CPSR cpsr, SCR scr,
                       bool isWfe) const
{
    Fault fault = NoFault;
    ExceptionLevel curr_el = currEL(tc);

    if (curr_el == EL0) {
        fault = checkForWFxTrap32(tc, EL1, isWfe);
    }

    if ((fault == NoFault) && EL2Enabled(tc) &&
        ((curr_el == EL0) || (curr_el == EL1))) {

        fault = checkForWFxTrap32(tc, EL2, isWfe);
    }

    if ((fault == NoFault) &&
        ArmSystem::haveEL(tc, EL3) && curr_el != EL3) {
        fault = checkForWFxTrap32(tc, EL3, isWfe);
    }

    return fault;
}

Fault
ArmStaticInst::checkSETENDEnabled(ThreadContext *tc, CPSR cpsr) const
{
    bool setend_disabled(false);
    ExceptionLevel pstate_el = currEL(tc);

    if (pstate_el == EL2) {
       setend_disabled = ((SCTLR)tc->readMiscRegNoEffect(MISCREG_HSCTLR)).sed;
    } else {
        // Please note: in the armarm pseudocode there is a distinction
        // whether EL1 is aarch32 or aarch64:
        // if ELUsingAArch32(EL1) then SCTLR.SED else SCTLR[].SED;
        // Considering that SETEND is aarch32 only, ELUsingAArch32(EL1)
        // will always be true (hence using SCTLR.SED) except for
        // instruction executed at EL0, and with an AArch64 EL1.
        // In this case SCTLR_EL1 will be used. In gem5 the register is
        // mapped to SCTLR_ns. We can safely use SCTLR and choose the
        // appropriate bank version.

        // Get the index of the banked version of SCTLR:
        // SCTLR_s or SCTLR_ns.
        auto banked_sctlr = snsBankedIndex(
            MISCREG_SCTLR, tc, !isSecure(tc));

        // SCTLR.SED bit is enabling/disabling the ue of SETEND instruction.
        setend_disabled = ((SCTLR)tc->readMiscRegNoEffect(banked_sctlr)).sed;
    }

    return setend_disabled ? undefinedFault32(tc, pstate_el) :
                             NoFault;
}

Fault
ArmStaticInst::undefinedFault32(ThreadContext *tc,
                                ExceptionLevel pstateEL) const
{
    // Even if we are running in aarch32, the fault might be dealt with in
    // aarch64 ISA.
    if (generalExceptionsToAArch64(tc, pstateEL)) {
        return undefinedFault64(tc, pstateEL);
    } else {
        // Please note: according to the ARM ARM pseudocode we should handle
        // the case when EL2 is aarch64 and HCR.TGE is 1 as well.
        // However this case is already handled by the routeToHyp method in
        // ArmFault class.
        return std::make_shared<UndefinedInstruction>(
            machInst, 0,
            ExceptionClass::UNKNOWN, mnemonic);
    }
}

Fault
ArmStaticInst::undefinedFault64(ThreadContext *tc,
                                ExceptionLevel pstateEL) const
{
    switch (pstateEL) {
      case EL0:
      case EL1:
        return std::make_shared<SupervisorTrap>(
            machInst, 0, ExceptionClass::UNKNOWN);
      case EL2:
        return std::make_shared<HypervisorTrap>(
            machInst, 0, ExceptionClass::UNKNOWN);
      case EL3:
        return std::make_shared<SecureMonitorTrap>(
            machInst, 0, ExceptionClass::UNKNOWN);
      default:
        panic("Unrecognized Exception Level: %d\n", pstateEL);
        break;
    }

    return NoFault;
}

Fault
ArmStaticInst::sveAccessTrap(ExceptionLevel el) const
{
    switch (el) {
      case EL1:
        return std::make_shared<SupervisorTrap>(
            machInst, 0, ExceptionClass::TRAPPED_SVE);
      case EL2:
        return std::make_shared<HypervisorTrap>(
            machInst, 0, ExceptionClass::TRAPPED_SVE);
      case EL3:
        return std::make_shared<SecureMonitorTrap>(
            machInst, 0, ExceptionClass::TRAPPED_SVE);

      default:
        panic("Illegal EL in sveAccessTrap\n");
    }
}

Fault
ArmStaticInst::checkSveEnabled(ThreadContext *tc, CPSR cpsr, CPACR cpacr) const
{
    // We first check if we are in streaming mode or not. If we are in
    // streaming mode, we actually check the SME traps, not the SVE traps!
    SVCR svcr_sm_check = tc->readMiscReg(MISCREG_SVCR);
    if (svcr_sm_check.sm) {
        return checkSmeEnabled(tc, cpsr, cpacr);
    }

    const ExceptionLevel el = (ExceptionLevel) (uint8_t) cpsr.el;
    // Check if access disabled in CPACR_EL1
    if (el <= EL1 && !ELIsInHost(tc, el)) {
        if ((el == EL0 && cpacr.zen == 0x1) ||
            (!(cpacr.zen & 0x1)))
            return sveAccessTrap(EL1);

        if ((el == EL0 && cpacr.fpen == 0x1) ||
            (!(cpacr.fpen & 0x1)))
            return advSIMDFPAccessTrap64(EL1);
    }

    // Check if access disabled in CPTR_EL2
    if (el <= EL2 && EL2Enabled(tc)) {
        CPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL2);
        HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        if (HaveExt(tc, ArmExtension::FEAT_VHE) && hcr.e2h) {
            if (((cptr_en_check.zen & 0x1) == 0x0) ||
                (cptr_en_check.zen == 0x1 && el == EL0 &&
                 hcr.tge == 0x1)) {
                return sveAccessTrap(EL2);
            }
            if (((cptr_en_check.fpen & 0x1) == 0x0) ||
                (cptr_en_check.fpen == 0x1 && el == EL0 &&
                 hcr.tge == 0x1)) {
                return advSIMDFPAccessTrap64(EL2);
            }
        } else {
            if (cptr_en_check.tz == 1)
                return sveAccessTrap(EL2);
            if (cptr_en_check.tfp == 1)
                return advSIMDFPAccessTrap64(EL2);
        }
    }

    // Check if access disabled in CPTR_EL3
    if (ArmSystem::haveEL(tc, EL3)) {
        CPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL3);
        if (!cptr_en_check.ez)
            return sveAccessTrap(EL3);
        if (cptr_en_check.tfp)
            return advSIMDFPAccessTrap64(EL3);
    }

    return NoFault;
}

Fault
ArmStaticInst::smeAccessTrap(ExceptionLevel el, uint32_t iss) const
{
    switch (el) {
      case EL1:
        return std::make_shared<SupervisorTrap>(
            machInst, iss, ExceptionClass::TRAPPED_SME);
      case EL2:
        return std::make_shared<HypervisorTrap>(
            machInst, iss, ExceptionClass::TRAPPED_SME);
      case EL3:
        return std::make_shared<SecureMonitorTrap>(
            machInst, iss, ExceptionClass::TRAPPED_SME);

      default:
        panic("Illegal EL in smeAccessTrap\n");
    }
}

Fault
ArmStaticInst::checkSmeEnabled(ThreadContext *tc, CPSR cpsr, CPACR cpacr) const
{
    const ExceptionLevel el = (ExceptionLevel) (uint8_t) cpsr.el;
    // Check if access disabled in CPACR_EL1
    if (el <= EL1 && !ELIsInHost(tc, el)) {
        if ((el == EL0 && cpacr.smen == 0x1) ||
            (!(cpacr.smen & 0x1)))
            return smeAccessTrap(EL1);

        if ((el == EL0 && cpacr.fpen == 0x1) ||
            (!(cpacr.fpen & 0x1)))
            return advSIMDFPAccessTrap64(EL1);
    }

    // Check if access disabled in CPTR_EL2
    if (el <= EL2 && EL2Enabled(tc)) {
        CPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL2);
        HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        if (HaveExt(tc, ArmExtension::FEAT_VHE) && hcr.e2h) {
            if (((cptr_en_check.smen & 0x1) == 0x0) ||
                (cptr_en_check.smen == 0x1 && el == EL0 &&
                 hcr.tge == 0x1)) {
                return smeAccessTrap(EL2);
            }
            if (((cptr_en_check.fpen & 0x1) == 0x0) ||
                (cptr_en_check.fpen == 0x1 && el == EL0 &&
                 hcr.tge == 0x1)) {
                return advSIMDFPAccessTrap64(EL2);
            }
        } else {
            if (cptr_en_check.tsm == 1)
                return smeAccessTrap(EL2);
            if (cptr_en_check.tfp == 1)
                return advSIMDFPAccessTrap64(EL2);
        }
    }

    // Check if access disabled in CPTR_EL3
    if (ArmSystem::haveEL(tc, EL3)) {
        CPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL3);
        if (!cptr_en_check.esm)
            return smeAccessTrap(EL3);
        if (cptr_en_check.tfp)
            return advSIMDFPAccessTrap64(EL3);
    }

    return NoFault;
}

Fault
ArmStaticInst::checkSmeAccess(ThreadContext *tc, CPSR cpsr, CPACR cpacr) const
{
    const ExceptionLevel el = (ExceptionLevel) (uint8_t) cpsr.el;
    // Check if access disabled in CPACR_EL1
    if (el <= EL1 && !ELIsInHost(tc, el)) {
        if ((el == EL0 && cpacr.smen == 0x1) || (!(cpacr.smen & 0x1))) {
            return smeAccessTrap(EL1);
        }
    }

    // Check if access disabled in CPTR_EL2
    if (el <= EL2 && EL2Enabled(tc)) {
        CPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL2);
        HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        if (HaveExt(tc, ArmExtension::FEAT_VHE) && hcr.e2h) {
            if (((cptr_en_check.smen & 0x1) == 0x0) ||
                (cptr_en_check.smen == 0x1 && el == EL0 &&
                 hcr.tge == 0x1)) {
                return smeAccessTrap(EL2);
            }
        } else {
            if (cptr_en_check.tsm == 1)
                return smeAccessTrap(EL2);
        }
    }

    // Check if access disabled in CPTR_EL3
    if (ArmSystem::haveEL(tc, EL3)) {
        CPTR cptr_en_check = tc->readMiscReg(MISCREG_CPTR_EL3);
        if (!cptr_en_check.esm)
            return smeAccessTrap(EL3);
    }

    return NoFault;
}

Fault
ArmStaticInst::checkSveSmeEnabled(ThreadContext *tc, CPSR cpsr,
                                  CPACR cpacr) const
{
    // If we are not in streaming mode, check the SVE traps, else check the SME
    // traps.
    SVCR svcr = tc->readMiscReg(MISCREG_SVCR);
    if (!svcr.sm) {
        return checkSveEnabled(tc, cpsr, cpacr);
    } else {
        return checkSmeEnabled(tc, cpsr, cpacr);
    }
}

static uint8_t
getRestoredITBits(ThreadContext *tc, CPSR spsr)
{
    // See: shared/functions/system/RestoredITBits in the ARM ARM

    const ExceptionLevel el = opModeToEL((OperatingMode) (uint8_t)spsr.mode);
    const uint8_t it = itState(spsr);

    if (!spsr.t || spsr.il)
        return 0;

    // The IT bits are forced to zero when they are set to a reserved
    // value.
    if (bits(it, 7, 4) != 0 && bits(it, 3, 0) == 0)
        return 0;

    const bool itd = el == EL2 ?
        ((SCTLR)tc->readMiscReg(MISCREG_HSCTLR)).itd :
        ((SCTLR)tc->readMiscReg(MISCREG_SCTLR)).itd;

    // The IT bits are forced to zero when returning to A32 state, or
    // when returning to an EL with the ITD bit set to 1, and the IT
    // bits are describing a multi-instruction block.
    if (itd && bits(it, 2, 0) != 0)
        return 0;

    return it;
}

static bool
illegalExceptionReturn(ThreadContext *tc, CPSR cpsr, CPSR spsr)
{
    const OperatingMode mode = (OperatingMode) (uint8_t)spsr.mode;
    if (unknownMode(mode))
        return true;

    SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

    //ELFromSPSR
    bool valid;
    ExceptionLevel target_el = opModeToEL(mode);
    if (!spsr.width) {
        if (!ArmSystem::highestELIs64(tc)) {
            valid = false;
        } else if (!ArmSystem::haveEL(tc, target_el)) {
            valid = false;
        } else if (spsr & 0x2) {
            valid = false;
        } else if (target_el == EL0 && spsr.sp) {
            valid = false;
        } else if (target_el == EL2 && ArmSystem::haveEL(tc, EL3) &&
                   !scr.ns && !IsSecureEL2Enabled(tc)) {
            valid = false;
        } else {
            valid = true;
        }
    } else {
        valid = !unknownMode32(mode);
    }
    if (!valid)
        return true;

    if (target_el > currEL(tc))
        return true;

    bool spsr_mode_is_aarch32 = (spsr.width == 1);
    auto [known, target_el_is_aarch32] = ELUsingAArch32K(tc, target_el);
    assert(known || (target_el == EL0 && ELIs64(tc, EL1)));

    if (known && (spsr_mode_is_aarch32 != target_el_is_aarch32))
        return true;

    if (target_el == EL1 && ArmSystem::haveEL(tc, EL2) && hcr.tge &&
            (IsSecureEL2Enabled(tc) || !isSecureBelowEL3(tc))) {
        return true;
    }

    return false;
}

CPSR
ArmStaticInst::getPSTATEFromPSR(ThreadContext *tc, CPSR cpsr, CPSR spsr) const
{
    CPSR new_cpsr = 0;
    ExceptionLevel dest;

    if (illegalExceptionReturn(tc, cpsr, spsr)) {
        // If the SPSR specifies an illegal exception return,
        // then PSTATE.{M, nRW, EL, SP} are unchanged and PSTATE.IL
        // is set to 1.
        new_cpsr.il = 1;
        if (cpsr.width) {
            new_cpsr.mode = cpsr.mode;
        } else {
            new_cpsr.width = cpsr.width;
            new_cpsr.el = cpsr.el;
            new_cpsr.sp = cpsr.sp;
        }
        dest = currEL(tc);
    } else {
        new_cpsr.il = spsr.il;
        if (spsr.width && unknownMode32((OperatingMode)(uint8_t)spsr.mode)) {
            new_cpsr.il = 1;
        } else if (spsr.width) {
            new_cpsr.mode = spsr.mode;
        } else {
            new_cpsr.el = spsr.el;
            new_cpsr.sp = spsr.sp;
        }
        dest = (ExceptionLevel)(uint8_t) spsr.el;
    }

    new_cpsr.nz = spsr.nz;
    new_cpsr.c = spsr.c;
    new_cpsr.v = spsr.v;
    new_cpsr.pan = spsr.pan;
    if (new_cpsr.width) {
        // aarch32
        const ITSTATE it = getRestoredITBits(tc, spsr);
        new_cpsr.q = spsr.q;
        new_cpsr.ge = spsr.ge;
        new_cpsr.e = spsr.e;
        new_cpsr.aif = spsr.aif;
        new_cpsr.t = spsr.t;
        new_cpsr.it2 = it.top6;
        new_cpsr.it1 = it.bottom2;
    } else {
        // aarch64
        new_cpsr.daif = spsr.daif;
        new_cpsr.uao = spsr.uao;
    }

    SelfDebug *sd = ArmISA::ISA::getSelfDebug(tc);
    SoftwareStep *ss = sd->getSstep();
    new_cpsr.ss = ss->debugExceptionReturnSS(tc, spsr, dest);

    return new_cpsr;
}

bool
ArmStaticInst::generalExceptionsToAArch64(ThreadContext *tc,
                                          ExceptionLevel pstateEL) const
{
    // Returns TRUE if exceptions normally routed to EL1 are being handled
    // at an Exception level using AArch64, because either EL1 is using
    // AArch64 or TGE is in force and EL2 is using AArch64.
    HCR hcr = ((HCR)tc->readMiscReg(MISCREG_HCR_EL2));
    return (pstateEL == EL0 && !ELIs32(tc, EL1)) ||
           (ArmSystem::haveEL(tc, EL2) && !isSecure(tc) &&
               !ELIs32(tc, EL2) && hcr.tge);
}

unsigned
ArmStaticInst::getCurSveVecLenInBits(ThreadContext *tc)
{
    auto *isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
    return isa->getCurSveVecLenInBits();
}

unsigned
ArmStaticInst::getCurSmeVecLenInBits(ThreadContext *tc)
{
    auto *isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
    return isa->getCurSmeVecLenInBits();
}


} // namespace ArmISA
} // namespace gem5
