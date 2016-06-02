/*
 * Copyright (c) 2010-2014, 2016 ARM Limited
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
 *
 * Authors: Stephen Hines
 */

#include "arch/arm/insts/static_inst.hh"
#include "arch/arm/faults.hh"
#include "base/loader/symtab.hh"
#include "base/condcodes.hh"
#include "base/cprintf.hh"
#include "cpu/reg_class.hh"

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
ArmStaticInst::printReg(std::ostream &os, int reg) const
{
    RegIndex rel_reg;

    switch (regIdxToClass(reg, &rel_reg)) {
      case IntRegClass:
        if (aarch64) {
            if (reg == INTREG_UREG0)
                ccprintf(os, "ureg0");
            else if (reg == INTREG_SPX)
               ccprintf(os, "%s%s", (intWidth == 32) ? "w" : "", "sp");
            else if (reg == INTREG_X31)
                ccprintf(os, "%szr", (intWidth == 32) ? "w" : "x");
            else
                ccprintf(os, "%s%d", (intWidth == 32) ? "w" : "x", reg);
        } else {
            switch (rel_reg) {
              case PCReg:
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
                ccprintf(os, "r%d", reg);
                break;
            }
        }
        break;
      case FloatRegClass:
        ccprintf(os, "f%d", rel_reg);
        break;
      case MiscRegClass:
        assert(rel_reg < NUM_MISCREGS);
        ccprintf(os, "%s", ArmISA::miscRegName[rel_reg]);
        break;
      case CCRegClass:
        ccprintf(os, "cc_%s", ArmISA::ccRegName[rel_reg]);
        break;
    }
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
                           const SymbolTable *symtab) const
{
    Addr symbolAddr;
    std::string symbol;

    if (symtab && symtab->findNearestSymbol(target, symbol, symbolAddr)) {
        ccprintf(os, "<%s", symbol);
        if (symbolAddr != target)
            ccprintf(os, "+%d>", target - symbolAddr);
        else
            ccprintf(os, ">");
    } else {
        ccprintf(os, "%#x", target);
    }
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
                              const SymbolTable *symtab,
                              const std::string &prefix,
                              const Addr addr,
                              const std::string &suffix) const
{
    Addr symbolAddr;
    std::string symbol;
    if (symtab && symtab->findNearestSymbol(addr, symbol, symbolAddr)) {
        ccprintf(os, "%s%s", prefix, symbol);
        if (symbolAddr != addr)
            ccprintf(os, "+%d", addr - symbolAddr);
        ccprintf(os, suffix);
    }
}

void
ArmStaticInst::printShiftOperand(std::ostream &os,
                                     IntRegIndex rm,
                                     bool immShift,
                                     uint32_t shiftAmt,
                                     IntRegIndex rs,
                                     ArmShiftType type) const
{
    bool firstOp = false;

    if (rm != INTREG_ZERO) {
        printReg(os, rm);
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
            printReg(os, rs);
    }
}

void
ArmStaticInst::printExtendOperand(bool firstOperand, std::ostream &os,
                                  IntRegIndex rm, ArmExtendType type,
                                  int64_t shiftAmt) const
{
    if (!firstOperand)
        ccprintf(os, ", ");
    printReg(os, rm);
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
        bool immShift, bool s, IntRegIndex rd, IntRegIndex rn,
        IntRegIndex rm, IntRegIndex rs, uint32_t shiftAmt,
        ArmShiftType type, uint64_t imm) const
{
    printMnemonic(os, s ? "s" : "");
    bool firstOp = true;

    // Destination
    if (rd != INTREG_ZERO) {
        firstOp = false;
        printReg(os, rd);
    }

    // Source 1.
    if (rn != INTREG_ZERO) {
        if (!firstOp)
            os << ", ";
        firstOp = false;
        printReg(os, rn);
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
                                   const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    return ss.str();
}


Fault
ArmStaticInst::advSIMDFPAccessTrap64(ExceptionLevel el) const
{
    switch (el) {
      case EL1:
        return std::make_shared<SupervisorTrap>(machInst, 0x1E00000,
                                                EC_TRAPPED_SIMD_FP);
      case EL2:
        return std::make_shared<HypervisorTrap>(machInst, 0x1E00000,
                                                EC_TRAPPED_SIMD_FP);
      case EL3:
        return std::make_shared<SecureMonitorTrap>(machInst, 0x1E00000,
                                                   EC_TRAPPED_SIMD_FP);

      default:
        panic("Illegal EL in advSIMDFPAccessTrap64\n");
    }
}


Fault
ArmStaticInst::checkFPAdvSIMDTrap64(ThreadContext *tc, CPSR cpsr) const
{
    const ExceptionLevel el = (ExceptionLevel) (uint8_t)cpsr.el;

    if (ArmSystem::haveVirtualization(tc) && el <= EL2) {
        HCPTR cptrEnCheck = tc->readMiscReg(MISCREG_CPTR_EL2);
        if (cptrEnCheck.tfp)
            return advSIMDFPAccessTrap64(EL2);
    }

    if (ArmSystem::haveSecurity(tc)) {
        HCPTR cptrEnCheck = tc->readMiscReg(MISCREG_CPTR_EL3);
        if (cptrEnCheck.tfp)
            return advSIMDFPAccessTrap64(EL3);
    }

    return NoFault;
}

Fault
ArmStaticInst::checkFPAdvSIMDEnabled64(ThreadContext *tc,
                                       CPSR cpsr, CPACR cpacr) const
{
    const ExceptionLevel el = (ExceptionLevel) (uint8_t)cpsr.el;
    if ((el == EL0 && cpacr.fpen != 0x3) ||
        (el == EL1 && !(cpacr.fpen & 0x1)))
        return advSIMDFPAccessTrap64(EL1);

    return checkFPAdvSIMDTrap64(tc, cpsr);
}

Fault
ArmStaticInst::checkAdvSIMDOrFPEnabled32(ThreadContext *tc,
                                         CPSR cpsr, CPACR cpacr,
                                         NSACR nsacr, FPEXC fpexc,
                                         bool fpexc_check, bool advsimd) const
{
    const bool have_virtualization = ArmSystem::haveVirtualization(tc);
    const bool have_security = ArmSystem::haveSecurity(tc);
    const bool is_secure = inSecureState(tc);
    const ExceptionLevel cur_el = opModeToEL(currOpMode(tc));

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
                    EC_TRAPPED_HCPTR, mnemonic);
            } else {
                return std::make_shared<HypervisorTrap>(
                    machInst, iss,
                    EC_TRAPPED_HCPTR);
            }

        }
    }

    if (have_security && ELIs64(tc, EL3)) {
        HCPTR cptrEnCheck = tc->readMiscReg(MISCREG_CPTR_EL3);
        if (cptrEnCheck.tfp)
            return advSIMDFPAccessTrap64(EL3);
    }

    return NoFault;
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
    if (badMode(mode))
        return true;

    const OperatingMode cur_mode = (OperatingMode) (uint8_t)cpsr.mode;
    const ExceptionLevel target_el = opModeToEL(mode);
    if (target_el > opModeToEL(cur_mode))
        return true;

    if (target_el == EL3 && !ArmSystem::haveSecurity(tc))
        return true;

    if (target_el == EL2 && !ArmSystem::haveVirtualization(tc))
        return true;

    if (!spsr.width) {
        // aarch64
        if (!ArmSystem::highestELIs64(tc))
            return true;

        if (spsr & 0x2)
            return true;
        if (target_el == EL0 && spsr.sp)
            return true;
        if (target_el == EL2 && !((SCR)tc->readMiscReg(MISCREG_SCR_EL3)).ns)
            return false;
    } else {
        return badMode32(mode);
    }

    return false;
}

CPSR
ArmStaticInst::getPSTATEFromPSR(ThreadContext *tc, CPSR cpsr, CPSR spsr) const
{
    CPSR new_cpsr = 0;

    // gem5 doesn't implement single-stepping, so force the SS bit to
    // 0.
    new_cpsr.ss = 0;

    if (illegalExceptionReturn(tc, cpsr, spsr)) {
        new_cpsr.il = 1;
    } else {
        new_cpsr.il = spsr.il;
        if (spsr.width && badMode32((OperatingMode)(uint8_t)spsr.mode)) {
            new_cpsr.il = 1;
        } else if (spsr.width) {
            new_cpsr.mode = spsr.mode;
        } else {
            new_cpsr.el = spsr.el;
            new_cpsr.sp = spsr.sp;
        }
    }

    new_cpsr.nz = spsr.nz;
    new_cpsr.c = spsr.c;
    new_cpsr.v = spsr.v;
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
    }

    return new_cpsr;
}



}
