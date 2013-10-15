/*
 * Copyright (c) 2010 ARM Limited
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
        break;
      case FloatRegClass:
        ccprintf(os, "f%d", rel_reg);
        break;
      case MiscRegClass:
        assert(rel_reg < NUM_MISCREGS);
        ccprintf(os, "%s", ArmISA::miscRegName[rel_reg]);
        break;
      case CCRegClass:
        panic("printReg: CCRegClass but ARM has no CC regs\n");
    }
}

void
ArmStaticInst::printMnemonic(std::ostream &os,
                             const std::string &suffix,
                             bool withPred) const
{
    os << "  " << mnemonic;
    if (withPred) {
        unsigned condCode = machInst.condCode;
        switch (condCode) {
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
            break;
          case COND_UC:
            // Unconditional.
            break;
          default:
            panic("Unrecognized condition code %d.\n", condCode);
        }
        os << suffix;
        if (machInst.bigThumb)
            os << ".w";
        os << "   ";
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
ArmStaticInst::printDataInst(std::ostream &os, bool withImm,
        bool immShift, bool s, IntRegIndex rd, IntRegIndex rn,
        IntRegIndex rm, IntRegIndex rs, uint32_t shiftAmt,
        ArmShiftType type, uint32_t imm) const
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
        ccprintf(os, "#%d", imm);
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
}
