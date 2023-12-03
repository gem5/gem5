/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
 * All rights reserved
 * Copyright 2017 Google Inc.
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

#include "arch/sparc/insts/static_inst.hh"

#include "arch/sparc/pcstate.hh"
#include "arch/sparc/regs/int.hh"
#include "arch/sparc/regs/misc.hh"
#include "base/bitunion.hh"

namespace gem5
{

namespace SparcISA
{

const char *CondTestAbbrev[] = { [Never] = "nev",
                                 [Equal] = "e",
                                 [LessOrEqual] = "le",
                                 [Less] = "l",
                                 [LessOrEqualUnsigned] = "leu",
                                 [CarrySet] = "c",
                                 [Negative] = "n",
                                 [OverflowSet] = "o",
                                 [Always] = "a",
                                 [NotEqual] = "ne",
                                 [Greater] = "g",
                                 [GreaterOrEqual] = "ge",
                                 [GreaterUnsigned] = "gu",
                                 [CarryClear] = "cc",
                                 [Positive] = "p",
                                 [OverflowClear] = "oc" };

void
SparcStaticInst::printMnemonic(std::ostream &os, const char *mnemonic)
{
    ccprintf(os, "\t%s   ", mnemonic);
}

void
SparcStaticInst::printRegArray(std::ostream &os, const RegId *indexArray,
                               int num) const
{
    if (num <= 0)
        return;
    printReg(os, indexArray[0]);
    for (int x = 1; x < num; x++) {
        os << ", ";
        printReg(os, indexArray[x]);
    }
}

void
SparcStaticInst::advancePC(PCStateBase &pcState) const
{
    pcState.as<PCState>().advance();
}

void
SparcStaticInst::advancePC(ThreadContext *tc) const
{
    PCState pc = tc->pcState().as<PCState>();
    pc.advance();
    tc->pcState(pc);
}

void
SparcStaticInst::printSrcReg(std::ostream &os, int reg) const
{
    if (_numSrcRegs > reg)
        printReg(os, srcRegIdx(reg));
}

void
SparcStaticInst::printDestReg(std::ostream &os, int reg) const
{
    if (_numDestRegs > reg)
        printReg(os, destRegIdx(reg));
}

void
SparcStaticInst::printReg(std::ostream &os, RegId reg)
{
    const int MaxGlobal = 8;
    const int MaxOutput = 16;
    const int MaxLocal = 24;
    const int MaxInput = 32;
    const int MaxMicroReg = 40;
    RegIndex reg_idx = reg.index();
    if (reg.is(IntRegClass)) {
        // If we used a register from the next or previous window,
        // take out the offset.
        while (reg_idx >= MaxMicroReg)
            reg_idx -= MaxMicroReg;
        if (reg_idx == FramePointerReg)
            ccprintf(os, "%%fp");
        else if (reg_idx == StackPointerReg)
            ccprintf(os, "%%sp");
        else if (reg_idx < MaxGlobal)
            ccprintf(os, "%%g%d", reg_idx);
        else if (reg_idx < MaxOutput)
            ccprintf(os, "%%o%d", reg_idx - MaxGlobal);
        else if (reg_idx < MaxLocal)
            ccprintf(os, "%%l%d", reg_idx - MaxOutput);
        else if (reg_idx < MaxInput)
            ccprintf(os, "%%i%d", reg_idx - MaxLocal);
        else if (reg_idx < MaxMicroReg)
            ccprintf(os, "%%u%d", reg_idx - MaxInput);
        // The fake int regs that are really control regs
        else {
            switch (reg_idx - MaxMicroReg) {
            case 1:
                ccprintf(os, "%%y");
                break;
            case 2:
                ccprintf(os, "%%ccr");
                break;
            case 3:
                ccprintf(os, "%%cansave");
                break;
            case 4:
                ccprintf(os, "%%canrestore");
                break;
            case 5:
                ccprintf(os, "%%cleanwin");
                break;
            case 6:
                ccprintf(os, "%%otherwin");
                break;
            case 7:
                ccprintf(os, "%%wstate");
                break;
            }
        }
    } else if (reg.is(FloatRegClass)) {
        ccprintf(os, "%%f%d", reg_idx);
    } else {
        switch (reg_idx) {
        case MISCREG_ASI:
            ccprintf(os, "%%asi");
            break;
        case MISCREG_FPRS:
            ccprintf(os, "%%fprs");
            break;
        case MISCREG_PCR:
            ccprintf(os, "%%pcr");
            break;
        case MISCREG_PIC:
            ccprintf(os, "%%pic");
            break;
        case MISCREG_GSR:
            ccprintf(os, "%%gsr");
            break;
        case MISCREG_SOFTINT:
            ccprintf(os, "%%softint");
            break;
        case MISCREG_SOFTINT_SET:
            ccprintf(os, "%%softint_set");
            break;
        case MISCREG_SOFTINT_CLR:
            ccprintf(os, "%%softint_clr");
            break;
        case MISCREG_TICK_CMPR:
            ccprintf(os, "%%tick_cmpr");
            break;
        case MISCREG_STICK:
            ccprintf(os, "%%stick");
            break;
        case MISCREG_STICK_CMPR:
            ccprintf(os, "%%stick_cmpr");
            break;
        case MISCREG_TPC:
            ccprintf(os, "%%tpc");
            break;
        case MISCREG_TNPC:
            ccprintf(os, "%%tnpc");
            break;
        case MISCREG_TSTATE:
            ccprintf(os, "%%tstate");
            break;
        case MISCREG_TT:
            ccprintf(os, "%%tt");
            break;
        case MISCREG_TICK:
            ccprintf(os, "%%tick");
            break;
        case MISCREG_TBA:
            ccprintf(os, "%%tba");
            break;
        case MISCREG_PSTATE:
            ccprintf(os, "%%pstate");
            break;
        case MISCREG_TL:
            ccprintf(os, "%%tl");
            break;
        case MISCREG_PIL:
            ccprintf(os, "%%pil");
            break;
        case MISCREG_CWP:
            ccprintf(os, "%%cwp");
            break;
        case MISCREG_GL:
            ccprintf(os, "%%gl");
            break;
        case MISCREG_HPSTATE:
            ccprintf(os, "%%hpstate");
            break;
        case MISCREG_HTSTATE:
            ccprintf(os, "%%htstate");
            break;
        case MISCREG_HINTP:
            ccprintf(os, "%%hintp");
            break;
        case MISCREG_HTBA:
            ccprintf(os, "%%htba");
            break;
        case MISCREG_HSTICK_CMPR:
            ccprintf(os, "%%hstick_cmpr");
            break;
        case MISCREG_HVER:
            ccprintf(os, "%%hver");
            break;
        case MISCREG_STRAND_STS_REG:
            ccprintf(os, "%%strand_sts_reg");
            break;
        case MISCREG_FSR:
            ccprintf(os, "%%fsr");
            break;
        default:
            ccprintf(os, "%%ctrl%d", reg_idx);
        }
    }
}

std::string
SparcStaticInst::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;

    printMnemonic(ss, mnemonic);

    // just print the first two source regs... if there's
    // a third one, it's a read-modify-write dest (Rc),
    // e.g. for CMOVxx
    if (_numSrcRegs > 0)
        printReg(ss, srcRegIdx(0));
    if (_numSrcRegs > 1) {
        ss << ",";
        printReg(ss, srcRegIdx(1));
    }

    // just print the first dest... if there's a second one,
    // it's generally implicit
    if (_numDestRegs > 0) {
        if (_numSrcRegs > 0)
            ss << ",";
        printReg(ss, destRegIdx(0));
    }

    return ss.str();
}

bool
SparcStaticInst::passesFpCondition(uint32_t fcc, uint32_t condition)
{
    bool u = (fcc == 3);
    bool g = (fcc == 2);
    bool l = (fcc == 1);
    bool e = (fcc == 0);

    switch (condition) {
    case FAlways:
        return 1;
    case FNever:
        return 0;
    case FUnordered:
        return u;
    case FGreater:
        return g;
    case FUnorderedOrGreater:
        return u || g;
    case FLess:
        return l;
    case FUnorderedOrLess:
        return u || l;
    case FLessOrGreater:
        return l || g;
    case FNotEqual:
        return l || g || u;
    case FEqual:
        return e;
    case FUnorderedOrEqual:
        return u || e;
    case FGreaterOrEqual:
        return g || e;
    case FUnorderedOrGreaterOrEqual:
        return u || g || e;
    case FLessOrEqual:
        return l || e;
    case FUnorderedOrLessOrEqual:
        return u || l || e;
    case FOrdered:
        return e || l || g;
    }
    panic("Tried testing condition nonexistant condition code %d", condition);
}

bool
SparcStaticInst::passesCondition(uint32_t codes, uint32_t condition)
{
    BitUnion32(CondCodes)
        Bitfield<0> c;
        Bitfield<1> v;
        Bitfield<2> z;
        Bitfield<3> n;
    EndBitUnion(CondCodes)
    CondCodes condCodes = codes;

    switch (condition) {
    case Always:
        return true;
    case Never:
        return false;
    case NotEqual:
        return !condCodes.z;
    case Equal:
        return condCodes.z;
    case Greater:
        return !(condCodes.z | (condCodes.n ^ condCodes.v));
    case LessOrEqual:
        return condCodes.z | (condCodes.n ^ condCodes.v);
    case GreaterOrEqual:
        return !(condCodes.n ^ condCodes.v);
    case Less:
        return (condCodes.n ^ condCodes.v);
    case GreaterUnsigned:
        return !(condCodes.c | condCodes.z);
    case LessOrEqualUnsigned:
        return (condCodes.c | condCodes.z);
    case CarryClear:
        return !condCodes.c;
    case CarrySet:
        return condCodes.c;
    case Positive:
        return !condCodes.n;
    case Negative:
        return condCodes.n;
    case OverflowClear:
        return !condCodes.v;
    case OverflowSet:
        return condCodes.v;
    }
    panic("Tried testing condition nonexistant "
          "condition code %d",
          condition);
}

} // namespace SparcISA
} // namespace gem5
