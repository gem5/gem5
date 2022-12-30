/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#include "arch/x86/insts/static_inst.hh"

#include "arch/x86/regs/segment.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

namespace X86ISA
{

void
X86StaticInst::printMnemonic(std::ostream &os, const char *mnemonic)
{
    ccprintf(os, "  %s   ", mnemonic);
}

void
X86StaticInst::printMnemonic(std::ostream &os, const char *instMnemonic,
        const char *mnemonic)
{
    ccprintf(os, "  %s : %s   ", instMnemonic, mnemonic);
}

void X86StaticInst::printSegment(std::ostream &os, int segment)
{
    switch (segment)
    {
      case segment_idx::Es:
        ccprintf(os, "ES");
        break;
      case segment_idx::Cs:
        ccprintf(os, "CS");
        break;
      case segment_idx::Ss:
        ccprintf(os, "SS");
        break;
      case segment_idx::Ds:
        ccprintf(os, "DS");
        break;
      case segment_idx::Fs:
        ccprintf(os, "FS");
        break;
      case segment_idx::Gs:
        ccprintf(os, "GS");
        break;
      case segment_idx::Hs:
        ccprintf(os, "HS");
        break;
      case segment_idx::Tsl:
        ccprintf(os, "TSL");
        break;
      case segment_idx::Tsg:
        ccprintf(os, "TSG");
        break;
      case segment_idx::Ls:
        ccprintf(os, "LS");
        break;
      case segment_idx::Ms:
        ccprintf(os, "MS");
        break;
      case segment_idx::Tr:
        ccprintf(os, "TR");
        break;
      case segment_idx::Idtr:
        ccprintf(os, "IDTR");
        break;
      default:
        panic("Unrecognized segment %d\n", segment);
    }
}

void
X86StaticInst::divideStep(uint64_t dividend, uint64_t divisor,
        uint64_t &quotient, uint64_t &remainder)
{
    // Check for divide by zero.
    assert(divisor != 0);
    // If the divisor is bigger than the dividend, don't do anything.
    if (divisor <= dividend) {
        // Shift the divisor so it's msb lines up with the dividend.
        int dividendMsb = findMsbSet(dividend);
        int divisorMsb = findMsbSet(divisor);
        int shift = dividendMsb - divisorMsb;
        divisor <<= shift;
        // Compute what we'll add to the quotient if the divisor isn't
        // now larger than the dividend.
        uint64_t quotientBit = 1;
        quotientBit <<= shift;
        // If we need to step back a bit (no pun intended) because the
        // divisor got too to large, do that here. This is the "or two"
        // part of one or two bit division.
        if (divisor > dividend) {
            quotientBit >>= 1;
            divisor >>= 1;
        }
        // Decrement the remainder and increment the quotient.
        quotient += quotientBit;
        remainder -= divisor;
    }
}

void
X86StaticInst::printReg(std::ostream &os, RegId reg, int size)
{
    assert(size == 1 || size == 2 || size == 4 || size == 8);
    static const char * abcdFormats[9] =
        {"", "%s",  "%sx",  "", "e%sx", "", "", "", "r%sx"};
    static const char * piFormats[9] =
        {"", "%s",  "%s",   "", "e%s",  "", "", "", "r%s"};
    static const char * longFormats[9] =
        {"", "r%sb", "r%sw", "", "r%sd", "", "", "", "r%s"};
    static const char * microFormats[9] =
        {"", "t%db", "t%dw", "", "t%dd", "", "", "", "t%d"};

    RegIndex reg_idx = reg.index();

    switch (reg.classValue()) {
      case IntRegClass:
        {
            const char * suffix = "";
            bool fold = reg_idx & IntFoldBit;
            reg_idx &= ~IntFoldBit;

            if (fold)
                suffix = "h";
            else if (reg_idx < 8 && size == 1)
                suffix = "l";

            switch (reg_idx) {
              case int_reg::Rax:
                ccprintf(os, abcdFormats[size], "a");
                break;
              case int_reg::Rbx:
                ccprintf(os, abcdFormats[size], "b");
                break;
              case int_reg::Rcx:
                ccprintf(os, abcdFormats[size], "c");
                break;
              case int_reg::Rdx:
                ccprintf(os, abcdFormats[size], "d");
                break;
              case int_reg::Rsp:
                ccprintf(os, piFormats[size], "sp");
                break;
              case int_reg::Rbp:
                ccprintf(os, piFormats[size], "bp");
                break;
              case int_reg::Rsi:
                ccprintf(os, piFormats[size], "si");
                break;
              case int_reg::Rdi:
                ccprintf(os, piFormats[size], "di");
                break;
              case int_reg::R8:
                ccprintf(os, longFormats[size], "8");
                break;
              case int_reg::R9:
                ccprintf(os, longFormats[size], "9");
                break;
              case int_reg::R10:
                ccprintf(os, longFormats[size], "10");
                break;
              case int_reg::R11:
                ccprintf(os, longFormats[size], "11");
                break;
              case int_reg::R12:
                ccprintf(os, longFormats[size], "12");
                break;
              case int_reg::R13:
                ccprintf(os, longFormats[size], "13");
                break;
              case int_reg::R14:
                ccprintf(os, longFormats[size], "14");
                break;
              case int_reg::R15:
                ccprintf(os, longFormats[size], "15");
                break;
              default:
                ccprintf(os, microFormats[size],
                        reg_idx - int_reg::MicroBegin);
            }
            ccprintf(os, suffix);
        }
        break;
      case FloatRegClass:
        if (reg_idx < NumMMXRegs) {
            ccprintf(os, "%%mmx%d", reg_idx);
            return;
        }
        reg_idx -= NumMMXRegs;
        if (reg_idx < NumXMMRegs * 2) {
            ccprintf(os, "%%xmm%d_%s", reg_idx / 2,
                    (reg_idx % 2) ? "high": "low");
            return;
        }
        reg_idx -= NumXMMRegs * 2;
        if (reg_idx < NumMicroFpRegs) {
            ccprintf(os, "%%ufp%d", reg_idx);
            return;
        }
        reg_idx -= NumMicroFpRegs;
        ccprintf(os, "%%st(%d)", reg_idx);
        break;
      case CCRegClass:
        ccprintf(os, "%%cc%d", reg_idx);
        break;
      case MiscRegClass:
        switch (reg_idx) {
          default:
            ccprintf(os, "%%ctrl%d", reg_idx);
        }
        break;
      default:
        panic("Unrecognized register class.");
    }
}

void
X86StaticInst::printMem(std::ostream &os, uint8_t segment,
        uint8_t scale, RegIndex index, RegIndex base,
        uint64_t disp, uint8_t addressSize, bool rip)
{
    bool someAddr = false;
    printSegment(os, segment);
    os << ":[";
    if (rip) {
        os << "rip";
        someAddr = true;
    } else {
        if (scale != 0 && index != int_reg::NumRegs) {
            if (scale != 1)
                ccprintf(os, "%d*", scale);
            printReg(os, intRegClass[index], addressSize);
            someAddr = true;
        }
        if (base != int_reg::NumRegs) {
            if (someAddr)
                os << " + ";
            printReg(os, intRegClass[base], addressSize);
            someAddr = true;
        }
    }
    if (disp != 0) {
        if (someAddr)
            os << " + ";
        ccprintf(os, "%#x", disp);
        someAddr = true;
    }
    if (!someAddr)
        os << "0";
    os << "]";
}

std::string
X86StaticInst::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;

    printMnemonic(ss, mnemonic);

    return ss.str();
}

} // namespace X86ISA
} // namespace gem5
