/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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
 * Authors: Gabe Black
 */

#include "arch/x86/insts/static_inst.hh"

namespace X86ISA
{
    void X86StaticInst::printMnemonic(std::ostream &os,
            const char * mnemonic) const
    {
        ccprintf(os, "\t%s   ", mnemonic);
    }

    void X86StaticInst::printMnemonic(std::ostream &os,
            const char * instMnemonic, const char * mnemonic) const
    {
        ccprintf(os, "\t%s : %s   ", instMnemonic, mnemonic);
    }

    void X86StaticInst::printSegment(std::ostream &os, int segment) const
    {
        switch (segment)
        {
          case 0:
            ccprintf(os, "ES");
            break;
          case 1:
            ccprintf(os, "CS");
            break;
          case 2:
            ccprintf(os, "SS");
            break;
          case 3:
            ccprintf(os, "DS");
            break;
          case 4:
            ccprintf(os, "FS");
            break;
          case 5:
            ccprintf(os, "GS");
            break;
          default:
            panic("Unrecognized segment %d\n", segment);
        }
    }

    void
    X86StaticInst::printSrcReg(std::ostream &os, int reg) const
    {
        if(_numSrcRegs > reg)
            printReg(os, _srcRegIdx[reg]);
    }

    void
    X86StaticInst::printDestReg(std::ostream &os, int reg) const
    {
        if(_numDestRegs > reg)
            printReg(os, _destRegIdx[reg]);
    }

    void
    X86StaticInst::printReg(std::ostream &os, int reg) const
    {
        if (reg < FP_Base_DepTag) {
            //FIXME These should print differently depending on the
            //mode etc, but for now this will get the point across
            switch (reg) {
              case INTREG_RAX:
                ccprintf(os, "rax");
                break;
              case INTREG_RBX:
                ccprintf(os, "rbx");
                break;
              case INTREG_RCX:
                ccprintf(os, "rcx");
                break;
              case INTREG_RDX:
                ccprintf(os, "rdx");
                break;
              case INTREG_RSP:
                ccprintf(os, "rsp");
                break;
              case INTREG_RBP:
                ccprintf(os, "rbp");
                break;
              case INTREG_RSI:
                ccprintf(os, "rsi");
                break;
              case INTREG_RDI:
                ccprintf(os, "rdi");
                break;
              case INTREG_R8W:
                ccprintf(os, "r8");
                break;
              case INTREG_R9W:
                ccprintf(os, "r9");
                break;
              case INTREG_R10W:
                ccprintf(os, "r10");
                break;
              case INTREG_R11W:
                ccprintf(os, "r11");
                break;
              case INTREG_R12W:
                ccprintf(os, "r12");
                break;
              case INTREG_R13W:
                ccprintf(os, "r13");
                break;
              case INTREG_R14W:
                ccprintf(os, "r14");
                break;
              case INTREG_R15W:
                ccprintf(os, "r15");
                break;
              default:
                ccprintf(os, "t%d", reg - NUM_INTREGS);
            }
        } else if (reg < Ctrl_Base_DepTag) {
            ccprintf(os, "%%f%d", reg - FP_Base_DepTag);
        } else {
            switch (reg - Ctrl_Base_DepTag) {
              default:
                ccprintf(os, "%%ctrl%d", reg - Ctrl_Base_DepTag);
            }
        }
    }

    std::string X86StaticInst::generateDisassembly(Addr pc,
        const SymbolTable *symtab) const
    {
        std::stringstream ss;

        printMnemonic(ss, mnemonic);

        return ss.str();
    }
}
