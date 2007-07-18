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

#include "arch/x86/insts/microregop.hh"
#include "arch/x86/miscregs.hh"
#include "base/condcodes.hh"
#include <string>

namespace X86ISA
{
    uint64_t RegOpBase::genFlags(uint64_t oldFlags, uint64_t flagMask,
            uint64_t _dest, uint64_t _src1, uint64_t _src2) const
    {
        uint64_t flags = oldFlags & ~flagMask;
        if(flagMask & CFBit && findCarry(dataSize, _dest, _src1, _src2))
            flags |= CFBit;
        if(flagMask & PFBit && findParity(dataSize, _dest))
            flags |= PFBit;
        if(flagMask & ECFBit && findCarry(dataSize, _dest, _src1, _src2))
            flags |= ECFBit;
        if(flagMask & AFBit && findCarry(4, _dest, _src1, _src2))
            flags |= AFBit;
        if(flagMask & EZFBit && findZero(dataSize, _dest))
            flags |= EZFBit;
        if(flagMask & ZFBit && findZero(dataSize, _dest))
            flags |= ZFBit;
        if(flagMask & SFBit && findNegative(dataSize, _dest))
            flags |= SFBit;
        if(flagMask & OFBit && findOverflow(dataSize, _dest, _src1, _src2))
            flags |= OFBit;
        return flags;
    }

    bool RegOpBase::checkCondition(uint64_t flags) const
    {
        CCFlagBits ccflags = flags;
        switch(ext)
        {
          case ConditionTests::True:
            return true;
          case ConditionTests::ECF:
            return ccflags.ECF;
          case ConditionTests::EZF:
            return ccflags.EZF;
          case ConditionTests::SZnZF:
            return !(!ccflags.EZF & ccflags.ZF);
          case ConditionTests::MSTRZ:
            panic("This condition is not implemented!");
          case ConditionTests::STRZ:
            panic("This condition is not implemented!");
          case ConditionTests::MSTRC:
            panic("This condition is not implemented!");
          case ConditionTests::STRZnZF:
            panic("This condition is not implemented!");
          case ConditionTests::OF:
            return ccflags.OF;
          case ConditionTests::CF:
            return ccflags.CF;
          case ConditionTests::ZF:
            return ccflags.ZF;
          case ConditionTests::CvZF:
            return ccflags.CF | ccflags.ZF;
          case ConditionTests::SF:
            return ccflags.SF;
          case ConditionTests::PF:
            return ccflags.PF;
          case ConditionTests::SxOF:
            return ccflags.SF ^ ccflags.OF;
          case ConditionTests::SxOvZF:
            return ccflags.SF ^ ccflags.OF | ccflags.ZF;
          case ConditionTests::False:
            return false;
          case ConditionTests::NotECF:
            return !ccflags.ECF;
          case ConditionTests::NotEZF:
            return !ccflags.EZF;
          case ConditionTests::NotSZnZF:
            return !ccflags.EZF & ccflags.ZF;
          case ConditionTests::NotMSTRZ:
            panic("This condition is not implemented!");
          case ConditionTests::NotSTRZ:
            panic("This condition is not implemented!");
          case ConditionTests::NotMSTRC:
            panic("This condition is not implemented!");
          case ConditionTests::NotSTRZnZF:
            panic("This condition is not implemented!");
          case ConditionTests::NotOF:
            return !ccflags.OF;
          case ConditionTests::NotCF:
            return !ccflags.CF;
          case ConditionTests::NotZF:
            return !ccflags.ZF;
          case ConditionTests::NotCvZF:
            return !(ccflags.CF | ccflags.ZF);
          case ConditionTests::NotSF:
            return !ccflags.SF;
          case ConditionTests::NotPF:
            return !ccflags.PF;
          case ConditionTests::NotSxOF:
            return !(ccflags.SF ^ ccflags.OF);
          case ConditionTests::NotSxOvZF:
            return !(ccflags.SF ^ ccflags.OF | ccflags.ZF);
        }
        panic("Unknown condition: %d\n", ext);
        return true;
    }

    std::string RegOp::generateDisassembly(Addr pc,
            const SymbolTable *symtab) const
    {
        std::stringstream response;

        printMnemonic(response, instMnem, mnemonic);
        printReg(response, dest, dataSize);
        response << ", ";
        printReg(response, src1, dataSize);
        response << ", ";
        printReg(response, src2, dataSize);
        return response.str();
    }

    std::string RegOpImm::generateDisassembly(Addr pc,
            const SymbolTable *symtab) const
    {
        std::stringstream response;

        printMnemonic(response, instMnem, mnemonic);
        printReg(response, dest, dataSize);
        response << ", ";
        printReg(response, src1, dataSize);
        ccprintf(response, ", %#x", imm8);
        return response.str();
    }
}
