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

#ifndef __ARCH_X86_INSTS_MICROREGOP_HH__
#define __ARCH_X86_INSTS_MICROREGOP_HH__

#include "arch/x86/insts/microop.hh"

namespace X86ISA
{
    namespace ConditionTests
    {
        enum CondTest {
            True,
            NotFalse = True,
            ECF,
            EZF,
            SZnZF,
            MSTRZ,
            STRZ,
            MSTRC,
            STRZnZF,
            OF,
            CF,
            ZF,
            CvZF,
            SF,
            PF,
            SxOF,
            SxOvZF,

            False,
            NotTrue = False,
            NotECF,
            NotEZF,
            NotSZnZF,
            NotMSTRZ,
            NotSTRZ,
            NotMSTRC,
            NotSTRZnZF,
            NotOF,
            NotCF,
            NotZF,
            NotCvZF,
            NotSF,
            NotPF,
            NotSxOF,
            NotSxOvZF
        };
    }

    /**
     * Base classes for RegOps which provides a generateDisassembly method.
     */
    class RegOpBase : public X86MicroopBase
    {
      protected:
        const RegIndex src1;
        const RegIndex dest;
        const uint8_t dataSize;
        const uint16_t ext;

        // Constructor
        RegOpBase(ExtMachInst _machInst,
                const char *mnem, const char *_instMnem,
                bool isMicro, bool isDelayed,
                bool isFirst, bool isLast,
                RegIndex _src1, RegIndex _dest,
                uint8_t _dataSize, uint16_t _ext,
                OpClass __opClass) :
            X86MicroopBase(_machInst, mnem, _instMnem,
                    isMicro, isDelayed, isFirst, isLast,
                    __opClass),
            src1(_src1), dest(_dest),
            dataSize(_dataSize), ext(_ext)
        {
        }

        //Figure out what the condition code flags should be.
        uint64_t genFlags(uint64_t oldFlags, uint64_t flagMask,
                uint64_t _dest, uint64_t _src1, uint64_t _src2) const;
        bool checkCondition(uint64_t flags) const;
    };

    class RegOp : public RegOpBase
    {
      protected:
        const RegIndex src2;

        // Constructor
        RegOp(ExtMachInst _machInst,
                const char *mnem, const char *_instMnem,
                bool isMicro, bool isDelayed,
                bool isFirst, bool isLast,
                RegIndex _src1, RegIndex _src2, RegIndex _dest,
                uint8_t _dataSize, uint16_t _ext,
                OpClass __opClass) :
            RegOpBase(_machInst, mnem, _instMnem,
                    isMicro, isDelayed, isFirst, isLast,
                    _src1, _dest, _dataSize, _ext,
                    __opClass),
            src2(_src2)
        {
        }

        std::string generateDisassembly(Addr pc,
            const SymbolTable *symtab) const;
    };

    class RegOpImm : public RegOpBase
    {
      protected:
        const uint8_t imm8;

        // Constructor
        RegOpImm(ExtMachInst _machInst,
                const char * mnem, const char *_instMnem,
                bool isMicro, bool isDelayed,
                bool isFirst, bool isLast,
                RegIndex _src1, uint8_t _imm8, RegIndex _dest,
                uint8_t _dataSize, uint16_t _ext,
                OpClass __opClass) :
            RegOpBase(_machInst, mnem, _instMnem,
                    isMicro, isDelayed, isFirst, isLast,
                    _src1, _dest, _dataSize, _ext,
                    __opClass),
            imm8(_imm8)
        {
        }

        std::string generateDisassembly(Addr pc,
            const SymbolTable *symtab) const;
    };
}

#endif //__ARCH_X86_INSTS_MICROREGOP_HH__
