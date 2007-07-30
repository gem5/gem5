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

#ifndef __ARCH_X86_INSTS_MICROLDSTOP_HH__
#define __ARCH_X86_INSTS_MICROLDSTOP_HH__

#include "arch/x86/insts/microop.hh"

namespace X86ISA
{
    /**
     * Base class for load and store ops
     */
    class LdStOp : public X86MicroopBase
    {
      protected:
        const uint8_t scale;
        const RegIndex index;
        const RegIndex base;
        const uint64_t disp;
        const uint8_t segment;
        const RegIndex data;
        const uint8_t dataSize;
        const uint8_t addressSize;
        RegIndex foldOBit, foldABit;

        //Constructor
        LdStOp(ExtMachInst _machInst,
                const char * mnem, const char * _instMnem,
                bool isMicro, bool isDelayed, bool isFirst, bool isLast,
                uint8_t _scale, RegIndex _index, RegIndex _base,
                uint64_t _disp, uint8_t _segment,
                RegIndex _data,
                uint8_t _dataSize, uint8_t _addressSize,
                OpClass __opClass) :
        X86MicroopBase(machInst, mnem, _instMnem,
                isMicro, isDelayed, isFirst, isLast, __opClass),
                scale(_scale), index(_index), base(_base),
                disp(_disp), segment(_segment),
                data(_data),
                dataSize(_dataSize), addressSize(_addressSize)
        {
            foldOBit = (dataSize == 1 && !_machInst.rex.present) ? 1 << 6 : 0;
            foldABit =
                (addressSize == 1 && !_machInst.rex.present) ? 1 << 6 : 0;
        }

        std::string generateDisassembly(Addr pc,
            const SymbolTable *symtab) const;

        template<class Context, class MemType>
        Fault read(Context *xc, Addr EA, MemType & Mem, unsigned flags) const
        {
            Fault fault = NoFault;
            int size = dataSize;
            Addr alignedEA = EA & ~(dataSize - 1);
            if (EA != alignedEA)
                size *= 2;
            switch(size)
            {
              case 1:
                fault = xc->read(alignedEA, (uint8_t&)Mem, flags);
                break;
              case 2:
                fault = xc->read(alignedEA, (uint16_t&)Mem, flags);
                break;
              case 4:
                fault = xc->read(alignedEA, (uint32_t&)Mem, flags);
                break;
              case 8:
                fault = xc->read(alignedEA, (uint64_t&)Mem, flags);
                break;
              default:
                panic("Bad operand size %d!\n", size);
            }
            return fault;
        }

        template<class Context, class MemType>
        Fault write(Context *xc, MemType & Mem, Addr EA, unsigned flags) const
        {
            Fault fault = NoFault;
            int size = dataSize;
            Addr alignedEA = EA & ~(dataSize - 1);
            if (EA != alignedEA)
                size *= 2;
            switch(size)
            {
              case 1:
                fault = xc->write((uint8_t&)Mem, alignedEA, flags, 0);
                break;
              case 2:
                fault = xc->write((uint16_t&)Mem, alignedEA, flags, 0);
                break;
              case 4:
                fault = xc->write((uint32_t&)Mem, alignedEA, flags, 0);
                break;
              case 8:
                fault = xc->write((uint64_t&)Mem, alignedEA, flags, 0);
                break;
              default:
                panic("Bad operand size %d!\n", size);
            }
            return fault;
        }
    };
}

#endif //__ARCH_X86_INSTS_MICROLDSTOP_HH__
