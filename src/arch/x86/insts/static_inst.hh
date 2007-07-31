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

#ifndef __ARCH_X86_INSTS_STATICINST_HH__
#define __ARCH_X86_INSTS_STATICINST_HH__

#include "base/trace.hh"
#include "cpu/static_inst.hh"

namespace X86ISA
{
    /**
     * Base class for all X86 static instructions.
     */

    class X86StaticInst : public StaticInst
    {
      protected:
        // Constructor.
        X86StaticInst(const char *mnem,
             ExtMachInst _machInst, OpClass __opClass)
                : StaticInst(mnem, _machInst, __opClass)
            {
            }

        std::string generateDisassembly(Addr pc,
            const SymbolTable *symtab) const;

        void printMnemonic(std::ostream &os, const char * mnemonic) const;
        void printMnemonic(std::ostream &os, const char * instMnemonic,
                const char * mnemonic) const;

        void printSegment(std::ostream &os, int segment) const;

        void printReg(std::ostream &os, int reg, int size) const;
        void printSrcReg(std::ostream &os, int reg, int size) const;
        void printDestReg(std::ostream &os, int reg, int size) const;

        inline uint64_t merge(uint64_t into, uint64_t val, int size) const
        {
            X86IntReg reg = into;
            if(_destRegIdx[0] & (1 << 6))
            {
                reg.H = val;
                return reg;
            }
            switch(size)
            {
              case 1:
                reg.L = val;
                break;
              case 2:
                reg.X = val;
                break;
              case 4:
                //XXX Check if this should be zeroed or sign extended
                reg = 0;
                reg.E = val;
                break;
              case 8:
                reg.R = val;
                break;
              default:
                panic("Tried to merge with unrecognized size %d.\n", size);
            }
            return reg;
        }

        inline uint64_t pick(uint64_t from, int idx, int size) const
        {
            X86IntReg reg = from;
            DPRINTF(X86, "Picking with size %d\n", size);
            if(_srcRegIdx[idx] & (1 << 6))
                return reg.H;
            switch(size)
            {
              case 1:
                return reg.L;
              case 2:
                return reg.X;
              case 4:
                return reg.E;
              case 8:
                return reg.R;
              default:
                panic("Tried to pick with unrecognized size %d.\n", size);
            }
        }
    };
}

#endif //__ARCH_X86_INSTS_STATICINST_HH__
