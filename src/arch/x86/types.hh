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

#ifndef __ARCH_X86_TYPES_HH__
#define __ARCH_X86_TYPES_HH__

#include <inttypes.h>
#include <iostream>

#include "base/bitfield.hh"
#include "base/cprintf.hh"

namespace X86ISA
{
    //This really determines how many bytes are passed to the predecoder.
    typedef uint64_t MachInst;

    enum Prefixes {
        NoOverride = 0,
        CSOverride = 1,
        DSOverride = 2,
        ESOverride = 3,
        FSOverride = 4,
        GSOverride = 5,
        SSOverride = 6,
        //The Rex prefix obviously doesn't fit in with the above, but putting
        //it here lets us save double the space the enums take up.
        RexPrefix = 7,
        //There can be only one segment override, so they share the
        //first 3 bits in the legacyPrefixes bitfield.
        SegmentOverride = 0x7,
        OperandSizeOverride = 8,
        AddressSizeOverride = 16,
        Lock = 32,
        Rep = 64,
        Repne = 128
    };

    BitUnion8(ModRM)
        Bitfield<7,6> mod;
        Bitfield<5,3> reg;
        Bitfield<2,0> rm;
    EndBitUnion(ModRM)

    BitUnion8(Sib)
        Bitfield<7,6> scale;
        Bitfield<5,3> index;
        Bitfield<2,0> base;
    EndBitUnion(Sib)

    BitUnion8(Rex)
        Bitfield<3> w;
        Bitfield<2> r;
        Bitfield<1> x;
        Bitfield<0> b;
    EndBitUnion(Rex)

    BitUnion8(Opcode)
        Bitfield<7,3> top5;
        Bitfield<2,0> bottom3;
    EndBitUnion(Opcode)

    //The intermediate structure the x86 predecoder returns.
    struct ExtMachInst
    {
        //Prefixes
        uint8_t legacy;
        Rex rex;
        //This holds all of the bytes of the opcode
        struct
        {
            //The number of bytes in this opcode. Right now, we ignore that
            //this can be 3 in some cases
            uint8_t num;
            //The first byte detected in a 2+ byte opcode. Should be 0xF0.
            uint8_t prefixA;
            //The second byte detected in a 3+ byte opcode. Could be 0xF0 for
            //3dnow instructions, or 0x38-0x3F for some SSE instructions.
            uint8_t prefixB;
            //The main opcode byte. The highest addressed byte in the opcode.
            Opcode op;
        } opcode;
        //Modifier bytes
        ModRM modRM;
        uint8_t sib;
        //Immediate fields
        uint64_t immediate;
        uint64_t displacement;
    };

    inline static std::ostream &
        operator << (std::ostream & os, const ExtMachInst & emi)
    {
        ccprintf(os, "\n{\n\tleg = %#x,\n\trex = %#x,\n\t"
                     "op = {\n\t\tnum = %d,\n\t\top = %#x,\n\t\t"
                           "prefixA = %#x,\n\t\tprefixB = %#x\n\t},\n\t"
                     "modRM = %#x,\n\tsib = %#x,\n\t"
                     "immediate = %#x,\n\tdisplacement = %#x\n}\n",
                     emi.legacy, (uint8_t)emi.rex,
                     emi.opcode.num, emi.opcode.op,
                     emi.opcode.prefixA, emi.opcode.prefixB,
                     (uint8_t)emi.modRM, (uint8_t)emi.sib,
                     emi.immediate, emi.displacement);
        return os;
    }

    inline static bool
        operator == (const ExtMachInst &emi1, const ExtMachInst &emi2)
    {
        if(emi1.legacy != emi2.legacy)
            return false;
        if(emi1.rex != emi2.rex)
            return false;
        if(emi1.opcode.num != emi2.opcode.num)
            return false;
        if(emi1.opcode.op != emi2.opcode.op)
            return false;
        if(emi1.opcode.prefixA != emi2.opcode.prefixA)
            return false;
        if(emi1.opcode.prefixB != emi2.opcode.prefixB)
            return false;
        if(emi1.modRM != emi2.modRM)
            return false;
        if(emi1.sib != emi2.sib)
            return false;
        if(emi1.immediate != emi2.immediate)
            return false;
        if(emi1.displacement != emi2.displacement)
            return false;
        return true;
    }

    typedef uint64_t IntReg;
    //XXX Should this be a 128 bit structure for XMM memory ops?
    typedef uint64_t LargestRead;
    typedef uint64_t MiscReg;

    //These floating point types are correct for mmx, but not
    //technically for x87 (80 bits) or at all for xmm (128 bits)
    typedef double FloatReg;
    typedef uint64_t FloatRegBits;
    typedef union
    {
        IntReg intReg;
        FloatReg fpReg;
        MiscReg ctrlReg;
    } AnyReg;

    //XXX This is very hypothetical. X87 instructions would need to
    //change their "context" constantly. It's also not clear how
    //this would be handled as far as out of order execution.
    //Maybe x87 instructions are in order?
    enum RegContextParam
    {
        CONTEXT_X87_TOP
    };

    typedef int RegContextVal;

    typedef uint8_t RegIndex;
};

#endif // __ARCH_X86_TYPES_HH__
