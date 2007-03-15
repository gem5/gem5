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
        Rex = 7,
        //There can be only one segment override, so they share the
        //first 3 bits in the legacyPrefixes bitfield.
        SegmentOverride = 0x7,
        OperandSizeOverride = 8,
        AddressSizeOverride = 16,
        Lock = 32,
        Rep = 64,
        Repne = 128
    };

    //The intermediate structure the x86 predecoder returns.
    struct ExtMachInst
    {
      public: //XXX These should be hidden in the future

        uint8_t legacyPrefixes;
        uint8_t rexPrefix;
        bool twoByteOpcode;
        uint8_t opcode;
        uint64_t immediate;
        uint64_t displacement;

      public:

        //These are to pacify the decoder for now. This will go away once
        //it can handle non integer inputs, and in the mean time allow me to
        //excercise the predecoder a little.
        operator unsigned int()
        {
            return 0;
        }

        ExtMachInst(unsigned int)
        {;}

        ExtMachInst()
        {;}
    };

    inline static std::ostream &
        operator << (std::ostream & os, const ExtMachInst & emi)
    {
        os << "{X86 ExtMachInst}";
        return os;
    }

    inline static bool
        operator == (const ExtMachInst &emi1, const ExtMachInst &emi2)
    {
        //Since this is empty, it's always equal
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
