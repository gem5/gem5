/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 */

#ifndef __ARCH_MIPS_TYPES_HH__
#define __ARCH_MIPS_TYPES_HH__

#include "sim/host.hh"

namespace MipsISA
{
    typedef uint32_t MachInst;
    typedef uint64_t ExtMachInst;
    typedef uint8_t  RegIndex;

    typedef uint32_t IntReg;

    // floating point register file entry type
    typedef double FloatReg;
    typedef uint32_t FloatReg32;
    typedef uint64_t FloatReg64;
    typedef uint64_t FloatRegBits;

    // cop-0/cop-1 system control register
    typedef uint64_t MiscReg;
    typedef uint64_t InternalProcReg;

    typedef union {
        IntReg   intreg;
        FloatReg fpreg;
        MiscReg  ctrlreg;
    } AnyReg;

    //used in FP convert & round function
    enum ConvertType{
        SINGLE_TO_DOUBLE,
        SINGLE_TO_WORD,
        SINGLE_TO_LONG,

        DOUBLE_TO_SINGLE,
        DOUBLE_TO_WORD,
        DOUBLE_TO_LONG,

        LONG_TO_SINGLE,
        LONG_TO_DOUBLE,
        LONG_TO_WORD,
        LONG_TO_PS,

        WORD_TO_SINGLE,
        WORD_TO_DOUBLE,
        WORD_TO_LONG,
        WORD_TO_PS,

        PL_TO_SINGLE,
        PU_TO_SINGLE
    };

    //used in FP convert & round function
    enum RoundMode{
        RND_ZERO,
        RND_DOWN,
        RND_UP,
        RND_NEAREST
   };

} // namespace MipsISA

#endif
