/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 *
 * Authors: Korey Sewell
 */

#ifndef __ARCH_MIPS_REGFILE_FLOAT_REGFILE_HH__
#define __ARCH_MIPS_REGFILE_FLOAT_REGFILE_HH__

#include "arch/mips/types.hh"
#include "arch/mips/isa_traits.hh"
#include "base/misc.hh"
#include "base/bitfield.hh"
#include "sim/faults.hh"

#include <string>

class Checkpoint;

namespace MipsISA
{
    const uint32_t MIPS32_QNAN = 0x7fbfffff;
    const uint64_t MIPS64_QNAN = ULL(0x7fbfffffffffffff);

    enum FPControlRegNums {
       FIR = NumFloatArchRegs,
       FCCR,
       FEXR,
       FENR,
       FCSR
    };

    enum FCSRBits {
        Inexact = 1,
        Underflow,
        Overflow,
        DivideByZero,
        Invalid,
        Unimplemented
    };

    enum FCSRFields {
        Flag_Field = 1,
        Enable_Field = 6,
        Cause_Field = 11
    };

    const int SingleWidth = 32;
    const int SingleBytes = SingleWidth / 4;

    const int DoubleWidth = 64;
    const int DoubleBytes = DoubleWidth / 4;

    const int QuadWidth = 128;
    const int QuadBytes = QuadWidth / 4;

    class FloatRegFile
    {
      protected:
        FloatReg32 regs[NumFloatRegs];

      public:
        static const int regWidth = SingleWidth;

        void clear();
        double readReg(int floatReg, int width, unsigned tid = 0);
        FloatRegBits readRegBits(int floatReg, int width, unsigned tid = 0);
        Fault setReg(int floatReg, const FloatRegVal &val, int width, unsigned tid = 0);
        Fault setRegBits(int floatReg, const FloatRegBits &val, int width, unsigned tid = 0);

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);
    };

} // namespace MipsISA

#endif
