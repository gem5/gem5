/*
 * Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Stephen Hines
 */

#ifndef __ARCH_ARM_REGFILE_FLOAT_REGFILE_HH__
#define __ARCH_ARM_REGFILE_FLOAT_REGFILE_HH__

#include "arch/arm/types.hh"
#include "arch/arm/isa_traits.hh"
#include "base/misc.hh"
#include "base/bitfield.hh"
#include "sim/faults.hh"
#include "sim/serialize.hh"

#include <string>

class Checkpoint;

namespace ArmISA
{
    static inline std::string getFloatRegName(RegIndex)
    {
        return "";
    }

    const uint32_t ARM32_QNAN = 0x7fbfffff;
    const uint64_t ARM64_QNAN = ULL(0x7fbfffffffffffff);

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

    class FloatRegFile
    {
      protected:
          union {
            FloatRegBits qregs[NumFloatRegs];
            FloatReg regs[NumFloatRegs];
          };

      public:

        void clear()
        {
            bzero(regs, sizeof(regs));
            regs[8] = 0.0;
            regs[9] = 1.0;
            regs[10] = 2.0;
            regs[11] = 3.0;
            regs[12] = 4.0;
            regs[13] = 5.0;
            regs[14] = 0.5;
            regs[15] = 10.0;
        }

        FloatReg readReg(int floatReg)
        {
            return regs[floatReg];
        }

        FloatRegBits readRegBits(int floatReg)
        {
            return qregs[floatReg];
        }

        Fault setReg(int floatReg, const FloatReg &val)
        {
            if (floatReg > 7)
                panic("Writing to a hard-wired FP register");
            regs[floatReg] = val;
            return NoFault;
        }

        Fault setRegBits(int floatReg, const FloatRegBits &val)
        {
            if (floatReg > 7)
                panic("Writing to a hard-wired FP register");
            qregs[floatReg] = val;
            return NoFault;
        }

        void serialize(std::ostream &os)
        {
            SERIALIZE_ARRAY(regs, NumFloatRegs);
        }

        void unserialize(Checkpoint *cp, const std::string &section)
        {
            UNSERIALIZE_ARRAY(regs, NumFloatRegs);
        }
    };

} // namespace ArmISA

#endif
