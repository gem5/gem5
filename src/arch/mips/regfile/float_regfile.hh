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
 *
 * Authors: Korey Sewell
 */

#ifndef __ARCH_MIPS_FLOAT_REGFILE_HH__
#define __ARCH_MIPS_FLOAT_REGFILE_HH__

#include "arch/mips/types.hh"
#include "arch/mips/constants.hh"
#include "base/misc.hh"
#include "config/full_system.hh"
#include "sim/byteswap.hh"
#include "sim/faults.hh"
#include "sim/host.hh"

class Checkpoint;
class ExecContext;
class Regfile;

namespace MipsISA
{
    class FloatRegFile
    {
      protected:
        FloatReg32 regs[NumFloatRegs];

      public:

        void clear()
        {
            bzero(regs, sizeof(regs));
        }

        double readReg(int floatReg, int width)
        {
            switch(width)
            {
              case SingleWidth:
                void *float_ptr = &regs[floatReg];
                return *(float *) float_ptr;

              case DoubleWidth:
                uint64_t double_val = (FloatReg64)regs[floatReg + 1] << 32 | regs[floatReg];
                void *double_ptr = &double_val;
                return *(double *) double_ptr;

              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
        }

        FloatRegBits readRegBits(int floatReg, int width)
        {
            if (floatReg < NumFloatArchRegs - 1) {
                switch(width)
                {
                  case SingleWidth:
                    return regs[floatReg];

                  case DoubleWidth:
                    return (FloatReg64)regs[floatReg + 1] << 32 | regs[floatReg];

                  default:
                    panic("Attempted to read a %d bit floating point register!", width);
                }
            } else {
                if (width > SingleWidth)
                    assert("Control Regs are only 32 bits wide");

                return regs[floatReg];
            }
        }

        Fault setReg(int floatReg, const FloatReg &val, int width)
        {

            switch(width)
            {
              case SingleWidth:
                float temp = val;
                void *float_ptr = &temp;
                regs[floatReg] = *(FloatReg32 *) float_ptr;
                break;

              case DoubleWidth:
                const void *double_ptr = &val;
                FloatReg64 temp_double = *(FloatReg64 *) double_ptr;
                regs[floatReg + 1] = temp_double >> 32;
                regs[floatReg] = temp_double;
                break;

              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }

            return NoFault;
        }

        Fault setRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            using namespace std;

            switch(width)
            {
              case SingleWidth:
                regs[floatReg] = val;
                break;

              case DoubleWidth:
                regs[floatReg + 1] = val >> 32;
                regs[floatReg] = val;
                break;

              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
            return NoFault;
        }

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);
    };

    enum MiscFloatRegNums {
       FIR = NumFloatArchRegs,
       FCCR,
       FEXR,
       FENR,
       FCSR
    };

} // namespace MipsISA

#endif
