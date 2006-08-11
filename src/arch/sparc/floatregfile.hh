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
 * Authors: Gabe Black
 *          Ali Saidi
 */

#ifndef __ARCH_SPARC_FLOATREGFILE_HH__
#define __ARCH_SPARC_FLOATREGFILE_HH__

#include "arch/sparc/faults.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/types.hh"

#include <string>

namespace SparcISA
{
    std::string getFloatRegName(RegIndex);

    typedef float float32_t;
    typedef double float64_t;
    //FIXME long double refers to a 10 byte float, rather than a
    //16 byte float as required. This data type may have to be emulated.
    typedef double float128_t;

    class FloatRegFile
    {
      public:
        static const int SingleWidth = 32;
        static const int DoubleWidth = 64;
        static const int QuadWidth = 128;

      protected:

        //Since the floating point registers overlap each other,
        //A generic storage space is used. The float to be returned is
        //pulled from the appropriate section of this region.
        char regSpace[(SingleWidth / 8) * NumFloatRegs];

      public:

        void clear();

        FloatReg readReg(int floatReg, int width);

        FloatRegBits readRegBits(int floatReg, int width);

        Fault setReg(int floatReg, const FloatReg &val, int width);

        Fault setRegBits(int floatReg, const FloatRegBits &val, int width);

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);
    };
}

#endif
