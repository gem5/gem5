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

#ifndef __ARCH_ARM_REGFILE_MISC_REGFILE_HH__
#define __ARCH_ARM_REGFILE_MISC_REGFILE_HH__

#include "arch/arm/isa_traits.hh"
#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "sim/faults.hh"

class ThreadContext;

namespace ArmISA
{
    const int NumMiscRegs = NUM_MISCREGS;

    static inline std::string getMiscRegName(RegIndex)
    {
        return "";
    }

    class MiscRegFile {

      protected:
        MiscReg miscRegFile[NumMiscRegs];

      public:
        void clear()
        {
            // Unknown startup state in misc register file currently
        }

        void copyMiscRegs(ThreadContext *tc);

        MiscReg readRegNoEffect(int misc_reg)
        {
            assert(misc_reg < NumMiscRegs);
            return miscRegFile[misc_reg];
        }

        MiscReg readReg(int misc_reg, ThreadContext *tc)
        {
            assert(misc_reg < NumMiscRegs);
            return miscRegFile[misc_reg];
        }

        void setRegNoEffect(int misc_reg, const MiscReg &val)
        {
            assert(misc_reg < NumMiscRegs);
            miscRegFile[misc_reg] = val;
        }

        void setReg(int misc_reg, const MiscReg &val,
                               ThreadContext *tc)
        {
            assert(misc_reg < NumMiscRegs);
            miscRegFile[misc_reg] = val;
        }

        friend class RegFile;
    };
} // namespace ArmISA

#endif
