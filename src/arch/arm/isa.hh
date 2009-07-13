/*
 * Copyright (c) 2009 The Regents of The University of Michigan
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
 */

#ifndef __ARCH_ARM_ISA_HH__
#define __ARCH_MRM_ISA_HH__

#include "arch/arm/registers.hh"
#include "arch/arm/types.hh"

class ThreadContext;
class Checkpoint;
class EventManager;

namespace ArmISA
{
    class ISA
    {
      protected:
        MiscReg miscRegs[NumMiscRegs];

      public:
        void clear()
        {
            // Unknown startup state currently
        }

        MiscReg
        readMiscRegNoEffect(int misc_reg)
        {
            assert(misc_reg < NumMiscRegs);
            return miscRegs[misc_reg];
        }

        MiscReg
        readMiscReg(int misc_reg, ThreadContext *tc)
        {
            assert(misc_reg < NumMiscRegs);
            return miscRegs[misc_reg];
        }

        void
        setMiscRegNoEffect(int misc_reg, const MiscReg &val)
        {
            assert(misc_reg < NumMiscRegs);
            miscRegs[misc_reg] = val;
        }

        void
        setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc)
        {
            assert(misc_reg < NumMiscRegs);
            miscRegs[misc_reg] = val;
        }

        int
        flattenIntIndex(int reg)
        {
            return reg;
        }

        int
        flattenFloatIndex(int reg)
        {
            return reg;
        }

        void serialize(std::ostream &os)
        {}
        void unserialize(Checkpoint *cp, const std::string &section)
        {}

        ISA()
        {
            clear();
        }
    };
}

#endif
