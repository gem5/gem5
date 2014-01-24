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

#ifndef __ARCH_X86_ISA_HH__
#define __ARCH_X86_ISA_HH__

#include <iostream>
#include <string>

#include "arch/x86/regs/float.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/registers.hh"
#include "base/types.hh"
#include "sim/sim_object.hh"

class Checkpoint;
class EventManager;
class ThreadContext;
struct X86ISAParams;

namespace X86ISA
{
    class ISA : public SimObject
    {
      protected:
        MiscReg regVal[NUM_MISCREGS];
        void updateHandyM5Reg(Efer efer, CR0 cr0,
                SegAttr csAttr, SegAttr ssAttr, RFLAGS rflags,
                ThreadContext *tc);

      public:
        typedef X86ISAParams Params;

        void clear();

        ISA(Params *p);
        const Params *params() const;

        MiscReg readMiscRegNoEffect(int miscReg);
        MiscReg readMiscReg(int miscReg, ThreadContext *tc);

        void setMiscRegNoEffect(int miscReg, MiscReg val);
        void setMiscReg(int miscReg, MiscReg val, ThreadContext *tc);

        int
        flattenIntIndex(int reg)
        {
            return reg & ~IntFoldBit;
        }

        int
        flattenFloatIndex(int reg)
        {
            if (reg >= NUM_FLOATREGS) {
                reg = FLOATREG_STACK(reg - NUM_FLOATREGS,
                                     regVal[MISCREG_X87_TOP]);
            }
            return reg;
        }

        int
        flattenCCIndex(int reg)
        {
            return reg;
        }

        int
        flattenMiscIndex(int reg)
        {
            return reg;
        }

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);
        void startup(ThreadContext *tc);

        /// Explicitly import the otherwise hidden startup
        using SimObject::startup;

    };
}

#endif
