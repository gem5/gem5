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

#ifndef __ARCH_X86_REGFILE_HH__
#define __ARCH_X86_REGFILE_HH__

#include "arch/x86/floatregfile.hh"
#include "arch/x86/intregfile.hh"
#include "arch/x86/isa_traits.hh"
#include "arch/x86/miscregfile.hh"
#include "arch/x86/types.hh"
#include "sim/host.hh"

#include <string>

class Checkpoint;

namespace X86ISA
{
    class RegFile
    {
      protected:
        Addr rip; //Program Counter
        Addr nextRip; //Next Program Counter

      public:
        Addr readPC();
        void setPC(Addr val);

        Addr readNextPC();
        void setNextPC(Addr val);

        Addr readNextNPC();
        void setNextNPC(Addr val);

      protected:
        IntRegFile intRegFile; // integer register file
        FloatRegFile floatRegFile; // floating point register file
        MiscRegFile miscRegFile; // control register file

      public:

        void clear();

        int FlattenIntIndex(int reg);

        MiscReg readMiscRegNoEffect(int miscReg);

        MiscReg readMiscReg(int miscReg, ThreadContext *tc);

        void setMiscRegNoEffect(int miscReg, const MiscReg &val);

        void setMiscReg(int miscReg, const MiscReg &val,
                ThreadContext * tc);

        int instAsid()
        {
            //XXX This doesn't make sense in x86
            return 0;
        }

        int dataAsid()
        {
            //XXX This doesn't make sense in x86
            return 0;
        }

        FloatReg readFloatReg(int floatReg, int width);

        FloatReg readFloatReg(int floatReg);

        FloatRegBits readFloatRegBits(int floatReg, int width);

        FloatRegBits readFloatRegBits(int floatReg);

        void setFloatReg(int floatReg, const FloatReg &val, int width);

        void setFloatReg(int floatReg, const FloatReg &val);

        void setFloatRegBits(int floatReg, const FloatRegBits &val, int width);

        void setFloatRegBits(int floatReg, const FloatRegBits &val);

        IntReg readIntReg(int intReg);

        void setIntReg(int intReg, const IntReg &val);

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);

      public:

        void changeContext(RegContextParam param, RegContextVal val);
    };

    int flattenIntIndex(ThreadContext * tc, int reg);

    void copyRegs(ThreadContext *src, ThreadContext *dest);

    void copyMiscRegs(ThreadContext *src, ThreadContext *dest);

    int InterruptLevel(uint64_t softint);

}; // namespace X86ISA

#endif // __ARCH_X86_REGFILE_HH__
