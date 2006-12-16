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

#ifndef __ARCH_SPARC_REGFILE_HH__
#define __ARCH_SPARC_REGFILE_HH__

#include "arch/sparc/floatregfile.hh"
#include "arch/sparc/intregfile.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/miscregfile.hh"
#include "arch/sparc/types.hh"
#include "sim/host.hh"

#include <string>

class Checkpoint;

namespace SparcISA
{
    class RegFile
    {
      protected:
        Addr pc;		// Program Counter
        Addr npc;		// Next Program Counter
        Addr nnpc;

      public:
        Addr readPC();
        void setPC(Addr val);

        Addr readNextPC();
        void setNextPC(Addr val);

        Addr readNextNPC();
        void setNextNPC(Addr val);

      protected:
        IntRegFile intRegFile;		// integer register file
        FloatRegFile floatRegFile;	// floating point register file
        MiscRegFile miscRegFile;	// control register file

      public:

        void clear();

        int FlattenIntIndex(int reg);

        MiscReg readMiscReg(int miscReg);

        MiscReg readMiscRegWithEffect(int miscReg, ThreadContext *tc);

        void setMiscReg(int miscReg, const MiscReg &val);

        void setMiscRegWithEffect(int miscReg, const MiscReg &val,
                ThreadContext * tc);

        int instAsid()
        {
            return miscRegFile.getInstAsid();
        }

        int dataAsid()
        {
            return miscRegFile.getDataAsid();
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

} // namespace SparcISA

#endif
