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

#ifndef __ARCH_MIPS_REGFILE_REGFILE_HH__
#define __ARCH_MIPS_REGFILE_REGFILE_HH__

#include "arch/mips/types.hh"
#include "arch/mips/isa_traits.hh"
//#include "arch/mips/mt.hh"
#include "arch/mips/regfile/int_regfile.hh"
#include "arch/mips/regfile/float_regfile.hh"
#include "arch/mips/regfile/misc_regfile.hh"
//#include "cpu/base.hh"
#include "sim/faults.hh"

class BaseCPU;
class Checkpoint;
class EventManager;

namespace MipsISA
{
    class RegFile {
      protected:
        Addr pc;                        // program counter
        Addr npc;                       // next-cycle program counter
        Addr nnpc;                      // next-next-cycle program counter
                                        // used to implement branch delay slot
                                        // not real register

        IntRegFile intRegFile;          // (signed) integer register file
        FloatRegFile floatRegFile;      // floating point register file
        MiscRegFile miscRegFile;        // control register file

      public:
        void clear();
        void reset(std::string core_name, unsigned num_threads, unsigned num_vpes, BaseCPU *_cpu);
        MiscRegFile *getMiscRegFilePtr();

        IntReg readIntReg(int intReg);
        Fault setIntReg(int intReg, const IntReg &val);


        MiscReg readMiscRegNoEffect(int miscReg, unsigned tid = 0);
        MiscReg readMiscReg(int miscReg, ThreadContext *tc,
                            unsigned tid = 0);
        void setMiscRegNoEffect(int miscReg, const MiscReg &val, unsigned tid = 0);
        void setMiscReg(int miscReg, const MiscReg &val,
                        ThreadContext * tc, unsigned tid = 0);

        FloatRegVal readFloatReg(int floatReg);
        FloatRegVal readFloatReg(int floatReg, int width);
        FloatRegBits readFloatRegBits(int floatReg);
        FloatRegBits readFloatRegBits(int floatReg, int width);
        Fault setFloatReg(int floatReg, const FloatRegVal &val);
        Fault setFloatReg(int floatReg, const FloatRegVal &val, int width);
        Fault setFloatRegBits(int floatReg, const FloatRegBits &val);
        Fault setFloatRegBits(int floatReg, const FloatRegBits &val, int width);


        void setShadowSet(int css);

        int instAsid();
        int dataAsid();

      public:
        Addr readPC();
        void setPC(Addr val);

        Addr readNextPC();
        void setNextPC(Addr val);

        Addr readNextNPC();
        void setNextNPC(Addr val);

        void serialize(EventManager *em, std::ostream &os);
        void unserialize(EventManager *em, Checkpoint *cp,
            const std::string &section);

        void changeContext(RegContextParam param, RegContextVal val)
        {
        }

    };

} // namespace MipsISA

#endif
