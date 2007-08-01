/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
#include "arch/mips/regfile/int_regfile.hh"
#include "arch/mips/regfile/float_regfile.hh"
#include "arch/mips/regfile/misc_regfile.hh"
#include "sim/faults.hh"

class Checkpoint;
class ThreadContext;

namespace MipsISA
{
    class RegFile {
      protected:
        IntRegFile intRegFile;		// (signed) integer register file
        FloatRegFile floatRegFile;	// floating point register file
        MiscRegFile miscRegFile;	// control register file

      public:
        void clear()
        {
            intRegFile.clear();
            floatRegFile.clear();
            miscRegFile.clear();
        }

        void reset(std::string core_name, unsigned num_threads, unsigned num_vpes)
        {
            bzero(&intRegFile, sizeof(intRegFile));
            bzero(&floatRegFile, sizeof(floatRegFile));
            miscRegFile.reset(core_name, num_threads, num_vpes);
        }

        IntReg readIntReg(int intReg)
        {
            return intRegFile.readReg(intReg);
        }

        Fault setIntReg(int intReg, const IntReg &val)
        {
            return intRegFile.setReg(intReg, val);
        }

        MiscReg readMiscRegNoEffect(int miscReg, unsigned tid = 0)
        {
            return miscRegFile.readRegNoEffect(miscReg, tid);
        }

        MiscReg readMiscReg(int miscReg, ThreadContext *tc,
                                      unsigned tid = 0)
        {
            return miscRegFile.readReg(miscReg, tc, tid);
        }

        void setMiscRegNoEffect(int miscReg, const MiscReg &val, unsigned tid = 0)
        {
            miscRegFile.setRegNoEffect(miscReg, val, tid);
        }

        void setMiscReg(int miscReg, const MiscReg &val,
                ThreadContext * tc, unsigned tid = 0)
        {
            miscRegFile.setReg(miscReg, val, tc, tid);
        }

        FloatRegVal readFloatReg(int floatReg)
        {
            return floatRegFile.readReg(floatReg,SingleWidth);
        }

        FloatRegVal readFloatReg(int floatReg, int width)
        {
            return floatRegFile.readReg(floatReg,width);
        }

        FloatRegBits readFloatRegBits(int floatReg)
        {
            return floatRegFile.readRegBits(floatReg,SingleWidth);
        }

        FloatRegBits readFloatRegBits(int floatReg, int width)
        {
            return floatRegFile.readRegBits(floatReg,width);
        }

        Fault setFloatReg(int floatReg, const FloatRegVal &val)
        {
            return floatRegFile.setReg(floatReg, val, SingleWidth);
        }

        Fault setFloatReg(int floatReg, const FloatRegVal &val, int width)
        {
            return floatRegFile.setReg(floatReg, val, width);
        }

        Fault setFloatRegBits(int floatReg, const FloatRegBits &val)
        {
            return floatRegFile.setRegBits(floatReg, val, SingleWidth);
        }

        Fault setFloatRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            return floatRegFile.setRegBits(floatReg, val, width);
        }

      protected:

        Addr pc;			// program counter
        Addr npc;			// next-cycle program counter
        Addr nnpc;			// next-next-cycle program counter
                                        // used to implement branch delay slot
                                        // not real register

      public:
        Addr readPC()
        {
            return pc;
        }

        void setPC(Addr val)
        {
            pc = val;
        }

        Addr readNextPC()
        {
            return npc;
        }

        void setNextPC(Addr val)
        {
            npc = val;
        }

        Addr readNextNPC()
        {
            return nnpc;
        }

        void setNextNPC(Addr val)
        {
            nnpc = val;
        }

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);

        void changeContext(RegContextParam param, RegContextVal val)
        {
        }
    };

    static inline int flattenIntIndex(ThreadContext * tc, int reg)
    {
        return reg;
    }

    void
    copyRegs(ThreadContext *src, ThreadContext *dest);

    void
    copyMiscRegs(ThreadContext *src, ThreadContext *dest);

} // namespace MipsISA

#endif
