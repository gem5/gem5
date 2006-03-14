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
 */

#ifndef __ARCH_ALPHA_REGFILE_HH__
#define __ARCH_ALPHA_REGFILE_HH__

#include "arch/alpha/types.hh"
#include "arch/alpha/constants.hh"
#include "sim/faults.hh"

class Checkpoint;
class ExecContext;

namespace AlphaISA
{

    typedef IntReg IntRegFile[NumIntRegs];

    class FloatRegFile
    {
      protected:

        union {
            uint64_t q[NumFloatRegs];	// integer qword view
            double d[NumFloatRegs];	// double-precision floating point view
        };

      public:

        FloatReg readReg(int floatReg)
        {
            return d[floatReg];
        }

        FloatReg readReg(int floatReg, int width)
        {
            return readReg(floatReg);
        }

        FloatRegBits readRegBits(int floatReg)
        {
            return q[floatReg];
        }

        FloatRegBits readRegBits(int floatReg, int width)
        {
            return readRegBits(floatReg);
        }

        Fault setReg(int floatReg, const FloatReg &val)
        {
            d[floatReg] = val;
            return NoFault;
        }

        Fault setReg(int floatReg, const FloatReg &val, int width)
        {
            return setReg(floatReg, val);
        }

        Fault setRegBits(int floatReg, const FloatRegBits &val)
        {
            q[floatReg] = val;
            return NoFault;
        }

        Fault setRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            return setRegBits(floatReg, val);
        }

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);

    };

    class MiscRegFile {
      protected:
        uint64_t	fpcr;		// floating point condition codes
        uint64_t	uniq;		// process-unique register
        bool		lock_flag;	// lock flag for LL/SC
        Addr		lock_addr;	// lock address for LL/SC

      public:
        MiscReg readReg(int misc_reg);

        //These functions should be removed once the simplescalar cpu model
        //has been replaced.
        int getInstAsid();
        int getDataAsid();

        MiscReg readRegWithEffect(int misc_reg, Fault &fault, ExecContext *xc);

        Fault setReg(int misc_reg, const MiscReg &val);

        Fault setRegWithEffect(int misc_reg, const MiscReg &val,
                               ExecContext *xc);

#if FULL_SYSTEM
      protected:
        typedef uint64_t InternalProcReg;

        InternalProcReg ipr[NumInternalProcRegs]; // Internal processor regs

      private:
        InternalProcReg readIpr(int idx, Fault &fault, ExecContext *xc);

        Fault setIpr(int idx, InternalProcReg val, ExecContext *xc);
#endif
        friend class RegFile;
    };

    struct RegFile {
        IntRegFile intRegFile;		// (signed) integer register file
        FloatRegFile floatRegFile;	// floating point register file
        MiscRegFile miscRegs;		// control register file
        Addr pc;			// program counter
        Addr npc;			// next-cycle program counter
        Addr nnpc;

#if FULL_SYSTEM
        int intrflag;			// interrupt flag
        inline int instAsid()
        { return miscRegs.getInstAsid(); }
        inline int dataAsid()
        { return miscRegs.getDataAsid(); }
#endif // FULL_SYSTEM

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);
    };

    void copyRegs(ExecContext *src, ExecContext *dest);

    void copyMiscRegs(ExecContext *src, ExecContext *dest);

#if FULL_SYSTEM
    void copyIprs(ExecContext *src, ExecContext *dest);
#endif
} // namespace AlphaISA

#endif
