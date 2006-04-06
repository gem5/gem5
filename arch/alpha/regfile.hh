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
    class IntRegFile
    {
      protected:
        IntReg regs[NumIntRegs];

      public:

        IntReg readReg(int intReg)
        {
            return regs[intReg];
        }

        Fault setReg(int intReg, const IntReg &val)
        {
            regs[intReg] = val;
            return NoFault;
        }

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);

    };

    class FloatRegFile
    {
      public:

        union {
            uint64_t q[NumFloatRegs];	// integer qword view
            double d[NumFloatRegs];	// double-precision floating point view
        };

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

        MiscReg readRegWithEffect(int misc_reg, Fault &fault,
                ExecContext *xc);

        //These functions should be removed once the simplescalar cpu model
        //has been replaced.
        int getInstAsid();
        int getDataAsid();

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

    class RegFile {

      protected:
        Addr pc;			// program counter
        Addr npc;			// next-cycle program counter
        Addr nnpc;

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

      protected:
        IntRegFile intRegFile;		// (signed) integer register file
        FloatRegFile floatRegFile;	// floating point register file
        MiscRegFile miscRegFile;	// control register file

      public:

#if FULL_SYSTEM
        int intrflag;			// interrupt flag
        inline int instAsid()
        { return miscRegs.getInstAsid(); }
        inline int dataAsid()
        { return miscRegs.getDataAsid(); }
#endif // FULL_SYSTEM

        void clear()
        {
            bzero(&intRegFile, sizeof(intRegFile));
            bzero(&floatRegFile, sizeof(floatRegFile));
            bzero(&miscRegFile, sizeof(miscRegFile));
        }

        MiscReg readMiscReg(int miscReg)
        {
            return miscRegFile.readReg(miscReg);
        }

        MiscReg readMiscRegWithEffect(int miscReg,
                Fault &fault, ExecContext *xc)
        {
            fault = NoFault;
            return miscRegFile.readRegWithEffect(miscReg, fault, xc);
        }

        Fault setMiscReg(int miscReg, const MiscReg &val)
        {
            return miscRegFile.setReg(miscReg, val);
        }

        Fault setMiscRegWithEffect(int miscReg, const MiscReg &val,
                ExecContext * xc)
        {
            return miscRegFile.setRegWithEffect(miscReg, val, xc);
        }

        FloatReg readFloatReg(int floatReg)
        {
            return floatRegFile.d[floatReg];
        }

        FloatReg readFloatReg(int floatReg, int width)
        {
            return readFloatReg(floatReg);
        }

        FloatRegBits readFloatRegBits(int floatReg)
        {
            return floatRegFile.q[floatReg];
        }

        FloatRegBits readFloatRegBits(int floatReg, int width)
        {
            return readFloatRegBits(floatReg);
        }

        Fault setFloatReg(int floatReg, const FloatReg &val)
        {
            floatRegFile.d[floatReg] = val;
            return NoFault;
        }

        Fault setFloatReg(int floatReg, const FloatReg &val, int width)
        {
            return setFloatReg(floatReg, val);
        }

        Fault setFloatRegBits(int floatReg, const FloatRegBits &val)
        {
            floatRegFile.q[floatReg] = val;
            return NoFault;
        }

        Fault setFloatRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            return setFloatRegBits(floatReg, val);
        }

        IntReg readIntReg(int intReg)
        {
            return intRegFile.readReg(intReg);
        }

        Fault setIntReg(int intReg, const IntReg &val)
        {
            return intRegFile.setReg(intReg, val);
        }

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);

        enum ContextParam
        {
            CONTEXT_PALMODE
        };

        typedef bool ContextVal;

        void changeContext(ContextParam param, ContextVal val)
        {
            //This would be an alternative place to call/implement
            //the swapPALShadow function
        }
    };

    void copyRegs(ExecContext *src, ExecContext *dest);

    void copyMiscRegs(ExecContext *src, ExecContext *dest);

#if FULL_SYSTEM
    void copyIprs(ExecContext *src, ExecContext *dest);
#endif
} // namespace AlphaISA

#endif
