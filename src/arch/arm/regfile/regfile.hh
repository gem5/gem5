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

#ifndef __ARCH_ARM_REGFILE_REGFILE_HH__
#define __ARCH_ARM_REGFILE_REGFILE_HH__

#include "arch/arm/types.hh"
#include "arch/arm/regfile/int_regfile.hh"
#include "arch/arm/regfile/float_regfile.hh"
#include "arch/arm/regfile/misc_regfile.hh"
#include "sim/faults.hh"

class Checkpoint;
class EventManager;
class ThreadContext;

namespace ArmISA
{
    class RegFile
    {
      protected:
        IntRegFile intRegFile;		// (signed) integer register file
        FloatRegFile floatRegFile;	// floating point register file

      public:

        void clear()
        {
            intRegFile.clear();
            floatRegFile.clear();
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

        void setFloatReg(int floatReg, const FloatRegVal &val)
        {
            floatRegFile.setReg(floatReg, val, SingleWidth);
        }

        void setFloatReg(int floatReg, const FloatRegVal &val, int width)
        {
            floatRegFile.setReg(floatReg, val, width);
        }

        void setFloatRegBits(int floatReg, const FloatRegBits &val)
        {
            floatRegFile.setRegBits(floatReg, val, SingleWidth);
        }

        void setFloatRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            floatRegFile.setRegBits(floatReg, val, width);
        }

        IntReg readIntReg(int intReg)
        {
            return intRegFile.readReg(intReg);
        }

        void setIntReg(int intReg, const IntReg &val)
        {
            intRegFile.setReg(intReg, val);
        }
      protected:

        Addr pc;			// program counter
        Addr npc;			// next-cycle program counter
        Addr nnpc;			// next-next-cycle program counter

      public:
        Addr readPC()
        {
            return intRegFile.readReg(PCReg);
            //return pc;
        }

        void setPC(Addr val)
        {
            intRegFile.setReg(PCReg, val);
            //pc = val;
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
            return npc + sizeof(MachInst);
        }

        void setNextNPC(Addr val)
        {
            //nnpc = val;
        }

        void serialize(EventManager *em, std::ostream &os);
        void unserialize(EventManager *em, Checkpoint *cp,
                         const std::string &section);
    };

    void copyRegs(ThreadContext *src, ThreadContext *dest);

    void copyMiscRegs(ThreadContext *src, ThreadContext *dest);

} // namespace ArmISA

#endif
