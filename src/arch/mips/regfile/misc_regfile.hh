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
 * Authors: Korey Sewell
 */

#ifndef __ARCH_MIPS_MISC_REGFILE_HH__
#define __ARCH_MIPS_MISC_REGFILE_HH__

#include "arch/mips/types.hh"
#include "arch/mips/constants.hh"
#include "sim/faults.hh"

class Checkpoint;
class ThreadContext;
class Regfile;

namespace MipsISA
{
    class MiscRegFile {

      protected:
        uint64_t	fpcr;		// floating point condition codes
        uint64_t	uniq;		// process-unique register
        bool		lock_flag;	// lock flag for LL/SC
        Addr		lock_addr;	// lock address for LL/SC

        MiscReg miscRegFile[NumMiscRegs];

      public:
        //These functions should be removed once the simplescalar cpu model
        //has been replaced.
        int getInstAsid();
        int getDataAsid();

        void copyMiscRegs(ThreadContext *tc);

        MiscReg readReg(int misc_reg)
        {
            return miscRegFile[misc_reg];
        }

        MiscReg readRegWithEffect(int misc_reg, Fault &fault, ThreadContext *tc)
        {
            return miscRegFile[misc_reg];
        }

        Fault setReg(int misc_reg, const MiscReg &val)
        {
            miscRegFile[misc_reg] = val; return NoFault;
        }

        Fault setRegWithEffect(int misc_reg, const MiscReg &val,
                               ThreadContext *tc)
        {
            miscRegFile[misc_reg] = val; return NoFault;
        }

#if FULL_SYSTEM
        void clearIprs() { }

      protected:
        InternalProcReg ipr[NumInternalProcRegs]; // Internal processor regs

      private:
        MiscReg readIpr(int idx, Fault &fault, ThreadContext *tc) { }

        Fault setIpr(int idx, uint64_t val, ThreadContext *tc) { }
#endif
        friend class RegFile;
    };
} // namespace MipsISA

#endif
