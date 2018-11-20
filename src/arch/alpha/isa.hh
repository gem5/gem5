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

#ifndef __ARCH_ALPHA_ISA_HH__
#define __ARCH_ALPHA_ISA_HH__

#include <cstring>
#include <iostream>
#include <string>

#include "arch/alpha/registers.hh"
#include "arch/alpha/types.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

struct AlphaISAParams;
class BaseCPU;
class Checkpoint;
class EventManager;
class ThreadContext;

namespace AlphaISA
{
    class ISA : public SimObject
    {
      public:
        typedef uint64_t InternalProcReg;
        typedef AlphaISAParams Params;

      protected:
        // Parent system
        System *system;

        uint64_t fpcr;       // floating point condition codes
        uint64_t uniq;       // process-unique register
        bool lock_flag;      // lock flag for LL/SC
        Addr lock_addr;      // lock address for LL/SC
        int intr_flag;

        InternalProcReg ipr[NumInternalProcRegs]; // Internal processor regs

      protected:
        InternalProcReg readIpr(int idx, ThreadContext *tc);
        void setIpr(int idx, InternalProcReg val, ThreadContext *tc);

      public:

        RegVal readMiscRegNoEffect(int misc_reg, ThreadID tid = 0) const;
        RegVal readMiscReg(int misc_reg, ThreadContext *tc, ThreadID tid = 0);

        void setMiscRegNoEffect(int misc_reg, RegVal val, ThreadID tid=0);
        void setMiscReg(int misc_reg, RegVal val, ThreadContext *tc,
                        ThreadID tid=0);

        void
        clear()
        {
            fpcr = 0;
            uniq = 0;
            lock_flag = 0;
            lock_addr = 0;
            intr_flag = 0;
            memset(ipr, 0, sizeof(ipr));
        }

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        RegId flattenRegId(const RegId& regId) const { return regId; }

        int
        flattenIntIndex(int reg) const
        {
            return reg;
        }

        int
        flattenFloatIndex(int reg) const
        {
            return reg;
        }

        int
        flattenVecIndex(int reg) const
        {
            return reg;
        }

        int
        flattenVecElemIndex(int reg) const
        {
            return reg;
        }

        int
        flattenVecPredIndex(int reg) const
        {
            return reg;
        }

        // dummy
        int
        flattenCCIndex(int reg) const
        {
            return reg;
        }

        int
        flattenMiscIndex(int reg) const
        {
            return reg;
        }

        const Params *params() const;

        ISA(Params *p);

        void startup(ThreadContext *tc) {}

        /// Explicitly import the otherwise hidden startup
        using SimObject::startup;
    };
}

#endif
