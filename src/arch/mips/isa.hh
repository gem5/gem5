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
 */

#ifndef __ARCH_MIPS_ISA_HH__
#define __ARCH_MIPS_ISA_HH__

#include <queue>
#include <string>
#include <vector>

#include "arch/generic/isa.hh"
#include "arch/mips/registers.hh"
#include "arch/mips/types.hh"
#include "cpu/reg_class.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

class BaseCPU;
class Checkpoint;
class EventManager;
struct MipsISAParams;
class ThreadContext;

namespace MipsISA
{
    class ISA : public BaseISA
    {
      public:
        // The MIPS name for this file is CP0 or Coprocessor 0
        typedef ISA CP0;

        typedef MipsISAParams Params;

      protected:
        // Number of threads and vpes an individual ISA state can handle
        uint8_t numThreads;
        uint8_t numVpes;

        enum BankType {
            perProcessor,
            perThreadContext,
            perVirtProcessor
        };

        std::vector<std::vector<RegVal> > miscRegFile;
        std::vector<std::vector<RegVal> > miscRegFile_WriteMask;
        std::vector<BankType> bankType;

      public:
        void clear(ThreadContext *tc) { clear(); }

      protected:
        void clear();

      public:
        void configCP();

        unsigned getVPENum(ThreadID tid) const;

        //////////////////////////////////////////////////////////
        //
        // READ/WRITE CP0 STATE
        //
        //
        //////////////////////////////////////////////////////////
        //@TODO: MIPS MT's register view automatically connects
        //       Status to TCStatus depending on current thread
        void updateCP0ReadView(int misc_reg, ThreadID tid) { }
        RegVal readMiscRegNoEffect(int misc_reg, ThreadID tid = 0) const;

        //template <class TC>
        RegVal readMiscReg(int misc_reg, ThreadContext *tc, ThreadID tid = 0);

        RegVal filterCP0Write(int misc_reg, int reg_sel, RegVal val);
        void setRegMask(int misc_reg, RegVal val, ThreadID tid = 0);
        void setMiscRegNoEffect(int misc_reg, RegVal val, ThreadID tid=0);

        //template <class TC>
        void setMiscReg(int misc_reg, RegVal val,
                        ThreadContext *tc, ThreadID tid=0);

        //////////////////////////////////////////////////////////
        //
        // DECLARE INTERFACE THAT WILL ALLOW A MiscRegFile (Cop0)
        // TO SCHEDULE EVENTS
        //
        //////////////////////////////////////////////////////////

        // Flag that is set when CP0 state has been written to.
        bool cp0Updated;

        // Enumerated List of CP0 Event Types
        enum CP0EventType {
            UpdateCP0
        };

        /** Process a CP0 event */
        void processCP0Event(BaseCPU *cpu, CP0EventType);

        // Schedule a CP0 Update Event
        void scheduleCP0Update(BaseCPU *cpu, Cycles delay = Cycles(0));

        // If any changes have been made, then check the state for changes
        // and if necessary alert the CPU
        void updateCPU(BaseCPU *cpu);

        static std::string miscRegNames[NumMiscRegs];

      public:
        void startup(ThreadContext *tc) {}

        /// Explicitly import the otherwise hidden startup
        using BaseISA::startup;

        const Params *params() const;

        ISA(Params *p);

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

    };
}

#endif
