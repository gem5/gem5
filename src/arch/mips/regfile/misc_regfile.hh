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

#ifndef __ARCH_MIPS_REGFILE_MISC_REGFILE_HH__
#define __ARCH_MIPS_REGFILE_MISC_REGFILE_HH__

#include "arch/mips/isa_traits.hh"
#include "arch/mips/types.hh"
#include "arch/mips/mt.hh"
#include "arch/mips/mt_constants.hh"
#include "base/bitfield.hh"
#include "cpu/base.hh"
#include "sim/faults.hh"
#include <queue>

class ThreadContext;

namespace MipsISA
{
    class MiscRegFile {
      public:
        // Give RegFile object, private access
        friend class RegFile;

        // The MIPS name for this file is CP0 or Coprocessor 0
        typedef MiscRegFile CP0;

      protected:
        enum BankType {
            perProcessor,
            perThreadContext,
            perVirtProcessor
        };

        std::vector<std::vector<MiscReg> > miscRegFile;
        std::vector<BankType> bankType;

        BaseCPU *cpu;

      public:
        MiscRegFile();
        MiscRegFile(BaseCPU *cpu);

        void init();

        void clear(unsigned tid_or_vpn = 0);

        void reset(std::string core_name, unsigned num_threads, unsigned num_vpes);

        void expandForMultithreading(unsigned num_threads, unsigned num_vpes);

        void copyMiscRegs(ThreadContext *tc);

        inline unsigned getVPENum(unsigned tid);

        //////////////////////////////////////////////////////////
        //
        // READ/WRITE CP0 STATE
        //
        //
        //////////////////////////////////////////////////////////
        //@TODO: MIPS MT's register view automatically connects
        //       Status to TCStatus depending on current thread
        void updateCP0ReadView(int misc_reg, unsigned tid) { }
        MiscReg readRegNoEffect(int misc_reg, unsigned tid = 0);
        MiscReg readReg(int misc_reg,
                        ThreadContext *tc,  unsigned tid = 0);

        MiscReg filterCP0Write(int misc_reg, MiscReg val) { return val; }
        void setRegNoEffect(int misc_reg, const MiscReg &val, unsigned tid = 0);
        void setReg(int misc_reg, const MiscReg &val,
                     ThreadContext *tc, unsigned tid = 0);

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

        // Declare A CP0Event Class for scheduling
        class CP0Event : public Event
        {
          protected:
            MiscRegFile::CP0 *cp0;
            BaseCPU *cpu;
            CP0EventType cp0EventType;
            Fault fault;

          public:
            /** Constructs a CP0 event. */
            CP0Event(CP0 *_cp0, BaseCPU *_cpu, CP0EventType e_type);

            /** Process this event. */
            virtual void process();

            /** Returns the description of this event. */
            const char *description();

            /** Schedule This Event */
            void scheduleEvent(int delay);

            /** Unschedule This Event */
            void unscheduleEvent();
        };

        // Schedule a CP0 Update Event
        void scheduleCP0Update(int delay = 0);

        // If any changes have been made, then check the state for changes
        // and if necessary alert the CPU
        void updateCPU();

        // Keep a List of CPU Events that need to be deallocated
        std::queue<CP0Event*> cp0EventRemoveList;

        static std::string miscRegNames[NumMiscRegs];
    };

    inline std::string getMiscRegName(unsigned reg_idx);
} // namespace MipsISA

#endif
