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
//#include "cpu/base.hh"
#include "sim/faults.hh"

class BaseCPU;
class Checkpoint;
class EventManager;

namespace MipsISA
{
    const uint32_t MIPS32_QNAN = 0x7fbfffff;
    const uint64_t MIPS64_QNAN = ULL(0x7fbfffffffffffff);

    enum FPControlRegNums {
       FIR = NumFloatArchRegs,
       FCCR,
       FEXR,
       FENR,
       FCSR
    };

    enum FCSRBits {
        Inexact = 1,
        Underflow,
        Overflow,
        DivideByZero,
        Invalid,
        Unimplemented
    };

    enum FCSRFields {
        Flag_Field = 1,
        Enable_Field = 6,
        Cause_Field = 11
    };

    class RegFile {
      protected:
        Addr pc;                        // program counter
        Addr npc;                       // next-cycle program counter
        Addr nnpc;                      // next-next-cycle program counter
                                        // used to implement branch delay slot
                                        // not real register

      public:
        void clear();
        void reset(std::string core_name, ThreadID num_threads,
                   unsigned num_vpes, BaseCPU *_cpu);

        void setShadowSet(int css);

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

    };

} // namespace MipsISA

#endif
