/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
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

#ifndef TRACECHILD_SPARC_HH
#define TRACECHILD_SPARC_HH

#include <asm-sparc64/reg.h>
#include <sys/ptrace.h>
#include <sys/types.h>
#include <stdint.h>

#include <cassert>
#include <ostream>
#include <string>

#include "base/tracechild.hh"

struct regs;

class SparcTraceChild : public TraceChild
{
  public:
    enum RegNum
    {
        // Global registers
        G0,
        G1,
        G2,
        G3,
        G4,
        G5,
        G6,
        G7,
        // Output registers
        O0,
        O1,
        O2,
        O3,
        O4,
        O5,
        O6,
        O7,
        // Local registers
        L0,
        L1,
        L2,
        L3,
        L4,
        L5,
        L6,
        L7,
        // Input registers
        I0,
        I1,
        I2,
        I3,
        I4,
        I5,
        I6,
        I7,
        // Floating point
        F0,
        F2,
        F4,
        F6,
        F8,
        F10,
        F12,
        F14,
        F16,
        F18,
        F20,
        F22,
        F24,
        F26,
        F28,
        F30,
        F32,
        F34,
        F36,
        F38,
        F40,
        F42,
        F44,
        F46,
        F48,
        F50,
        F52,
        F54,
        F56,
        F58,
        F60,
        F62,
        // Miscelaneous
        FSR,
        FPRS,
        PC,
        NPC,
        Y,
        CWP,
        PSTATE,
        ASI,
        CCR,
        numregs
    };

  private:
    regs theregs;
    regs oldregs;
    fpu thefpregs;
    fpu oldfpregs;
    uint64_t locals[8];
    uint64_t oldLocals[8];
    uint64_t inputs[8];
    uint64_t oldInputs[8];
    bool regDiffSinceUpdate[numregs];

    // This calculates where the pc might go after the current instruction.
    // while this equals npc for most instructions, it doesn't for all of
    // them. The return value is the number of actual potential targets.
    int getTargets(uint32_t inst, uint64_t pc, uint64_t npc, uint64_t &target1,
                   uint64_t &target2);

  protected:
    bool update(int pid);

  public:
    SparcTraceChild();

    bool sendState(int socket);

    int64_t getRegVal(int num);

    int64_t getOldRegVal(int num);

    bool step();

    uint64_t
    getPC()
    {
        return getRegVal(PC);
    }

    uint64_t
    getSP()
    {
        return getRegVal(O6);
    }

    std::ostream &outputStartState(std::ostream &os);
};

#endif
