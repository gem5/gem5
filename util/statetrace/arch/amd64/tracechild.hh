/*
 * Copyright (c) 2007 The Regents of The University of Michigan
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

#ifndef REGSTATE_AMD64_HH
#define REGSTATE_AMD64_HH

#include <sys/ptrace.h>
#include <sys/types.h>
#include <sys/user.h>

#include <cassert>
#include <string>

#include "base/tracechild.hh"

class AMD64TraceChild : public TraceChild
{
  public:
    enum RegNum
    {
        // GPRs
        RAX,
        RCX,
        RDX,
        RBX,
        // Base pointer and stack pointer
        RSP,
        RBP,
        // Index registers
        RSI,
        RDI,
        // New 64 bit mode registers
        R8,
        R9,
        R10,
        R11,
        R12,
        R13,
        R14,
        R15,
        // Segmentation registers
        CS,
        DS,
        ES,
        FS,
        GS,
        SS,
        FS_BASE,
        GS_BASE,
        // PC
        RIP,
        // Flags
        EFLAGS,
        // MMX
        MMX0_0,
        MMX0_1,
        MMX1_0,
        MMX1_1,
        MMX2_0,
        MMX2_1,
        MMX3_0,
        MMX3_1,
        MMX4_0,
        MMX4_1,
        MMX5_0,
        MMX5_1,
        MMX6_0,
        MMX6_1,
        MMX7_0,
        MMX7_1,
        // XMM
        XMM0_0,
        XMM0_1,
        XMM0_2,
        XMM0_3,
        XMM1_0,
        XMM1_1,
        XMM1_2,
        XMM1_3,
        XMM2_0,
        XMM2_1,
        XMM2_2,
        XMM2_3,
        XMM3_0,
        XMM3_1,
        XMM3_2,
        XMM3_3,
        XMM4_0,
        XMM4_1,
        XMM4_2,
        XMM4_3,
        XMM5_0,
        XMM5_1,
        XMM5_2,
        XMM5_3,
        XMM6_0,
        XMM6_1,
        XMM6_2,
        XMM6_3,
        XMM7_0,
        XMM7_1,
        XMM7_2,
        XMM7_3,
        XMM8_0,
        XMM8_1,
        XMM8_2,
        XMM8_3,
        XMM9_0,
        XMM9_1,
        XMM9_2,
        XMM9_3,
        XMM10_0,
        XMM10_1,
        XMM10_2,
        XMM10_3,
        XMM11_0,
        XMM11_1,
        XMM11_2,
        XMM11_3,
        XMM12_0,
        XMM12_1,
        XMM12_2,
        XMM12_3,
        XMM13_0,
        XMM13_1,
        XMM13_2,
        XMM13_3,
        XMM14_0,
        XMM14_1,
        XMM14_2,
        XMM14_3,
        XMM15_0,
        XMM15_1,
        XMM15_2,
        XMM15_3,
        numregs
    };

  private:
    int64_t getRegs(user_regs_struct &myregs, user_fpregs_struct &myfpregs,
                    int num);
    user_regs_struct regs;
    user_regs_struct oldregs;
    user_fpregs_struct fpregs;
    user_fpregs_struct oldfpregs;
    bool regDiffSinceUpdate[numregs];

    uint64_t findSyscall();

  protected:
    bool update(int pid);

  public:
    AMD64TraceChild();

    bool sendState(int socket);

    int64_t getRegVal(int num);
    int64_t getOldRegVal(int num);

    uint64_t
    getPC()
    {
        return getRegVal(RIP);
    }

    uint64_t
    getSP()
    {
        return getRegVal(RSP);
    }

    std::ostream &outputStartState(std::ostream &output);

    bool step();
};

#endif
