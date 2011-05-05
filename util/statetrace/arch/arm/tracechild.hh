/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 * Authors: Ali Saidi
 *          Gabe Black
 */

#ifndef TRACECHILD_ARM_HH
#define TRACECHILD_ARM_HH

#include <sys/ptrace.h>
#include <sys/user.h>

#include <cassert>
#include <string>

#include "base/tracechild.hh"

class ARMTraceChild : public TraceChild
{
  public:
    enum RegNum
    {
        // r0 - r3 argument, temp, caller save
        // r4 - r10 callee save
        // r11 - FP
        // r12 - temp
        // r13 - stack
        // r14 - link
        // r15 - pc
        R0, R1, R2, R3, R4, R5, R6, R7,
        R8, R9, R10, FP, R12, SP, LR, PC,
        CPSR,
        F0, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15,
        F16, F17, F18, F19, F20, F21, F22, F23, F24, F25, F26, F27, F28, F29,
        F30, F31, FPSCR,
        numregs
    };

    struct vfp_regs {
        uint64_t fpregs[32];
        uint32_t fpscr;
    };

  private:
    uint32_t getRegs(user_regs& myregs, int num);
    uint64_t getFpRegs(vfp_regs &myfpregs, int num);

    user_regs regs;
    user_regs oldregs;

    vfp_regs fpregs;
    vfp_regs oldfpregs;

    bool regDiffSinceUpdate[numregs];
    bool foundMvn;

  protected:
    bool update(int pid);

  public:
    ARMTraceChild();
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
        return getRegVal(SP);
    }

    std::ostream & outputStartState(std::ostream & os);

};

#endif

