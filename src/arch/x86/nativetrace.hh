/*
 * Copyright (c) 2007-2009 The Regents of The University of Michigan
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

#ifndef __ARCH_X86_NATIVETRACE_HH__
#define __ARCH_X86_NATIVETRACE_HH__

#include "base/types.hh"
#include "cpu/nativetrace.hh"

class ThreadContext;

namespace Trace {

class X86NativeTrace : public NativeTrace
{
  protected:
    bool checkRcx;
    bool checkR11;
    uint64_t oldRcxVal, oldR11Val;
    uint64_t oldRealRcxVal, oldRealR11Val;

    struct ThreadState {
        uint64_t rax;
        uint64_t rcx;
        uint64_t rdx;
        uint64_t rbx;
        uint64_t rsp;
        uint64_t rbp;
        uint64_t rsi;
        uint64_t rdi;
        uint64_t r8;
        uint64_t r9;
        uint64_t r10;
        uint64_t r11;
        uint64_t r12;
        uint64_t r13;
        uint64_t r14;
        uint64_t r15;
        uint64_t rip;
        //This should be expanded to 16 if x87 registers are considered
        uint64_t mmx[8];
        uint64_t xmm[32];

        void update(NativeTrace *parent);
        void update(ThreadContext *tc);
    };

    ThreadState nState;
    ThreadState mState;

    bool checkRcxReg(const char * regName, uint64_t &, uint64_t &);
    bool checkR11Reg(const char * regName, uint64_t &, uint64_t &);
    bool checkXMM(int num, uint64_t mXmmBuf[], uint64_t nXmmBuf[]);

  public:
    X86NativeTrace(const Params *p);

    void check(NativeTraceRecord *record);
};

} // namespace Trace

#endif // __ARCH_X86_NATIVETRACE_HH__
