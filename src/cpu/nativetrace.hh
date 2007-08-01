/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#ifndef __NATIVETRACE_HH__
#define __NATIVETRACE_HH__

#include "base/trace.hh"
#include "cpu/static_inst.hh"
#include "sim/host.hh"
#include "sim/insttracer.hh"
#include "arch/x86/intregs.hh"

class ThreadContext;


namespace Trace {

class NativeTrace;

class NativeTraceRecord : public InstRecord
{
  protected:
    NativeTrace * parent;

  public:
    NativeTraceRecord(NativeTrace * _parent,
               Tick _when, ThreadContext *_thread,
               const StaticInstPtr &_staticInst, Addr _pc, bool spec)
        : InstRecord(_when, _thread, _staticInst, _pc, spec), parent(_parent)
    {
    }

    void dump();
};

class NativeTrace : public InstTracer
{
  protected:
    int fd;

    ListenSocket native_listener;

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

        void update(int fd)
        {
            int bytesLeft = sizeof(ThreadState);
            int bytesRead = 0;
            do
            {
                int res = read(fd, ((char *)this) + bytesRead, bytesLeft);
                if(res < 0)
                    panic("Read call failed! %s\n", strerror(errno));
                bytesLeft -= res;
                bytesRead += res;
            } while(bytesLeft);
            rax = TheISA::gtoh(rax);
            rcx = TheISA::gtoh(rcx);
            rdx = TheISA::gtoh(rdx);
            rbx = TheISA::gtoh(rbx);
            rsp = TheISA::gtoh(rsp);
            rbp = TheISA::gtoh(rbp);
            rsi = TheISA::gtoh(rsi);
            rdi = TheISA::gtoh(rdi);
            r8 = TheISA::gtoh(r8);
            r9 = TheISA::gtoh(r9);
            r10 = TheISA::gtoh(r10);
            r11 = TheISA::gtoh(r11);
            r12 = TheISA::gtoh(r12);
            r13 = TheISA::gtoh(r13);
            r14 = TheISA::gtoh(r14);
            r15 = TheISA::gtoh(r15);
            rip = TheISA::gtoh(rip);
        }

        void update(ThreadContext * tc)
        {
            rax = tc->readIntReg(X86ISA::INTREG_RAX);
            rcx = tc->readIntReg(X86ISA::INTREG_RCX);
            rdx = tc->readIntReg(X86ISA::INTREG_RDX);
            rbx = tc->readIntReg(X86ISA::INTREG_RBX);
            rsp = tc->readIntReg(X86ISA::INTREG_RSP);
            rbp = tc->readIntReg(X86ISA::INTREG_RBP);
            rsi = tc->readIntReg(X86ISA::INTREG_RSI);
            rdi = tc->readIntReg(X86ISA::INTREG_RDI);
            r8 = tc->readIntReg(X86ISA::INTREG_R8);
            r9 = tc->readIntReg(X86ISA::INTREG_R9);
            r10 = tc->readIntReg(X86ISA::INTREG_R10);
            r11 = tc->readIntReg(X86ISA::INTREG_R11);
            r12 = tc->readIntReg(X86ISA::INTREG_R12);
            r13 = tc->readIntReg(X86ISA::INTREG_R13);
            r14 = tc->readIntReg(X86ISA::INTREG_R14);
            r15 = tc->readIntReg(X86ISA::INTREG_R15);
            rip = tc->readNextPC();
        }

    };

    ThreadState nState;
    ThreadState mState;


  public:

    template<class T>
    bool
    checkReg(const char * regName, T &val, T &realVal)
    {
        if(val != realVal)
        {
            DPRINTFN("Register %s should be %#x but is %#x.\n",
                    regName, realVal, val);
            return false;
        }
        return true;
    }

    bool
    checkRcxReg(const char * regName, uint64_t &, uint64_t &);

    bool
    checkR11Reg(const char * regName, uint64_t &, uint64_t &);

    NativeTrace(const std::string & name);

    NativeTraceRecord *
    getInstRecord(Tick when, ThreadContext *tc,
            const StaticInstPtr staticInst, Addr pc)
    {
        if (tc->misspeculating())
            return NULL;

        return new NativeTraceRecord(this, when, tc,
                staticInst, pc, tc->misspeculating());
    }

    void
    check(ThreadContext *, bool syscall);

    friend class NativeTraceRecord;
};

/* namespace Trace */ }

#endif // __EXETRACE_HH__
