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
 */

#include "arch/x86/nativetrace.hh"

#include "arch/x86/isa_traits.hh"
#include "arch/x86/regs/float.hh"
#include "arch/x86/regs/int.hh"
#include "cpu/thread_context.hh"
#include "debug/ExecRegDelta.hh"
#include "params/X86NativeTrace.hh"
#include "sim/byteswap.hh"

namespace Trace {

void
X86NativeTrace::ThreadState::update(NativeTrace *parent)
{
    parent->read(this, sizeof(*this));
    rax = letoh(rax);
    rcx = letoh(rcx);
    rdx = letoh(rdx);
    rbx = letoh(rbx);
    rsp = letoh(rsp);
    rbp = letoh(rbp);
    rsi = letoh(rsi);
    rdi = letoh(rdi);
    r8 = letoh(r8);
    r9 = letoh(r9);
    r10 = letoh(r10);
    r11 = letoh(r11);
    r12 = letoh(r12);
    r13 = letoh(r13);
    r14 = letoh(r14);
    r15 = letoh(r15);
    rip = letoh(rip);
    //This should be expanded if x87 registers are considered
    for (int i = 0; i < 8; i++)
        mmx[i] = letoh(mmx[i]);
    for (int i = 0; i < 32; i++)
        xmm[i] = letoh(xmm[i]);
}

void
X86NativeTrace::ThreadState::update(ThreadContext *tc)
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
    rip = tc->pcState().npc();
    //This should be expanded if x87 registers are considered
    for (int i = 0; i < 8; i++)
        mmx[i] = tc->readFloatReg(X86ISA::FLOATREG_MMX(i));
    for (int i = 0; i < 32; i++)
        xmm[i] = tc->readFloatReg(X86ISA::FLOATREG_XMM_BASE + i);
}


X86NativeTrace::X86NativeTrace(const Params *p)
    : NativeTrace(p)
{
    checkRcx = true;
    checkR11 = true;
}

bool
X86NativeTrace::checkRcxReg(const char * name, uint64_t &mVal, uint64_t &nVal)
{
    if (!checkRcx)
        checkRcx = (mVal != oldRcxVal || nVal != oldRealRcxVal);
    if (checkRcx)
        return checkReg(name, mVal, nVal);
    return true;
}

bool
X86NativeTrace::checkR11Reg(const char * name, uint64_t &mVal, uint64_t &nVal)
{
    if (!checkR11)
        checkR11 = (mVal != oldR11Val || nVal != oldRealR11Val);
    if (checkR11)
        return checkReg(name, mVal, nVal);
    return true;
}

bool
X86NativeTrace::checkXMM(int num, uint64_t mXmmBuf[], uint64_t nXmmBuf[])
{
    if (mXmmBuf[num * 2]     != nXmmBuf[num * 2] ||
        mXmmBuf[num * 2 + 1] != nXmmBuf[num * 2 + 1]) {
        DPRINTF(ExecRegDelta,
                "Register xmm%d should be 0x%016x%016x but is 0x%016x%016x.\n",
                num, nXmmBuf[num * 2 + 1], nXmmBuf[num * 2],
                     mXmmBuf[num * 2 + 1], mXmmBuf[num * 2]);
        return false;
    }
    return true;
}

void
X86NativeTrace::check(NativeTraceRecord *record)
{
    nState.update(this);
    mState.update(record->getThread());

    if (record->getStaticInst()->isSyscall())
    {
        checkRcx = false;
        checkR11 = false;
        oldRcxVal = mState.rcx;
        oldRealRcxVal = nState.rcx;
        oldR11Val = mState.r11;
        oldRealR11Val = nState.r11;
    }

    checkReg("rax", mState.rax, nState.rax);
    checkRcxReg("rcx", mState.rcx, nState.rcx);
    checkReg("rdx", mState.rdx, nState.rdx);
    checkReg("rbx", mState.rbx, nState.rbx);
    checkReg("rsp", mState.rsp, nState.rsp);
    checkReg("rbp", mState.rbp, nState.rbp);
    checkReg("rsi", mState.rsi, nState.rsi);
    checkReg("rdi", mState.rdi, nState.rdi);
    checkReg("r8",  mState.r8,  nState.r8);
    checkReg("r9",  mState.r9,  nState.r9);
    checkReg("r10", mState.r10, nState.r10);
    checkR11Reg("r11", mState.r11, nState.r11);
    checkReg("r12", mState.r12, nState.r12);
    checkReg("r13", mState.r13, nState.r13);
    checkReg("r14", mState.r14, nState.r14);
    checkReg("r15", mState.r15, nState.r15);
    checkReg("rip", mState.rip, nState.rip);
    checkXMM(0, mState.xmm, nState.xmm);
    checkXMM(1, mState.xmm, nState.xmm);
    checkXMM(2, mState.xmm, nState.xmm);
    checkXMM(3, mState.xmm, nState.xmm);
    checkXMM(4, mState.xmm, nState.xmm);
    checkXMM(5, mState.xmm, nState.xmm);
    checkXMM(6, mState.xmm, nState.xmm);
    checkXMM(7, mState.xmm, nState.xmm);
    checkXMM(8, mState.xmm, nState.xmm);
    checkXMM(9, mState.xmm, nState.xmm);
    checkXMM(10, mState.xmm, nState.xmm);
    checkXMM(11, mState.xmm, nState.xmm);
    checkXMM(12, mState.xmm, nState.xmm);
    checkXMM(13, mState.xmm, nState.xmm);
    checkXMM(14, mState.xmm, nState.xmm);
    checkXMM(15, mState.xmm, nState.xmm);
}

} // namespace Trace

////////////////////////////////////////////////////////////////////////
//
//  ExeTracer Simulation Object
//
Trace::X86NativeTrace *
X86NativeTraceParams::create()
{
    return new Trace::X86NativeTrace(this);
}
