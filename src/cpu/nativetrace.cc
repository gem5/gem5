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
 *          Lisa Hsu
 *          Nathan Binkert
 *          Steve Raasch
 */

#include <errno.h>

#include "arch/regfile.hh"
#include "arch/utility.hh"
#include "base/loader/symtab.hh"
#include "base/socket.hh"
#include "cpu/nativetrace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "params/NativeTrace.hh"

//XXX This is temporary
#include "arch/isa_specific.hh"

using namespace std;
using namespace TheISA;

namespace Trace {

NativeTrace::NativeTrace(const std::string & _name) : InstTracer(_name)
{
    int port = 8000;
    while(!native_listener.listen(port, true))
    {
        DPRINTF(GDBMisc, "Can't bind port %d\n", port);
        port++;
    }
    ccprintf(cerr, "Listening for native process on port %d\n", port);
    fd = native_listener.accept();
    checkRcx = true;
    checkR11 = true;
}

bool
NativeTrace::checkRcxReg(const char * name, uint64_t &mVal, uint64_t &nVal)
{
    if(!checkRcx)
        checkRcx = (mVal != oldRcxVal || nVal != oldRealRcxVal);
    if(checkRcx)
        return checkReg(name, mVal, nVal);
    return true;
}

bool
NativeTrace::checkR11Reg(const char * name, uint64_t &mVal, uint64_t &nVal)
{
    if(!checkR11)
        checkR11 = (mVal != oldR11Val || nVal != oldRealR11Val);
    if(checkR11)
        return checkReg(name, mVal, nVal);
    return true;
}

void
Trace::NativeTraceRecord::dump()
{
    //Don't print what happens for each micro-op, just print out
    //once at the last op, and for regular instructions.
    if(!staticInst->isMicroop() || staticInst->isLastMicroop())
    parent->check(thread, staticInst->isSyscall());
}

void
Trace::NativeTrace::check(ThreadContext * tc, bool isSyscall)
{
//    ostream &outs = Trace::output();
    nState.update(fd);
    mState.update(tc);

    if(isSyscall)
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
#if THE_ISA == SPARC_ISA
    /*for(int f = 0; f <= 62; f+=2)
    {
        uint64_t regVal;
        int res = read(fd, &regVal, sizeof(regVal));
        if(res < 0)
            panic("First read call failed! %s\n", strerror(errno));
        regVal = TheISA::gtoh(regVal);
        uint64_t realRegVal = thread->readFloatRegBits(f, 64);
        if(regVal != realRegVal)
        {
            DPRINTF(ExecRegDelta, "Register f%d should be %#x but is %#x.\n", f, regVal, realRegVal);
        }
    }*/
    uint64_t regVal;
    int res = read(fd, &regVal, sizeof(regVal));
    if(res < 0)
        panic("First read call failed! %s\n", strerror(errno));
    regVal = TheISA::gtoh(regVal);
    uint64_t realRegVal = thread->readNextPC();
    if(regVal != realRegVal)
    {
        DPRINTF(ExecRegDelta,
                "Register pc should be %#x but is %#x.\n",
                regVal, realRegVal);
    }
    res = read(fd, &regVal, sizeof(regVal));
    if(res < 0)
        panic("First read call failed! %s\n", strerror(errno));
    regVal = TheISA::gtoh(regVal);
    realRegVal = thread->readNextNPC();
    if(regVal != realRegVal)
    {
        DPRINTF(ExecRegDelta,
                "Register npc should be %#x but is %#x.\n",
                regVal, realRegVal);
    }
    res = read(fd, &regVal, sizeof(regVal));
    if(res < 0)
        panic("First read call failed! %s\n", strerror(errno));
    regVal = TheISA::gtoh(regVal);
    realRegVal = thread->readIntReg(SparcISA::NumIntArchRegs + 2);
    if((regVal & 0xF) != (realRegVal & 0xF))
    {
        DPRINTF(ExecRegDelta,
                "Register ccr should be %#x but is %#x.\n",
                regVal, realRegVal);
    }
#endif
}

/* namespace Trace */ }

////////////////////////////////////////////////////////////////////////
//
//  ExeTracer Simulation Object
//
Trace::NativeTrace *
NativeTraceParams::create()
{
    return new Trace::NativeTrace(name);
};
