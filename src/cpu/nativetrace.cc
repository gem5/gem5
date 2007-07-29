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
}

bool
NativeTraceRecord::checkIntReg(const char * regName, int index, int size)
{
    uint64_t regVal;
    int res = read(parent->fd, &regVal, size);
    if(res < 0)
        panic("Read call failed! %s\n", strerror(errno));
    regVal = TheISA::gtoh(regVal);
    uint64_t realRegVal = thread->readIntReg(index);
    if(regVal != realRegVal)
    {
        DPRINTFN("Register %s should be %#x but is %#x.\n",
                regName, regVal, realRegVal);
        return false;
    }
    return true;
}

bool NativeTraceRecord::checkPC(const char * regName, int size)
{
    uint64_t regVal;
    int res = read(parent->fd, &regVal, size);
    if(res < 0)
        panic("Read call failed! %s\n", strerror(errno));
    regVal = TheISA::gtoh(regVal);
    uint64_t realRegVal = thread->readNextPC();
    if(regVal != realRegVal)
    {
        DPRINTFN("%s should be %#x but is %#x.\n",
                regName, regVal, realRegVal);
        return false;
    }
    return true;
}

void
Trace::NativeTraceRecord::dump()
{
//    ostream &outs = Trace::output();

    //Don't print what happens for each micro-op, just print out
    //once at the last op, and for regular instructions.
    if(!staticInst->isMicroop() || staticInst->isLastMicroop())
    {
        checkIntReg("rax", INTREG_RAX, sizeof(uint64_t));
        checkIntReg("rcx", INTREG_RCX, sizeof(uint64_t));
        checkIntReg("rdx", INTREG_RDX, sizeof(uint64_t));
        checkIntReg("rbx", INTREG_RBX, sizeof(uint64_t));
        checkIntReg("rsp", INTREG_RSP, sizeof(uint64_t));
        checkIntReg("rbp", INTREG_RBP, sizeof(uint64_t));
        checkIntReg("rsi", INTREG_RSI, sizeof(uint64_t));
        checkIntReg("rdi", INTREG_RDI, sizeof(uint64_t));
        checkIntReg("r8", INTREG_R8, sizeof(uint64_t));
        checkIntReg("r9", INTREG_R9, sizeof(uint64_t));
        checkIntReg("r10", INTREG_R10, sizeof(uint64_t));
        checkIntReg("r11", INTREG_R11, sizeof(uint64_t));
        checkIntReg("r12", INTREG_R12, sizeof(uint64_t));
        checkIntReg("r13", INTREG_R13, sizeof(uint64_t));
        checkIntReg("r14", INTREG_R14, sizeof(uint64_t));
        checkIntReg("r15", INTREG_R15, sizeof(uint64_t));
        checkPC("rip", sizeof(uint64_t));
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
