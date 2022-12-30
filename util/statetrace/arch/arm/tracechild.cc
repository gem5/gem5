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
 * Copyright (c) 2006-2009 The Regents of The University of Michigan
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

#include <stdint.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "arch/arm/tracechild.hh"

using namespace std;

ARMTraceChild::ARMTraceChild()
{
    foundMvn = false;

    memset(&regs, 0, sizeof(regs));
    memset(&oldregs, 0, sizeof(regs));
    memset(&fpregs, 0, sizeof(vfp_regs));
    memset(&oldfpregs, 0, sizeof(vfp_regs));

    for (int x = 0; x < numregs; x++) {
        regDiffSinceUpdate[x] = false;
    }

    assert(sizeof(regs.uregs)/sizeof(regs.uregs[0]) > CPSR);
}

bool
ARMTraceChild::sendState(int socket)
{
    uint32_t regVal = 0;
    uint64_t message[numregs + 1];
    int pos = 1;
    message[0] = 0;
    for (int x = 0; x < numregs; x++) {
        if (regDiffSinceUpdate[x]) {
            message[0] = message[0] | (1ULL << x);
            message[pos++] = getRegVal(x);
        }
    }

    size_t sent = 0;
    size_t toSend = pos * sizeof(message[0]);
    uint8_t *messagePtr = (uint8_t *)message;
    while (toSend != 0) {
        sent = write(socket, messagePtr, toSend);
        if (sent == -1) {
            cerr << "Write failed! " << strerror(errno) << endl;
            tracing = false;
            return false;
        }
        toSend -= sent;
        messagePtr += sent;
    }

    return true;
}

uint32_t
ARMTraceChild::getRegs(user_regs &myregs, int num)
{
    assert(num <= CPSR && num >= 0);
    return myregs.uregs[num];
}

uint64_t
ARMTraceChild::getFpRegs(vfp_regs &my_fp_regs, int num)
{
    assert(num >= F0 && num < numregs);
    if (num == FPSCR)
        return my_fp_regs.fpscr;

    num -= F0;
    return my_fp_regs.fpregs[num];
}

bool
ARMTraceChild::update(int pid)
{
    oldregs = regs;
    if (ptrace(PTRACE_GETREGS, pid, 0, &regs) != 0) {
        cerr << "update: " << strerror(errno) << endl;
        return false;
    }

    const uint32_t get_vfp_regs = 32;

    oldfpregs = fpregs;
    if (ptrace((__ptrace_request)get_vfp_regs, pid, 0, &fpregs) != 0) {
        cerr << "update: " << strerror(errno) << endl;
        return false;
    }

    for (unsigned int x = 0; x < numregs; x++)
        regDiffSinceUpdate[x] = (getRegVal(x) != getOldRegVal(x));

    return true;
}

int64_t
ARMTraceChild::getRegVal(int num)
{
    if (num <= CPSR)
        return getRegs(regs, num);
    else
        return (int64_t)getFpRegs(fpregs, num);
}

int64_t
ARMTraceChild::getOldRegVal(int num)
{
    if (num <= CPSR)
        return getRegs(oldregs, num);
    else
        return (int64_t)getFpRegs(oldfpregs, num);
}

ostream &
ARMTraceChild::outputStartState(ostream & os)
{
    uint32_t sp = getSP();
    uint32_t pc = getPC();
    uint32_t highestInfo = 0;
    char obuf[1024];
    sprintf(obuf, "Initial stack pointer = 0x%08x\n", sp);
    os << obuf;
    sprintf(obuf, "Initial program counter = 0x%08x\n", pc);
    os << obuf;

    //Output the argument count
    int32_t cargc = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
    sprintf(obuf, "0x%08x: Argc = 0x%08x\n", sp, cargc);
    os << obuf;
    sp += 4;

    //Output argv pointers
    int argCount = 0;
    int32_t cargv;
    do {
        cargv = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%08x: argv[%d] = 0x%08x\n",
                sp, argCount++, cargv);
        if (cargv)
            if (highestInfo < cargv)
                highestInfo = cargv;
        os << obuf;
        sp += 4;
    } while (cargv);

    //Output the envp pointers
    int envCount = 0;
    uint32_t cenvp;
    do {
        cenvp = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%08x: envp[%d] = 0x%08x\n",
                sp, envCount++, cenvp);
        os << obuf;
        sp += 4;
    } while (cenvp);
    uint32_t auxType, auxVal;
    do {
        auxType = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sp += 4;
        auxVal = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sp += 4;
        sprintf(obuf, "0x%08x: Auxiliary vector = {0x%08x, 0x%08x}\n",
                sp - 8, auxType, auxVal);
        os << obuf;
    } while (auxType != 0 || auxVal != 0);
    //Print out the argument strings, environment strings, and file name.
    string current;
    uint32_t buf;
    uint32_t currentStart = sp;
    bool clearedInitialPadding = false;
    do {
        buf = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        char * cbuf = (char *)&buf;
        for (int x = 0; x < sizeof(uint32_t); x++) {
            if (cbuf[x])
                current += cbuf[x];
            else {
                sprintf(obuf, "0x%08x: \"%s\"\n",
                        currentStart, current.c_str());
                os << obuf;
                current = "";
                currentStart = sp + x + 1;
            }
        }
        sp += 4;
        clearedInitialPadding = clearedInitialPadding || buf != 0;
    } while (!clearedInitialPadding || buf != 0 || sp <= highestInfo);
    return os;
}

bool
ARMTraceChild::step()
{
    const uint32_t bkpt_inst = 0xe7f001f0;

    uint32_t lr = getRegVal(14);
    uint32_t pc = getPC();
    uint32_t lrOp, subsOp;
    char obuf[128];
    bool patch = false;

    // Since ARM uses software breakpoints behind the scenes, they don't work
    // in read only areas like the page of routines provided by the kernel. The
    // link register generally holds the address the process wants to the
    // kernel to return to after it's done, so we'll install a software
    // breakpoint there.
    //
    // Calls into the kernel user page always follow the form:
    //  MVN ...
    //  <possible MOV lr,...>
    //  SUB PC, ...
    //
    //  So we look for this pattern and set a breakpoint on the LR at the SUB
    //  instruction.


    subsOp = ptrace(PTRACE_PEEKDATA, pid, pc, 0);
    if ((subsOp & 0xFFFF0FFF) == 0xe3e00a0f)
        foundMvn = true;

    if (foundMvn && ((subsOp & 0xFFF0F000) == 0xe240f000)) {
        foundMvn = false;
        lrOp = ptrace(PTRACE_PEEKDATA, pid, lr, 0);
        ptrace(PTRACE_POKEDATA, pid, lr, bkpt_inst);
        patch = true;
    }
    ptraceSingleStep();

    if (patch)
        ptrace(PTRACE_POKEDATA, pid, lr, lrOp);
}


TraceChild *
genTraceChild()
{
    return new ARMTraceChild;
}
