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
 *
 * Authors: Gabe Black
 */

#include <iostream>
#include <errno.h>
#include <sys/ptrace.h>
#include <stdint.h>

#include "tracechild_amd64.hh"

using namespace std;

char * AMD64TraceChild::regNames[numregs] = {
                //GPRs
                "rax", "rbx", "rcx", "rdx",
                //Index registers
                "rsi", "rdi",
                //Base pointer and stack pointer
                "rbp", "rsp",
                //New 64 bit mode registers
                "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
                //Segmentation registers
                "cs", "ds", "es", "fs", "gs", "ss", "fs_base", "gs_base",
                //PC
                "rip",
                //Flags
                "eflags"};

bool AMD64TraceChild::sendState(int socket)
{
    uint64_t regVal = 0;
    for(int x = 0; x <= R15; x++)
    {
        regVal = getRegVal(x);
        if(write(socket, &regVal, sizeof(regVal)) == -1)
        {
            cerr << "Write failed! " << strerror(errno) << endl;
            tracing = false;
            return false;
        }
    }
    regVal = getRegVal(RIP);
    if(write(socket, &regVal, sizeof(regVal)) == -1)
    {
        cerr << "Write failed! " << strerror(errno) << endl;
        tracing = false;
        return false;
    }
    return true;
}

int64_t AMD64TraceChild::getRegs(user_regs_struct & myregs, int num)
{
        assert(num < numregs && num >= 0);
        switch(num)
        {
                //GPRs
                case RAX: return myregs.rax;
                case RBX: return myregs.rbx;
                case RCX: return myregs.rcx;
                case RDX: return myregs.rdx;
                //Index registers
                case RSI: return myregs.rsi;
                case RDI: return myregs.rdi;
                //Base pointer and stack pointer
                case RBP: return myregs.rbp;
                case RSP: return myregs.rsp;
                //New 64 bit mode registers
                case R8: return myregs.r8;
                case R9: return myregs.r9;
                case R10: return myregs.r10;
                case R11: return myregs.r11;
                case R12: return myregs.r12;
                case R13: return myregs.r13;
                case R14: return myregs.r14;
                case R15: return myregs.r15;
                //Segmentation registers
                case CS: return myregs.cs;
                case DS: return myregs.ds;
                case ES: return myregs.es;
                case FS: return myregs.fs;
                case GS: return myregs.gs;
                case SS: return myregs.ss;
                case FS_BASE: return myregs.fs_base;
                case GS_BASE: return myregs.gs_base;
                //PC
                case RIP: return myregs.rip;
                //Flags
                case EFLAGS: return myregs.eflags;
                default:
                        assert(0);
                        return 0;
        }
}

bool AMD64TraceChild::update(int pid)
{
    oldregs = regs;
    if(ptrace(PTRACE_GETREGS, pid, 0, &regs) != 0)
    {
        cerr << "update: " << strerror(errno) << endl;
        return false;
    }
    for(unsigned int x = 0; x < numregs; x++)
        regDiffSinceUpdate[x] = (getRegVal(x) != getOldRegVal(x));
    return true;
}

AMD64TraceChild::AMD64TraceChild()
{
    for(unsigned int x = 0; x < numregs; x++)
        regDiffSinceUpdate[x] = false;
}

int64_t AMD64TraceChild::getRegVal(int num)
{
        return getRegs(regs, num);
}

int64_t AMD64TraceChild::getOldRegVal(int num)
{
        return getRegs(oldregs, num);
}

char * AMD64TraceChild::printReg(int num)
{
        sprintf(printBuffer, "0x%08X", getRegVal(num));
        return printBuffer;
}

ostream & AMD64TraceChild::outputStartState(ostream & os)
{
    uint64_t sp = getSP();
    uint64_t pc = getPC();
    char obuf[1024];
    sprintf(obuf, "Initial stack pointer = 0x%016llx\n", sp);
    os << obuf;
    sprintf(obuf, "Initial program counter = 0x%016llx\n", pc);
    os << obuf;

    //Output the argument count
    uint64_t cargc = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
    sprintf(obuf, "0x%016llx: Argc = 0x%016llx\n", sp, cargc);
    os << obuf;
    sp += 8;

    //Output argv pointers
    int argCount = 0;
    uint64_t cargv;
    do
    {
        cargv = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%016llx: argv[%d] = 0x%016llx\n",
                sp, argCount++, cargv);
        os << obuf;
        sp += 8;
    } while(cargv);

    //Output the envp pointers
    int envCount = 0;
    uint64_t cenvp;
    do
    {
        cenvp = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%016llx: envp[%d] = 0x%016llx\n",
                sp, envCount++, cenvp);
        os << obuf;
        sp += 8;
    } while(cenvp);
    uint64_t auxType, auxVal;
    do
    {
        auxType = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sp += 8;
        auxVal = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sp += 8;
        sprintf(obuf, "0x%016llx: Auxiliary vector = {0x%016llx, 0x%016llx}\n",
                sp - 8, auxType, auxVal);
        os << obuf;
    } while(auxType != 0 || auxVal != 0);
    //Print out the argument strings, environment strings, and file name.
    string current;
    uint64_t buf;
    uint64_t currentStart = sp;
    bool clearedInitialPadding = false;
    do
    {
        buf = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        char * cbuf = (char *)&buf;
        for(int x = 0; x < sizeof(uint64_t); x++)
        {
            if(cbuf[x])
                current += cbuf[x];
            else
            {
                sprintf(obuf, "0x%016llx: \"%s\"\n",
                        currentStart, current.c_str());
                os << obuf;
                current = "";
                currentStart = sp + x + 1;
            }
        }
        sp += 8;
        clearedInitialPadding = clearedInitialPadding || buf != 0;
    } while(!clearedInitialPadding || buf != 0);
    return os;
}

TraceChild * genTraceChild()
{
        return new AMD64TraceChild;
}
