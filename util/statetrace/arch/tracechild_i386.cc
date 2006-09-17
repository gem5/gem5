/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "tracechild_i386.hh"

using namespace std;

char * I386TraceChild::regNames[numregs] = {
                //GPRs
                "eax", "ebx", "ecx", "edx",
                //Index registers
                "esi", "edi",
                //Base pointer and stack pointer
                "ebp", "esp",
                //Segmentation registers
                "cs", "ds", "es", "fs", "gs", "ss",
                //PC
                "eip"};

int64_t I386TraceChild::getRegs(user_regs_struct & myregs, int num)
{
        assert(num < numregs && num >= 0);
        switch(num)
        {
                //GPRs
                case EAX: return myregs.eax;
                case EBX: return myregs.ebx;
                case ECX: return myregs.ecx;
                case EDX: return myregs.edx;
                //Index registers
                case ESI: return myregs.esi;
                case EDI: return myregs.edi;
                //Base pointer and stack pointer
                case EBP: return myregs.ebp;
                case ESP: return myregs.esp;
                //Segmentation registers
                case CS: return myregs.cs;
                case DS: return myregs.ds;
                case ES: return myregs.es;
                case FS: return myregs.fs;
                case GS: return myregs.gs;
                case SS: return myregs.ss;
                //PC
                case EIP: return myregs.eip;
                default:
                        assert(0);
                        return 0;
        }
}

bool I386TraceChild::update(int pid)
{
        oldregs = regs;
        if(ptrace(PTRACE_GETREGS, pid, 0, &regs) != 0)
                return false;
        for(unsigned int x = 0; x < numregs; x++)
        {
                regDiffSinceUpdate[x] =
                        (getRegVal(x) != getOldRegVal(x));
        }
}

I386TraceChild::I386TraceChild()
{
        for(unsigned int x = 0; x < numregs; x++)
                regDiffSinceUpdate[x] = false;
}

int64_t I386TraceChild::getRegVal(int num)
{
        return getRegs(regs, num);
}

int64_t I386TraceChild::getOldRegVal(int num)
{
        return getRegs(oldregs, num);
}

char * I386TraceChild::printReg(int num)
{
        sprintf(printBuffer, "0x%08X", getRegVal(num));
        return printBuffer;
}

TraceChild * genTraceChild()
{
        return new I386TraceChild;
}
