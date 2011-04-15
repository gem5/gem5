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

#include <sys/ptrace.h>
#include <stdint.h>

#include <cerrno>
#include <iostream>

#include "arch/i686/tracechild.hh"

using namespace std;

int64_t
I686TraceChild::getRegs(user_regs_struct & myregs, int num)
{
    assert(num < numregs && num >= 0);
    switch (num) {
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
      case CS: return myregs.xcs;
      case DS: return myregs.xds;
      case ES: return myregs.xes;
      case FS: return myregs.xfs;
      case GS: return myregs.xgs;
      case SS: return myregs.xss;
      //PC
      case EIP: return myregs.eip;
      default:
        assert(0);
        return 0;
    }
}

bool
I686TraceChild::update(int pid)
{
    oldregs = regs;
    if (ptrace(PTRACE_GETREGS, pid, 0, &regs) != 0)
        return false;
    for (unsigned int x = 0; x < numregs; x++) {
        regDiffSinceUpdate[x] = (getRegVal(x) != getOldRegVal(x));
    }
}

I686TraceChild::I686TraceChild()
{
    for (unsigned int x = 0; x < numregs; x++)
        regDiffSinceUpdate[x] = false;
}

int64_t
I686TraceChild::getRegVal(int num)
{
    return getRegs(regs, num);
}

int64_t
I686TraceChild::getOldRegVal(int num)
{
    return getRegs(oldregs, num);
}

bool
I686TraceChild::sendState(int socket)
{
    return false;
}

TraceChild *
genTraceChild()
{
    return new I686TraceChild;
}
