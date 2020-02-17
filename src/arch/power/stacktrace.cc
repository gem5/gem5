/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#include "arch/power/stacktrace.hh"

#include <string>

#include "base/trace.hh"

using namespace std;

namespace PowerISA {

ProcessInfo::ProcessInfo(ThreadContext *_tc)
{
    panic("ProcessInfo constructor not implemented.\n");
}

Addr
ProcessInfo::task(Addr ksp) const
{
    panic("ProcessInfo::task not implemented.\n");
    return 0;
}

int
ProcessInfo::pid(Addr ksp) const
{
    panic("ProcessInfo::pid not implemented.\n");
    return 0;
}

string
ProcessInfo::name(Addr ksp) const
{
    panic("ProcessInfo::name not implemented.\n");
    return "";
}

StackTrace::StackTrace()
    : tc(0), stack(64)
{
    panic("StackTrace constructor not implemented.\n");
}

StackTrace::StackTrace(ThreadContext *_tc, const StaticInstPtr &inst)
    : tc(0), stack(64)
{
    panic("StackTrace constructor not implemented.\n");
}

StackTrace::~StackTrace()
{
    panic("StackTrace destructor not implemented.\n");
}

void
StackTrace::trace(ThreadContext *_tc, bool is_call)
{
    panic("StackTrace::trace not implemented.\n");
}

bool
StackTrace::isEntry(Addr addr)
{
    panic("StackTrace::isEntry not implemented.\n");
    return false;
}

bool
StackTrace::decodeStack(MachInst inst, int &disp)
{
    panic("StackTrace::decodeStack not implemented.\n");
    return false;
}

bool
StackTrace::decodeSave(MachInst inst, int &reg, int &disp)
{
    panic("StackTrace::decodeSave not implemented.\n");
    return true;
}

/*
 * Decode the function prologue for the function we're in, and note
 * which registers are stored where, and how large the stack frame is.
 */
bool
StackTrace::decodePrologue(Addr sp, Addr callpc, Addr func, int &size,
                           Addr &ra)
{
    panic("StackTrace::decodePrologue not implemented.\n");
    return true;
}

#if TRACING_ON
void
StackTrace::dump()
{
    panic("StackTrace::dump not implemented.\n");
}
#endif

} // namespace PowerISA
