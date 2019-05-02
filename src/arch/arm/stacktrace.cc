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
 *
 * Authors: Nathan Binkert
 */

#include "arch/arm/stacktrace.hh"

#include <string>

#include "arch/arm/isa_traits.hh"
#include "arch/arm/vtophys.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "sim/system.hh"

namespace ArmISA
{

static int32_t
readSymbol(ThreadContext *tc, const std::string name)
{
    PortProxy &vp = tc->getVirtProxy();
    SymbolTable *symtab = tc->getSystemPtr()->kernelSymtab;

    Addr addr;
    if (!symtab->findAddress(name, addr))
        panic("thread info not compiled into kernel\n");

    return vp.read<int32_t>(addr, GuestByteOrder);
}

ProcessInfo::ProcessInfo(ThreadContext *_tc) : tc(_tc)
{
    thread_info_size = readSymbol(tc, "thread_info_size");
    task_struct_size = readSymbol(tc, "task_struct_size");
    task_off = readSymbol(tc, "thread_info_task");
    pid_off = readSymbol(tc, "task_struct_pid");
    name_off = readSymbol(tc, "task_struct_comm");
}

Addr
ProcessInfo::task(Addr ksp) const
{
    Addr base = ksp & ~0x1fff;
    if (base == ULL(0xffffffffc0000000))
        return 0;

    Addr tsk;

    PortProxy &vp = tc->getVirtProxy();
    tsk = vp.read<Addr>(base + task_off, GuestByteOrder);

    return tsk;
}

int
ProcessInfo::pid(Addr ksp) const
{
    Addr task = this->task(ksp);
    if (!task)
        return -1;

    uint16_t pd;

    PortProxy &vp = tc->getVirtProxy();
    pd = vp.read<uint16_t>(task + pid_off, GuestByteOrder);

    return pd;
}

std::string
ProcessInfo::name(Addr ksp) const
{
    Addr task = this->task(ksp);
    if (!task)
        return "unknown";

    char comm[256];
    tc->getVirtProxy().readString(comm, task + name_off, sizeof(comm));
    if (!comm[0])
        return "startup";

    return comm;
}

StackTrace::StackTrace()
    : tc(0), stack(64)
{
}

StackTrace::StackTrace(ThreadContext *_tc, const StaticInstPtr &inst)
    : tc(0), stack(64)
{
    trace(_tc, inst);
}

StackTrace::~StackTrace()
{
}

void
StackTrace::trace(ThreadContext *_tc, bool is_call)
{
}

bool
StackTrace::isEntry(Addr addr)
{
    return false;
}

bool
StackTrace::decodeStack(MachInst inst, int &disp)
{
    return false;
}

bool
StackTrace::decodeSave(MachInst inst, int &reg, int &disp)
{
    return false;
}

/*
 * Decode the function prologue for the function we're in, and note
 * which registers are stored where, and how large the stack frame is.
 */
bool
StackTrace::decodePrologue(Addr sp, Addr callpc, Addr func,
                           int &size, Addr &ra)
{
    return false;
}

#if TRACING_ON
void
StackTrace::dump()
{
    DPRINTFN("------ Stack ------\n");

    DPRINTFN(" Not implemented\n");
}
#endif
}
