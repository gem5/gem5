/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "arch/mips/stacktrace.hh"

#include <string>

#include "arch/mips/isa_traits.hh"
#include "arch/mips/vtophys.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "sim/system.hh"

using namespace MipsISA;

ProcessInfo::ProcessInfo(ThreadContext *_tc) : tc(_tc)
{}

Addr
ProcessInfo::task(Addr ksp) const
{
    Addr base = ksp & ~0x3fff;
    if (base == ULL(0xfffffc0000000000))
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
        return "console";

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
    tc = _tc;
    bool usermode = 0;

    if (usermode) {
        stack.push_back(user);
        return;
    }
}

bool
StackTrace::isEntry(Addr addr)
{
    return false;
}

bool
StackTrace::decodeStack(MachInst inst, int &disp)
{
    // lda $sp, -disp($sp)
    //
    // Opcode<31:26> == 0x08
    // RA<25:21> == 30
    // RB<20:16> == 30
    // Disp<15:0>
    const MachInst mem_mask = 0xffff0000;
    const MachInst lda_pattern = 0x23de0000;
    const MachInst lda_disp_mask = 0x0000ffff;

    // subq $sp, disp, $sp
    // addq $sp, disp, $sp
    //
    // Opcode<31:26> == 0x10
    // RA<25:21> == 30
    // Lit<20:13>
    // One<12> = 1
    // Func<11:5> == 0x20 (addq)
    // Func<11:5> == 0x29 (subq)
    // RC<4:0> == 30
    const MachInst intop_mask = 0xffe01fff;
    const MachInst addq_pattern = 0x43c0141e;
    const MachInst subq_pattern = 0x43c0153e;
    const MachInst intop_disp_mask = 0x001fe000;
    const int intop_disp_shift = 13;

    if ((inst & mem_mask) == lda_pattern)
        disp = -sext<16>(inst & lda_disp_mask);
    else if ((inst & intop_mask) == addq_pattern)
        disp = -int((inst & intop_disp_mask) >> intop_disp_shift);
    else if ((inst & intop_mask) == subq_pattern)
        disp = int((inst & intop_disp_mask) >> intop_disp_shift);
    else
        return false;

    return true;
}

bool
StackTrace::decodeSave(MachInst inst, int &reg, int &disp)
{
    // lda $stq, disp($sp)
    //
    // Opcode<31:26> == 0x08
    // RA<25:21> == ?
    // RB<20:16> == 30
    // Disp<15:0>
    const MachInst stq_mask = 0xfc1f0000;
    const MachInst stq_pattern = 0xb41e0000;
    const MachInst stq_disp_mask = 0x0000ffff;
    const MachInst reg_mask = 0x03e00000;
    const int reg_shift = 21;

    if ((inst & stq_mask) == stq_pattern) {
        reg = (inst & reg_mask) >> reg_shift;
        disp = sext<16>(inst & stq_disp_mask);
    } else {
        return false;
    }

    return true;
}

/*
 * Decode the function prologue for the function we're in, and note
 * which registers are stored where, and how large the stack frame is.
 */
bool
StackTrace::decodePrologue(Addr sp, Addr callpc, Addr func,
                           int &size, Addr &ra)
{
    size = 0;
    ra = 0;

    for (Addr pc = func; pc < callpc; pc += sizeof(MachInst)) {
        MachInst inst = tc->getVirtProxy().read<MachInst>(pc);

        int reg, disp;
        if (decodeStack(inst, disp)) {
            if (size) {
                return true;
            }
            size += disp;
        } else if (decodeSave(inst, reg, disp)) {
            if (!ra && reg == ReturnAddressReg) {
                ra = tc->getVirtProxy().read<Addr>(sp + disp);
                if (!ra) {
                    return false;
                }
            }
        }
    }

    return true;
}

#if TRACING_ON
void
StackTrace::dump()
{
    panic("Stack trace dump not implemented.\n");
}
#endif
