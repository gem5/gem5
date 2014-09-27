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

#include <string>

#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/stacktrace.hh"
#include "arch/alpha/vtophys.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "sim/system.hh"

using namespace std;

namespace AlphaISA {

ProcessInfo::ProcessInfo(ThreadContext *_tc)
    : tc(_tc)
{
    Addr addr = 0;
    FSTranslatingPortProxy &vp = tc->getVirtProxy();
    SymbolTable *symtab = tc->getSystemPtr()->kernelSymtab;

    if (!symtab->findAddress("thread_info_size", addr))
        panic("thread info not compiled into kernel\n");
    thread_info_size = vp.readGtoH<int32_t>(addr);

    if (!symtab->findAddress("task_struct_size", addr))
        panic("thread info not compiled into kernel\n");
    task_struct_size = vp.readGtoH<int32_t>(addr);

    if (!symtab->findAddress("thread_info_task", addr))
        panic("thread info not compiled into kernel\n");
    task_off = vp.readGtoH<int32_t>(addr);

    if (!symtab->findAddress("task_struct_pid", addr))
        panic("thread info not compiled into kernel\n");
    pid_off = vp.readGtoH<int32_t>(addr);

    if (!symtab->findAddress("task_struct_comm", addr))
        panic("thread info not compiled into kernel\n");
    name_off = vp.readGtoH<int32_t>(addr);
}

Addr
ProcessInfo::task(Addr ksp) const
{
    Addr base = ksp & ~0x3fff;
    if (base == ULL(0xfffffc0000000000))
        return 0;

    Addr tsk;

    FSTranslatingPortProxy &vp = tc->getVirtProxy();
    tsk = vp.readGtoH<Addr>(base + task_off);

    return tsk;
}

int
ProcessInfo::pid(Addr ksp) const
{
    Addr task = this->task(ksp);
    if (!task)
        return -1;

    uint16_t pd;

    FSTranslatingPortProxy &vp = tc->getVirtProxy();
    pd = vp.readGtoH<uint16_t>(task + pid_off);

    return pd;
}

string
ProcessInfo::name(Addr ksp) const
{
    Addr task = this->task(ksp);
    if (!task)
        return "console";

    char comm[256];
    CopyStringOut(tc, comm, task + name_off, sizeof(comm));
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

    System *sys = tc->getSystemPtr();

    bool usermode =
        (tc->readMiscRegNoEffect(IPR_DTB_CM) & 0x18) != 0;

    Addr pc = tc->pcState().pc();
    bool kernel = sys->kernelStart <= pc && pc <= sys->kernelEnd;

    if (usermode) {
        stack.push_back(user);
        return;
    }

    if (!kernel) {
        stack.push_back(console);
        return;
    }

    SymbolTable *symtab = sys->kernelSymtab;
    Addr ksp = tc->readIntReg(StackPointerReg);
    Addr bottom = ksp & ~0x3fff;

    if (is_call) {
        Addr addr;
        if (!symtab->findNearestAddr(pc, addr))
            panic("could not find address %#x", pc);

        stack.push_back(addr);
        pc = tc->pcState().pc();
    }

    while (ksp > bottom) {
        Addr addr;
        if (!symtab->findNearestAddr(pc, addr))
            panic("could not find symbol for pc=%#x", pc);
        assert(pc >= addr && "symbol botch: callpc < func");

        stack.push_back(addr);

        if (isEntry(addr))
            return;

        Addr ra;
        int size;
        if (decodePrologue(ksp, pc, addr, size, ra)) {
            if (!ra)
                return;

            if (size <= 0) {
                stack.push_back(unknown);
                return;
            }

            pc = ra;
            ksp += size;
        } else {
            stack.push_back(unknown);
            return;
        }

        kernel = sys->kernelStart <= pc && pc <= sys->kernelEnd;
        if (!kernel)
            return;

        if (stack.size() >= 1000)
            panic("unwinding too far");
    }

    panic("unwinding too far");
}

bool
StackTrace::isEntry(Addr addr)
{
    if (addr == tc->readMiscRegNoEffect(IPR_PALtemp12))
        return true;

    if (addr == tc->readMiscRegNoEffect(IPR_PALtemp7))
        return true;

    if (addr == tc->readMiscRegNoEffect(IPR_PALtemp11))
        return true;

    if (addr == tc->readMiscRegNoEffect(IPR_PALtemp21))
        return true;

    if (addr == tc->readMiscRegNoEffect(IPR_PALtemp9))
        return true;

    if (addr == tc->readMiscRegNoEffect(IPR_PALtemp2))
        return true;

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
StackTrace::decodePrologue(Addr sp, Addr callpc, Addr func, int &size,
                           Addr &ra)
{
    size = 0;
    ra = 0;

    for (Addr pc = func; pc < callpc; pc += sizeof(MachInst)) {
        MachInst inst;
        CopyOut(tc, (uint8_t *)&inst, pc, sizeof(MachInst));

        int reg, disp;
        if (decodeStack(inst, disp)) {
            if (size) {
                // panic("decoding frame size again");
                return true;
            }
            size += disp;
        } else if (decodeSave(inst, reg, disp)) {
            if (!ra && reg == ReturnAddressReg) {
                CopyOut(tc, (uint8_t *)&ra, sp + disp, sizeof(Addr));
                if (!ra) {
                    // panic("no return address value pc=%#x\n", pc);
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
    StringWrap name(tc->getCpuPtr()->name());
    SymbolTable *symtab = tc->getSystemPtr()->kernelSymtab;

    DPRINTFN("------ Stack ------\n");

    string symbol;
    for (int i = 0, size = stack.size(); i < size; ++i) {
        Addr addr = stack[size - i - 1];
        if (addr == user)
            symbol = "user";
        else if (addr == console)
            symbol = "console";
        else if (addr == unknown)
            symbol = "unknown";
        else
            symtab->findSymbol(addr, symbol);

        DPRINTFN("%#x: %s\n", addr, symbol);
    }
}
#endif

} // namespace AlphaISA
