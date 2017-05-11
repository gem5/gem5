/*
 * Copyright 2015 LabWare
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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
 *          Boris Shingarov
 */

#include "arch/x86/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/vtophys.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/process.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBAcc.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"

using namespace std;
using namespace X86ISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *c) :
    BaseRemoteGDB(_system, c), regCache32(this), regCache64(this)
{}

bool
RemoteGDB::acc(Addr va, size_t len)
{
    if (FullSystem) {
        Walker *walker = context->getDTBPtr()->getWalker();
        unsigned logBytes;
        Fault fault = walker->startFunctional(context, va, logBytes,
                                              BaseTLB::Read);
        if (fault != NoFault)
            return false;

        Addr endVa = va + len - 1;
        if ((va & ~mask(logBytes)) == (endVa & ~mask(logBytes)))
            return true;

        fault = walker->startFunctional(context, endVa, logBytes,
                                        BaseTLB::Read);
        return fault == NoFault;
    } else {
        TlbEntry entry;
        return context->getProcessPtr()->pTable->lookup(va, entry);
    }
}

RemoteGDB::BaseGdbRegCache*
RemoteGDB::gdbRegs()
{
    HandyM5Reg m5reg = context->readMiscRegNoEffect(MISCREG_M5_REG);
    if (m5reg.submode == SixtyFourBitMode)
        return &regCache64;
    else
        return &regCache32;
}



void
RemoteGDB::AMD64GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");
    r.rax = context->readIntReg(INTREG_RAX);
    r.rbx = context->readIntReg(INTREG_RBX);
    r.rcx = context->readIntReg(INTREG_RCX);
    r.rdx = context->readIntReg(INTREG_RDX);
    r.rsi = context->readIntReg(INTREG_RSI);
    r.rdi = context->readIntReg(INTREG_RDI);
    r.rbp = context->readIntReg(INTREG_RBP);
    r.rsp = context->readIntReg(INTREG_RSP);
    r.r8 = context->readIntReg(INTREG_R8);
    r.r9 = context->readIntReg(INTREG_R9);
    r.r10 = context->readIntReg(INTREG_R10);
    r.r11 = context->readIntReg(INTREG_R11);
    r.r12 = context->readIntReg(INTREG_R12);
    r.r13 = context->readIntReg(INTREG_R13);
    r.r14 = context->readIntReg(INTREG_R14);
    r.r15 = context->readIntReg(INTREG_R15);
    r.rip = context->pcState().pc();
    r.eflags = context->readMiscRegNoEffect(MISCREG_RFLAGS);
    r.cs = context->readMiscRegNoEffect(MISCREG_CS);
    r.ss = context->readMiscRegNoEffect(MISCREG_SS);
    r.ds = context->readMiscRegNoEffect(MISCREG_DS);
    r.es = context->readMiscRegNoEffect(MISCREG_ES);
    r.fs = context->readMiscRegNoEffect(MISCREG_FS);
    r.gs = context->readMiscRegNoEffect(MISCREG_GS);
}

void
RemoteGDB::X86GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");
    r.eax = context->readIntReg(INTREG_RAX);
    r.ecx = context->readIntReg(INTREG_RCX);
    r.edx = context->readIntReg(INTREG_RDX);
    r.ebx = context->readIntReg(INTREG_RBX);
    r.esp = context->readIntReg(INTREG_RSP);
    r.ebp = context->readIntReg(INTREG_RBP);
    r.esi = context->readIntReg(INTREG_RSI);
    r.edi = context->readIntReg(INTREG_RDI);
    r.eip = context->pcState().pc();
    r.eflags = context->readMiscRegNoEffect(MISCREG_RFLAGS);
    r.cs = context->readMiscRegNoEffect(MISCREG_CS);
    r.ss = context->readMiscRegNoEffect(MISCREG_SS);
    r.ds = context->readMiscRegNoEffect(MISCREG_DS);
    r.es = context->readMiscRegNoEffect(MISCREG_ES);
    r.fs = context->readMiscRegNoEffect(MISCREG_FS);
    r.gs = context->readMiscRegNoEffect(MISCREG_GS);
}

void
RemoteGDB::AMD64GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");
    context->setIntReg(INTREG_RAX, r.rax);
    context->setIntReg(INTREG_RBX, r.rbx);
    context->setIntReg(INTREG_RCX, r.rcx);
    context->setIntReg(INTREG_RDX, r.rdx);
    context->setIntReg(INTREG_RSI, r.rsi);
    context->setIntReg(INTREG_RDI, r.rdi);
    context->setIntReg(INTREG_RBP, r.rbp);
    context->setIntReg(INTREG_RSP, r.rsp);
    context->setIntReg(INTREG_R8, r.r8);
    context->setIntReg(INTREG_R9, r.r9);
    context->setIntReg(INTREG_R10, r.r10);
    context->setIntReg(INTREG_R11, r.r11);
    context->setIntReg(INTREG_R12, r.r12);
    context->setIntReg(INTREG_R13, r.r13);
    context->setIntReg(INTREG_R14, r.r14);
    context->setIntReg(INTREG_R15, r.r15);
    context->pcState(r.rip);
    context->setMiscReg(MISCREG_RFLAGS, r.eflags);
    if (r.cs != context->readMiscRegNoEffect(MISCREG_CS))
        warn("Remote gdb: Ignoring update to CS.\n");
    if (r.ss != context->readMiscRegNoEffect(MISCREG_SS))
        warn("Remote gdb: Ignoring update to SS.\n");
    if (r.ds != context->readMiscRegNoEffect(MISCREG_DS))
        warn("Remote gdb: Ignoring update to DS.\n");
    if (r.es != context->readMiscRegNoEffect(MISCREG_ES))
        warn("Remote gdb: Ignoring update to ES.\n");
    if (r.fs != context->readMiscRegNoEffect(MISCREG_FS))
        warn("Remote gdb: Ignoring update to FS.\n");
    if (r.gs != context->readMiscRegNoEffect(MISCREG_GS))
        warn("Remote gdb: Ignoring update to GS.\n");
}

void
RemoteGDB::X86GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");
    context->setIntReg(INTREG_RAX, r.eax);
    context->setIntReg(INTREG_RCX, r.ecx);
    context->setIntReg(INTREG_RDX, r.edx);
    context->setIntReg(INTREG_RBX, r.ebx);
    context->setIntReg(INTREG_RSP, r.esp);
    context->setIntReg(INTREG_RBP, r.ebp);
    context->setIntReg(INTREG_RSI, r.esi);
    context->setIntReg(INTREG_RDI, r.edi);
    context->pcState(r.eip);
    context->setMiscReg(MISCREG_RFLAGS, r.eflags);
    if (r.cs != context->readMiscRegNoEffect(MISCREG_CS))
        warn("Remote gdb: Ignoring update to CS.\n");
    if (r.ss != context->readMiscRegNoEffect(MISCREG_SS))
        warn("Remote gdb: Ignoring update to SS.\n");
    if (r.ds != context->readMiscRegNoEffect(MISCREG_DS))
        warn("Remote gdb: Ignoring update to DS.\n");
    if (r.es != context->readMiscRegNoEffect(MISCREG_ES))
        warn("Remote gdb: Ignoring update to ES.\n");
    if (r.fs != context->readMiscRegNoEffect(MISCREG_FS))
        warn("Remote gdb: Ignoring update to FS.\n");
    if (r.gs != context->readMiscRegNoEffect(MISCREG_GS))
        warn("Remote gdb: Ignoring update to GS.\n");
}
