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
 */

#include "arch/x86/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/x86/mmu.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/process.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBAcc.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"
#include "sim/workload.hh"

namespace gem5
{

using namespace X86ISA;

RemoteGDB::RemoteGDB(System *_system, ListenSocketConfig _listen_config) :
    BaseRemoteGDB(_system, _listen_config),
    regCache32(this), regCache64(this)
{}

bool
RemoteGDB::acc(Addr va, size_t len)
{
    if (FullSystem) {
        Walker *walker = dynamic_cast<MMU *>(
            context()->getMMUPtr())->getDataWalker();
        unsigned logBytes;
        Fault fault = walker->startFunctional(context(), va, logBytes,
                                              BaseMMU::Read);
        if (fault != NoFault)
            return false;

        Addr endVa = va + len - 1;
        if ((va & ~mask(logBytes)) == (endVa & ~mask(logBytes)))
            return true;

        fault = walker->startFunctional(context(), endVa, logBytes,
                                        BaseMMU::Read);
        return fault == NoFault;
    } else {
        return context()->getProcessPtr()->pTable->lookup(va) != nullptr;
    }
}

BaseGdbRegCache*
RemoteGDB::gdbRegs()
{
    // First, try to figure out which type of register cache to return based
    // on the architecture reported by the workload.
    if (system()->workload) {
        auto arch = system()->workload->getArch();
        if (arch == loader::X86_64) {
            return &regCache64;
        } else if (arch == loader::I386) {
            return &regCache32;
        } else if (arch != loader::UnknownArch) {
            panic("Unrecognized workload arch %s.",
                    loader::archToString(arch));
        }
    }

    // If that didn't work, decide based on the current mode of the context.
    HandyM5Reg m5reg = context()->readMiscRegNoEffect(misc_reg::M5Reg);
    if (m5reg.submode == SixtyFourBitMode)
        return &regCache64;
    else
        return &regCache32;
}



void
RemoteGDB::AMD64GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");
    r.rax = context->getReg(int_reg::Rax);
    r.rbx = context->getReg(int_reg::Rbx);
    r.rcx = context->getReg(int_reg::Rcx);
    r.rdx = context->getReg(int_reg::Rdx);
    r.rsi = context->getReg(int_reg::Rsi);
    r.rdi = context->getReg(int_reg::Rdi);
    r.rbp = context->getReg(int_reg::Rbp);
    r.rsp = context->getReg(int_reg::Rsp);
    r.r8 = context->getReg(int_reg::R8);
    r.r9 = context->getReg(int_reg::R9);
    r.r10 = context->getReg(int_reg::R10);
    r.r11 = context->getReg(int_reg::R11);
    r.r12 = context->getReg(int_reg::R12);
    r.r13 = context->getReg(int_reg::R13);
    r.r14 = context->getReg(int_reg::R14);
    r.r15 = context->getReg(int_reg::R15);
    r.rip = context->pcState().instAddr();
    r.eflags = context->readMiscRegNoEffect(misc_reg::Rflags);
    r.cs = context->readMiscRegNoEffect(misc_reg::Cs);
    r.ss = context->readMiscRegNoEffect(misc_reg::Ss);
    r.ds = context->readMiscRegNoEffect(misc_reg::Ds);
    r.es = context->readMiscRegNoEffect(misc_reg::Es);
    r.fs = context->readMiscRegNoEffect(misc_reg::Fs);
    r.gs = context->readMiscRegNoEffect(misc_reg::Gs);
}

void
RemoteGDB::X86GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");
    r.eax = context->getReg(int_reg::Rax);
    r.ecx = context->getReg(int_reg::Rcx);
    r.edx = context->getReg(int_reg::Rdx);
    r.ebx = context->getReg(int_reg::Rbx);
    r.esp = context->getReg(int_reg::Rsp);
    r.ebp = context->getReg(int_reg::Rbp);
    r.esi = context->getReg(int_reg::Rsi);
    r.edi = context->getReg(int_reg::Rdi);
    r.eip = context->pcState().instAddr();
    r.eflags = context->readMiscRegNoEffect(misc_reg::Rflags);
    r.cs = context->readMiscRegNoEffect(misc_reg::Cs);
    r.ss = context->readMiscRegNoEffect(misc_reg::Ss);
    r.ds = context->readMiscRegNoEffect(misc_reg::Ds);
    r.es = context->readMiscRegNoEffect(misc_reg::Es);
    r.fs = context->readMiscRegNoEffect(misc_reg::Fs);
    r.gs = context->readMiscRegNoEffect(misc_reg::Gs);
}

void
RemoteGDB::AMD64GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");
    context->setReg(int_reg::Rax, r.rax);
    context->setReg(int_reg::Rbx, r.rbx);
    context->setReg(int_reg::Rcx, r.rcx);
    context->setReg(int_reg::Rdx, r.rdx);
    context->setReg(int_reg::Rsi, r.rsi);
    context->setReg(int_reg::Rdi, r.rdi);
    context->setReg(int_reg::Rbp, r.rbp);
    context->setReg(int_reg::Rsp, r.rsp);
    context->setReg(int_reg::R8, r.r8);
    context->setReg(int_reg::R9, r.r9);
    context->setReg(int_reg::R10, r.r10);
    context->setReg(int_reg::R11, r.r11);
    context->setReg(int_reg::R12, r.r12);
    context->setReg(int_reg::R13, r.r13);
    context->setReg(int_reg::R14, r.r14);
    context->setReg(int_reg::R15, r.r15);
    context->pcState(r.rip);
    context->setMiscReg(misc_reg::Rflags, r.eflags);
    if (r.cs != context->readMiscRegNoEffect(misc_reg::Cs))
        warn("Remote gdb: Ignoring update to CS.\n");
    if (r.ss != context->readMiscRegNoEffect(misc_reg::Ss))
        warn("Remote gdb: Ignoring update to SS.\n");
    if (r.ds != context->readMiscRegNoEffect(misc_reg::Ds))
        warn("Remote gdb: Ignoring update to DS.\n");
    if (r.es != context->readMiscRegNoEffect(misc_reg::Es))
        warn("Remote gdb: Ignoring update to ES.\n");
    if (r.fs != context->readMiscRegNoEffect(misc_reg::Fs))
        warn("Remote gdb: Ignoring update to FS.\n");
    if (r.gs != context->readMiscRegNoEffect(misc_reg::Gs))
        warn("Remote gdb: Ignoring update to GS.\n");
}

void
RemoteGDB::X86GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");
    context->setReg(int_reg::Rax, r.eax);
    context->setReg(int_reg::Rcx, r.ecx);
    context->setReg(int_reg::Rdx, r.edx);
    context->setReg(int_reg::Rbx, r.ebx);
    context->setReg(int_reg::Rsp, r.esp);
    context->setReg(int_reg::Rbp, r.ebp);
    context->setReg(int_reg::Rsi, r.esi);
    context->setReg(int_reg::Rdi, r.edi);
    context->pcState(r.eip);
    context->setMiscReg(misc_reg::Rflags, r.eflags);
    if (r.cs != context->readMiscRegNoEffect(misc_reg::Cs))
        warn("Remote gdb: Ignoring update to CS.\n");
    if (r.ss != context->readMiscRegNoEffect(misc_reg::Ss))
        warn("Remote gdb: Ignoring update to SS.\n");
    if (r.ds != context->readMiscRegNoEffect(misc_reg::Ds))
        warn("Remote gdb: Ignoring update to DS.\n");
    if (r.es != context->readMiscRegNoEffect(misc_reg::Es))
        warn("Remote gdb: Ignoring update to ES.\n");
    if (r.fs != context->readMiscRegNoEffect(misc_reg::Fs))
        warn("Remote gdb: Ignoring update to FS.\n");
    if (r.gs != context->readMiscRegNoEffect(misc_reg::Gs))
        warn("Remote gdb: Ignoring update to GS.\n");
}

} // namespace gem5
