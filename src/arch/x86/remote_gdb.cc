/*
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
 */

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/process.hh"
#include "arch/x86/remote_gdb.hh"
#include "arch/vtophys.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"

using namespace std;
using namespace X86ISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *c) :
    BaseRemoteGDB(_system, c, GDB_REG_BYTES)
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

void
RemoteGDB::getregs()
{
    HandyM5Reg m5reg = context->readMiscRegNoEffect(MISCREG_M5_REG);
    if (m5reg.submode == SixtyFourBitMode) {
        gdbregs.regs64[GDB64_RAX] = context->readIntReg(INTREG_RAX);
        gdbregs.regs64[GDB64_RBX] = context->readIntReg(INTREG_RBX);
        gdbregs.regs64[GDB64_RCX] = context->readIntReg(INTREG_RCX);
        gdbregs.regs64[GDB64_RDX] = context->readIntReg(INTREG_RDX);
        gdbregs.regs64[GDB64_RSI] = context->readIntReg(INTREG_RSI);
        gdbregs.regs64[GDB64_RDI] = context->readIntReg(INTREG_RDI);
        gdbregs.regs64[GDB64_RBP] = context->readIntReg(INTREG_RBP);
        gdbregs.regs64[GDB64_RSP] = context->readIntReg(INTREG_RSP);
        gdbregs.regs64[GDB64_R8] = context->readIntReg(INTREG_R8);
        gdbregs.regs64[GDB64_R9] = context->readIntReg(INTREG_R9);
        gdbregs.regs64[GDB64_R10] = context->readIntReg(INTREG_R10);
        gdbregs.regs64[GDB64_R11] = context->readIntReg(INTREG_R11);
        gdbregs.regs64[GDB64_R12] = context->readIntReg(INTREG_R12);
        gdbregs.regs64[GDB64_R13] = context->readIntReg(INTREG_R13);
        gdbregs.regs64[GDB64_R14] = context->readIntReg(INTREG_R14);
        gdbregs.regs64[GDB64_R15] = context->readIntReg(INTREG_R15);
        gdbregs.regs64[GDB64_RIP] = context->pcState().pc();
        gdbregs.regs32[GDB64_RFLAGS_32] =
            context->readMiscRegNoEffect(MISCREG_RFLAGS);
        gdbregs.regs32[GDB64_CS_32] = context->readMiscRegNoEffect(MISCREG_CS);
        gdbregs.regs32[GDB64_SS_32] = context->readMiscRegNoEffect(MISCREG_SS);
        gdbregs.regs32[GDB64_DS_32] = context->readMiscRegNoEffect(MISCREG_DS);
        gdbregs.regs32[GDB64_ES_32] = context->readMiscRegNoEffect(MISCREG_ES);
        gdbregs.regs32[GDB64_FS_32] = context->readMiscRegNoEffect(MISCREG_FS);
        gdbregs.regs32[GDB64_GS_32] = context->readMiscRegNoEffect(MISCREG_GS);
    } else {
        gdbregs.regs32[GDB32_EAX] = context->readIntReg(INTREG_RAX);
        gdbregs.regs32[GDB32_ECX] = context->readIntReg(INTREG_RCX);
        gdbregs.regs32[GDB32_EDX] = context->readIntReg(INTREG_RDX);
        gdbregs.regs32[GDB32_EBX] = context->readIntReg(INTREG_RBX);
        gdbregs.regs32[GDB32_ESP] = context->readIntReg(INTREG_RSP);
        gdbregs.regs32[GDB32_EBP] = context->readIntReg(INTREG_RBP);
        gdbregs.regs32[GDB32_ESI] = context->readIntReg(INTREG_RSI);
        gdbregs.regs32[GDB32_EDI] = context->readIntReg(INTREG_RDI);
        gdbregs.regs32[GDB32_EIP] = context->pcState().pc();
        gdbregs.regs32[GDB32_EFLAGS] =
            context->readMiscRegNoEffect(MISCREG_RFLAGS);
        gdbregs.regs32[GDB32_CS] = context->readMiscRegNoEffect(MISCREG_CS);
        gdbregs.regs32[GDB32_CS] = context->readMiscRegNoEffect(MISCREG_SS);
        gdbregs.regs32[GDB32_CS] = context->readMiscRegNoEffect(MISCREG_DS);
        gdbregs.regs32[GDB32_CS] = context->readMiscRegNoEffect(MISCREG_ES);
        gdbregs.regs32[GDB32_CS] = context->readMiscRegNoEffect(MISCREG_FS);
        gdbregs.regs32[GDB32_CS] = context->readMiscRegNoEffect(MISCREG_GS);
    }
}

void
RemoteGDB::setregs()
{
    HandyM5Reg m5reg = context->readMiscRegNoEffect(MISCREG_M5_REG);
    if (m5reg.submode == SixtyFourBitMode) {
        context->setIntReg(INTREG_RAX, gdbregs.regs64[GDB64_RAX]);
        context->setIntReg(INTREG_RBX, gdbregs.regs64[GDB64_RBX]);
        context->setIntReg(INTREG_RCX, gdbregs.regs64[GDB64_RCX]);
        context->setIntReg(INTREG_RDX, gdbregs.regs64[GDB64_RDX]);
        context->setIntReg(INTREG_RSI, gdbregs.regs64[GDB64_RSI]);
        context->setIntReg(INTREG_RDI, gdbregs.regs64[GDB64_RDI]);
        context->setIntReg(INTREG_RBP, gdbregs.regs64[GDB64_RBP]);
        context->setIntReg(INTREG_RSP, gdbregs.regs64[GDB64_RSP]);
        context->setIntReg(INTREG_R8, gdbregs.regs64[GDB64_R8]);
        context->setIntReg(INTREG_R9, gdbregs.regs64[GDB64_R9]);
        context->setIntReg(INTREG_R10, gdbregs.regs64[GDB64_R10]);
        context->setIntReg(INTREG_R11, gdbregs.regs64[GDB64_R11]);
        context->setIntReg(INTREG_R12, gdbregs.regs64[GDB64_R12]);
        context->setIntReg(INTREG_R13, gdbregs.regs64[GDB64_R13]);
        context->setIntReg(INTREG_R14, gdbregs.regs64[GDB64_R14]);
        context->setIntReg(INTREG_R15, gdbregs.regs64[GDB64_R15]);
        context->pcState(gdbregs.regs64[GDB64_RIP]);
        context->setMiscReg(MISCREG_RFLAGS, gdbregs.regs32[GDB64_RFLAGS_32]);
        if (gdbregs.regs32[GDB64_CS_32] !=
            context->readMiscRegNoEffect(MISCREG_CS)) {
            warn("Remote gdb: Ignoring update to CS.\n");
        }
        if (gdbregs.regs32[GDB64_SS_32] !=
            context->readMiscRegNoEffect(MISCREG_SS)) {
            warn("Remote gdb: Ignoring update to SS.\n");
        }
        if (gdbregs.regs32[GDB64_DS_32] !=
            context->readMiscRegNoEffect(MISCREG_DS)) {
            warn("Remote gdb: Ignoring update to DS.\n");
        }
        if (gdbregs.regs32[GDB64_ES_32] !=
            context->readMiscRegNoEffect(MISCREG_ES)) {
            warn("Remote gdb: Ignoring update to ES.\n");
        }
        if (gdbregs.regs32[GDB64_FS_32] !=
            context->readMiscRegNoEffect(MISCREG_FS)) {
            warn("Remote gdb: Ignoring update to FS.\n");
        }
        if (gdbregs.regs32[GDB64_GS_32] !=
            context->readMiscRegNoEffect(MISCREG_GS)) {
            warn("Remote gdb: Ignoring update to GS.\n");
        }
    } else {
        context->setIntReg(INTREG_RAX, gdbregs.regs32[GDB32_EAX]);
        context->setIntReg(INTREG_RCX, gdbregs.regs32[GDB32_ECX]);
        context->setIntReg(INTREG_RDX, gdbregs.regs32[GDB32_EDX]);
        context->setIntReg(INTREG_RBX, gdbregs.regs32[GDB32_EBX]);
        context->setIntReg(INTREG_RSP, gdbregs.regs32[GDB32_ESP]);
        context->setIntReg(INTREG_RBP, gdbregs.regs32[GDB32_EBP]);
        context->setIntReg(INTREG_RSI, gdbregs.regs32[GDB32_ESI]);
        context->setIntReg(INTREG_RDI, gdbregs.regs32[GDB32_EDI]);
        context->pcState(gdbregs.regs32[GDB32_EIP]);
        context->setMiscReg(MISCREG_RFLAGS, gdbregs.regs32[GDB32_EFLAGS]);
        if (gdbregs.regs32[GDB64_CS_32] !=
            context->readMiscRegNoEffect(MISCREG_CS)) {
            warn("Remote gdb: Ignoring update to CS.\n");
        }
        if (gdbregs.regs32[GDB32_SS] !=
            context->readMiscRegNoEffect(MISCREG_SS)) {
            warn("Remote gdb: Ignoring update to SS.\n");
        }
        if (gdbregs.regs32[GDB32_DS] !=
            context->readMiscRegNoEffect(MISCREG_DS)) {
            warn("Remote gdb: Ignoring update to DS.\n");
        }
        if (gdbregs.regs32[GDB32_ES] !=
            context->readMiscRegNoEffect(MISCREG_ES)) {
            warn("Remote gdb: Ignoring update to ES.\n");
        }
        if (gdbregs.regs32[GDB32_FS] !=
            context->readMiscRegNoEffect(MISCREG_FS)) {
            warn("Remote gdb: Ignoring update to FS.\n");
        }
        if (gdbregs.regs32[GDB32_GS] !=
            context->readMiscRegNoEffect(MISCREG_GS)) {
            warn("Remote gdb: Ignoring update to GS.\n");
        }
    }
}
