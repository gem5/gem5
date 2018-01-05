/*
 * Copyright 2015 LabWare
 * Copyright 2014 Google Inc.
 * Copyright (c) 2010, 2013, 2016 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *          William Wang
 *          Boris Shingarov
 */

/*
 * Copyright (c) 1990, 1993 The Regents of the University of California
 * All rights reserved
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Lawrence Berkeley Laboratories.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *      @(#)kgdb_stub.c 8.4 (Berkeley) 1/12/94
 */

/*-
 * Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the NetBSD
 *      Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * $NetBSD: kgdb_stub.c,v 1.8 2001/07/07 22:58:00 wdk Exp $
 *
 * Taken from NetBSD
 *
 * "Stub" to allow remote cpu to debug over a serial line using gdb.
 */

#include "arch/arm/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/arm/decoder.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/registers.hh"
#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "arch/arm/vtophys.hh"
#include "base/chunk_generator.hh"
#include "base/intmath.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "debug/GDBAcc.hh"
#include "debug/GDBMisc.hh"
#include "mem/page_table.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;
using namespace ArmISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *tc, int _port)
    : BaseRemoteGDB(_system, tc, _port), regCache32(this), regCache64(this)
{
}

/*
 * Determine if the mapping at va..(va+len) is valid.
 */
bool
RemoteGDB::acc(Addr va, size_t len)
{
    if (FullSystem) {
        for (ChunkGenerator gen(va, len, PageBytes); !gen.done(); gen.next()) {
            if (!virtvalid(context(), gen.addr())) {
                DPRINTF(GDBAcc, "acc:   %#x mapping is invalid\n", va);
                return false;
            }
        }

        DPRINTF(GDBAcc, "acc:   %#x mapping is valid\n", va);
        return true;
    } else {
        // Check to make sure the first byte is mapped into the processes
        // address space.
        return context()->getProcessPtr()->pTable->lookup(va) != nullptr;
    }
}

void
RemoteGDB::AArch64GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");

    for (int i = 0; i < 31; ++i)
        r.x[i] = context->readIntReg(INTREG_X0 + i);
    r.spx = context->readIntReg(INTREG_SPX);
    r.pc = context->pcState().pc();
    r.cpsr = context->readMiscRegNoEffect(MISCREG_CPSR);

    for (int i = 0; i < 32*4; i += 4) {
        r.v[i + 0] = context->readFloatRegBits(i + 2);
        r.v[i + 1] = context->readFloatRegBits(i + 3);
        r.v[i + 2] = context->readFloatRegBits(i + 0);
        r.v[i + 3] = context->readFloatRegBits(i + 1);
    }

    for (int i = 0; i < 32; i ++) {
        r.vec[i] = context->readVecReg(RegId(VecRegClass,i));
    }
}

void
RemoteGDB::AArch64GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");

    for (int i = 0; i < 31; ++i)
        context->setIntReg(INTREG_X0 + i, r.x[i]);
    context->pcState(r.pc);
    context->setMiscRegNoEffect(MISCREG_CPSR, r.cpsr);
    // Update the stack pointer. This should be done after
    // updating CPSR/PSTATE since that might affect how SPX gets
    // mapped.
    context->setIntReg(INTREG_SPX, r.spx);

    for (int i = 0; i < 32*4; i += 4) {
        context->setFloatRegBits(i + 2, r.v[i + 0]);
        context->setFloatRegBits(i + 3, r.v[i + 1]);
        context->setFloatRegBits(i + 0, r.v[i + 2]);
        context->setFloatRegBits(i + 1, r.v[i + 3]);
    }

    for (int i = 0; i < 32; i ++) {
        context->setVecReg(RegId(VecRegClass, i), r.vec[i]);
    }
}

void
RemoteGDB::AArch32GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");

    r.gpr[0] = context->readIntReg(INTREG_R0);
    r.gpr[1] = context->readIntReg(INTREG_R1);
    r.gpr[2] = context->readIntReg(INTREG_R2);
    r.gpr[3] = context->readIntReg(INTREG_R3);
    r.gpr[4] = context->readIntReg(INTREG_R4);
    r.gpr[5] = context->readIntReg(INTREG_R5);
    r.gpr[6] = context->readIntReg(INTREG_R6);
    r.gpr[7] = context->readIntReg(INTREG_R7);
    r.gpr[8] = context->readIntReg(INTREG_R8);
    r.gpr[9] = context->readIntReg(INTREG_R9);
    r.gpr[10] = context->readIntReg(INTREG_R10);
    r.gpr[11] = context->readIntReg(INTREG_R11);
    r.gpr[12] = context->readIntReg(INTREG_R12);
    r.gpr[13] = context->readIntReg(INTREG_SP);
    r.gpr[14] = context->readIntReg(INTREG_LR);
    r.gpr[15] = context->pcState().pc();

    // One day somebody will implement transfer of FPRs correctly.
    for (int i=0; i<8*3; i++) r.fpr[i] = 0;

    r.fpscr = context->readMiscRegNoEffect(MISCREG_FPSCR);
    r.cpsr = context->readMiscRegNoEffect(MISCREG_CPSR);
}

void
RemoteGDB::AArch32GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");

    context->setIntReg(INTREG_R0, r.gpr[0]);
    context->setIntReg(INTREG_R1, r.gpr[1]);
    context->setIntReg(INTREG_R2, r.gpr[2]);
    context->setIntReg(INTREG_R3, r.gpr[3]);
    context->setIntReg(INTREG_R4, r.gpr[4]);
    context->setIntReg(INTREG_R5, r.gpr[5]);
    context->setIntReg(INTREG_R6, r.gpr[6]);
    context->setIntReg(INTREG_R7, r.gpr[7]);
    context->setIntReg(INTREG_R8, r.gpr[8]);
    context->setIntReg(INTREG_R9, r.gpr[9]);
    context->setIntReg(INTREG_R10, r.gpr[10]);
    context->setIntReg(INTREG_R11, r.gpr[11]);
    context->setIntReg(INTREG_R12, r.gpr[12]);
    context->setIntReg(INTREG_SP, r.gpr[13]);
    context->setIntReg(INTREG_LR, r.gpr[14]);
    context->pcState(r.gpr[15]);

    // One day somebody will implement transfer of FPRs correctly.

    context->setMiscReg(MISCREG_FPSCR, r.fpscr);
    context->setMiscRegNoEffect(MISCREG_CPSR, r.cpsr);
}

BaseGdbRegCache*
RemoteGDB::gdbRegs()
{
    if (inAArch64(context()))
        return &regCache64;
    else
        return &regCache32;
}
