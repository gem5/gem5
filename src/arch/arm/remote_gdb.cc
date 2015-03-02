/*
 * Copyright 2014 Google Inc.
 * Copyright (c) 2010, 2013 ARM Limited
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

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/arm/decoder.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/registers.hh"
#include "arch/arm/remote_gdb.hh"
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

RemoteGDB::RemoteGDB(System *_system, ThreadContext *tc)
    : BaseRemoteGDB(_system, tc, GDB_REG_BYTES)
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
            if (!virtvalid(context, gen.addr())) {
                DPRINTF(GDBAcc, "acc:   %#x mapping is invalid\n", va);
                return false;
            }
        }

        DPRINTF(GDBAcc, "acc:   %#x mapping is valid\n", va);
        return true;
    } else {
        TlbEntry entry;
        //Check to make sure the first byte is mapped into the processes address
        //space.
        if (context->getProcessPtr()->pTable->lookup(va, entry))
            return true;
        return false;
    }
}

/*
 * Translate the kernel debugger register format into the GDB register
 * format.
 */
void
RemoteGDB::getregs()
{
    DPRINTF(GDBAcc, "getregs in remotegdb \n");

    memset(gdbregs.regs, 0, gdbregs.bytes());

    if (inAArch64(context)) {  // AArch64
        // x0-x30
        for (int i = 0; i < 31; ++i)
            gdbregs.regs64[GDB64_X0 + i] = context->readIntReg(INTREG_X0 + i);
        gdbregs.regs64[GDB64_SPX] = context->readIntReg(INTREG_SPX);
        // pc
        gdbregs.regs64[GDB64_PC] = context->pcState().pc();
        // cpsr
        gdbregs.regs64[GDB64_CPSR] =
            context->readMiscRegNoEffect(MISCREG_CPSR);
        // v0-v31
        for (int i = 0; i < 128; i += 4) {
            int gdboff = GDB64_V0_32 + i;
            gdbregs.regs32[gdboff + 0] = context->readFloatRegBits(i + 2);
            gdbregs.regs32[gdboff + 1] = context->readFloatRegBits(i + 3);
            gdbregs.regs32[gdboff + 2] = context->readFloatRegBits(i + 0);
            gdbregs.regs32[gdboff + 3] = context->readFloatRegBits(i + 1);
        }
    } else {  // AArch32
        // R0-R15 supervisor mode
        gdbregs.regs32[GDB32_R0 + 0] = context->readIntReg(INTREG_R0);
        gdbregs.regs32[GDB32_R0 + 1] = context->readIntReg(INTREG_R1);
        gdbregs.regs32[GDB32_R0 + 2] = context->readIntReg(INTREG_R2);
        gdbregs.regs32[GDB32_R0 + 3] = context->readIntReg(INTREG_R3);
        gdbregs.regs32[GDB32_R0 + 4] = context->readIntReg(INTREG_R4);
        gdbregs.regs32[GDB32_R0 + 5] = context->readIntReg(INTREG_R5);
        gdbregs.regs32[GDB32_R0 + 6] = context->readIntReg(INTREG_R6);
        gdbregs.regs32[GDB32_R0 + 7] = context->readIntReg(INTREG_R7);
        gdbregs.regs32[GDB32_R0 + 8] = context->readIntReg(INTREG_R8);
        gdbregs.regs32[GDB32_R0 + 9] = context->readIntReg(INTREG_R9);
        gdbregs.regs32[GDB32_R0 + 10] = context->readIntReg(INTREG_R10);
        gdbregs.regs32[GDB32_R0 + 11] = context->readIntReg(INTREG_R11);
        gdbregs.regs32[GDB32_R0 + 12] = context->readIntReg(INTREG_R12);
        gdbregs.regs32[GDB32_R0 + 13] = context->readIntReg(INTREG_SP);
        gdbregs.regs32[GDB32_R0 + 14] = context->readIntReg(INTREG_LR);
        gdbregs.regs32[GDB32_R0 + 15] = context->pcState().pc();

        // CPSR
        gdbregs.regs32[GDB32_CPSR] = context->readMiscRegNoEffect(MISCREG_CPSR);

        // vfpv3/neon floating point registers (32 double or 64 float)
        for (int i = 0; i < NumFloatV7ArchRegs; ++i)
            gdbregs.regs32[GDB32_F0 + i] = context->readFloatRegBits(i);

        // FPSCR
        gdbregs.regs32[GDB32_FPSCR] =
            context->readMiscRegNoEffect(MISCREG_FPSCR);
    }
}

/*
 * Translate the GDB register format into the kernel debugger register
 * format.
 */
void
RemoteGDB::setregs()
{

    DPRINTF(GDBAcc, "setregs in remotegdb \n");
    if (inAArch64(context)) {  // AArch64
        // x0-x30
        for (int i = 0; i < 31; ++i)
            context->setIntReg(INTREG_X0 + i, gdbregs.regs64[GDB64_X0 + i]);
        // pc
        context->pcState(gdbregs.regs64[GDB64_PC]);
        // cpsr
        context->setMiscRegNoEffect(MISCREG_CPSR, gdbregs.regs64[GDB64_CPSR]);
        // Update the stack pointer. This should be done after
        // updating CPSR/PSTATE since that might affect how SPX gets
        // mapped.
        context->setIntReg(INTREG_SPX, gdbregs.regs64[GDB64_SPX]);
        // v0-v31
        for (int i = 0; i < 128; i += 4) {
            int gdboff = GDB64_V0_32 + i;
            context->setFloatRegBits(i + 2, gdbregs.regs32[gdboff + 0]);
            context->setFloatRegBits(i + 3, gdbregs.regs32[gdboff + 1]);
            context->setFloatRegBits(i + 0, gdbregs.regs32[gdboff + 2]);
            context->setFloatRegBits(i + 1, gdbregs.regs32[gdboff + 3]);
        }
    } else {  // AArch32
        // R0-R15 supervisor mode
        // arm registers are 32 bits wide, gdb registers are 64 bits wide
        // two arm registers are packed into one gdb register (little endian)
        context->setIntReg(INTREG_R0, gdbregs.regs32[GDB32_R0 + 0]);
        context->setIntReg(INTREG_R1, gdbregs.regs32[GDB32_R0 + 1]);
        context->setIntReg(INTREG_R2, gdbregs.regs32[GDB32_R0 + 2]);
        context->setIntReg(INTREG_R3, gdbregs.regs32[GDB32_R0 + 3]);
        context->setIntReg(INTREG_R4, gdbregs.regs32[GDB32_R0 + 4]);
        context->setIntReg(INTREG_R5, gdbregs.regs32[GDB32_R0 + 5]);
        context->setIntReg(INTREG_R6, gdbregs.regs32[GDB32_R0 + 6]);
        context->setIntReg(INTREG_R7, gdbregs.regs32[GDB32_R0 + 7]);
        context->setIntReg(INTREG_R8, gdbregs.regs32[GDB32_R0 + 8]);
        context->setIntReg(INTREG_R9, gdbregs.regs32[GDB32_R0 + 9]);
        context->setIntReg(INTREG_R10, gdbregs.regs32[GDB32_R0 + 10]);
        context->setIntReg(INTREG_R11, gdbregs.regs32[GDB32_R0 + 11]);
        context->setIntReg(INTREG_R12, gdbregs.regs32[GDB32_R0 + 12]);
        context->setIntReg(INTREG_SP, gdbregs.regs32[GDB32_R0 + 13]);
        context->setIntReg(INTREG_LR, gdbregs.regs32[GDB32_R0 + 14]);
        context->pcState(gdbregs.regs32[GDB32_R0 + 7]);

        //CPSR
        context->setMiscRegNoEffect(MISCREG_CPSR, gdbregs.regs32[GDB32_CPSR]);

        //vfpv3/neon floating point registers (32 double or 64 float)
        for (int i = 0; i < NumFloatV7ArchRegs; ++i)
            context->setFloatRegBits(i, gdbregs.regs32[GDB32_F0 + i]);

        //FPSCR
        context->setMiscReg(MISCREG_FPSCR, gdbregs.regs32[GDB32_FPSCR]);
    }
}

// Write bytes to kernel address space for debugger.
bool
RemoteGDB::write(Addr vaddr, size_t size, const char *data)
{
    return BaseRemoteGDB::write(vaddr, size, data);
}

