/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2010 ARM Limited
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
 *          Deyuan Guo
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

#include "arch/mips/decoder.hh"
#include "arch/mips/remote_gdb.hh"
#include "arch/mips/vtophys.hh"
#include "cpu/thread_state.hh"
#include "debug/GDBAcc.hh"
#include "debug/GDBMisc.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"

using namespace std;
using namespace MipsISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *tc)
    : BaseRemoteGDB(_system, tc, GdbNumRegs * sizeof(uint32_t))
{
}

/*
 * Determine if the mapping at va..(va+len) is valid.
 */
bool
RemoteGDB::acc(Addr va, size_t len)
{
    TlbEntry entry;
    //Check to make sure the first byte is mapped into the processes address
    //space.
    if (FullSystem)
        panic("acc not implemented for MIPS FS!");
    else
        return context->getProcessPtr()->pTable->lookup(va, entry);
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

    // MIPS registers are 32 bits wide, gdb registers are 64 bits wide
    // two MIPS registers are packed into one gdb register (little endian)

    // INTREG: R0~R31
    for (int i = 0; i < GdbIntArchRegs; i++)
        gdbregs.regs32[i] = context->readIntReg(i);
    // SR, LO, HI, BADVADDR, CAUSE, PC
    gdbregs.regs32[GdbIntArchRegs + 0] =
        context->readMiscRegNoEffect(MISCREG_STATUS);
    gdbregs.regs32[GdbIntArchRegs + 1] = context->readIntReg(INTREG_LO);
    gdbregs.regs32[GdbIntArchRegs + 2] = context->readIntReg(INTREG_HI);
    gdbregs.regs32[GdbIntArchRegs + 3] =
        context->readMiscRegNoEffect(MISCREG_BADVADDR);
    gdbregs.regs32[GdbIntArchRegs + 4] =
        context->readMiscRegNoEffect(MISCREG_CAUSE);
    gdbregs.regs32[GdbIntArchRegs + 5] = context->pcState().pc();
    // FLOATREG: F0~F31
    for (int i = 0; i < GdbFloatArchRegs; i++)
        gdbregs.regs32[GdbIntRegs + i] = context->readFloatRegBits(i);
    // FCR, FIR
    gdbregs.regs32[GdbIntRegs + GdbFloatArchRegs + 0] =
        context->readFloatRegBits(FLOATREG_FCCR);
    gdbregs.regs32[GdbIntRegs + GdbFloatArchRegs + 1] =
        context->readFloatRegBits(FLOATREG_FIR);
}

/*
 * Translate the GDB register format into the kernel debugger register
 * format.
 */
void
RemoteGDB::setregs()
{
    DPRINTF(GDBAcc, "setregs in remotegdb \n");

    // INTREG: R0~R31
    for (int i = 1; i < GdbIntArchRegs; i++)
        context->setIntReg(i, gdbregs.regs32[i]);
    // SR, LO, HI, BADVADDR, CAUSE, PC
    context->setMiscRegNoEffect(MISCREG_STATUS,
        gdbregs.regs32[GdbIntArchRegs + 0]);
    context->setIntReg(INTREG_LO, gdbregs.regs32[GdbIntArchRegs + 1]);
    context->setIntReg(INTREG_HI, gdbregs.regs32[GdbIntArchRegs + 2]);
    context->setMiscRegNoEffect(MISCREG_BADVADDR,
        gdbregs.regs32[GdbIntArchRegs + 3]);
    context->setMiscRegNoEffect(MISCREG_CAUSE,
        gdbregs.regs32[GdbIntArchRegs + 4]);
    context->pcState(gdbregs.regs32[GdbIntArchRegs + 5]);
    // FLOATREG: F0~F31
    for (int i = 0; i < GdbFloatArchRegs; i++)
        context->setFloatRegBits(i, gdbregs.regs32[GdbIntRegs + i]);
    // FCR, FIR
    context->setFloatRegBits(FLOATREG_FCCR,
        gdbregs.regs32[GdbIntRegs + GdbFloatArchRegs + 0]);
    context->setFloatRegBits(FLOATREG_FIR,
        gdbregs.regs32[GdbIntRegs + GdbFloatArchRegs + 1]);
}
