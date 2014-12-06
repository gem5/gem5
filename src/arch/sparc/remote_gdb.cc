/*
 * Copyright 2014 Google, Inc.
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

#include <signal.h>
#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/sparc/remote_gdb.hh"
#include "arch/vtophys.hh"
#include "base/intmath.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBRead.hh"
#include "mem/page_table.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/byteswap.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/system.hh"

using namespace std;
using namespace SparcISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *c)
    : BaseRemoteGDB(_system, c, NumGDBRegs * sizeof(uint64_t))
{}

///////////////////////////////////////////////////////////
// RemoteGDB::acc
//
//      Determine if the mapping at va..(va+len) is valid.
//
bool
RemoteGDB::acc(Addr va, size_t len)
{
    //@Todo In NetBSD, this function checks if all addresses
    // from va to va + len have valid page map entries. Not
    // sure how this will work for other OSes or in general.
    if (FullSystem) {
        if (va)
            return true;
        return false;
    } else {
        TlbEntry entry;
        // Check to make sure the first byte is mapped into the processes
        // address space.
        if (context->getProcessPtr()->pTable->lookup(va, entry))
            return true;
        return false;
    }
}

///////////////////////////////////////////////////////////
// RemoteGDB::getregs
//
//      Translate the kernel debugger register format into
//      the GDB register format.
void
RemoteGDB::getregs()
{
    memset(gdbregs.regs, 0, gdbregs.size);

    PCState pc = context->pcState();
    PSTATE pstate = context->readMiscReg(MISCREG_PSTATE);

    if (pstate.am) {
        gdbregs.regs32[Reg32Pc] = htobe((uint32_t)pc.pc());
        gdbregs.regs32[Reg32Npc] = htobe((uint32_t)pc.npc());
        for (int x = RegG0; x <= RegI0 + 7; x++)
            gdbregs.regs32[x] = htobe((uint32_t)context->readIntReg(x - RegG0));

        gdbregs.regs32[Reg32Y] =
            htobe((uint32_t)context->readIntReg(NumIntArchRegs + 1));
        gdbregs.regs32[Reg32Psr] = htobe((uint32_t)pstate);
        gdbregs.regs32[Reg32Fsr] =
            htobe((uint32_t)context->readMiscReg(MISCREG_FSR));
        gdbregs.regs32[Reg32Csr] =
            htobe((uint32_t)context->readIntReg(NumIntArchRegs + 2));
    } else {
        gdbregs.regs64[RegPc] = htobe(pc.pc());
        gdbregs.regs64[RegNpc] = htobe(pc.npc());
        for (int x = RegG0; x <= RegI0 + 7; x++)
            gdbregs.regs64[x] = htobe(context->readIntReg(x - RegG0));

        gdbregs.regs64[RegFsr] = htobe(context->readMiscReg(MISCREG_FSR));
        gdbregs.regs64[RegFprs] = htobe(context->readMiscReg(MISCREG_FPRS));
        gdbregs.regs64[RegY] = htobe(context->readIntReg(NumIntArchRegs + 1));
        gdbregs.regs64[RegState] = htobe(
            context->readMiscReg(MISCREG_CWP) |
            pstate << 8 |
            context->readMiscReg(MISCREG_ASI) << 24 |
            context->readIntReg(NumIntArchRegs + 2) << 32);
    }

    DPRINTF(GDBRead, "PC=%#x\n", gdbregs.regs64[RegPc]);

    // Floating point registers are left at 0 in netbsd
    // All registers other than the pc, npc and int regs
    // are ignored as well.
}

///////////////////////////////////////////////////////////
// RemoteGDB::setregs
//
//      Translate the GDB register format into the kernel
//      debugger register format.
//
void
RemoteGDB::setregs()
{
    PCState pc;
    pc.pc(gdbregs.regs64[RegPc]);
    pc.npc(gdbregs.regs64[RegNpc]);
    pc.nnpc(pc.npc() + sizeof(MachInst));
    pc.upc(0);
    pc.nupc(1);
    context->pcState(pc);
    for (int x = RegG0; x <= RegI0 + 7; x++)
        context->setIntReg(x - RegG0, gdbregs.regs64[x]);
    // Only the integer registers, pc and npc are set in netbsd
}
