/*
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
 * Copyright (c) 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Lawrence Berkeley Laboratories.
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
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
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
 *	@(#)kgdb_stub.c	8.4 (Berkeley) 1/12/94
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
 *	This product includes software developed by the NetBSD
 *	Foundation, Inc. and its contributors.
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

#include <string>
#include <unistd.h>

#include "arch/vtophys.hh"
#include "arch/sparc/remote_gdb.hh"
#include "base/intmath.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/thread_context.hh"
#include "cpu/static_inst.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *c)
    : BaseRemoteGDB(_system, c, NumGDBRegs)
{}

///////////////////////////////////////////////////////////
// RemoteGDB::acc
//
//	Determine if the mapping at va..(va+len) is valid.
//
bool
RemoteGDB::acc(Addr va, size_t len)
{
#if 0
    Addr last_va;

    va = TheISA::TruncPage(va);
    last_va = TheISA::RoundPage(va + len);

    do  {
        if (TheISA::IsK0Seg(va)) {
            if (va < (TheISA::K0SegBase + pmem->size())) {
                DPRINTF(GDBAcc, "acc:   Mapping is valid  K0SEG <= "
                        "%#x < K0SEG + size\n", va);
                return true;
            } else {
                DPRINTF(GDBAcc, "acc:   Mapping invalid %#x > K0SEG + size\n",
                        va);
                return false;
            }
        }

    /**
     * This code says that all accesses to palcode (instruction and data)
     * are valid since there isn't a va->pa mapping because palcode is
     * accessed physically. At some point this should probably be cleaned up
     * but there is no easy way to do it.
     */

        if (AlphaISA::PcPAL(va) || va < 0x10000)
            return true;

        Addr ptbr = context->readMiscReg(AlphaISA::IPR_PALtemp20);
        TheISA::PageTableEntry pte = TheISA::kernel_pte_lookup(context->getPhysPort(), ptbr, va);
        if (!pte.valid()) {
            DPRINTF(GDBAcc, "acc:   %#x pte is invalid\n", va);
            return false;
        }
        va += TheISA::PageBytes;
    } while (va < last_va);

    DPRINTF(GDBAcc, "acc:   %#x mapping is valid\n", va);
#endif
    return true;
}

///////////////////////////////////////////////////////////
// RemoteGDB::getregs
//
//	Translate the kernel debugger register format into
//	the GDB register format.
void
RemoteGDB::getregs()
{
    memset(gdbregs.regs, 0, gdbregs.size);

    gdbregs.regs[RegPc] = context->readPC();
    gdbregs.regs[RegNpc] = context->readNextPC();
    for(int x = RegG0; x <= RegI7; x++)
        gdbregs.regs[x] = context->readIntReg(x - RegG0);
    for(int x = RegF0; x <= RegF31; x++)
        gdbregs.regs[x] = context->readFloatRegBits(x - RegF0);
    gdbregs.regs[RegY] = context->readMiscReg(MISCREG_Y);
    //XXX need to also load up Psr, Wim, Tbr, Fpsr, and Cpsr
}

///////////////////////////////////////////////////////////
// RemoteGDB::setregs
//
//	Translate the GDB register format into the kernel
//	debugger register format.
//
void
RemoteGDB::setregs()
{
    context->setPC(gdbregs.regs[RegPc]);
    context->setNextPC(gdbregs.regs[RegNpc]);
    for(int x = RegG0; x <= RegI7; x++)
        context->setIntReg(x - RegG0, gdbregs.regs[x]);
    for(int x = RegF0; x <= RegF31; x++)
        context->setFloatRegBits(x - RegF0, gdbregs.regs[x]);
    context->setMiscRegWithEffect(MISCREG_Y, gdbregs.regs[RegY]);
    //XXX need to also set Psr, Wim, Tbr, Fpsr, and Cpsr
}

void
RemoteGDB::clearSingleStep()
{
#if 0
    DPRINTF(GDBMisc, "clearSingleStep bt_addr=%#x nt_addr=%#x\n",
            takenBkpt.address, notTakenBkpt.address);

    if (takenBkpt.address != 0)
        clearTempBreakpoint(takenBkpt);

    if (notTakenBkpt.address != 0)
        clearTempBreakpoint(notTakenBkpt);
#endif
}

void
RemoteGDB::setSingleStep()
{
#if 0
    Addr pc = context->readPC();
    Addr npc, bpc;
    bool set_bt = false;

    npc = pc + sizeof(MachInst);

    // User was stopped at pc, e.g. the instruction at pc was not
    // executed.
    MachInst inst = read<MachInst>(pc);
    StaticInstPtr si(inst);
    if (si->hasBranchTarget(pc, context, bpc)) {
        // Don't bother setting a breakpoint on the taken branch if it
        // is the same as the next pc
        if (bpc != npc)
            set_bt = true;
    }

    DPRINTF(GDBMisc, "setSingleStep bt_addr=%#x nt_addr=%#x\n",
            takenBkpt.address, notTakenBkpt.address);

    setTempBreakpoint(notTakenBkpt, npc);

    if (set_bt)
        setTempBreakpoint(takenBkpt, bpc);
#endif
}
