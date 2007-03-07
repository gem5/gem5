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

#include "config/full_system.hh"
#if FULL_SYSTEM
#include "arch/alpha/vtophys.hh"
#endif

#include "arch/alpha/kgdb.h"
#include "arch/alpha/utility.hh"
#include "arch/alpha/remote_gdb.hh"
#include "base/intmath.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "cpu/static_inst.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *c)
    : BaseRemoteGDB(_system, c, KGDB_NUMREGS)
{
    memset(gdbregs.regs, 0, gdbregs.bytes());
}

///////////////////////////////////////////////////////////
// RemoteGDB::acc
//
//	Determine if the mapping at va..(va+len) is valid.
//
bool
RemoteGDB::acc(Addr va, size_t len)
{
#if !FULL_SYSTEM
    panic("acc function needs to be rewritten for SE mode\n");
#else
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

        Addr ptbr = context->readMiscRegNoEffect(AlphaISA::IPR_PALtemp20);
        TheISA::PageTableEntry pte = TheISA::kernel_pte_lookup(context->getPhysPort(), ptbr, va);
        if (!pte.valid()) {
            DPRINTF(GDBAcc, "acc:   %#x pte is invalid\n", va);
            return false;
        }
        va += TheISA::PageBytes;
    } while (va < last_va);

    DPRINTF(GDBAcc, "acc:   %#x mapping is valid\n", va);
    return true;
#endif
}

///////////////////////////////////////////////////////////
// RemoteGDB::getregs
//
//	Translate the kernel debugger register format into
//	the GDB register format.
void
RemoteGDB::getregs()
{
    memset(gdbregs.regs, 0, gdbregs.bytes());

    gdbregs.regs[KGDB_REG_PC] = context->readPC();

    // @todo: Currently this is very Alpha specific.
    if (AlphaISA::PcPAL(gdbregs.regs[KGDB_REG_PC])) {
        for (int i = 0; i < TheISA::NumIntArchRegs; ++i) {
            gdbregs.regs[i] = context->readIntReg(AlphaISA::reg_redir[i]);
        }
    } else {
        for (int i = 0; i < TheISA::NumIntArchRegs; ++i) {
            gdbregs.regs[i] = context->readIntReg(i);
        }
    }

#ifdef KGDB_FP_REGS
    for (int i = 0; i < TheISA::NumFloatArchRegs; ++i) {
        gdbregs.regs[i + KGDB_REG_F0] = context->readFloatRegBits(i);
    }
#endif
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
    // @todo: Currently this is very Alpha specific.
    if (AlphaISA::PcPAL(gdbregs.regs[KGDB_REG_PC])) {
        for (int i = 0; i < TheISA::NumIntArchRegs; ++i) {
            context->setIntReg(AlphaISA::reg_redir[i], gdbregs.regs[i]);
        }
    } else {
        for (int i = 0; i < TheISA::NumIntArchRegs; ++i) {
            context->setIntReg(i, gdbregs.regs[i]);
        }
    }

#ifdef KGDB_FP_REGS
    for (int i = 0; i < TheISA::NumFloatArchRegs; ++i) {
        context->setFloatRegBits(i, gdbregs.regs[i + KGDB_REG_F0]);
    }
#endif
    context->setPC(gdbregs.regs[KGDB_REG_PC]);
}

void
RemoteGDB::clearSingleStep()
{
    DPRINTF(GDBMisc, "clearSingleStep bt_addr=%#x nt_addr=%#x\n",
            takenBkpt, notTakenBkpt);

    if (takenBkpt != 0)
        clearTempBreakpoint(takenBkpt);

    if (notTakenBkpt != 0)
        clearTempBreakpoint(notTakenBkpt);
}

void
RemoteGDB::setSingleStep()
{
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
            takenBkpt, notTakenBkpt);

    setTempBreakpoint(notTakenBkpt = npc);

    if (set_bt)
        setTempBreakpoint(takenBkpt = bpc);
}

// Write bytes to kernel address space for debugger.
bool
RemoteGDB::write(Addr vaddr, size_t size, const char *data)
{
    if (BaseRemoteGDB::write(vaddr, size, data)) {
#ifdef IMB
        alpha_pal_imb();
#endif
        return true;
    } else {
        return false;
    }
}

