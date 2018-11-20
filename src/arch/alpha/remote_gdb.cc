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
 * All rights reserved.
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

#include "arch/alpha/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/alpha/decoder.hh"
#include "arch/alpha/regredir.hh"
#include "arch/alpha/utility.hh"
#include "arch/alpha/vtophys.hh"
#include "base/intmath.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBAcc.hh"
#include "debug/GDBMisc.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;
using namespace AlphaISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *tc, int _port)
    : BaseRemoteGDB(_system, tc, _port)
{
    warn_once("Breakpoints do not work in Alpha PAL mode.\n"
              "      See PCEventQueue::doService() in cpu/pc_event.cc.\n");
}

/*
 * Determine if the mapping at va..(va+len) is valid.
 */
bool
RemoteGDB::acc(Addr va, size_t len)
{
    if (!FullSystem)
        panic("acc function needs to be rewritten for SE mode\n");

    Addr last_va;

    va = TruncPage(va);
    last_va = RoundPage(va + len);

    do  {
        if (IsK0Seg(va)) {
            if (va < (K0SegBase + system()->memSize())) {
                DPRINTF(GDBAcc, "acc:   Mapping is valid  K0SEG <= "
                        "%#x < K0SEG + size\n", va);
                return true;
            } else {
                DPRINTF(GDBAcc, "acc:   Mapping invalid %#x "
                        "> K0SEG + size\n", va);
                return false;
            }
        }

        /**
         * This code says that all accesses to palcode (instruction
         * and data) are valid since there isn't a va->pa mapping
         * because palcode is accessed physically. At some point this
         * should probably be cleaned up but there is no easy way to
         * do it.
         */

        if (PcPAL(va) || va < 0x10000)
            return true;

        Addr ptbr = context()->readMiscRegNoEffect(IPR_PALtemp20);
        PageTableEntry pte =
            kernel_pte_lookup(context()->getPhysProxy(), ptbr, va);
        if (!pte.valid()) {
            DPRINTF(GDBAcc, "acc:   %#x pte is invalid\n", va);
            return false;
        }
        va += PageBytes;
    } while (va < last_va);

    DPRINTF(GDBAcc, "acc:   %#x mapping is valid\n", va);
    return true;
}

void
RemoteGDB::AlphaGdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");

    r.pc = context->pcState().pc();

    if (PcPAL(r.pc)) {
        for (int i = 0; i < 32; ++i)
            r.gpr[i] = context->readIntReg(reg_redir[i]);
    } else {
        for (int i = 0; i < 32; ++i)
            r.gpr[i] = context->readIntReg(i);
    }

    for (int i = 0; i < 32; ++i)
#ifdef KGDB_FP_REGS
       r.fpr[i] = context->readFloatReg(i);
#else
       r.fpr[i] = 0;
#endif
}

void
RemoteGDB::AlphaGdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");

    if (PcPAL(r.pc)) {
        for (int i = 0; i < 32; ++i) {
            context->setIntReg(reg_redir[i], r.gpr[i]);
        }
    } else {
        for (int i = 0; i < 32; ++i) {
            context->setIntReg(i, r.gpr[i]);
        }
    }

#ifdef KGDB_FP_REGS
    for (int i = 0; i < NumFloatArchRegs; ++i) {
        context->setFloatReg(i, gdbregs.regs64[i + KGDB_REG_F0]);
    }
#endif
    context->pcState(r.pc);
}


BaseGdbRegCache*
RemoteGDB::gdbRegs()
{
    return new AlphaGdbRegCache(this);
}

