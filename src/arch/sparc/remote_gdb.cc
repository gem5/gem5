/*
 * Copyright 2015 LabWare
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

#include "arch/sparc/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <csignal>
#include <string>

#include "base/intmath.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBAcc.hh"
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

RemoteGDB::RemoteGDB(System *_system, ThreadContext *c, int _port)
    : BaseRemoteGDB(_system, c, _port), regCache32(this), regCache64(this)
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
        return va != 0;
    } else {
        // Check to make sure the first byte is mapped into the processes
        // address space.
        return context()->getProcessPtr()->pTable->lookup(va) != nullptr;
    }
}

void
RemoteGDB::SPARCGdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");
    for (int i = 0; i < 32; i++) r.gpr[i] = htobe((uint32_t)context->readIntReg(i));
    PCState pc = context->pcState();
    r.pc = htobe((uint32_t)pc.pc());
    r.npc = htobe((uint32_t)pc.npc());
    r.y = htobe((uint32_t)context->readIntReg(INTREG_Y));
    PSTATE pstate = context->readMiscReg(MISCREG_PSTATE);
    r.psr = htobe((uint32_t)pstate);
    r.fsr = htobe((uint32_t)context->readMiscReg(MISCREG_FSR));
    r.csr = htobe((uint32_t)context->readIntReg(INTREG_CCR));
}

void
RemoteGDB::SPARC64GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");
    for (int i = 0; i < 32; i++) r.gpr[i] = htobe(context->readIntReg(i));
    for (int i = 0; i < 32; i++) r.fpr[i] = 0;
    PCState pc = context->pcState();
    r.pc = htobe(pc.pc());
    r.npc = htobe(pc.npc());
    r.fsr = htobe(context->readMiscReg(MISCREG_FSR));
    r.fprs = htobe(context->readMiscReg(MISCREG_FPRS));
    r.y = htobe(context->readIntReg(INTREG_Y));
    PSTATE pstate = context->readMiscReg(MISCREG_PSTATE);
    r.state = htobe(
        context->readMiscReg(MISCREG_CWP) |
        pstate << 8 |
        context->readMiscReg(MISCREG_ASI) << 24 |
        context->readIntReg(INTREG_CCR) << 32);
}

void
RemoteGDB::SPARCGdbRegCache::setRegs(ThreadContext *context) const
{
    for (int i = 0; i < 32; i++) context->setIntReg(i, r.gpr[i]);
    PCState pc;
    pc.pc(r.pc);
    pc.npc(r.npc);
    pc.nnpc(pc.npc() + sizeof(MachInst));
    pc.upc(0);
    pc.nupc(1);
    context->pcState(pc);
    // Floating point registers are left at 0 in netbsd
    // All registers other than the pc, npc and int regs
    // are ignored as well.
}

void
RemoteGDB::SPARC64GdbRegCache::setRegs(ThreadContext *context) const
{
    for (int i = 0; i < 32; i++) context->setIntReg(i, r.gpr[i]);
    PCState pc;
    pc.pc(r.pc);
    pc.npc(r.npc);
    pc.nnpc(pc.npc() + sizeof(MachInst));
    pc.upc(0);
    pc.nupc(1);
    context->pcState(pc);
    // Floating point registers are left at 0 in netbsd
    // All registers other than the pc, npc and int regs
    // are ignored as well.
}


BaseGdbRegCache*
RemoteGDB::gdbRegs()
{
    PSTATE pstate = context()->readMiscReg(MISCREG_PSTATE);
    if (pstate.am) {
        return &regCache32;
    } else {
        return &regCache64;
    }
}
