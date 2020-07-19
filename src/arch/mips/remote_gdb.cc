/*
 * Copyright 2015-2020 LabWare
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

#include "arch/mips/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/mips/decoder.hh"
#include "blobs/gdb_xml_mips.hh"
#include "cpu/thread_state.hh"
#include "debug/GDBAcc.hh"
#include "debug/GDBMisc.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"

using namespace std;
using namespace MipsISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *tc, int _port)
    : BaseRemoteGDB(_system, tc, _port), regCache(this)
{
}

/*
 * Determine if the mapping at va..(va+len) is valid.
 */
bool
RemoteGDB::acc(Addr va, size_t len)
{
    // Check to make sure the first byte is mapped into the processes address
    // space.
    panic_if(FullSystem, "acc not implemented for MIPS FS!");
    return context()->getProcessPtr()->pTable->lookup(va) != nullptr;
}

void
RemoteGDB::MipsGdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getregs in remotegdb \n");

    for (int i = 0; i < 32; i++) r.gpr[i] = context->readIntReg(i);
    r.sr = context->readMiscRegNoEffect(MISCREG_STATUS);
    r.lo = context->readIntReg(INTREG_LO);
    r.hi = context->readIntReg(INTREG_HI);
    r.badvaddr = context->readMiscRegNoEffect(MISCREG_BADVADDR);
    r.cause = context->readMiscRegNoEffect(MISCREG_CAUSE);
    r.pc = context->pcState().pc();
    for (int i = 0; i < 32; i++) r.fpr[i] = context->readFloatReg(i);
    r.fsr = context->readFloatReg(FLOATREG_FCCR);
    r.fir = context->readFloatReg(FLOATREG_FIR);
}

void
RemoteGDB::MipsGdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setregs in remotegdb \n");

    for (int i = 1; i < 32; i++) context->setIntReg(i, r.gpr[i]);
    context->setMiscRegNoEffect(MISCREG_STATUS, r.sr);
    context->setIntReg(INTREG_LO, r.lo);
    context->setIntReg(INTREG_HI, r.hi);
    context->setMiscRegNoEffect(MISCREG_BADVADDR, r.badvaddr);
    context->setMiscRegNoEffect(MISCREG_CAUSE, r.cause);
    context->pcState(r.pc);
    for (int i = 0; i < 32; i++) context->setFloatReg(i, r.fpr[i]);
    context->setFloatReg(FLOATREG_FCCR, r.fsr);
    context->setFloatReg(FLOATREG_FIR, r.fir);
}

BaseGdbRegCache*
RemoteGDB::gdbRegs()
{
    return &regCache;
}

bool
RemoteGDB::getXferFeaturesRead(const std::string &annex, std::string &output)
{
#define GDB_XML(x, s) \
        { x, std::string(reinterpret_cast<const char *>(Blobs::s), \
        Blobs::s ## _len) }
    static const std::map<std::string, std::string> annexMap {
        GDB_XML("target.xml", gdb_xml_mips),
    };
#undef GDB_XML
    auto it = annexMap.find(annex);
    if (it == annexMap.end())
        return false;
    output = it->second;
    return true;
}
