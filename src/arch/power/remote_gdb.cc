/*
 * Copyright 2015 LabWare
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2010 ARM Limited
 * Copyright (c) 2021 IBM Corporation
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

#include "arch/power/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/power/gdb-xml/gdb_xml_power64_core.hh"
#include "arch/power/gdb-xml/gdb_xml_power_core.hh"
#include "arch/power/gdb-xml/gdb_xml_power_fpu.hh"
#include "arch/power/gdb-xml/gdb_xml_powerpc_32.hh"
#include "arch/power/gdb-xml/gdb_xml_powerpc_64.hh"
#include "arch/power/pcstate.hh"
#include "arch/power/regs/misc.hh"
#include "cpu/thread_state.hh"
#include "debug/GDBAcc.hh"
#include "debug/GDBMisc.hh"
#include "mem/page_table.hh"
#include "sim/byteswap.hh"

namespace gem5
{

using namespace PowerISA;

RemoteGDB::RemoteGDB(System *_system, ListenSocketConfig _listen_config)
    : BaseRemoteGDB(_system, _listen_config),
      regCache32(this),
      regCache64(this)
{}

/*
 * Determine if the mapping at va..(va+len) is valid.
 */
bool
RemoteGDB::acc(Addr va, size_t len)
{
    // Check to make sure the first byte is mapped into the processes address
    // space.  At the time of this writing, the acc() check is used when
    // processing the MemR/MemW packets before actually asking the translating
    // port proxy to read/writeBlob.  I (bgs) am not convinced the first byte
    // check is enough.
    panic_if(FullSystem, "acc not implemented for POWER FS!");
    return context()->getProcessPtr()->pTable->lookup(va) != nullptr;
}

void
RemoteGDB::PowerGdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");

    Msr msr = context->getReg(int_reg::Msr);
    ByteOrder order = (msr.le ? ByteOrder::little : ByteOrder::big);

    // Default order on 32-bit PowerPC:
    // R0-R31 (32-bit each), F0-F31 (64-bit IEEE754 double),
    // PC, MSR, CR, LR, CTR, XER, FPSCR (32-bit each)

    for (int i = 0; i < int_reg::NumArchRegs; i++) {
        RegId reg = intRegClass[i];
        r.gpr[i] = htog((uint32_t)context->getReg(reg), order);
    }

    for (int i = 0; i < float_reg::NumArchRegs; i++)
        r.fpr[i] = context->getReg(floatRegClass[i]);

    r.pc = htog((uint32_t)context->pcState().instAddr(), order);
    r.msr = 0; // MSR is privileged, hence not exposed here
    r.cr = htog((uint32_t)context->getReg(int_reg::Cr), order);
    r.lr = htog((uint32_t)context->getReg(int_reg::Lr), order);
    r.ctr = htog((uint32_t)context->getReg(int_reg::Ctr), order);
    r.xer = htog((uint32_t)context->getReg(int_reg::Xer), order);
    r.fpscr = htog((uint32_t)context->getReg(int_reg::Fpscr), order);
}

void
RemoteGDB::PowerGdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");

    Msr msr = context->getReg(int_reg::Msr);
    ByteOrder order = (msr.le ? ByteOrder::little : ByteOrder::big);

    for (int i = 0; i < int_reg::NumArchRegs; i++)
        context->setReg(intRegClass[i], gtoh(r.gpr[i], order));

    for (int i = 0; i < float_reg::NumArchRegs; i++)
        context->setReg(floatRegClass[i], r.fpr[i]);

    auto pc = context->pcState().as<PowerISA::PCState>();
    pc.byteOrder(order);
    pc.set(gtoh(r.pc, order));
    context->pcState(pc);
    // MSR is privileged, hence not modified here
    context->setReg(int_reg::Cr, gtoh(r.cr, order));
    context->setReg(int_reg::Lr, gtoh(r.lr, order));
    context->setReg(int_reg::Ctr, gtoh(r.ctr, order));
    context->setReg(int_reg::Xer, gtoh(r.xer, order));
    context->setReg(int_reg::Fpscr, gtoh(r.fpscr, order));
}

void
RemoteGDB::Power64GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");

    Msr msr = context->getReg(int_reg::Msr);
    ByteOrder order = (msr.le ? ByteOrder::little : ByteOrder::big);

    // Default order on 64-bit PowerPC:
    // GPRR0-GPRR31 (64-bit each), FPR0-FPR31 (64-bit IEEE754 double),
    // CIA, MSR, CR, LR, CTR, XER, FPSCR (only CR, XER, FPSCR are 32-bit
    // each and the rest are 64-bit)

    for (int i = 0; i < int_reg::NumArchRegs; i++)
        r.gpr[i] = htog(context->getReg(intRegClass[i]), order);

    for (int i = 0; i < float_reg::NumArchRegs; i++)
        r.fpr[i] = context->getReg(floatRegClass[i]);

    r.pc = htog(context->pcState().instAddr(), order);
    r.msr = 0; // MSR is privileged, hence not exposed here
    r.cr = htog((uint32_t)context->getReg(int_reg::Cr), order);
    r.lr = htog(context->getReg(int_reg::Lr), order);
    r.ctr = htog(context->getReg(int_reg::Ctr), order);
    r.xer = htog((uint32_t)context->getReg(int_reg::Xer), order);
    r.fpscr = htog((uint32_t)context->getReg(int_reg::Fpscr), order);
}

void
RemoteGDB::Power64GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");

    Msr msr = context->getReg(int_reg::Msr);
    ByteOrder order = (msr.le ? ByteOrder::little : ByteOrder::big);

    for (int i = 0; i < int_reg::NumArchRegs; i++)
        context->setReg(intRegClass[i], gtoh(r.gpr[i], order));

    for (int i = 0; i < float_reg::NumArchRegs; i++)
        context->setReg(floatRegClass[i], r.fpr[i]);

    auto pc = context->pcState().as<PowerISA::PCState>();
    pc.byteOrder(order);
    pc.set(gtoh(r.pc, order));
    context->pcState(pc);
    // MSR is privileged, hence not modified here
    context->setReg(int_reg::Cr, gtoh(r.cr, order));
    context->setReg(int_reg::Lr, gtoh(r.lr, order));
    context->setReg(int_reg::Ctr, gtoh(r.ctr, order));
    context->setReg(int_reg::Xer, gtoh(r.xer, order));
    context->setReg(int_reg::Fpscr, gtoh(r.fpscr, order));
}

BaseGdbRegCache *
RemoteGDB::gdbRegs()
{
    Msr msr = context()->getReg(int_reg::Msr);
    if (msr.sf)
        return &regCache64;
    else
        return &regCache32;
}

bool
RemoteGDB::getXferFeaturesRead(const std::string &annex, std::string &output)
{
#define GDB_XML(x, s)                                                         \
    {                                                                         \
        x, std::string(reinterpret_cast<const char *>(Blobs::s),              \
                       Blobs::s##_len)                                        \
    }
    static const std::map<std::string, std::string> annexMap32{
        GDB_XML("target.xml", gdb_xml_powerpc_32),
        GDB_XML("power-core.xml", gdb_xml_power_core),
        GDB_XML("power-fpu.xml", gdb_xml_power_fpu)
    };
    static const std::map<std::string, std::string> annexMap64{
        GDB_XML("target.xml", gdb_xml_powerpc_64),
        GDB_XML("power64-core.xml", gdb_xml_power64_core),
        GDB_XML("power-fpu.xml", gdb_xml_power_fpu)
    };
#undef GDB_XML

    Msr msr = context()->getReg(int_reg::Msr);
    auto &annexMap = msr.sf ? annexMap64 : annexMap32;
    auto it = annexMap.find(annex);
    if (it == annexMap.end())
        return false;
    output = it->second;
    return true;
}

} // namespace gem5
