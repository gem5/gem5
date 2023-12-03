/*
 * Copyright 2015 LabWare
 * Copyright 2014 Google Inc.
 * Copyright (c) 2010, 2013, 2016, 2018-2019 ARM Limited
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

#include "arch/arm/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/arm/decoder.hh"
#include "arch/arm/gdb-xml/gdb_xml_aarch64_core.hh"
#include "arch/arm/gdb-xml/gdb_xml_aarch64_fpu.hh"
#include "arch/arm/gdb-xml/gdb_xml_aarch64_target.hh"
#include "arch/arm/gdb-xml/gdb_xml_arm_core.hh"
#include "arch/arm/gdb-xml/gdb_xml_arm_target.hh"
#include "arch/arm/gdb-xml/gdb_xml_arm_vfpv3.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/regs/vec.hh"
#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/mmu.hh"
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

namespace gem5
{

using namespace ArmISA;

namespace
{

// https://sourceware.org/gdb/current/onlinedocs/gdb/ARM-Breakpoint-Kinds.html
enum class ArmBpKind
{
    THUMB = 2,
    THUMB_2 = 3,
    ARM = 4,
};

} // namespace

static bool
tryTranslate(ThreadContext *tc, Addr addr)
{
    // Set up a functional memory Request to pass to the TLB
    // to get it to translate the vaddr to a paddr
    auto req = std::make_shared<Request>(addr, 64, 0x40, -1, 0, 0);

    // Check the TLBs for a translation
    // It's possible that there is a valid translation in the tlb
    // that is no loger valid in the page table in memory
    // so we need to check here first
    //
    // Calling translateFunctional invokes a table-walk if required
    // so we should always succeed
    auto *mmu = tc->getMMUPtr();
    return mmu->translateFunctional(req, tc, BaseMMU::Read) == NoFault ||
           mmu->translateFunctional(req, tc, BaseMMU::Execute) == NoFault;
}

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
    if (FullSystem) {
        for (ChunkGenerator gen(va, len, PageBytes); !gen.done(); gen.next()) {
            if (!tryTranslate(context(), gen.addr())) {
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
        r.x[i] = context->getReg(int_reg::x(i));
    r.spx = context->getReg(int_reg::Spx);
    r.pc = context->pcState().instAddr();
    r.cpsr = context->readMiscRegNoEffect(MISCREG_CPSR);

    size_t base = 0;
    for (int i = 0; i < NumVecV8ArchRegs; i++) {
        ArmISA::VecRegContainer vc;
        context->getReg(vecRegClass[i], &vc);
        auto v = vc.as<VecElem>();
        for (size_t j = 0; j < NumVecElemPerNeonVecReg; j++) {
            r.v[base] = v[j];
            base++;
        }
    }
    r.fpsr = context->readMiscRegNoEffect(MISCREG_FPSR);
    r.fpcr = context->readMiscRegNoEffect(MISCREG_FPCR);
}

void
RemoteGDB::AArch64GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");

    for (int i = 0; i < 31; ++i)
        context->setReg(int_reg::x(i), r.x[i]);
    auto pc_state = context->pcState().as<PCState>();
    pc_state.set(r.pc);
    context->pcState(pc_state);
    context->setMiscRegNoEffect(MISCREG_CPSR, r.cpsr);
    // Update the stack pointer. This should be done after
    // updating CPSR/PSTATE since that might affect how SPX gets
    // mapped.
    context->setReg(int_reg::Spx, r.spx);

    size_t base = 0;
    for (int i = 0; i < NumVecV8ArchRegs; i++) {
        auto *vc = static_cast<ArmISA::VecRegContainer *>(
            context->getWritableReg(vecRegClass[i]));
        auto v = vc->as<VecElem>();
        for (size_t j = 0; j < NumVecElemPerNeonVecReg; j++) {
            v[j] = r.v[base];
            base++;
        }
    }
    context->setMiscRegNoEffect(MISCREG_FPSR, r.fpsr);
    context->setMiscRegNoEffect(MISCREG_FPCR, r.fpcr);
}

void
RemoteGDB::AArch32GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");

    r.gpr[0] = context->getReg(int_reg::R0);
    r.gpr[1] = context->getReg(int_reg::R1);
    r.gpr[2] = context->getReg(int_reg::R2);
    r.gpr[3] = context->getReg(int_reg::R3);
    r.gpr[4] = context->getReg(int_reg::R4);
    r.gpr[5] = context->getReg(int_reg::R5);
    r.gpr[6] = context->getReg(int_reg::R6);
    r.gpr[7] = context->getReg(int_reg::R7);
    r.gpr[8] = context->getReg(int_reg::R8);
    r.gpr[9] = context->getReg(int_reg::R9);
    r.gpr[10] = context->getReg(int_reg::R10);
    r.gpr[11] = context->getReg(int_reg::R11);
    r.gpr[12] = context->getReg(int_reg::R12);
    r.gpr[13] = context->getReg(int_reg::Sp);
    r.gpr[14] = context->getReg(int_reg::Lr);
    r.gpr[15] = context->pcState().instAddr();
    r.cpsr = context->readMiscRegNoEffect(MISCREG_CPSR);

    // One day somebody will implement transfer of FPRs correctly.
    for (int i = 0; i < 32; i++)
        r.fpr[i] = 0;

    r.fpscr = context->readMiscRegNoEffect(MISCREG_FPSCR);
}

void
RemoteGDB::AArch32GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");

    context->setReg(int_reg::R0, r.gpr[0]);
    context->setReg(int_reg::R1, r.gpr[1]);
    context->setReg(int_reg::R2, r.gpr[2]);
    context->setReg(int_reg::R3, r.gpr[3]);
    context->setReg(int_reg::R4, r.gpr[4]);
    context->setReg(int_reg::R5, r.gpr[5]);
    context->setReg(int_reg::R6, r.gpr[6]);
    context->setReg(int_reg::R7, r.gpr[7]);
    context->setReg(int_reg::R8, r.gpr[8]);
    context->setReg(int_reg::R9, r.gpr[9]);
    context->setReg(int_reg::R10, r.gpr[10]);
    context->setReg(int_reg::R11, r.gpr[11]);
    context->setReg(int_reg::R12, r.gpr[12]);
    context->setReg(int_reg::Sp, r.gpr[13]);
    context->setReg(int_reg::Lr, r.gpr[14]);
    PCState pc_state = context->pcState().as<PCState>();
    pc_state.set(r.gpr[15]);
    context->pcState(pc_state);

    // One day somebody will implement transfer of FPRs correctly.

    context->setMiscReg(MISCREG_FPSCR, r.fpscr);
    context->setMiscRegNoEffect(MISCREG_CPSR, r.cpsr);
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
        GDB_XML("target.xml", gdb_xml_arm_target),
        GDB_XML("arm-core.xml", gdb_xml_arm_core),
        GDB_XML("arm-vfpv3.xml", gdb_xml_arm_vfpv3),
    };
    static const std::map<std::string, std::string> annexMap64{
        GDB_XML("target.xml", gdb_xml_aarch64_target),
        GDB_XML("aarch64-core.xml", gdb_xml_aarch64_core),
        GDB_XML("aarch64-fpu.xml", gdb_xml_aarch64_fpu),
    };
#undef GDB_XML
    auto &annexMap = inAArch64(context()) ? annexMap64 : annexMap32;
    auto it = annexMap.find(annex);
    if (it == annexMap.end())
        return false;
    output = it->second;
    return true;
}

BaseGdbRegCache *
RemoteGDB::gdbRegs()
{
    if (inAArch64(context()))
        return &regCache64;
    else
        return &regCache32;
}

bool
RemoteGDB::checkBpKind(size_t kind)
{
    switch (ArmBpKind(kind)) {
    case ArmBpKind::THUMB:
    case ArmBpKind::THUMB_2:
    case ArmBpKind::ARM:
        return true;
    default:
        return false;
    }
}

} // namespace gem5
