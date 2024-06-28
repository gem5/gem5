/*
 * Copyright (c) 2021 Huawei International
 * Copyright 2015 LabWare
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2010 ARM Limited
 * Copyright (c) 2020 Barkhausen Institut
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
 * Copyright (c) 2017 The University of Virginia
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

#include "arch/riscv/remote_gdb.hh"

#include <string>

#include "arch/riscv/gdb-xml/gdb_xml_riscv_32bit_cpu.hh"
#include "arch/riscv/gdb-xml/gdb_xml_riscv_32bit_csr.hh"
#include "arch/riscv/gdb-xml/gdb_xml_riscv_32bit_fpu.hh"
#include "arch/riscv/gdb-xml/gdb_xml_riscv_32bit_target.hh"
#include "arch/riscv/gdb-xml/gdb_xml_riscv_64bit_cpu.hh"
#include "arch/riscv/gdb-xml/gdb_xml_riscv_64bit_csr.hh"
#include "arch/riscv/gdb-xml/gdb_xml_riscv_64bit_fpu.hh"
#include "arch/riscv/gdb-xml/gdb_xml_riscv_64bit_target.hh"
#include "arch/riscv/mmu.hh"
#include "arch/riscv/pagetable_walker.hh"
#include "arch/riscv/regs/float.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/tlb.hh"
#include "cpu/thread_state.hh"
#include "debug/GDBAcc.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"

namespace gem5
{

using namespace RiscvISA;

static RiscvType
getRvType(ThreadContext* tc)
{
    auto isa = dynamic_cast<ISA*>(tc->getIsaPtr());
    panic_if(!isa, "Cannot derive rv_type from non-riscv isa");
    return isa->rvType();
}

static PrivilegeModeSet
getPrivilegeModeSet(ThreadContext* tc)
{
    auto isa = dynamic_cast<ISA*>(tc->getIsaPtr());
    panic_if(!isa, "Cannot derive rv_type from non-riscv isa");
    return isa->getPrivilegeModeSet();
}

template <typename xint>
static void
setRegNoEffectWithMask(
        ThreadContext *context, RiscvType type, PrivilegeModeSet pms,
        CSRIndex idx, xint val)
{
    RegVal oldVal, newVal;
    RegVal mask = CSRMasks[type][pms].at(idx);
    oldVal = context->readMiscRegNoEffect(CSRData.at(idx).physIndex);
    newVal = (oldVal & ~mask) | (val & mask);
    context->setMiscRegNoEffect(CSRData.at(idx).physIndex, newVal);
}

template <typename xint>
static void
setRegWithMask(
        ThreadContext *context, RiscvType type, PrivilegeModeSet pms,
        CSRIndex idx, xint val)
{
    RegVal oldVal, newVal;
    RegVal mask = CSRMasks[type][pms].at(idx);
    oldVal = context->readMiscReg(CSRData.at(idx).physIndex);
    newVal = (oldVal & ~mask) | (val & mask);
    context->setMiscReg(CSRData.at(idx).physIndex, newVal);
}

RemoteGDB::RemoteGDB(System *_system, ListenSocketConfig _listen_config)
    : BaseRemoteGDB(_system, _listen_config),
    regCache32(this), regCache64(this)
{
}

bool
RemoteGDB::acc(Addr va, size_t len)
{
    if (FullSystem)
    {
        MMU *mmu = static_cast<MMU *>(context()->getMMUPtr());
        unsigned logBytes;
        Addr paddr = va;

        PrivilegeMode pmode = mmu->getMemPriv(context(), BaseMMU::Read);
        SATP satp = context()->readMiscReg(MISCREG_SATP);
        MISA misa = tc->readMiscRegNoEffect(MISCREG_ISA);
        if (misa.rvs && pmode != PrivilegeMode::PRV_M &&
            satp.mode != AddrXlateMode::BARE) {
            Walker *walker = mmu->getDataWalker();
            Fault fault = walker->startFunctional(
                context(), paddr, logBytes, BaseMMU::Read);
            if (fault != NoFault)
                return false;
        }
        return true;
    }

    return context()->getProcessPtr()->pTable->lookup(va) != nullptr;
}

void
RemoteGDB::Riscv32GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getregs in remotegdb, size %lu\n", size());
    PrivilegeModeSet pms = getPrivilegeModeSet(context);
    auto& RVxCSRMasks = CSRMasks[RV32][pms];

    // General registers
    for (int i = 0; i < int_reg::NumArchRegs; i++) {
        r.gpr[i] = context->getReg(intRegClass[i]);
    }
    r.pc = context->pcState().instAddr();

    // Floating point registers
    for (int i = 0; i < float_reg::NumRegs; i++)
        r.fpu[i] = context->getReg(floatRegClass[i]);
    r.fflags = context->readMiscRegNoEffect(
        CSRData.at(CSR_FFLAGS).physIndex) & RVxCSRMasks.at(CSR_FFLAGS);
    r.frm = context->readMiscRegNoEffect(
        CSRData.at(CSR_FRM).physIndex) & RVxCSRMasks.at(CSR_FRM);
    r.fcsr = context->readMiscReg(
        CSRData.at(CSR_FCSR).physIndex) & RVxCSRMasks.at(CSR_FCSR);

    // CSR registers
    r.cycle = context->readMiscRegNoEffect(
        CSRData.at(CSR_CYCLE).physIndex);
    r.cycleh = context->readMiscRegNoEffect(
        CSRData.at(CSR_CYCLEH).physIndex);
    r.time = context->readMiscRegNoEffect(
        CSRData.at(CSR_TIME).physIndex);
    r.timeh = context->readMiscRegNoEffect(
        CSRData.at(CSR_TIMEH).physIndex);

    // U mode CSR
    r.ustatus = context->readMiscReg(
        CSRData.at(CSR_USTATUS).physIndex) & RVxCSRMasks.at(CSR_USTATUS);
    r.uie = context->readMiscReg(
        CSRData.at(CSR_UIE).physIndex) & RVxCSRMasks.at(CSR_UIE);
    r.utvec = context->readMiscRegNoEffect(
        CSRData.at(CSR_UTVEC).physIndex);
    r.uscratch = context->readMiscRegNoEffect(
        CSRData.at(CSR_USCRATCH).physIndex);
    r.uepc = context->readMiscRegNoEffect(
        CSRData.at(CSR_UEPC).physIndex);
    r.ucause = context->readMiscRegNoEffect(
        CSRData.at(CSR_UCAUSE).physIndex);
    r.utval = context->readMiscRegNoEffect(
        CSRData.at(CSR_UTVAL).physIndex);
    r.uip = context->readMiscReg(
        CSRData.at(CSR_UIP).physIndex) & RVxCSRMasks.at(CSR_UIP);

    // S mode CSR
    r.sstatus = context->readMiscReg(
        CSRData.at(CSR_SSTATUS).physIndex) & RVxCSRMasks.at(CSR_SSTATUS);
    r.sedeleg = context->readMiscRegNoEffect(
        CSRData.at(CSR_SEDELEG).physIndex);
    r.sideleg = context->readMiscRegNoEffect(
        CSRData.at(CSR_SIDELEG).physIndex);
    r.sie = context->readMiscReg(
        CSRData.at(CSR_SIE).physIndex) & RVxCSRMasks.at(CSR_SIE);
    r.stvec = context->readMiscRegNoEffect(
        CSRData.at(CSR_STVEC).physIndex);
    r.scounteren = context->readMiscRegNoEffect(
        CSRData.at(CSR_SCOUNTEREN).physIndex);
    r.sscratch = context->readMiscRegNoEffect(
        CSRData.at(CSR_SSCRATCH).physIndex);
    r.sepc = context->readMiscReg(
        CSRData.at(CSR_SEPC).physIndex);
    r.scause = context->readMiscRegNoEffect(
        CSRData.at(CSR_SCAUSE).physIndex);
    r.stval = context->readMiscRegNoEffect(
        CSRData.at(CSR_STVAL).physIndex);
    r.sip = context->readMiscReg(
        CSRData.at(CSR_SIP).physIndex) & RVxCSRMasks.at(CSR_SIP);
    r.satp = context->readMiscRegNoEffect(
        CSRData.at(CSR_SATP).physIndex);

    // M mode CSR
    r.mvendorid = context->readMiscRegNoEffect(
        CSRData.at(CSR_MVENDORID).physIndex);
    r.marchid = context->readMiscRegNoEffect(
        CSRData.at(CSR_MARCHID).physIndex);
    r.mimpid = context->readMiscRegNoEffect(
        CSRData.at(CSR_MIMPID).physIndex);
    r.mhartid = context->contextId();
    r.mstatus = context->readMiscReg(
        CSRData.at(CSR_MSTATUS).physIndex) & RVxCSRMasks.at(CSR_MSTATUS);
    r.misa = context->readMiscRegNoEffect(
        CSRData.at(CSR_MISA).physIndex) & RVxCSRMasks.at(CSR_MISA);
    r.medeleg = context->readMiscRegNoEffect(
        CSRData.at(CSR_MEDELEG).physIndex);
    r.mideleg = context->readMiscRegNoEffect(
        CSRData.at(CSR_MIDELEG).physIndex);
    r.mie = context->readMiscReg(
        CSRData.at(CSR_MIE).physIndex) & RVxCSRMasks.at(CSR_MIE);
    r.mtvec = context->readMiscRegNoEffect(
        CSRData.at(CSR_MTVEC).physIndex);
    r.mcounteren = context->readMiscRegNoEffect(
        CSRData.at(CSR_MCOUNTEREN).physIndex);
    r.mstatush = context->readMiscReg(
        CSRData.at(CSR_MSTATUSH).physIndex) & RVxCSRMasks.at(CSR_MSTATUSH);
    r.mscratch = context->readMiscRegNoEffect(
        CSRData.at(CSR_MSCRATCH).physIndex);
    r.mepc = context->readMiscReg(
        CSRData.at(CSR_MEPC).physIndex);
    r.mcause = context->readMiscRegNoEffect(
        CSRData.at(CSR_MCAUSE).physIndex);
    r.mtval = context->readMiscRegNoEffect(
        CSRData.at(CSR_MTVAL).physIndex);
    r.mip = context->readMiscReg(
        CSRData.at(CSR_MIP).physIndex) & RVxCSRMasks.at(CSR_MIP);

    // H mode CSR (to be implemented)
}

void
RemoteGDB::Riscv32GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setregs in remotegdb \n");
    PrivilegeModeSet pms = getPrivilegeModeSet(context);
    for (int i = 0; i < int_reg::NumArchRegs; i++)
        context->setReg(intRegClass[i], r.gpr[i]);
    context->pcState(r.pc);

    // Floating point registers
    for (int i = 0; i < float_reg::NumRegs; i++)
        context->setReg(floatRegClass[i], r.fpu[i]);

    setRegNoEffectWithMask(context, RV32, pms, CSR_FFLAGS, r.fflags);
    setRegNoEffectWithMask(context, RV32, pms, CSR_FRM, r.frm);
    setRegWithMask(context, RV32, pms, CSR_FCSR, r.fcsr);

    // TODO: implement CSR counter registers for mcycle(h), minstret(h)

    // U mode CSR
    setRegNoEffectWithMask(context, RV32, pms, CSR_USTATUS, r.ustatus);
    setRegWithMask(context, RV32, pms, CSR_UIE, r.uie);
    setRegWithMask(context, RV32, pms, CSR_UIP, r.uip);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_UTVEC).physIndex, r.utvec);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_USCRATCH).physIndex, r.uscratch);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_UEPC).physIndex, r.uepc);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_UCAUSE).physIndex, r.ucause);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_UTVAL).physIndex, r.utval);

    // S mode CSR
    setRegNoEffectWithMask(context, RV32, pms, CSR_SSTATUS, r.sstatus);
    setRegWithMask(context, RV32, pms, CSR_SIE, r.sie);
    setRegWithMask(context, RV32, pms, CSR_SIP, r.sip);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SEDELEG).physIndex, r.sedeleg);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SIDELEG).physIndex, r.sideleg);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_STVEC).physIndex, r.stvec);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SCOUNTEREN).physIndex, r.scounteren);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SSCRATCH).physIndex, r.sscratch);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SEPC).physIndex, r.sepc);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SCAUSE).physIndex, r.scause);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_STVAL).physIndex, r.stval);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SATP).physIndex, r.satp);

    // M mode CSR
    setRegNoEffectWithMask(context, RV32, pms, CSR_MSTATUS, r.mstatus);
    setRegNoEffectWithMask(context, RV32, pms, CSR_MISA, r.misa);
    setRegWithMask(context, RV32, pms, CSR_MIE, r.mie);
    setRegWithMask(context, RV32, pms, CSR_MIP, r.mip);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MEDELEG).physIndex, r.medeleg);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MIDELEG).physIndex, r.mideleg);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MTVEC).physIndex, r.mtvec);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MCOUNTEREN).physIndex, r.mcounteren);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MSCRATCH).physIndex, r.mscratch);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MEPC).physIndex, r.mepc);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MCAUSE).physIndex, r.mcause);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MTVAL).physIndex, r.mtval);

    // H mode CSR (to be implemented)
}

void
RemoteGDB::Riscv64GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getregs in remotegdb, size %lu\n", size());
    PrivilegeModeSet pms = getPrivilegeModeSet(context);
    auto& RVxCSRMasks = CSRMasks[RV64][pms];

    // General registers
    for (int i = 0; i < int_reg::NumArchRegs; i++) {
        r.gpr[i] = context->getReg(intRegClass[i]);
    }
    r.pc = context->pcState().instAddr();

    // Floating point registers
    for (int i = 0; i < float_reg::NumRegs; i++)
        r.fpu[i] = context->getReg(floatRegClass[i]);
    r.fflags = context->readMiscRegNoEffect(
        CSRData.at(CSR_FFLAGS).physIndex) & RVxCSRMasks.at(CSR_FFLAGS);
    r.frm = context->readMiscRegNoEffect(
        CSRData.at(CSR_FRM).physIndex) & RVxCSRMasks.at(CSR_FRM);
    r.fcsr = context->readMiscReg(
        CSRData.at(CSR_FCSR).physIndex) & RVxCSRMasks.at(CSR_FCSR);

    // CSR registers
    r.cycle = context->readMiscRegNoEffect(
        CSRData.at(CSR_CYCLE).physIndex);
    r.time = context->readMiscRegNoEffect(
        CSRData.at(CSR_TIME).physIndex);

    // U mode CSR
    r.ustatus = context->readMiscReg(
        CSRData.at(CSR_USTATUS).physIndex) & RVxCSRMasks.at(CSR_USTATUS);
    r.uie = context->readMiscReg(
        CSRData.at(CSR_UIE).physIndex) & RVxCSRMasks.at(CSR_UIE);
    r.utvec = context->readMiscRegNoEffect(
        CSRData.at(CSR_UTVEC).physIndex);
    r.uscratch = context->readMiscRegNoEffect(
        CSRData.at(CSR_USCRATCH).physIndex);
    r.uepc = context->readMiscRegNoEffect(
        CSRData.at(CSR_UEPC).physIndex);
    r.ucause = context->readMiscRegNoEffect(
        CSRData.at(CSR_UCAUSE).physIndex);
    r.utval = context->readMiscRegNoEffect(
        CSRData.at(CSR_UTVAL).physIndex);
    r.uip = context->readMiscReg(
        CSRData.at(CSR_UIP).physIndex) & RVxCSRMasks.at(CSR_UIP);

    // S mode CSR
    r.sstatus = context->readMiscReg(
        CSRData.at(CSR_SSTATUS).physIndex) & RVxCSRMasks.at(CSR_SSTATUS);
    r.sedeleg = context->readMiscRegNoEffect(
        CSRData.at(CSR_SEDELEG).physIndex);
    r.sideleg = context->readMiscRegNoEffect(
        CSRData.at(CSR_SIDELEG).physIndex);
    r.sie = context->readMiscReg(
        CSRData.at(CSR_SIE).physIndex) & RVxCSRMasks.at(CSR_SIE);
    r.stvec = context->readMiscRegNoEffect(
        CSRData.at(CSR_STVEC).physIndex);
    r.scounteren = context->readMiscRegNoEffect(
        CSRData.at(CSR_SCOUNTEREN).physIndex);
    r.sscratch = context->readMiscRegNoEffect(
        CSRData.at(CSR_SSCRATCH).physIndex);
    r.sepc = context->readMiscReg(
        CSRData.at(CSR_SEPC).physIndex);
    r.scause = context->readMiscRegNoEffect(
        CSRData.at(CSR_SCAUSE).physIndex);
    r.stval = context->readMiscRegNoEffect(
        CSRData.at(CSR_STVAL).physIndex);
    r.sip = context->readMiscReg(
        CSRData.at(CSR_SIP).physIndex) & RVxCSRMasks.at(CSR_SIP);
    r.satp = context->readMiscRegNoEffect(
        CSRData.at(CSR_SATP).physIndex);

    // M mode CSR
    r.mvendorid = context->readMiscRegNoEffect(
        CSRData.at(CSR_MVENDORID).physIndex);
    r.marchid = context->readMiscRegNoEffect(
        CSRData.at(CSR_MARCHID).physIndex);
    r.mimpid = context->readMiscRegNoEffect(
        CSRData.at(CSR_MIMPID).physIndex);
    r.mhartid = context->contextId();
    r.mstatus = context->readMiscReg(
        CSRData.at(CSR_MSTATUS).physIndex) & RVxCSRMasks.at(CSR_MSTATUS);
    r.misa = context->readMiscRegNoEffect(
        CSRData.at(CSR_MISA).physIndex) & RVxCSRMasks.at(CSR_MISA);
    r.medeleg = context->readMiscRegNoEffect(
        CSRData.at(CSR_MEDELEG).physIndex);
    r.mideleg = context->readMiscRegNoEffect(
        CSRData.at(CSR_MIDELEG).physIndex);
    r.mie = context->readMiscReg(
        CSRData.at(CSR_MIE).physIndex) & RVxCSRMasks.at(CSR_MIE);
    r.mtvec = context->readMiscRegNoEffect(
        CSRData.at(CSR_MTVEC).physIndex);
    r.mcounteren = context->readMiscRegNoEffect(
        CSRData.at(CSR_MCOUNTEREN).physIndex);
    r.mscratch = context->readMiscRegNoEffect(
        CSRData.at(CSR_MSCRATCH).physIndex);
    r.mepc = context->readMiscReg(
        CSRData.at(CSR_MEPC).physIndex);
    r.mcause = context->readMiscRegNoEffect(
        CSRData.at(CSR_MCAUSE).physIndex);
    r.mtval = context->readMiscRegNoEffect(
        CSRData.at(CSR_MTVAL).physIndex);
    r.mip = context->readMiscReg(
        CSRData.at(CSR_MIP).physIndex) & RVxCSRMasks.at(CSR_MIP);

    // H mode CSR (to be implemented)
}

void
RemoteGDB::Riscv64GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setregs in remotegdb \n");
    PrivilegeModeSet pms = getPrivilegeModeSet(context);
    for (int i = 0; i < int_reg::NumArchRegs; i++)
        context->setReg(intRegClass[i], r.gpr[i]);
    context->pcState(r.pc);

    // Floating point registers
    for (int i = 0; i < float_reg::NumRegs; i++)
        context->setReg(floatRegClass[i], r.fpu[i]);

    setRegNoEffectWithMask(context, RV64, pms, CSR_FFLAGS, r.fflags);
    setRegNoEffectWithMask(context, RV64, pms, CSR_FRM, r.frm);
    setRegWithMask(context, RV64, pms, CSR_FCSR, r.fcsr);

    // TODO: implement CSR counter registers for mcycle, minstret

    // U mode CSR
    setRegNoEffectWithMask(context, RV64, pms, CSR_USTATUS, r.ustatus);
    setRegWithMask(context, RV64, pms, CSR_UIE, r.uie);
    setRegWithMask(context, RV64, pms, CSR_UIP, r.uip);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_UTVEC).physIndex, r.utvec);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_USCRATCH).physIndex, r.uscratch);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_UEPC).physIndex, r.uepc);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_UCAUSE).physIndex, r.ucause);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_UTVAL).physIndex, r.utval);

    // S mode CSR
    setRegNoEffectWithMask(
        context, RV64, pms, CSR_SSTATUS, r.sstatus);
    setRegWithMask(context, RV64, pms, CSR_SIE, r.sie);
    setRegWithMask(context, RV64, pms, CSR_SIP, r.sip);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SEDELEG).physIndex, r.sedeleg);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SIDELEG).physIndex, r.sideleg);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_STVEC).physIndex, r.stvec);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SCOUNTEREN).physIndex, r.scounteren);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SSCRATCH).physIndex, r.sscratch);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SEPC).physIndex, r.sepc);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SCAUSE).physIndex, r.scause);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_STVAL).physIndex, r.stval);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_SATP).physIndex, r.satp);

    // M mode CSR
    setRegNoEffectWithMask(
        context, RV64, pms, CSR_MSTATUS, r.mstatus);
    setRegNoEffectWithMask(context, RV64, pms, CSR_MISA, r.misa);
    setRegWithMask(context, RV64, pms, CSR_MIE, r.mie);
    setRegWithMask(context, RV64, pms, CSR_MIP, r.mip);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MEDELEG).physIndex, r.medeleg);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MIDELEG).physIndex, r.mideleg);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MTVEC).physIndex, r.mtvec);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MCOUNTEREN).physIndex, r.mcounteren);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MSCRATCH).physIndex, r.mscratch);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MEPC).physIndex, r.mepc);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MCAUSE).physIndex, r.mcause);
    context->setMiscRegNoEffect(
        CSRData.at(CSR_MTVAL).physIndex, r.mtval);

    // H mode CSR (to be implemented)
}

bool
RemoteGDB::getXferFeaturesRead(const std::string &annex, std::string &output)
{
    /**
     * Blobs e.g. gdb_xml_riscv_target are generated by adding
     * GdbXml(<xml_file_name>, <blob_name>) to src/arch/riscv/Sconscript.
     *
     * Import using #include blobs/<blob_name>.hh
     */
#define GDB_XML(x, s)                                            \
    {                                                            \
        x, std::string(reinterpret_cast<const char *>(Blobs::s), \
                       Blobs::s##_len)                           \
    }
    static const std::map<std::string, std::string> annexMaps[enums::Num_RiscvType] = {
        [RV32] = {GDB_XML("target.xml", gdb_xml_riscv_32bit_target),
                  GDB_XML("riscv-32bit-cpu.xml", gdb_xml_riscv_32bit_cpu),
                  GDB_XML("riscv-32bit-fpu.xml", gdb_xml_riscv_32bit_fpu),
                  GDB_XML("riscv-32bit-csr.xml", gdb_xml_riscv_32bit_csr)},
        [RV64] = {GDB_XML("target.xml", gdb_xml_riscv_64bit_target),
                  GDB_XML("riscv-64bit-cpu.xml", gdb_xml_riscv_64bit_cpu),
                  GDB_XML("riscv-64bit-fpu.xml", gdb_xml_riscv_64bit_fpu),
                  GDB_XML("riscv-64bit-csr.xml", gdb_xml_riscv_64bit_csr)},
    };
    auto& annexMap = annexMaps[getRvType(context())];
    auto it = annexMap.find(annex);
    if (it == annexMap.end())
        return false;
    output = it->second;
    return true;
}

BaseGdbRegCache *
RemoteGDB::gdbRegs()
{
    BaseGdbRegCache* regs[enums::Num_RiscvType] = {
        [RV32] = &regCache32,
        [RV64] = &regCache64,
    };
    return regs[getRvType(context())];
}

} // namespace gem5
