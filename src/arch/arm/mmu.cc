/*
 * Copyright (c) 2020 ARM Limited
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

#include "arch/arm/mmu.hh"
#include "arch/arm/table_walker.hh"
#include "arch/arm/tlbi_op.hh"

namespace gem5
{

using namespace ArmISA;

MMU::MMU(const ArmMMUParams &p)
  : BaseMMU(p),
    itbStage2(p.stage2_itb), dtbStage2(p.stage2_dtb),
    iport(p.itb_walker, p.sys->getRequestorId(p.itb_walker)),
    dport(p.dtb_walker, p.sys->getRequestorId(p.dtb_walker)),
    itbWalker(p.itb_walker), dtbWalker(p.dtb_walker),
    itbStage2Walker(p.stage2_itb_walker),
    dtbStage2Walker(p.stage2_dtb_walker)
{
    itbWalker->setPort(&iport);
    dtbWalker->setPort(&dport);
    itbStage2Walker->setPort(&iport);
    dtbStage2Walker->setPort(&dport);
}

void
MMU::init()
{
    itbWalker->setMmu(this);
    dtbWalker->setMmu(this);
    itbStage2Walker->setMmu(this);
    dtbStage2Walker->setMmu(this);

    itbStage2->setTableWalker(itbStage2Walker);
    dtbStage2->setTableWalker(dtbStage2Walker);

    getITBPtr()->setStage2Tlb(itbStage2);
    getITBPtr()->setTableWalker(itbWalker);
    getDTBPtr()->setStage2Tlb(dtbStage2);
    getDTBPtr()->setTableWalker(dtbWalker);
}

TLB *
MMU::getTlb(BaseMMU::Mode mode, bool stage2) const
{
    if (mode == BaseMMU::Execute) {
        if (stage2)
            return itbStage2;
        else
            return getITBPtr();
    } else {
        if (stage2)
            return dtbStage2;
        else
            return getDTBPtr();
    }
}

bool
MMU::translateFunctional(ThreadContext *tc, Addr vaddr, Addr &paddr)
{
    return getDTBPtr()->translateFunctional(tc, vaddr, paddr);
}

Fault
MMU::translateFunctional(const RequestPtr &req, ThreadContext *tc,
    BaseMMU::Mode mode, TLB::ArmTranslationType tran_type)
{
    return translateFunctional(req, tc, mode, tran_type, false);
}

Fault
MMU::translateFunctional(const RequestPtr &req, ThreadContext *tc,
    BaseMMU::Mode mode, TLB::ArmTranslationType tran_type,
    bool stage2)
{
    return getTlb(mode, stage2)->translateFunctional(
        req, tc, mode, tran_type);
}

Fault
MMU::translateAtomic(const RequestPtr &req, ThreadContext *tc,
    BaseMMU::Mode mode, bool stage2)
{
    return getTlb(mode, stage2)->translateAtomic(req, tc, mode);
}

void
MMU::translateTiming(const RequestPtr &req, ThreadContext *tc,
    BaseMMU::Translation *translation, BaseMMU::Mode mode, bool stage2)
{
    return getTlb(mode, stage2)->translateTiming(req, tc, translation, mode);
}

void
MMU::invalidateMiscReg(TLBType type)
{
    if (type & TLBType::I_TLBS) {
        getITBPtr()->invalidateMiscReg();
    }
    if (type & TLBType::D_TLBS) {
        getDTBPtr()->invalidateMiscReg();
    }
}

} // namespace gem5
