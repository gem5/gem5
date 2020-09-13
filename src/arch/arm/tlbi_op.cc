/*
 * Copyright (c) 2018-2019 ARM Limited
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

#include "arch/arm/tlbi_op.hh"

#include "arch/arm/mmu.hh"
#include "cpu/checker/cpu.hh"

namespace ArmISA {

void
TLBIALL::operator()(ThreadContext* tc)
{
    HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool in_host = (hcr.tge == 1 && hcr.e2h == 1);
    getMMUPtr(tc)->flushAllSecurity(secureLookup, targetEL,
        MMU::ALL_TLBS, in_host);

    // If CheckerCPU is connected, need to notify it of a flush
    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushAllSecurity(
            secureLookup, targetEL, MMU::ALL_TLBS, in_host);
    }
}

void
ITLBIALL::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushAllSecurity(secureLookup, targetEL, MMU::I_TLBS);
}

void
DTLBIALL::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushAllSecurity(secureLookup, targetEL, MMU::D_TLBS);
}

void
TLBIASID::operator()(ThreadContext* tc)
{
    HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool in_host = (hcr.tge == 1 && hcr.e2h == 1);
    getMMUPtr(tc)->flushAsid(asid, secureLookup, targetEL,
        MMU::ALL_TLBS, in_host);
    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushAsid(asid, secureLookup, targetEL,
            MMU::ALL_TLBS, in_host);
    }
}

void
ITLBIASID::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushAsid(asid, secureLookup, targetEL, MMU::I_TLBS);
}

void
DTLBIASID::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushAsid(asid, secureLookup, targetEL, MMU::D_TLBS);
}

void
TLBIALLN::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushAllNs(targetEL);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushAllNs(targetEL);
    }
}

void
TLBIMVAA::operator()(ThreadContext* tc)
{
    HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool in_host = (hcr.tge == 1 && hcr.e2h == 1);
    getMMUPtr(tc)->flushMva(addr, secureLookup, targetEL,
        MMU::ALL_TLBS, in_host);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushMva(addr, secureLookup, targetEL,
            MMU::ALL_TLBS, in_host);
    }
}

void
TLBIMVA::operator()(ThreadContext* tc)
{
    HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool in_host = (hcr.tge == 1 && hcr.e2h == 1);
    getMMUPtr(tc)->flushMvaAsid(addr, asid,
        secureLookup, targetEL, MMU::ALL_TLBS, in_host);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushMvaAsid(
            addr, asid, secureLookup, targetEL, MMU::ALL_TLBS, in_host);
    }
}

void
ITLBIMVA::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushMvaAsid(
        addr, asid, secureLookup, targetEL, MMU::I_TLBS);
}

void
DTLBIMVA::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushMvaAsid(
        addr, asid, secureLookup, targetEL, MMU::D_TLBS);
}

void
TLBIIPA::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushIpaVmid(addr,
        secureLookup, targetEL);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushIpaVmid(addr,
            secureLookup, targetEL);
    }
}

} // namespace ArmISA
