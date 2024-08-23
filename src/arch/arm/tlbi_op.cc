/*
 * Copyright (c) 2018-2024 Arm Limited
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

namespace gem5
{

namespace ArmISA {

bool
TLBIOp::match(TlbEntry* te, vmid_t vmid) const
{
    return matchEntry(te, vmid) && (attr != Attr::ExcludeXS || !te->xs);
}

void
TLBIALL::operator()(ThreadContext* tc)
{
    el2Enabled = EL2Enabled(tc);
    currentEL = currEL(tc);

    getMMUPtr(tc)->flush(*this);

    // If CheckerCPU is connected, need to notify it of a flush
    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flush(*this);
    }
}

bool
TLBIALL::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return te->valid && ss == te->ss &&
        (te->vmid == vmid || el2Enabled) &&
        te->checkRegime(targetRegime);
}

void
ITLBIALL::operator()(ThreadContext* tc)
{
    el2Enabled = EL2Enabled(tc);
    getMMUPtr(tc)->iflush(*this);
}

bool
ITLBIALL::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return TLBIALL::matchEntry(te, vmid) && (te->type & TypeTLB::instruction);
}

void
DTLBIALL::operator()(ThreadContext* tc)
{
    el2Enabled = EL2Enabled(tc);
    getMMUPtr(tc)->dflush(*this);
}

bool
DTLBIALL::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return TLBIALL::matchEntry(te, vmid) && (te->type & TypeTLB::data);
}

void
TLBIALLEL::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flush(*this);

    // If CheckerCPU is connected, need to notify it of a flush
    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flush(*this);
    }
}

bool
TLBIALLEL::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return te->valid && ss == te->ss &&
        te->checkRegime(targetRegime);
}

void
TLBIVMALL::operator()(ThreadContext* tc)
{
    el2Enabled = EL2Enabled(tc);

    getMMUPtr(tc)->flush(*this);

    // If CheckerCPU is connected, need to notify it of a flush
    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flush(*this);
    }
}

bool
TLBIVMALL::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return te->valid && ss == te->ss &&
        te->checkRegime(targetRegime) &&
        (te->vmid == vmid || !el2Enabled || !useVMID(targetRegime));
}

void
TLBIASID::operator()(ThreadContext* tc)
{
    el2Enabled = EL2Enabled(tc);

    getMMUPtr(tc)->flushStage1(*this);
    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushStage1(*this);
    }
}

bool
TLBIASID::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return te->valid && te->asid == asid &&
        ss == te->ss &&
        te->checkRegime(targetRegime) &&
        (te->vmid == vmid || !el2Enabled || !useVMID(targetRegime));
}

void
ITLBIASID::operator()(ThreadContext* tc)
{
    el2Enabled = EL2Enabled(tc);
    getMMUPtr(tc)->iflush(*this);
}

bool
ITLBIASID::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return TLBIASID::matchEntry(te, vmid) && (te->type & TypeTLB::instruction);
}

void
DTLBIASID::operator()(ThreadContext* tc)
{
    el2Enabled = EL2Enabled(tc);
    getMMUPtr(tc)->dflush(*this);
}

bool
DTLBIASID::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return TLBIASID::matchEntry(te, vmid) && (te->type & TypeTLB::data);
}

void
TLBIALLN::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flush(*this);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flush(*this);
    }
}

bool
TLBIALLN::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return te->valid && te->ss == SecurityState::NonSecure &&
        te->checkRegime(targetRegime);
}

TlbEntry::Lookup
TLBIMVAA::lookupGen(vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data;
    lookup_data.va = sext<56>(addr);
    lookup_data.ignoreAsn = true;
    lookup_data.vmid = vmid;
    lookup_data.ss = ss;
    lookup_data.functional = true;
    lookup_data.targetRegime = targetRegime;
    lookup_data.mode = BaseMMU::Read;
    return lookup_data;
}

void
TLBIMVAA::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushStage1(*this);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushStage1(*this);
    }
}

bool
TLBIMVAA::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data = lookupGen(vmid);

    return te->match(lookup_data) && (!lastLevel || !te->partial);
}

TlbEntry::Lookup
TLBIMVA::lookupGen(vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data;
    lookup_data.va = sext<56>(addr);
    lookup_data.asn = asid;
    lookup_data.ignoreAsn = false;
    lookup_data.vmid = vmid;
    lookup_data.ss = ss;
    lookup_data.functional = true;
    lookup_data.targetRegime = targetRegime;
    lookup_data.mode = BaseMMU::Read;

    return lookup_data;
}

void
TLBIMVA::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushStage1(*this);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushStage1(*this);
    }
}

bool
TLBIMVA::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data = lookupGen(vmid);

    return te->match(lookup_data) && (!lastLevel || !te->partial);
}

void
ITLBIMVA::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->iflush(*this);
}

bool
ITLBIMVA::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return TLBIMVA::matchEntry(te, vmid) && (te->type & TypeTLB::instruction);
}

void
DTLBIMVA::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->dflush(*this);
}

bool
DTLBIMVA::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    return TLBIMVA::matchEntry(te, vmid) && (te->type & TypeTLB::data);
}

void
TLBIIPA::operator()(ThreadContext* tc)
{
    getMMUPtr(tc)->flushStage2(*this);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getMMUPtr(checker)->flushStage2(*this);
    }
}

TlbEntry::Lookup
TLBIIPA::lookupGen(vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data;
    lookup_data.va = szext<56>(addr);
    lookup_data.ignoreAsn = true;
    lookup_data.vmid = vmid;
    lookup_data.ss = ss;
    lookup_data.functional = true;
    lookup_data.targetRegime = targetRegime;
    lookup_data.mode = BaseMMU::Read;
    return lookup_data;
}

bool
TLBIIPA::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data = lookupGen(vmid);

    return te->match(lookup_data) && (!lastLevel || !te->partial) &&
        ipaSpace == te->ipaSpace;
}

bool
TLBIRMVA::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data = lookupGen(vmid);
    lookup_data.size = rangeSize();

    auto addr_match = te->match(lookup_data) && (!lastLevel || !te->partial);
    if (addr_match) {
        return tgMap[rangeData.tg] == te->tg &&
        (resTLBIttl(rangeData.tg, rangeData.ttl) ||
            rangeData.ttl == te->lookupLevel);
    } else {
        return false;
    }
}

bool
TLBIRMVAA::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data = lookupGen(vmid);
    lookup_data.size = rangeSize();

    auto addr_match = te->match(lookup_data) && (!lastLevel || !te->partial);
    if (addr_match) {
        return tgMap[rangeData.tg] == te->tg &&
        (resTLBIttl(rangeData.tg, rangeData.ttl) ||
            rangeData.ttl == te->lookupLevel);
    } else {
        return false;
    }
}

bool
TLBIRIPA::matchEntry(TlbEntry* te, vmid_t vmid) const
{
    TlbEntry::Lookup lookup_data = lookupGen(vmid);
    lookup_data.size = rangeSize();

    auto addr_match = te->match(lookup_data) && (!lastLevel || !te->partial);
    if (addr_match) {
        return ipaSpace == te->ipaSpace &&
            tgMap[rangeData.tg] == te->tg &&
            (resTLBIttl(rangeData.tg, rangeData.ttl) ||
                rangeData.ttl == te->lookupLevel);
    } else {
        return false;
    }
}

} // namespace ArmISA
} // namespace gem5
