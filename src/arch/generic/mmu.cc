/*
 * Copyright (c) 2011-2012,2016-2017, 2019-2021 Arm Limited
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
 * Copyright (c) 2011 Regents of the University of California
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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

#include "arch/generic/mmu.hh"
#include "arch/generic/tlb.hh"
#include "cpu/thread_context.hh"
#include "sim/system.hh"

namespace gem5
{

void
BaseMMU::init()
{
    auto traverse_hierarchy = [this](BaseTLB *starter) {
        for (BaseTLB *tlb = starter; tlb; tlb = tlb->nextLevel()) {
            switch (tlb->type()) {
              case TypeTLB::instruction:
                if (instruction.find(tlb) == instruction.end())
                    instruction.insert(tlb);
                break;
              case TypeTLB::data:
                if (data.find(tlb) == data.end())
                    data.insert(tlb);
                break;
              case TypeTLB::unified:
                if (unified.find(tlb) == unified.end())
                    unified.insert(tlb);
                break;
              default:
                panic("Invalid TLB type\n");
            }
        }
    };

    traverse_hierarchy(itb);
    traverse_hierarchy(dtb);
}

void
BaseMMU::flushAll()
{
    for (auto tlb : instruction) {
        tlb->flushAll();
    }

    for (auto tlb : data) {
        tlb->flushAll();
    }

    for (auto tlb : unified) {
        tlb->flushAll();
    }
}

void
BaseMMU::reset()
{
    // flush the TLBs by defaults
    flushAll();
}

void
BaseMMU::demapPage(Addr vaddr, uint64_t asn)
{
    itb->demapPage(vaddr, asn);
    dtb->demapPage(vaddr, asn);
}

Fault
BaseMMU::translateAtomic(const RequestPtr &req, ThreadContext *tc,
                         BaseMMU::Mode mode)
{
    return getTlb(mode)->translateAtomic(req, tc, mode);
}

void
BaseMMU::translateTiming(const RequestPtr &req, ThreadContext *tc,
                         BaseMMU::Translation *translation, BaseMMU::Mode mode)
{
    return getTlb(mode)->translateTiming(req, tc, translation, mode);
}

Fault
BaseMMU::translateFunctional(const RequestPtr &req, ThreadContext *tc,
                             BaseMMU::Mode mode)
{
    return getTlb(mode)->translateFunctional(req, tc, mode);
}

Addr
BaseMMU::getValidAddr(Addr vaddr, ThreadContext *tc, Mode mode)
{
    return vaddr;
}

Fault
BaseMMU::finalizePhysical(const RequestPtr &req, ThreadContext *tc,
                          BaseMMU::Mode mode) const
{
    return getTlb(mode)->finalizePhysical(req, tc, mode);
}

BaseMMU::MMUTranslationGen::MMUTranslationGen(Addr page_bytes,
        Addr new_start, Addr new_size, ThreadContext *new_tc,
        BaseMMU *new_mmu, BaseMMU::Mode new_mode, Request::Flags new_flags) :
    TranslationGen(new_start, new_size), tc(new_tc), cid(tc->contextId()),
    mmu(new_mmu), mode(new_mode), flags(new_flags),
    pageBytes(page_bytes)
{}

void
BaseMMU::MMUTranslationGen::translate(Range &range) const
{
    Addr next = roundUp(range.vaddr, pageBytes);
    if (next == range.vaddr)
        next += pageBytes;
    range.size = std::min(range.size, next - range.vaddr);

    auto req = std::make_shared<Request>(
            range.vaddr, range.size, flags, Request::funcRequestorId, 0, cid);

    range.fault = mmu->translateFunctional(req, tc, mode);

    if (range.fault == NoFault) {
        range.paddr = req->getPaddr();
        range.flags = req->getFlags();
    }
}

void
BaseMMU::takeOverFrom(BaseMMU *old_mmu)
{
    Port *old_itb_port = old_mmu->itb->getTableWalkerPort();
    Port *old_dtb_port = old_mmu->dtb->getTableWalkerPort();
    Port *new_itb_port = itb->getTableWalkerPort();
    Port *new_dtb_port = dtb->getTableWalkerPort();

    // Move over any table walker ports if they exist
    if (new_itb_port)
        new_itb_port->takeOverFrom(old_itb_port);
    if (new_dtb_port)
        new_dtb_port->takeOverFrom(old_dtb_port);

    itb->takeOverFrom(old_mmu->itb);
    dtb->takeOverFrom(old_mmu->dtb);
}

} // namespace gem5
