/*
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
 * Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Ali Saidi
 *          Nathan Binkert
 *          Stephen Hines
 */

#include <string>

#include "arch/arm/table_walker.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/vtophys.hh"
#include "base/chunk_generator.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"

using namespace std;
using namespace ArmISA;

Addr
ArmISA::vtophys(Addr vaddr)
{
    fatal("VTOPHYS: Can't convert vaddr to paddr on ARM without a thread context");
}

Addr
ArmISA::vtophys(ThreadContext *tc, Addr addr)
{
    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
    if (!sctlr.m) {
        // Translation is currently disabled PA == VA
        return addr;
    }
    bool success;
    Addr pa;
    ArmISA::TLB *tlb;

    // Check the TLBs far a translation
    // It's possible that there is a validy translation in the tlb
    // that is no loger valid in the page table in memory
    // so we need to check here first
    tlb = static_cast<ArmISA::TLB*>(tc->getDTBPtr());
    success = tlb->translateFunctional(tc, addr, pa);
    if (success)
        return pa;

    tlb = static_cast<ArmISA::TLB*>(tc->getITBPtr());
    success = tlb->translateFunctional(tc, addr, pa);
    if (success)
        return pa;

    // We've failed everything, so we need to do a
    // hardware tlb walk without messing with any
    // state

    uint32_t N = tc->readMiscReg(MISCREG_TTBCR);
    Addr ttbr;
    if (N == 0 || !mbits(addr, 31, 32-N)) {
        ttbr = tc->readMiscReg(MISCREG_TTBR0);
    } else {
        ttbr = tc->readMiscReg(MISCREG_TTBR1);
        N = 0;
    }

    PortProxy &port = tc->getPhysProxy();
    Addr l1desc_addr = mbits(ttbr, 31, 14-N) | (bits(addr,31-N,20) << 2);

    TableWalker::L1Descriptor l1desc;
    l1desc.data = port.read<uint32_t>(l1desc_addr);
    if (l1desc.type() == TableWalker::L1Descriptor::Ignore ||
            l1desc.type() == TableWalker::L1Descriptor::Reserved) {
        warn("Unable to translate virtual address: %#x\n", addr);
        return -1;
    }
    if (l1desc.type() == TableWalker::L1Descriptor::Section)
        return l1desc.paddr(addr);

    // Didn't find it at the first level, try againt
    Addr l2desc_addr = l1desc.l2Addr() | (bits(addr, 19, 12) << 2);
    TableWalker::L2Descriptor l2desc;
    l2desc.data = port.read<uint32_t>(l2desc_addr);

    if (l2desc.invalid()) {
        warn("Unable to translate virtual address: %#x\n", addr);
        return -1;
    }

    return l2desc.paddr(addr);
}

bool
ArmISA::virtvalid(ThreadContext *tc, Addr vaddr)
{
    if (vtophys(tc, vaddr) != -1)
        return true;
    return false;
}


