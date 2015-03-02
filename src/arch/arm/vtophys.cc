/*
 * Copyright (c) 2010, 2012-2013 ARM Limited
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

#include "arch/arm/faults.hh"
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

static std::pair<bool, Addr>
try_translate(ThreadContext *tc, Addr addr)
{
    Fault fault;
    // Set up a functional memory Request to pass to the TLB
    // to get it to translate the vaddr to a paddr
    Request req(0, addr, 64, 0x40, -1, 0, 0, 0);
    ArmISA::TLB *tlb;

    // Check the TLBs for a translation
    // It's possible that there is a valid translation in the tlb
    // that is no loger valid in the page table in memory
    // so we need to check here first
    //
    // Calling translateFunctional invokes a table-walk if required
    // so we should always succeed
    tlb = static_cast<ArmISA::TLB*>(tc->getDTBPtr());
    fault = tlb->translateFunctional(&req, tc, BaseTLB::Read, TLB::NormalTran);
    if (fault == NoFault)
        return std::make_pair(true, req.getPaddr());

    tlb = static_cast<ArmISA::TLB*>(tc->getITBPtr());
    fault = tlb->translateFunctional(&req, tc, BaseTLB::Read, TLB::NormalTran);
    if (fault == NoFault)
        return std::make_pair(true, req.getPaddr());

    return std::make_pair(false, 0);
}

Addr
ArmISA::vtophys(ThreadContext *tc, Addr addr)
{
    const std::pair<bool, Addr> translation(try_translate(tc, addr));

    if (translation.first)
        return translation.second;
    else
        panic("Table walkers support functional accesses. We should never get here\n");
}

bool
ArmISA::virtvalid(ThreadContext *tc, Addr vaddr)
{
    const std::pair<bool, Addr> translation(try_translate(tc, vaddr));

    return translation.first;
}


