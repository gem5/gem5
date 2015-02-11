/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include "arch/generic/tlb.hh"

#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

Fault
GenericTLB::translateAtomic(RequestPtr req, ThreadContext *tc, Mode)
{
    if (FullSystem)
        panic("Generic translation shouldn't be used in full system mode.\n");

    Process * p = tc->getProcessPtr();

    Fault fault = p->pTable->translate(req);
    if(fault != NoFault)
        return fault;

    return NoFault;
}

void
GenericTLB::translateTiming(RequestPtr req, ThreadContext *tc,
        Translation *translation, Mode mode)
{
    assert(translation);
    translation->finish(translateAtomic(req, tc, mode), req, tc, mode);
}

Fault
GenericTLB::finalizePhysical(RequestPtr req, ThreadContext *tc, Mode mode) const
{
    return NoFault;
}

void
GenericTLB::demapPage(Addr vaddr, uint64_t asn)
{
    warn("Demapping pages in the generic TLB is unnecessary.\n");
}
