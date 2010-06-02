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
 * Authors: Ali Saidi
 *          Nathan Binkert
 *          Steve Reinhardt
 */

#include <string>
#include <vector>

#include "arch/arm/faults.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "params/ArmTLB.hh"
#include "sim/process.hh"


using namespace std;
using namespace ArmISA;

TLB::TLB(const Params *p)
    : BaseTLB(p), size(p->size), nlu(0)
{
    table = new ArmISA::PTE[size];
    memset(table, 0, sizeof(ArmISA::PTE[size]));

}

TLB::~TLB()
{
    if (table)
        delete [] table;
}

ArmISA::PTE *
TLB::lookup(Addr vpn, uint8_t asn) const
{
    panic("lookup() not implemented for ARM\n");
}

// insert a new TLB entry
void
TLB::insert(Addr addr, ArmISA::PTE &pte)
{
  fatal("TLB Insert not yet implemented\n");
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "flushAll\n");
    memset(table, 0, sizeof(ArmISA::PTE[size]));
    lookupTable.clear();
    nlu = 0;
}

void
TLB::serialize(ostream &os)
{
    SERIALIZE_SCALAR(size);
    SERIALIZE_SCALAR(nlu);

    for (int i = 0; i < size; i++) {
        nameOut(os, csprintf("%s.PTE%d", name(), i));
        table[i].serialize(os);
    }
}

void
TLB::unserialize(Checkpoint *cp, const string &section)
{
    UNSERIALIZE_SCALAR(size);
    UNSERIALIZE_SCALAR(nlu);

    panic("Need to properly unserialize TLB\n");
    for (int i = 0; i < size; i++) {
        table[i].unserialize(cp, csprintf("%s.PTE%d", section, i));
    }
}

void
TLB::regStats()
{
    read_hits
        .name(name() + ".read_hits")
        .desc("DTB read hits")
        ;

    read_misses
        .name(name() + ".read_misses")
        .desc("DTB read misses")
        ;


    read_accesses
        .name(name() + ".read_accesses")
        .desc("DTB read accesses")
        ;

    write_hits
        .name(name() + ".write_hits")
        .desc("DTB write hits")
        ;

    write_misses
        .name(name() + ".write_misses")
        .desc("DTB write misses")
        ;


    write_accesses
        .name(name() + ".write_accesses")
        .desc("DTB write accesses")
        ;

    hits
        .name(name() + ".hits")
        .desc("DTB hits")
        ;

    misses
        .name(name() + ".misses")
        .desc("DTB misses")
        ;

    invalids
        .name(name() + ".invalids")
        .desc("DTB access violations")
        ;

    accesses
        .name(name() + ".accesses")
        .desc("DTB accesses")
        ;

    hits = read_hits + write_hits;
    misses = read_misses + write_misses;
    accesses = read_accesses + write_accesses;
}

Fault
TLB::translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode)
{
    Addr vaddr = req->getVaddr() & ~PcModeMask;
    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
    uint32_t flags = req->getFlags();

    if (mode != Execute) {
        assert(flags & MustBeOne);

        if (sctlr.a || (flags & AllowUnaligned) == 0) {
            if ((vaddr & flags & AlignmentMask) != 0) {
                return new DataAbort(vaddr, (mode == Write), 0,
                            ArmFault::AlignmentFault);
            }
        }
    }
#if !FULL_SYSTEM
    Process * p = tc->getProcessPtr();

    Addr paddr;
    if (!p->pTable->translate(vaddr, paddr))
        return Fault(new GenericPageTableFault(vaddr));
    req->setPaddr(paddr);

    return NoFault;
#else
    if (!sctlr.m) {
        req->setPaddr(vaddr);
        return NoFault;
    }
    warn_once("MPU translation not implemented\n");
    req->setPaddr(vaddr);
    return NoFault;
    

#endif
}

void
TLB::translateTiming(RequestPtr req, ThreadContext *tc,
        Translation *translation, Mode mode)
{
    assert(translation);
    translation->finish(translateAtomic(req, tc, mode), req, tc, mode);
}

ArmISA::TLB *
ArmTLBParams::create()
{
    return new ArmISA::TLB(this);
}
