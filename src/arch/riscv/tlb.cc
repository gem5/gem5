/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Jaidev Patwardhan
 *          Zhengxing Li
 *          Deyuan Guo
 */

#include "arch/riscv/tlb.hh"

#include <string>
#include <vector>

#include "arch/riscv/faults.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/pra_constants.hh"
#include "arch/riscv/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/RiscvTLB.hh"
#include "debug/TLB.hh"
#include "mem/page_table.hh"
#include "params/RiscvTLB.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

using namespace std;
using namespace RiscvISA;

///////////////////////////////////////////////////////////////////////
//
//  RISC-V TLB
//

TLB::TLB(const Params *p)
    : BaseTLB(p), size(p->size), nlu(0)
{
    table = new PTE[size];
    memset(table, 0, sizeof(PTE[size]));
    smallPages = 0;
}

TLB::~TLB()
{
    if (table)
        delete [] table;
}

// look up an entry in the TLB
RiscvISA::PTE *
TLB::lookup(Addr vpn, uint8_t asn) const
{
    // assume not found...
    PTE *retval = nullptr;
    PageTable::const_iterator i = lookupTable.find(vpn);
    if (i != lookupTable.end()) {
        while (i->first == vpn) {
            int index = i->second;
            PTE *pte = &table[index];

            /* 1KB TLB Lookup code - from MIPS ARM Volume III - Rev. 2.50 */
            Addr Mask = pte->Mask;
            Addr InvMask = ~Mask;
            Addr VPN  = pte->VPN;
            if (((vpn & InvMask) == (VPN & InvMask)) &&
                    (pte->G  || (asn == pte->asid))) {
                // We have a VPN + ASID Match
                retval = pte;
                break;
            }
            ++i;
        }
    }

    DPRINTF(TLB, "lookup %#x, asn %#x -> %s ppn %#x\n", vpn, (int)asn,
            retval ? "hit" : "miss", retval ? retval->PFN1 : 0);
    return retval;
}

RiscvISA::PTE*
TLB::getEntry(unsigned Index) const
{
    // Make sure that Index is valid
    assert(Index<size);
    return &table[Index];
}

int
TLB::probeEntry(Addr vpn, uint8_t asn) const
{
    // assume not found...
    int Ind = -1;
    PageTable::const_iterator i = lookupTable.find(vpn);
    if (i != lookupTable.end()) {
        while (i->first == vpn) {
            int index = i->second;
            PTE *pte = &table[index];

            /* 1KB TLB Lookup code - from MIPS ARM Volume III - Rev. 2.50 */
            Addr Mask = pte->Mask;
            Addr InvMask = ~Mask;
            Addr VPN = pte->VPN;
            if (((vpn & InvMask) == (VPN & InvMask)) &&
                    (pte->G  || (asn == pte->asid))) {
                // We have a VPN + ASID Match
                Ind = index;
                break;
            }
            ++i;
        }
    }
    DPRINTF(RiscvTLB,"VPN: %x, asid: %d, Result of TLBP: %d\n",vpn,asn,Ind);
    return Ind;
}

inline Fault
TLB::checkCacheability(RequestPtr &req)
{
    Addr VAddrUncacheable = 0xA0000000;
    // In MIPS, cacheability is controlled by certain bits of the virtual
    // address or by the TLB entry
    if ((req->getVaddr() & VAddrUncacheable) == VAddrUncacheable) {
        // mark request as uncacheable
        req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
    }
    return NoFault;
}

void
TLB::insertAt(PTE &pte, unsigned Index, int _smallPages)
{
    smallPages = _smallPages;
    if (Index > size) {
        warn("Attempted to write at index (%d) beyond TLB size (%d)",
                Index, size);
    } else {
        // Update TLB
        DPRINTF(TLB, "TLB[%d]: %x %x %x %x\n",
                Index, pte.Mask << 11,
                ((pte.VPN << 11) | pte.asid),
                ((pte.PFN0 << 6) | (pte.C0 << 3) |
                 (pte.D0 << 2) | (pte.V0 <<1) | pte.G),
                ((pte.PFN1 <<6) | (pte.C1 << 3) |
                 (pte.D1 << 2) | (pte.V1 <<1) | pte.G));
        if (table[Index].V0 || table[Index].V1) {
            // Previous entry is valid
            PageTable::iterator i = lookupTable.find(table[Index].VPN);
            lookupTable.erase(i);
        }
        table[Index]=pte;
        // Update fast lookup table
        lookupTable.insert(make_pair(table[Index].VPN, Index));
    }
}

// insert a new TLB entry
void
TLB::insert(Addr addr, PTE &pte)
{
    fatal("TLB Insert not yet implemented\n");
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "flushAll\n");
    memset(table, 0, sizeof(PTE[size]));
    lookupTable.clear();
    nlu = 0;
}

void
TLB::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(size);
    SERIALIZE_SCALAR(nlu);

    for (int i = 0; i < size; i++) {
        ScopedCheckpointSection sec(cp, csprintf("PTE%d", i));
        table[i].serialize(cp);
    }
}

void
TLB::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(size);
    UNSERIALIZE_SCALAR(nlu);

    for (int i = 0; i < size; i++) {
        ScopedCheckpointSection sec(cp, csprintf("PTE%d", i));
        table[i].unserialize(cp);
        if (table[i].V0 || table[i].V1) {
            lookupTable.insert(make_pair(table[i].VPN, i));
        }
    }
}

void
TLB::regStats()
{
    BaseTLB::regStats();

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

    accesses
        .name(name() + ".accesses")
        .desc("DTB accesses")
        ;

    hits = read_hits + write_hits;
    misses = read_misses + write_misses;
    accesses = read_accesses + write_accesses;
}

Fault
TLB::translateInst(RequestPtr req, ThreadContext *tc)
{
    if (FullSystem)
        panic("translateInst not implemented in RISC-V.\n");

    Process * p = tc->getProcessPtr();

    Fault fault = p->pTable->translate(req);
    if (fault != NoFault)
        return fault;

    return NoFault;
}

Fault
TLB::translateData(RequestPtr req, ThreadContext *tc, bool write)
{
    if (FullSystem)
        panic("translateData not implemented in RISC-V.\n");

    // In the O3 CPU model, sometimes a memory access will be speculatively
    // executed along a branch that will end up not being taken where the
    // address is invalid.  In that case, return a fault rather than trying
    // to translate it (which will cause a panic).  Since RISC-V allows
    // unaligned memory accesses, this should only happen if the request's
    // length is long enough to wrap around from the end of the memory to the
    // start.
    assert(req->getSize() > 0);
    if (req->getVaddr() + req->getSize() - 1 < req->getVaddr())
        return make_shared<GenericPageTableFault>(req->getVaddr());

    Process * p = tc->getProcessPtr();

    Fault fault = p->pTable->translate(req);
    if (fault != NoFault)
        return fault;

    return NoFault;
}

Fault
TLB::translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode)
{
    if (mode == Execute)
        return translateInst(req, tc);
    else
        return translateData(req, tc, mode == Write);
}

void
TLB::translateTiming(RequestPtr req, ThreadContext *tc,
        Translation *translation, Mode mode)
{
    assert(translation);
    translation->finish(translateAtomic(req, tc, mode), req, tc, mode);
}

Fault
TLB::finalizePhysical(RequestPtr req, ThreadContext *tc, Mode mode) const
{
    return NoFault;
}


RiscvISA::PTE &
TLB::index(bool advance)
{
    PTE *pte = &table[nlu];

    if (advance)
        nextnlu();

    return *pte;
}

RiscvISA::TLB *
RiscvTLBParams::create()
{
    return new TLB(this);
}
