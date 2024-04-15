/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
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

#include "arch/power/tlb.hh"

#include <string>
#include <vector>

#include "arch/power/faults.hh"
#include "arch/power/pagetable.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Power.hh"
#include "debug/TLB.hh"
#include "mem/page_table.hh"
#include "params/PowerTLB.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

namespace gem5
{

using namespace PowerISA;

///////////////////////////////////////////////////////////////////////
//
//  POWER TLB
//

#define MODE2MASK(X) (1 << (X))

TLB::TLB(const Params &p) : BaseTLB(p), size(p.size), nlu(0)
{
    table = new PowerISA::PTE[size];
    memset(table, 0, sizeof(PowerISA::PTE[size]));
    smallPages = 0;
}

TLB::~TLB()
{
    if (table)
        delete[] table;
}

// look up an entry in the TLB
PowerISA::PTE *
TLB::lookup(Addr vpn, uint8_t asn) const
{
    // assume not found...
    PowerISA::PTE *retval = NULL;
    PageTable::const_iterator i = lookupTable.find(vpn);
    if (i != lookupTable.end()) {
        while (i->first == vpn) {
            int index = i->second;
            PowerISA::PTE *pte = &table[index];
            Addr Mask = pte->Mask;
            Addr InvMask = ~Mask;
            Addr VPN = pte->VPN;
            if (((vpn & InvMask) == (VPN & InvMask)) &&
                (pte->G || (asn == pte->asid))) {
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

PowerISA::PTE *
TLB::getEntry(unsigned Index) const
{
    // Make sure that Index is valid
    assert(Index < size);
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
            PowerISA::PTE *pte = &table[index];
            Addr Mask = pte->Mask;
            Addr InvMask = ~Mask;
            Addr VPN = pte->VPN;
            if (((vpn & InvMask) == (VPN & InvMask)) &&
                (pte->G || (asn == pte->asid))) {
                // We have a VPN + ASID Match
                Ind = index;
                break;
            }
            ++i;
        }
    }

    DPRINTF(Power, "VPN: %x, asid: %d, Result of TLBP: %d\n", vpn, asn, Ind);
    return Ind;
}

inline Fault
TLB::checkCacheability(const RequestPtr &req)
{
    Addr VAddrUncacheable = 0xA0000000;
    if ((req->getVaddr() & VAddrUncacheable) == VAddrUncacheable) {
        // mark request as uncacheable
        req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
    }
    return NoFault;
}

void
TLB::insertAt(PowerISA::PTE &pte, unsigned Index, int _smallPages)
{
    smallPages = _smallPages;
    if (Index > size) {
        warn("Attempted to write at index (%d) beyond TLB size (%d)", Index,
             size);
    } else {
        // Update TLB
        if (table[Index].V0 || table[Index].V1) {
            // Previous entry is valid
            PageTable::iterator i = lookupTable.find(table[Index].VPN);
            lookupTable.erase(i);
        }
        table[Index] = pte;

        // Update fast lookup table
        lookupTable.insert(std::make_pair(table[Index].VPN, Index));
    }
}

// insert a new TLB entry
void
TLB::insert(Addr addr, PowerISA::PTE &pte)
{
    fatal("TLB Insert not yet implemented\n");
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "flushAll\n");
    memset(table, 0, sizeof(PowerISA::PTE[size]));
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
        if (table[i].V0 || table[i].V1) {
            lookupTable.insert(std::make_pair(table[i].VPN, i));
        }
    }
}

Fault
TLB::translateInst(const RequestPtr &req, ThreadContext *tc)
{
    Addr vaddr = req->getVaddr();

    // Instruction accesses must be word-aligned
    if (vaddr & 0x3) {
        DPRINTF(TLB, "Alignment Fault on %#x, size = %d\n", vaddr,
                req->getSize());
        return std::make_shared<AlignmentFault>(vaddr);
    }

    return tc->getProcessPtr()->pTable->translate(req);
}

Fault
TLB::translateData(const RequestPtr &req, ThreadContext *tc, bool write)
{
    return tc->getProcessPtr()->pTable->translate(req);
}

Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Mode mode)
{
    panic_if(FullSystem,
             "translateAtomic not yet implemented for full system.");

    if (mode == BaseMMU::Execute)
        return translateInst(req, tc);
    else
        return translateData(req, tc, mode == BaseMMU::Write);
}

Fault
TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc,
                         BaseMMU::Mode mode)
{
    panic_if(FullSystem,
             "translateFunctional not implemented for full system.");
    return tc->getProcessPtr()->pTable->translate(req);
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Translation *translation, BaseMMU::Mode mode)
{
    assert(translation);
    translation->finish(translateAtomic(req, tc, mode), req, tc, mode);
}

Fault
TLB::finalizePhysical(const RequestPtr &req, ThreadContext *tc,
                      BaseMMU::Mode mode) const
{
    return NoFault;
}

PowerISA::PTE &
TLB::index(bool advance)
{
    PowerISA::PTE *pte = &table[nlu];

    if (advance)
        nextnlu();

    return *pte;
}

} // namespace gem5
