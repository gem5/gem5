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
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Jaidev Patwardhan
 *          Stephen Hines
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

///////////////////////////////////////////////////////////////////////
//
//  ARM TLB
//

#define MODE2MASK(X)			(1 << (X))

TLB::TLB(const Params *p)
    : BaseTLB(p), size(p->size), nlu(0)
{
    table = new ArmISA::PTE[size];
    memset(table, 0, sizeof(ArmISA::PTE[size]));
    smallPages=0;
}

TLB::~TLB()
{
    if (table)
        delete [] table;
}

// look up an entry in the TLB
ArmISA::PTE *
TLB::lookup(Addr vpn, uint8_t asn) const
{
    // assume not found...
    ArmISA::PTE *retval = NULL;
    PageTable::const_iterator i = lookupTable.find(vpn);
    if (i != lookupTable.end()) {
        while (i->first == vpn) {
            int index = i->second;
            ArmISA::PTE *pte = &table[index];

            /* 1KB TLB Lookup code - from ARM ARM Volume III - Rev. 2.50 */
            Addr Mask = pte->Mask;
            Addr InvMask = ~Mask;
            Addr VPN  = pte->VPN;
            //	    warn("Valid: %d - %d\n",pte->V0,pte->V1);
            if(((vpn & InvMask) == (VPN & InvMask)) && (pte->G  || (asn == pte->asid)))
              { // We have a VPN + ASID Match
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

ArmISA::PTE* TLB::getEntry(unsigned Index) const
{
    // Make sure that Index is valid
    assert(Index<size);
    return &table[Index];
}

int TLB::probeEntry(Addr vpn,uint8_t asn) const
{
    // assume not found...
    ArmISA::PTE *retval = NULL;
    int Ind=-1;
    PageTable::const_iterator i = lookupTable.find(vpn);
    if (i != lookupTable.end()) {
        while (i->first == vpn) {
            int index = i->second;
            ArmISA::PTE *pte = &table[index];

            /* 1KB TLB Lookup code - from ARM ARM Volume III - Rev. 2.50 */
            Addr Mask = pte->Mask;
            Addr InvMask = ~Mask;
            Addr VPN  = pte->VPN;
            if(((vpn & InvMask) == (VPN & InvMask)) && (pte->G  || (asn == pte->asid)))
              { // We have a VPN + ASID Match
                retval = pte;
                Ind = index;
                break;
              }

            ++i;
        }
    }
    DPRINTF(Arm,"VPN: %x, asid: %d, Result of TLBP: %d\n",vpn,asn,Ind);
    return Ind;
}
Fault inline
TLB::checkCacheability(RequestPtr &req)
{
  Addr VAddrUncacheable = 0xA0000000;
  // In ARM, cacheability is controlled by certain bits of the virtual address
  // or by the TLB entry
  if((req->getVaddr() & VAddrUncacheable) == VAddrUncacheable) {
    // mark request as uncacheable
    req->setFlags(Request::UNCACHEABLE);
  }
  return NoFault;
}
void TLB::insertAt(ArmISA::PTE &pte, unsigned Index, int _smallPages)
{
  smallPages=_smallPages;
  if(Index > size){
    warn("Attempted to write at index (%d) beyond TLB size (%d)",Index,size);
  } else {
    // Update TLB
    DPRINTF(TLB,"TLB[%d]: %x %x %x %x\n",Index,pte.Mask<<11,((pte.VPN << 11) | pte.asid),((pte.PFN0 <<6) | (pte.C0 << 3) | (pte.D0 << 2) | (pte.V0 <<1) | pte.G),
            ((pte.PFN1 <<6) | (pte.C1 << 3) | (pte.D1 << 2) | (pte.V1 <<1) | pte.G));
    if(table[Index].V0 == true || table[Index].V1 == true){ // Previous entry is valid
      PageTable::iterator i = lookupTable.find(table[Index].VPN);
      lookupTable.erase(i);
    }
    table[Index]=pte;
    // Update fast lookup table
    lookupTable.insert(make_pair(table[Index].VPN, Index));
    //    int TestIndex=probeEntry(pte.VPN,pte.asid);
    //    warn("Inserted at: %d, Found at: %d (%x)\n",Index,TestIndex,pte.Mask);
  }

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

    for (int i = 0; i < size; i++) {
        table[i].unserialize(cp, csprintf("%s.PTE%d", section, i));
        if (table[i].V0 || table[i].V1) {
            lookupTable.insert(make_pair(table[i].VPN, i));
        }
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

ArmISA::PTE &
TLB::index(bool advance)
{
    ArmISA::PTE *pte = &table[nlu];

    if (advance)
        nextnlu();

    return *pte;
}

ArmISA::TLB *
ArmTLBParams::create()
{
    return new ArmISA::TLB(this);
}
