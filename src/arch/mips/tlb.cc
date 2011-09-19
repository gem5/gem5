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
 */

#include <string>
#include <vector>

#include "arch/mips/faults.hh"
#include "arch/mips/pagetable.hh"
#include "arch/mips/pra_constants.hh"
#include "arch/mips/tlb.hh"
#include "arch/mips/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/MipsPRA.hh"
#include "debug/TLB.hh"
#include "mem/page_table.hh"
#include "params/MipsTLB.hh"
#include "sim/process.hh"

using namespace std;
using namespace MipsISA;

///////////////////////////////////////////////////////////////////////
//
//  MIPS TLB
//

static inline mode_type
getOperatingMode(MiscReg Stat)
{
    if ((Stat & 0x10000006) != 0 || (Stat & 0x18) ==0) {
        return mode_kernel;
    } else if ((Stat & 0x18) == 0x8) {
        return mode_supervisor;
    } else if ((Stat & 0x18) == 0x10) {
        return mode_user;
    } else {
        return mode_number;
    }
}


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
MipsISA::PTE *
TLB::lookup(Addr vpn, uint8_t asn) const
{
    // assume not found...
    PTE *retval = NULL;
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

MipsISA::PTE*
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
    PTE *retval = NULL;
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
                retval = pte;
                Ind = index;
                break;
            }
            ++i;
        }
    }
    DPRINTF(MipsPRA,"VPN: %x, asid: %d, Result of TLBP: %d\n",vpn,asn,Ind);
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
        req->setFlags(Request::UNCACHEABLE);
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
        if (table[Index].V0 == true || table[Index].V1 == true) {
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
#if !FULL_SYSTEM
    Process * p = tc->getProcessPtr();

    Fault fault = p->pTable->translate(req);
    if (fault != NoFault)
        return fault;

    return NoFault;
#else
    Addr vaddr = req->getVaddr();

    bool misaligned = (req->getSize() - 1) & vaddr;

    if (IsKSeg0(vaddr)) {
        // Address will not be translated through TLB, set response, and go!
        req->setPaddr(KSeg02Phys(vaddr));
        if (getOperatingMode(tc->readMiscReg(MISCREG_STATUS)) != mode_kernel ||
                misaligned) {
            return new AddressErrorFault(vaddr, false);
        }
    } else if(IsKSeg1(vaddr)) {
        // Address will not be translated through TLB, set response, and go!
        req->setPaddr(KSeg02Phys(vaddr));
    } else {
      /* 
       * This is an optimization - smallPages is updated every time a TLB
       * operation is performed. That way, we don't need to look at
       * Config3 _ SP and PageGrain _ ESP every time we do a TLB lookup
       */
      Addr VPN;
      if (smallPages == 1) {
        VPN = (vaddr >> 11);
      } else {
        VPN = ((vaddr >> 11) & 0xFFFFFFFC);
      }
      uint8_t Asid = req->getAsid();
      if (misaligned) {
          // Unaligned address!
          return new AddressErrorFault(vaddr, false);
      }
      PTE *pte = lookup(VPN,Asid);
      if (pte != NULL) {
          // Ok, found something
          /* Check for valid bits */
          int EvenOdd;
          bool Valid;
          if ((((vaddr) >> pte->AddrShiftAmount) & 1) == 0) {
              // Check even bits
              Valid = pte->V0;
              EvenOdd = 0;
          } else {
              // Check odd bits
              Valid = pte->V1;
              EvenOdd = 1;
          }

          if (Valid == false) {
              return new InvalidFault(Asid, vaddr, vpn, false);
          } else {
              // Ok, this is really a match, set paddr
              Addr PAddr;
              if (EvenOdd == 0) {
                PAddr = pte->PFN0;
              } else {
                PAddr = pte->PFN1;
              }
              PAddr >>= (pte->AddrShiftAmount - 12);
              PAddr <<= pte->AddrShiftAmount;
              PAddr |= (vaddr & pte->OffsetMask);
              req->setPaddr(PAddr);
            }
        } else {
            // Didn't find any match, return a TLB Refill Exception
            return new RefillFault(Asid, vaddr, vpn, false);
        }
    }
    return checkCacheability(req);
#endif
}

Fault
TLB::translateData(RequestPtr req, ThreadContext *tc, bool write)
{
#if !FULL_SYSTEM
    //@TODO: This should actually use TLB instead of going directly
    //       to the page table in syscall mode.
    /**
     * Check for alignment faults
     */
    if (req->getVaddr() & (req->getSize() - 1)) {
        DPRINTF(TLB, "Alignment Fault on %#x, size = %d", req->getVaddr(),
                req->getSize());
        return new AddressErrorFault(req->getVaddr(), write);
    }


    Process * p = tc->getProcessPtr();

    Fault fault = p->pTable->translate(req);
    if (fault != NoFault)
        return fault;

    return NoFault;
#else
    Addr vaddr = req->getVaddr();

    bool misaligned = (req->getSize() - 1) & vaddr;

    if (IsKSeg0(vaddr)) {
        // Address will not be translated through TLB, set response, and go!
        req->setPaddr(KSeg02Phys(vaddr));
        if (getOperatingMode(tc->readMiscReg(MISCREG_STATUS)) != mode_kernel ||
                misaligned) {
            return new AddressErrorFault(vaddr, true);
        }
    } else if(IsKSeg1(vaddr)) {
      // Address will not be translated through TLB, set response, and go!
      req->setPaddr(KSeg02Phys(vaddr));
    } else {
        /* 
         * This is an optimization - smallPages is updated every time a TLB
         * operation is performed. That way, we don't need to look at
         * Config3 _ SP and PageGrain _ ESP every time we do a TLB lookup
         */
        Addr VPN = (vaddr >> 11) & 0xFFFFFFFC;
        if (smallPages == 1) {
            VPN = vaddr >> 11;
        }
        uint8_t Asid = req->getAsid();
        PTE *pte = lookup(VPN, Asid);
        if (misaligned) {
            return new AddressErrorFault(vaddr, true);
        }
        if (pte != NULL) {
            // Ok, found something
            /* Check for valid bits */
            int EvenOdd;
            bool Valid;
            bool Dirty;
            if ((((vaddr >> pte->AddrShiftAmount) & 1)) == 0) {
                // Check even bits
                Valid = pte->V0;
                Dirty = pte->D0;
                EvenOdd = 0;
            } else {
                // Check odd bits
                Valid = pte->V1;
                Dirty = pte->D1;
                EvenOdd = 1;
            }

            if (Valid == false) {
                return new InvalidFault(Asid, vaddr, VPN, true);
            } else {
                // Ok, this is really a match, set paddr
                if (!Dirty) {
                    return new TLBModifiedFault(Asid, vaddr, VPN);
                }
                Addr PAddr;
                if (EvenOdd == 0) {
                    PAddr = pte->PFN0;
                } else {
                    PAddr = pte->PFN1;
                }
                PAddr >>= (pte->AddrShiftAmount - 12);
                PAddr <<= pte->AddrShiftAmount;
                PAddr |= (vaddr & pte->OffsetMask);
                req->setPaddr(PAddr);
            }
        } else {
            // Didn't find any match, return a TLB Refill Exception
            return new RefillFault(Asid, vaddr, VPN, true);
        }
    }
    return checkCacheability(req);
#endif
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


MipsISA::PTE &
TLB::index(bool advance)
{
    PTE *pte = &table[nlu];

    if (advance)
        nextnlu();

    return *pte;
}

MipsISA::TLB *
MipsTLBParams::create()
{
    return new TLB(this);
}
