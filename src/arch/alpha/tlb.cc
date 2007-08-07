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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Andrew Schultz
 */

#include <string>
#include <vector>

#include "arch/alpha/pagetable.hh"
#include "arch/alpha/tlb.hh"
#include "arch/alpha/faults.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "config/alpha_tlaser.hh"
#include "cpu/thread_context.hh"
#include "params/AlphaDTB.hh"
#include "params/AlphaITB.hh"

using namespace std;
using namespace EV5;

namespace AlphaISA {
///////////////////////////////////////////////////////////////////////
//
//  Alpha TLB
//
#ifdef DEBUG
bool uncacheBit39 = false;
bool uncacheBit40 = false;
#endif

#define MODE2MASK(X)			(1 << (X))

TLB::TLB(const string &name, int s)
    : SimObject(name), size(s), nlu(0)
{
    table = new PTE[size];
    memset(table, 0, sizeof(PTE[size]));
    flushCache();
}

TLB::~TLB()
{
    if (table)
        delete [] table;
}

// look up an entry in the TLB
PTE *
TLB::lookup(Addr vpn, uint8_t asn) const
{
    // assume not found...
    PTE *retval = NULL;

    if (PTECache[0] && vpn == PTECache[0]->tag &&
        (PTECache[0]->asma || PTECache[0]->asn == asn))
        retval = PTECache[0];
    else if (PTECache[1] && vpn == PTECache[1]->tag &&
        (PTECache[1]->asma || PTECache[1]->asn == asn))
        retval = PTECache[1];
    else if (PTECache[2] && vpn == PTECache[2]->tag &&
        (PTECache[2]->asma || PTECache[2]->asn == asn))
        retval = PTECache[2];
    else {
        PageTable::const_iterator i = lookupTable.find(vpn);
        if (i != lookupTable.end()) {
            while (i->first == vpn) {
                int index = i->second;
                PTE *pte = &table[index];
                assert(pte->valid);
                if (vpn == pte->tag && (pte->asma || pte->asn == asn)) {
                    retval = pte;
                    break;
                }

                ++i;
            }
        }
    }

    DPRINTF(TLB, "lookup %#x, asn %#x -> %s ppn %#x\n", vpn, (int)asn,
            retval ? "hit" : "miss", retval ? retval->ppn : 0);
    return retval;
}


Fault
TLB::checkCacheability(RequestPtr &req)
{
// in Alpha, cacheability is controlled by upper-level bits of the
// physical address

/*
 * We support having the uncacheable bit in either bit 39 or bit 40.
 * The Turbolaser platform (and EV5) support having the bit in 39, but
 * Tsunami (which Linux assumes uses an EV6) generates accesses with
 * the bit in 40.  So we must check for both, but we have debug flags
 * to catch a weird case where both are used, which shouldn't happen.
 */


#if ALPHA_TLASER
    if (req->getPaddr() & PAddrUncachedBit39)
#else
    if (req->getPaddr() & PAddrUncachedBit43)
#endif
    {
        // IPR memory space not implemented
        if (PAddrIprSpace(req->getPaddr())) {
            return new UnimpFault("IPR memory space not implemented!");
        } else {
            // mark request as uncacheable
            req->setFlags(req->getFlags() | UNCACHEABLE);

#if !ALPHA_TLASER
            // Clear bits 42:35 of the physical address (10-2 in Tsunami manual)
            req->setPaddr(req->getPaddr() & PAddrUncachedMask);
#endif
        }
    }
    return NoFault;
}


// insert a new TLB entry
void
TLB::insert(Addr addr, PTE &pte)
{
    flushCache();
    VAddr vaddr = addr;
    if (table[nlu].valid) {
        Addr oldvpn = table[nlu].tag;
        PageTable::iterator i = lookupTable.find(oldvpn);

        if (i == lookupTable.end())
            panic("TLB entry not found in lookupTable");

        int index;
        while ((index = i->second) != nlu) {
            if (table[index].tag != oldvpn)
                panic("TLB entry not found in lookupTable");

            ++i;
        }

        DPRINTF(TLB, "remove @%d: %#x -> %#x\n", nlu, oldvpn, table[nlu].ppn);

        lookupTable.erase(i);
    }

    DPRINTF(TLB, "insert @%d: %#x -> %#x\n", nlu, vaddr.vpn(), pte.ppn);

    table[nlu] = pte;
    table[nlu].tag = vaddr.vpn();
    table[nlu].valid = true;

    lookupTable.insert(make_pair(vaddr.vpn(), nlu));
    nextnlu();
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "flushAll\n");
    memset(table, 0, sizeof(PTE[size]));
    flushCache();
    lookupTable.clear();
    nlu = 0;
}

void
TLB::flushProcesses()
{
    flushCache();
    PageTable::iterator i = lookupTable.begin();
    PageTable::iterator end = lookupTable.end();
    while (i != end) {
        int index = i->second;
        PTE *pte = &table[index];
        assert(pte->valid);

        // we can't increment i after we erase it, so save a copy and
        // increment it to get the next entry now
        PageTable::iterator cur = i;
        ++i;

        if (!pte->asma) {
            DPRINTF(TLB, "flush @%d: %#x -> %#x\n", index, pte->tag, pte->ppn);
            pte->valid = false;
            lookupTable.erase(cur);
        }
    }
}

void
TLB::flushAddr(Addr addr, uint8_t asn)
{
    flushCache();
    VAddr vaddr = addr;

    PageTable::iterator i = lookupTable.find(vaddr.vpn());
    if (i == lookupTable.end())
        return;

    while (i != lookupTable.end() && i->first == vaddr.vpn()) {
        int index = i->second;
        PTE *pte = &table[index];
        assert(pte->valid);

        if (vaddr.vpn() == pte->tag && (pte->asma || pte->asn == asn)) {
            DPRINTF(TLB, "flushaddr @%d: %#x -> %#x\n", index, vaddr.vpn(),
                    pte->ppn);

            // invalidate this entry
            pte->valid = false;

            lookupTable.erase(i++);
        } else {
            ++i;
        }
    }
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
        if (table[i].valid) {
            lookupTable.insert(make_pair(table[i].tag, i));
        }
    }
}


///////////////////////////////////////////////////////////////////////
//
//  Alpha ITB
//
ITB::ITB(const std::string &name, int size)
    : TLB(name, size)
{}


void
ITB::regStats()
{
    hits
        .name(name() + ".hits")
        .desc("ITB hits");
    misses
        .name(name() + ".misses")
        .desc("ITB misses");
    acv
        .name(name() + ".acv")
        .desc("ITB acv");
    accesses
        .name(name() + ".accesses")
        .desc("ITB accesses");

    accesses = hits + misses;
}


Fault
ITB::translate(RequestPtr &req, ThreadContext *tc) const
{
    //If this is a pal pc, then set PHYSICAL
    if(FULL_SYSTEM && PcPAL(req->getPC()))
        req->setFlags(req->getFlags() | PHYSICAL);

    if (PcPAL(req->getPC())) {
        // strip off PAL PC marker (lsb is 1)
        req->setPaddr((req->getVaddr() & ~3) & PAddrImplMask);
        hits++;
        return NoFault;
    }

    if (req->getFlags() & PHYSICAL) {
        req->setPaddr(req->getVaddr());
    } else {
        // verify that this is a good virtual address
        if (!validVirtualAddress(req->getVaddr())) {
            acv++;
            return new ItbAcvFault(req->getVaddr());
        }


        // VA<42:41> == 2, VA<39:13> maps directly to PA<39:13> for EV5
        // VA<47:41> == 0x7e, VA<40:13> maps directly to PA<40:13> for EV6
#if ALPHA_TLASER
        if ((MCSR_SP(tc->readMiscRegNoEffect(IPR_MCSR)) & 2) &&
            VAddrSpaceEV5(req->getVaddr()) == 2)
#else
        if (VAddrSpaceEV6(req->getVaddr()) == 0x7e)
#endif
        {
            // only valid in kernel mode
            if (ICM_CM(tc->readMiscRegNoEffect(IPR_ICM)) !=
                mode_kernel) {
                acv++;
                return new ItbAcvFault(req->getVaddr());
            }

            req->setPaddr(req->getVaddr() & PAddrImplMask);

#if !ALPHA_TLASER
            // sign extend the physical address properly
            if (req->getPaddr() & PAddrUncachedBit40)
                req->setPaddr(req->getPaddr() | ULL(0xf0000000000));
            else
                req->setPaddr(req->getPaddr() & ULL(0xffffffffff));
#endif

        } else {
            // not a physical address: need to look up pte
            int asn = DTB_ASN_ASN(tc->readMiscRegNoEffect(IPR_DTB_ASN));
            PTE *pte = lookup(VAddr(req->getVaddr()).vpn(),
                              asn);

            if (!pte) {
                misses++;
                return new ItbPageFault(req->getVaddr());
            }

            req->setPaddr((pte->ppn << PageShift) +
                          (VAddr(req->getVaddr()).offset()
                           & ~3));

            // check permissions for this access
            if (!(pte->xre &
                  (1 << ICM_CM(tc->readMiscRegNoEffect(IPR_ICM))))) {
                // instruction access fault
                acv++;
                return new ItbAcvFault(req->getVaddr());
            }

            hits++;
        }
    }

    // check that the physical address is ok (catch bad physical addresses)
    if (req->getPaddr() & ~PAddrImplMask)
        return genMachineCheckFault();

    return checkCacheability(req);

}

///////////////////////////////////////////////////////////////////////
//
//  Alpha DTB
//
 DTB::DTB(const std::string &name, int size)
     : TLB(name, size)
{}

void
DTB::regStats()
{
    read_hits
        .name(name() + ".read_hits")
        .desc("DTB read hits")
        ;

    read_misses
        .name(name() + ".read_misses")
        .desc("DTB read misses")
        ;

    read_acv
        .name(name() + ".read_acv")
        .desc("DTB read access violations")
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

    write_acv
        .name(name() + ".write_acv")
        .desc("DTB write access violations")
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

    acv
        .name(name() + ".acv")
        .desc("DTB access violations")
        ;

    accesses
        .name(name() + ".accesses")
        .desc("DTB accesses")
        ;

    hits = read_hits + write_hits;
    misses = read_misses + write_misses;
    acv = read_acv + write_acv;
    accesses = read_accesses + write_accesses;
}

Fault
DTB::translate(RequestPtr &req, ThreadContext *tc, bool write) const
{
    Addr pc = tc->readPC();

    mode_type mode =
        (mode_type)DTB_CM_CM(tc->readMiscRegNoEffect(IPR_DTB_CM));


    /**
     * Check for alignment faults
     */
    if (req->getVaddr() & (req->getSize() - 1)) {
        DPRINTF(TLB, "Alignment Fault on %#x, size = %d", req->getVaddr(),
                req->getSize());
        uint64_t flags = write ? MM_STAT_WR_MASK : 0;
        return new DtbAlignmentFault(req->getVaddr(), req->getFlags(), flags);
    }

    if (PcPAL(pc)) {
        mode = (req->getFlags() & ALTMODE) ?
            (mode_type)ALT_MODE_AM(
                tc->readMiscRegNoEffect(IPR_ALT_MODE))
            : mode_kernel;
    }

    if (req->getFlags() & PHYSICAL) {
        req->setPaddr(req->getVaddr());
    } else {
        // verify that this is a good virtual address
        if (!validVirtualAddress(req->getVaddr())) {
            if (write) { write_acv++; } else { read_acv++; }
            uint64_t flags = (write ? MM_STAT_WR_MASK : 0) |
                MM_STAT_BAD_VA_MASK |
                MM_STAT_ACV_MASK;
            return new DtbPageFault(req->getVaddr(), req->getFlags(), flags);
        }

        // Check for "superpage" mapping
#if ALPHA_TLASER
        if ((MCSR_SP(tc->readMiscRegNoEffect(IPR_MCSR)) & 2) &&
            VAddrSpaceEV5(req->getVaddr()) == 2)
#else
        if (VAddrSpaceEV6(req->getVaddr()) == 0x7e)
#endif
        {

            // only valid in kernel mode
            if (DTB_CM_CM(tc->readMiscRegNoEffect(IPR_DTB_CM)) !=
                mode_kernel) {
                if (write) { write_acv++; } else { read_acv++; }
                uint64_t flags = ((write ? MM_STAT_WR_MASK : 0) |
                                  MM_STAT_ACV_MASK);
                return new DtbAcvFault(req->getVaddr(), req->getFlags(), flags);
            }

            req->setPaddr(req->getVaddr() & PAddrImplMask);

#if !ALPHA_TLASER
            // sign extend the physical address properly
            if (req->getPaddr() & PAddrUncachedBit40)
                req->setPaddr(req->getPaddr() | ULL(0xf0000000000));
            else
                req->setPaddr(req->getPaddr() & ULL(0xffffffffff));
#endif

        } else {
            if (write)
                write_accesses++;
            else
                read_accesses++;

            int asn = DTB_ASN_ASN(tc->readMiscRegNoEffect(IPR_DTB_ASN));

            // not a physical address: need to look up pte
            PTE *pte = lookup(VAddr(req->getVaddr()).vpn(),
                              asn);

            if (!pte) {
                // page fault
                if (write) { write_misses++; } else { read_misses++; }
                uint64_t flags = (write ? MM_STAT_WR_MASK : 0) |
                    MM_STAT_DTB_MISS_MASK;
                return (req->getFlags() & VPTE) ?
                    (Fault)(new PDtbMissFault(req->getVaddr(), req->getFlags(),
                                              flags)) :
                    (Fault)(new NDtbMissFault(req->getVaddr(), req->getFlags(),
                                              flags));
            }

            req->setPaddr((pte->ppn << PageShift) +
                          VAddr(req->getVaddr()).offset());

            if (write) {
                if (!(pte->xwe & MODE2MASK(mode))) {
                    // declare the instruction access fault
                    write_acv++;
                    uint64_t flags = MM_STAT_WR_MASK |
                        MM_STAT_ACV_MASK |
                        (pte->fonw ? MM_STAT_FONW_MASK : 0);
                    return new DtbPageFault(req->getVaddr(), req->getFlags(), flags);
                }
                if (pte->fonw) {
                    write_acv++;
                    uint64_t flags = MM_STAT_WR_MASK |
                        MM_STAT_FONW_MASK;
                    return new DtbPageFault(req->getVaddr(), req->getFlags(), flags);
                }
            } else {
                if (!(pte->xre & MODE2MASK(mode))) {
                    read_acv++;
                    uint64_t flags = MM_STAT_ACV_MASK |
                        (pte->fonr ? MM_STAT_FONR_MASK : 0);
                    return new DtbAcvFault(req->getVaddr(), req->getFlags(), flags);
                }
                if (pte->fonr) {
                    read_acv++;
                    uint64_t flags = MM_STAT_FONR_MASK;
                    return new DtbPageFault(req->getVaddr(), req->getFlags(), flags);
                }
            }
        }

        if (write)
            write_hits++;
        else
            read_hits++;
    }

    // check that the physical address is ok (catch bad physical addresses)
    if (req->getPaddr() & ~PAddrImplMask)
        return genMachineCheckFault();

    return checkCacheability(req);
}

PTE &
TLB::index(bool advance)
{
    PTE *pte = &table[nlu];

    if (advance)
        nextnlu();

    return *pte;
}

/* end namespace AlphaISA */ }

AlphaISA::ITB *
AlphaITBParams::create()
{
    return new AlphaISA::ITB(name, size);
}

AlphaISA::DTB *
AlphaDTBParams::create()
{
    return new AlphaISA::DTB(name, size);
}
