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

#include "arch/alpha/tlb.hh"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "arch/alpha/faults.hh"
#include "arch/alpha/pagetable.hh"
#include "arch/generic/debugfaults.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/TLB.hh"
#include "sim/full_system.hh"

using namespace std;

namespace AlphaISA {

///////////////////////////////////////////////////////////////////////
//
//  Alpha TLB
//

#ifdef DEBUG
bool uncacheBit39 = false;
bool uncacheBit40 = false;
#endif

#define MODE2MASK(X) (1 << (X))

TLB::TLB(const Params *p)
    : BaseTLB(p), table(p->size), nlu(0)
{
    flushCache();
}

TLB::~TLB()
{
}

void
TLB::regStats()
{
    BaseTLB::regStats();

    fetch_hits
        .name(name() + ".fetch_hits")
        .desc("ITB hits");
    fetch_misses
        .name(name() + ".fetch_misses")
        .desc("ITB misses");
    fetch_acv
        .name(name() + ".fetch_acv")
        .desc("ITB acv");
    fetch_accesses
        .name(name() + ".fetch_accesses")
        .desc("ITB accesses");

    fetch_accesses = fetch_hits + fetch_misses;

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

    data_hits
        .name(name() + ".data_hits")
        .desc("DTB hits")
        ;

    data_misses
        .name(name() + ".data_misses")
        .desc("DTB misses")
        ;

    data_acv
        .name(name() + ".data_acv")
        .desc("DTB access violations")
        ;

    data_accesses
        .name(name() + ".data_accesses")
        .desc("DTB accesses")
        ;

    data_hits = read_hits + write_hits;
    data_misses = read_misses + write_misses;
    data_acv = read_acv + write_acv;
    data_accesses = read_accesses + write_accesses;
}

// look up an entry in the TLB
TlbEntry *
TLB::lookup(Addr vpn, uint8_t asn)
{
    // assume not found...
    TlbEntry *retval = NULL;

    if (EntryCache[0]) {
        if (vpn == EntryCache[0]->tag &&
            (EntryCache[0]->asma || EntryCache[0]->asn == asn))
            retval = EntryCache[0];
        else if (EntryCache[1]) {
            if (vpn == EntryCache[1]->tag &&
                (EntryCache[1]->asma || EntryCache[1]->asn == asn))
                retval = EntryCache[1];
            else if (EntryCache[2] && vpn == EntryCache[2]->tag &&
                     (EntryCache[2]->asma || EntryCache[2]->asn == asn))
                retval = EntryCache[2];
        }
    }

    if (retval == NULL) {
        PageTable::const_iterator i = lookupTable.find(vpn);
        if (i != lookupTable.end()) {
            while (i->first == vpn) {
                int index = i->second;
                TlbEntry *entry = &table[index];
                assert(entry->valid);
                if (vpn == entry->tag && (entry->asma || entry->asn == asn)) {
                    retval = updateCache(entry);
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
TLB::checkCacheability(RequestPtr &req, bool itb)
{
    // in Alpha, cacheability is controlled by upper-level bits of the
    // physical address

    /*
     * We support having the uncacheable bit in either bit 39 or bit
     * 40.  The Turbolaser platform (and EV5) support having the bit
     * in 39, but Tsunami (which Linux assumes uses an EV6) generates
     * accesses with the bit in 40.  So we must check for both, but we
     * have debug flags to catch a weird case where both are used,
     * which shouldn't happen.
     */


    if (req->getPaddr() & PAddrUncachedBit43) {
        // IPR memory space not implemented
        if (PAddrIprSpace(req->getPaddr())) {
            return std::make_shared<UnimpFault>(
                "IPR memory space not implemented!");
        } else {
            // mark request as uncacheable
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);

            // Clear bits 42:35 of the physical address (10-2 in
            // Tsunami manual)
            req->setPaddr(req->getPaddr() & PAddrUncachedMask);
        }
        // We shouldn't be able to read from an uncachable address in Alpha as
        // we don't have a ROM and we don't want to try to fetch from a device
        // register as we destroy any data that is clear-on-read.
        if (req->isUncacheable() && itb)
            return std::make_shared<UnimpFault>(
                "CPU trying to fetch from uncached I/O");

    }
    return NoFault;
}


// insert a new TLB entry
void
TLB::insert(Addr addr, TlbEntry &entry)
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

    DPRINTF(TLB, "insert @%d: %#x -> %#x\n", nlu, vaddr.vpn(), entry.ppn);

    table[nlu] = entry;
    table[nlu].tag = vaddr.vpn();
    table[nlu].valid = true;

    lookupTable.insert(make_pair(vaddr.vpn(), nlu));
    nextnlu();
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "flushAll\n");
    std::fill(table.begin(), table.end(), TlbEntry());
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
        TlbEntry *entry = &table[index];
        assert(entry->valid);

        // we can't increment i after we erase it, so save a copy and
        // increment it to get the next entry now
        PageTable::iterator cur = i;
        ++i;

        if (!entry->asma) {
            DPRINTF(TLB, "flush @%d: %#x -> %#x\n", index,
                    entry->tag, entry->ppn);
            entry->valid = false;
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
        TlbEntry *entry = &table[index];
        assert(entry->valid);

        if (vaddr.vpn() == entry->tag && (entry->asma || entry->asn == asn)) {
            DPRINTF(TLB, "flushaddr @%d: %#x -> %#x\n", index, vaddr.vpn(),
                    entry->ppn);

            // invalidate this entry
            entry->valid = false;

            lookupTable.erase(i++);
        } else {
            ++i;
        }
    }
}


void
TLB::serialize(CheckpointOut &cp) const
{
    const unsigned size(table.size());
    SERIALIZE_SCALAR(size);
    SERIALIZE_SCALAR(nlu);

    for (int i = 0; i < size; i++)
        table[i].serializeSection(cp, csprintf("Entry%d", i));
}

void
TLB::unserialize(CheckpointIn &cp)
{
    unsigned size(0);
    UNSERIALIZE_SCALAR(size);
    UNSERIALIZE_SCALAR(nlu);

    table.resize(size);
    for (int i = 0; i < size; i++) {
        table[i].unserializeSection(cp, csprintf("Entry%d", i));
        if (table[i].valid) {
            lookupTable.insert(make_pair(table[i].tag, i));
        }
    }
}

Fault
TLB::translateInst(RequestPtr req, ThreadContext *tc)
{
    //If this is a pal pc, then set PHYSICAL
    if (FullSystem && PcPAL(req->getPC()))
        req->setFlags(Request::PHYSICAL);

    if (PcPAL(req->getPC())) {
        // strip off PAL PC marker (lsb is 1)
        req->setPaddr((req->getVaddr() & ~3) & PAddrImplMask);
        fetch_hits++;
        return NoFault;
    }

    if (req->getFlags() & Request::PHYSICAL) {
        req->setPaddr(req->getVaddr());
    } else {
        // verify that this is a good virtual address
        if (!validVirtualAddress(req->getVaddr())) {
            fetch_acv++;
            return std::make_shared<ItbAcvFault>(req->getVaddr());
        }


        // VA<42:41> == 2, VA<39:13> maps directly to PA<39:13> for EV5
        // VA<47:41> == 0x7e, VA<40:13> maps directly to PA<40:13> for EV6
        if (VAddrSpaceEV6(req->getVaddr()) == 0x7e) {
            // only valid in kernel mode
            if (ICM_CM(tc->readMiscRegNoEffect(IPR_ICM)) !=
                mode_kernel) {
                fetch_acv++;
                return std::make_shared<ItbAcvFault>(req->getVaddr());
            }

            req->setPaddr(req->getVaddr() & PAddrImplMask);

            // sign extend the physical address properly
            if (req->getPaddr() & PAddrUncachedBit40)
                req->setPaddr(req->getPaddr() | ULL(0xf0000000000));
            else
                req->setPaddr(req->getPaddr() & ULL(0xffffffffff));
        } else {
            // not a physical address: need to look up pte
            int asn = DTB_ASN_ASN(tc->readMiscRegNoEffect(IPR_DTB_ASN));
            TlbEntry *entry = lookup(VAddr(req->getVaddr()).vpn(),
                              asn);

            if (!entry) {
                fetch_misses++;
                return std::make_shared<ItbPageFault>(req->getVaddr());
            }

            req->setPaddr((entry->ppn << PageShift) +
                          (VAddr(req->getVaddr()).offset()
                           & ~3));

            // check permissions for this access
            if (!(entry->xre &
                  (1 << ICM_CM(tc->readMiscRegNoEffect(IPR_ICM))))) {
                // instruction access fault
                fetch_acv++;
                return std::make_shared<ItbAcvFault>(req->getVaddr());
            }

            fetch_hits++;
        }
    }

    // check that the physical address is ok (catch bad physical addresses)
    if (req->getPaddr() & ~PAddrImplMask) {
        return std::make_shared<MachineCheckFault>();
    }

    return checkCacheability(req, true);

}

Fault
TLB::translateData(RequestPtr req, ThreadContext *tc, bool write)
{
    mode_type mode =
        (mode_type)DTB_CM_CM(tc->readMiscRegNoEffect(IPR_DTB_CM));

    /**
     * Check for alignment faults
     */
    if (req->getVaddr() & (req->getSize() - 1)) {
        DPRINTF(TLB, "Alignment Fault on %#x, size = %d\n", req->getVaddr(),
                req->getSize());
        uint64_t flags = write ? MM_STAT_WR_MASK : 0;
        return std::make_shared<DtbAlignmentFault>(req->getVaddr(),
                                                   req->getFlags(),
                                                   flags);
    }

    if (PcPAL(req->getPC())) {
        mode = (req->getFlags() & AlphaRequestFlags::ALTMODE) ?
            (mode_type)ALT_MODE_AM(
                tc->readMiscRegNoEffect(IPR_ALT_MODE))
            : mode_kernel;
    }

    if (req->getFlags() & Request::PHYSICAL) {
        req->setPaddr(req->getVaddr());
    } else {
        // verify that this is a good virtual address
        if (!validVirtualAddress(req->getVaddr())) {
            if (write) { write_acv++; } else { read_acv++; }
            uint64_t flags = (write ? MM_STAT_WR_MASK : 0) |
                MM_STAT_BAD_VA_MASK |
                MM_STAT_ACV_MASK;
            return std::make_shared<DtbPageFault>(req->getVaddr(),
                                                  req->getFlags(),
                                                  flags);
        }

        // Check for "superpage" mapping
        if (VAddrSpaceEV6(req->getVaddr()) == 0x7e) {
            // only valid in kernel mode
            if (DTB_CM_CM(tc->readMiscRegNoEffect(IPR_DTB_CM)) !=
                mode_kernel) {
                if (write) { write_acv++; } else { read_acv++; }
                uint64_t flags = ((write ? MM_STAT_WR_MASK : 0) |
                                  MM_STAT_ACV_MASK);

                return std::make_shared<DtbAcvFault>(req->getVaddr(),
                                                     req->getFlags(),
                                                     flags);
            }

            req->setPaddr(req->getVaddr() & PAddrImplMask);

            // sign extend the physical address properly
            if (req->getPaddr() & PAddrUncachedBit40)
                req->setPaddr(req->getPaddr() | ULL(0xf0000000000));
            else
                req->setPaddr(req->getPaddr() & ULL(0xffffffffff));
        } else {
            if (write)
                write_accesses++;
            else
                read_accesses++;

            int asn = DTB_ASN_ASN(tc->readMiscRegNoEffect(IPR_DTB_ASN));

            // not a physical address: need to look up pte
            TlbEntry *entry = lookup(VAddr(req->getVaddr()).vpn(), asn);

            if (!entry) {
                // page fault
                if (write) { write_misses++; } else { read_misses++; }
                uint64_t flags = (write ? MM_STAT_WR_MASK : 0) |
                    MM_STAT_DTB_MISS_MASK;
                return (req->getFlags() & AlphaRequestFlags::VPTE) ?
                    (Fault)(std::make_shared<PDtbMissFault>(req->getVaddr(),
                                                            req->getFlags(),
                                                            flags)) :
                    (Fault)(std::make_shared<NDtbMissFault>(req->getVaddr(),
                                                            req->getFlags(),
                                                            flags));
            }

            req->setPaddr((entry->ppn << PageShift) +
                          VAddr(req->getVaddr()).offset());

            if (write) {
                if (!(entry->xwe & MODE2MASK(mode))) {
                    // declare the instruction access fault
                    write_acv++;
                    uint64_t flags = MM_STAT_WR_MASK |
                        MM_STAT_ACV_MASK |
                        (entry->fonw ? MM_STAT_FONW_MASK : 0);
                    return std::make_shared<DtbPageFault>(req->getVaddr(),
                                                          req->getFlags(),
                                                          flags);
                }
                if (entry->fonw) {
                    write_acv++;
                    uint64_t flags = MM_STAT_WR_MASK | MM_STAT_FONW_MASK;
                    return std::make_shared<DtbPageFault>(req->getVaddr(),
                                                          req->getFlags(),
                                                          flags);
                }
            } else {
                if (!(entry->xre & MODE2MASK(mode))) {
                    read_acv++;
                    uint64_t flags = MM_STAT_ACV_MASK |
                        (entry->fonr ? MM_STAT_FONR_MASK : 0);
                    return std::make_shared<DtbAcvFault>(req->getVaddr(),
                                                         req->getFlags(),
                                                         flags);
                }
                if (entry->fonr) {
                    read_acv++;
                    uint64_t flags = MM_STAT_FONR_MASK;
                    return std::make_shared<DtbPageFault>(req->getVaddr(),
                                                          req->getFlags(),
                                                          flags);
                }
            }
        }

        if (write)
            write_hits++;
        else
            read_hits++;
    }

    // check that the physical address is ok (catch bad physical addresses)
    if (req->getPaddr() & ~PAddrImplMask) {
        return std::make_shared<MachineCheckFault>();
    }

    return checkCacheability(req);
}

TlbEntry &
TLB::index(bool advance)
{
    TlbEntry *entry = &table[nlu];

    if (advance)
        nextnlu();

    return *entry;
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
TLB::translateFunctional(RequestPtr req, ThreadContext *tc, Mode mode)
{
    panic("Not implemented\n");
    return NoFault;
}

Fault
TLB::finalizePhysical(RequestPtr req, ThreadContext *tc, Mode mode) const
{
    return NoFault;
}

} // namespace AlphaISA

AlphaISA::TLB *
AlphaTLBParams::create()
{
    return new AlphaISA::TLB(this);
}
