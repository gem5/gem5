/*
 * Copyright (c) 2010-2013, 2016-2021 Arm Limited
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
 */

#include "arch/arm/tlb.hh"

#include <memory>
#include <string>
#include <vector>

#include "arch/arm/table_walker.hh"
#include "arch/arm/tlbi_op.hh"
#include "arch/arm/utility.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "params/ArmTLB.hh"

namespace gem5
{

using namespace ArmISA;

TLB::TLB(const ArmTLBParams &p)
    : BaseTLB(p), table(new TlbEntry[p.size]), size(p.size),
      isStage2(p.is_stage2),
      tableWalker(nullptr),
      stats(*this), rangeMRU(1), vmid(0)
{
}

TLB::~TLB()
{
    delete[] table;
}

void
TLB::setTableWalker(TableWalker *table_walker)
{
    tableWalker = table_walker;
    tableWalker->setTlb(this);
}

TlbEntry*
TLB::lookup(const Lookup &lookup_data)
{
    TlbEntry *retval = NULL;
    const auto functional = lookup_data.functional;
    const auto mode = lookup_data.mode;

    // Maintaining LRU array
    int x = 0;
    while (retval == NULL && x < size) {
        if (table[x].match(lookup_data)) {
            // We only move the hit entry ahead when the position is higher
            // than rangeMRU
            if (x > rangeMRU && !functional) {
                TlbEntry tmp_entry = table[x];
                for (int i = x; i > 0; i--)
                    table[i] = table[i - 1];
                table[0] = tmp_entry;
                retval = &table[0];
            } else {
                retval = &table[x];
            }
            break;
        }
        ++x;
    }

    DPRINTF(TLBVerbose, "Lookup %#x, asn %#x -> %s vmn 0x%x hyp %d secure %d "
            "ppn %#x size: %#x pa: %#x ap:%d ns:%d nstid:%d g:%d asid: %d "
            "el: %d\n",
            lookup_data.va, lookup_data.asn, retval ? "hit" : "miss",
            lookup_data.vmid, lookup_data.hyp, lookup_data.secure,
            retval ? retval->pfn       : 0, retval ? retval->size  : 0,
            retval ? retval->pAddr(lookup_data.va) : 0,
            retval ? retval->ap        : 0,
            retval ? retval->ns        : 0, retval ? retval->nstid : 0,
            retval ? retval->global    : 0, retval ? retval->asid  : 0,
            retval ? retval->el        : 0);

    // Updating stats if this was not a functional lookup
    if (!functional) {
        if (!retval) {
            if (mode == BaseMMU::Execute)
                stats.instMisses++;
            else if (mode == BaseMMU::Write)
                stats.writeMisses++;
            else
                stats.readMisses++;
        } else {
            if (mode == BaseMMU::Execute)
                stats.instHits++;
            else if (mode == BaseMMU::Write)
               stats.writeHits++;
            else
                stats.readHits++;
        }
    }

    return retval;
}

TlbEntry*
TLB::multiLookup(const Lookup &lookup_data)
{
    TlbEntry* te = lookup(lookup_data);

    if (te) {
        checkPromotion(te, lookup_data.mode);
    } else {
        if (auto tlb = static_cast<TLB*>(nextLevel())) {
            te = tlb->multiLookup(lookup_data);
            if (te && !lookup_data.functional)
                insert(*te);
        }
    }

    return te;
}

void
TLB::checkPromotion(TlbEntry *entry, BaseMMU::Mode mode)
{
    TypeTLB acc_type = (mode == BaseMMU::Execute) ?
       TypeTLB::instruction : TypeTLB::data;

    // Hitting an instruction TLB entry on a data access or
    // a data TLB entry on an instruction access:
    // promoting the entry to unified
    if (!(entry->type & acc_type))
       entry->type = TypeTLB::unified;
}

// insert a new TLB entry
void
TLB::insert(TlbEntry &entry)
{
    DPRINTF(TLB, "Inserting entry into TLB with pfn:%#x size:%#x vpn: %#x"
            " asid:%d vmid:%d N:%d global:%d valid:%d nc:%d xn:%d"
            " ap:%#x domain:%#x ns:%d nstid:%d isHyp:%d\n", entry.pfn,
            entry.size, entry.vpn, entry.asid, entry.vmid, entry.N,
            entry.global, entry.valid, entry.nonCacheable, entry.xn,
            entry.ap, static_cast<uint8_t>(entry.domain), entry.ns, entry.nstid,
            entry.isHyp);

    if (table[size - 1].valid)
        DPRINTF(TLB, " - Replacing Valid entry %#x, asn %d vmn %d ppn %#x "
                "size: %#x ap:%d ns:%d nstid:%d g:%d isHyp:%d el: %d\n",
                table[size-1].vpn << table[size-1].N, table[size-1].asid,
                table[size-1].vmid, table[size-1].pfn << table[size-1].N,
                table[size-1].size, table[size-1].ap, table[size-1].ns,
                table[size-1].nstid, table[size-1].global, table[size-1].isHyp,
                table[size-1].el);

    // inserting to MRU position and evicting the LRU one
    for (int i = size - 1; i > 0; --i)
        table[i] = table[i-1];
    table[0] = entry;

    stats.inserts++;
    ppRefills->notify(1);
}

void
TLB::multiInsert(TlbEntry &entry)
{
    insert(entry);

    if (auto next_level = static_cast<TLB*>(nextLevel())) {
        next_level->multiInsert(entry);
    }
}

void
TLB::printTlb() const
{
    int x = 0;
    TlbEntry *te;
    DPRINTF(TLB, "Current TLB contents:\n");
    while (x < size) {
        te = &table[x];
        if (te->valid)
            DPRINTF(TLB, " *  %s\n", te->print());
        ++x;
    }
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "Flushing all TLB entries\n");
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];

        DPRINTF(TLB, " -  %s\n", te->print());
        te->valid = false;
        stats.flushedEntries++;
        ++x;
    }

    stats.flushTlb++;
}

void
TLB::flush(const TLBIALL& tlbi_op)
{
    DPRINTF(TLB, "Flushing all TLB entries (%s lookup)\n",
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];
        const bool el_match = te->checkELMatch(
            tlbi_op.targetEL, tlbi_op.inHost);
        if (te->valid && tlbi_op.secureLookup == !te->nstid &&
            (te->vmid == vmid || tlbi_op.el2Enabled) && el_match) {

            DPRINTF(TLB, " -  %s\n", te->print());
            te->valid = false;
            stats.flushedEntries++;
        }
        ++x;
    }

    stats.flushTlb++;
}

void
TLB::flush(const ITLBIALL& tlbi_op)
{
    DPRINTF(TLB, "Flushing all ITLB entries (%s lookup)\n",
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];
        const bool el_match = te->checkELMatch(
            tlbi_op.targetEL, tlbi_op.inHost);
        if (te->type & TypeTLB::instruction && te->valid &&
            tlbi_op.secureLookup == !te->nstid &&
            (te->vmid == vmid || tlbi_op.el2Enabled) && el_match) {

            DPRINTF(TLB, " -  %s\n", te->print());
            te->valid = false;
            stats.flushedEntries++;
        }
        ++x;
    }

    stats.flushTlb++;
}

void
TLB::flush(const DTLBIALL& tlbi_op)
{
    DPRINTF(TLB, "Flushing all DTLB entries (%s lookup)\n",
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];
        const bool el_match = te->checkELMatch(
            tlbi_op.targetEL, tlbi_op.inHost);
        if (te->type & TypeTLB::data && te->valid &&
            tlbi_op.secureLookup == !te->nstid &&
            (te->vmid == vmid || tlbi_op.el2Enabled) && el_match) {

            DPRINTF(TLB, " -  %s\n", te->print());
            te->valid = false;
            stats.flushedEntries++;
        }
        ++x;
    }

    stats.flushTlb++;
}

void
TLB::flush(const TLBIALLEL &tlbi_op)
{
    DPRINTF(TLB, "Flushing all TLB entries (%s lookup)\n",
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];
        const bool el_match = te->checkELMatch(
            tlbi_op.targetEL, tlbi_op.inHost);
        if (te->valid && tlbi_op.secureLookup == !te->nstid && el_match) {

            DPRINTF(TLB, " -  %s\n", te->print());
            te->valid = false;
            stats.flushedEntries++;
        }
        ++x;
    }

    stats.flushTlb++;
}

void
TLB::flush(const TLBIVMALL &tlbi_op)
{
    DPRINTF(TLB, "Flushing all TLB entries (%s lookup)\n",
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];
        const bool el_match = te->checkELMatch(
            tlbi_op.targetEL, tlbi_op.inHost);

        const bool vmid_match =
            te->vmid == vmid ||
            !tlbi_op.el2Enabled ||
            (!tlbi_op.stage2Flush() && tlbi_op.inHost);

        if (te->valid && tlbi_op.secureLookup == !te->nstid &&
            el_match && vmid_match) {

            DPRINTF(TLB, " -  %s\n", te->print());
            te->valid = false;
            stats.flushedEntries++;
        }
        ++x;
    }

    stats.flushTlb++;
}

void
TLB::flush(const TLBIALLN &tlbi_op)
{
    bool hyp = tlbi_op.targetEL == EL2;

    DPRINTF(TLB, "Flushing all NS TLB entries (%s lookup)\n",
            (hyp ? "hyp" : "non-hyp"));
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];
        const bool el_match = te->checkELMatch(tlbi_op.targetEL, false);

        if (te->valid && te->nstid && te->isHyp == hyp && el_match) {

            DPRINTF(TLB, " -  %s\n", te->print());
            stats.flushedEntries++;
            te->valid = false;
        }
        ++x;
    }

    stats.flushTlb++;
}

void
TLB::flush(const TLBIMVA &tlbi_op)
{
    DPRINTF(TLB, "Flushing TLB entries with mva: %#x, asid: %#x "
            "(%s lookup)\n", tlbi_op.addr, tlbi_op.asid,
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    _flushMva(tlbi_op.addr, tlbi_op.asid, tlbi_op.secureLookup, false,
        tlbi_op.targetEL, tlbi_op.inHost, TypeTLB::unified);
    stats.flushTlbMvaAsid++;
}

void
TLB::flush(const ITLBIMVA &tlbi_op)
{
    DPRINTF(TLB, "Flushing ITLB entries with mva: %#x, asid: %#x "
            "(%s lookup)\n", tlbi_op.addr, tlbi_op.asid,
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    _flushMva(tlbi_op.addr, tlbi_op.asid, tlbi_op.secureLookup, false,
        tlbi_op.targetEL, tlbi_op.inHost, TypeTLB::instruction);
    stats.flushTlbMvaAsid++;
}

void
TLB::flush(const DTLBIMVA &tlbi_op)
{
    DPRINTF(TLB, "Flushing DTLB entries with mva: %#x, asid: %#x "
            "(%s lookup)\n", tlbi_op.addr, tlbi_op.asid,
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    _flushMva(tlbi_op.addr, tlbi_op.asid, tlbi_op.secureLookup, false,
        tlbi_op.targetEL, tlbi_op.inHost, TypeTLB::data);
    stats.flushTlbMvaAsid++;
}

void
TLB::flush(const TLBIASID &tlbi_op)
{
    DPRINTF(TLB, "Flushing TLB entries with asid: %#x (%s lookup)\n",
            tlbi_op.asid, (tlbi_op.secureLookup ? "secure" : "non-secure"));

    int x = 0 ;
    TlbEntry *te;

    while (x < size) {
        te = &table[x];

        const bool el_match = te->checkELMatch(
            tlbi_op.targetEL, tlbi_op.inHost);

        const bool vmid_match =
            te->vmid == vmid || !tlbi_op.el2Enabled || tlbi_op.inHost;

        if (te->valid && te->asid == tlbi_op.asid &&
            tlbi_op.secureLookup == !te->nstid &&
            vmid_match && el_match) {

            te->valid = false;
            DPRINTF(TLB, " -  %s\n", te->print());
            stats.flushedEntries++;
        }
        ++x;
    }
    stats.flushTlbAsid++;
}

void
TLB::flush(const ITLBIASID &tlbi_op)
{
    DPRINTF(TLB, "Flushing ITLB entries with asid: %#x (%s lookup)\n",
            tlbi_op.asid, (tlbi_op.secureLookup ? "secure" : "non-secure"));

    int x = 0 ;
    TlbEntry *te;

    while (x < size) {
        te = &table[x];
        if (te->type & TypeTLB::instruction &&
            te->valid && te->asid == tlbi_op.asid &&
            tlbi_op.secureLookup == !te->nstid &&
            (te->vmid == vmid || tlbi_op.el2Enabled) &&
            te->checkELMatch(tlbi_op.targetEL, tlbi_op.inHost)) {

            te->valid = false;
            DPRINTF(TLB, " -  %s\n", te->print());
            stats.flushedEntries++;
        }
        ++x;
    }
    stats.flushTlbAsid++;
}

void
TLB::flush(const DTLBIASID &tlbi_op)
{
    DPRINTF(TLB, "Flushing DTLB entries with asid: %#x (%s lookup)\n",
            tlbi_op.asid, (tlbi_op.secureLookup ? "secure" : "non-secure"));

    int x = 0 ;
    TlbEntry *te;

    while (x < size) {
        te = &table[x];
        if (te->type & TypeTLB::data &&
            te->valid && te->asid == tlbi_op.asid &&
            tlbi_op.secureLookup == !te->nstid &&
            (te->vmid == vmid || tlbi_op.el2Enabled) &&
            te->checkELMatch(tlbi_op.targetEL, tlbi_op.inHost)) {

            te->valid = false;
            DPRINTF(TLB, " -  %s\n", te->print());
            stats.flushedEntries++;
        }
        ++x;
    }
    stats.flushTlbAsid++;
}

void
TLB::flush(const TLBIMVAA &tlbi_op) {

    DPRINTF(TLB, "Flushing TLB entries with mva: %#x (%s lookup)\n",
            tlbi_op.addr,
            (tlbi_op.secureLookup ? "secure" : "non-secure"));
    _flushMva(tlbi_op.addr, 0xbeef, tlbi_op.secureLookup, true,
        tlbi_op.targetEL, tlbi_op.inHost, TypeTLB::unified);
    stats.flushTlbMva++;
}

void
TLB::_flushMva(Addr mva, uint64_t asn, bool secure_lookup,
               bool ignore_asn, ExceptionLevel target_el, bool in_host,
               TypeTLB entry_type)
{
    TlbEntry *te;
    Lookup lookup_data;

    lookup_data.va = sext<56>(mva);
    lookup_data.asn = asn;
    lookup_data.ignoreAsn = ignore_asn;
    lookup_data.vmid = vmid;
    lookup_data.hyp = target_el == EL2;
    lookup_data.secure = secure_lookup;
    lookup_data.functional = true;
    lookup_data.targetEL = target_el;
    lookup_data.inHost = in_host;
    lookup_data.mode = BaseMMU::Read;

    te = lookup(lookup_data);
    while (te != NULL) {
        bool matching_type = (te->type & entry_type);
        if (matching_type && secure_lookup == !te->nstid) {
            DPRINTF(TLB, " -  %s\n", te->print());
            te->valid = false;
            stats.flushedEntries++;
        }
        te = lookup(lookup_data);
    }
}

void
TLB::takeOverFrom(BaseTLB *_otlb)
{
}

TLB::TlbStats::TlbStats(TLB &parent)
  : statistics::Group(&parent), tlb(parent),
    ADD_STAT(instHits, statistics::units::Count::get(), "Inst hits"),
    ADD_STAT(instMisses, statistics::units::Count::get(), "Inst misses"),
    ADD_STAT(readHits, statistics::units::Count::get(), "Read hits"),
    ADD_STAT(readMisses, statistics::units::Count::get(),  "Read misses"),
    ADD_STAT(writeHits, statistics::units::Count::get(), "Write hits"),
    ADD_STAT(writeMisses, statistics::units::Count::get(), "Write misses"),
    ADD_STAT(inserts, statistics::units::Count::get(),
             "Number of times an entry is inserted into the TLB"),
    ADD_STAT(flushTlb, statistics::units::Count::get(),
             "Number of times complete TLB was flushed"),
    ADD_STAT(flushTlbMva, statistics::units::Count::get(),
             "Number of times TLB was flushed by MVA"),
    ADD_STAT(flushTlbMvaAsid, statistics::units::Count::get(),
             "Number of times TLB was flushed by MVA & ASID"),
    ADD_STAT(flushTlbAsid, statistics::units::Count::get(),
             "Number of times TLB was flushed by ASID"),
    ADD_STAT(flushedEntries, statistics::units::Count::get(),
             "Number of entries that have been flushed from TLB"),
    ADD_STAT(readAccesses, statistics::units::Count::get(), "Read accesses",
             readHits + readMisses),
    ADD_STAT(writeAccesses, statistics::units::Count::get(), "Write accesses",
             writeHits + writeMisses),
    ADD_STAT(instAccesses, statistics::units::Count::get(), "Inst accesses",
             instHits + instMisses),
    ADD_STAT(hits, statistics::units::Count::get(),
             "Total TLB (inst and data) hits",
             readHits + writeHits + instHits),
    ADD_STAT(misses, statistics::units::Count::get(),
             "Total TLB (inst and data) misses",
             readMisses + writeMisses + instMisses),
    ADD_STAT(accesses, statistics::units::Count::get(),
             "Total TLB (inst and data) accesses",
             readAccesses + writeAccesses + instAccesses)
{
    // If this is a pure Data TLB, mark the instruction
    // stats as nozero, so that they won't make it in
    // into the final stats file
    if (tlb.type() == TypeTLB::data) {
        instHits.flags(statistics::nozero);
        instMisses.flags(statistics::nozero);

        instAccesses.flags(statistics::nozero);
    }

    // If this is a pure Instruction TLB, mark the data
    // stats as nozero, so that they won't make it in
    // into the final stats file
    if (tlb.type() & TypeTLB::instruction) {
        readHits.flags(statistics::nozero);
        readMisses.flags(statistics::nozero);
        writeHits.flags(statistics::nozero);
        writeMisses.flags(statistics::nozero);

        readAccesses.flags(statistics::nozero);
        writeAccesses.flags(statistics::nozero);
    }
}

void
TLB::regProbePoints()
{
    ppRefills.reset(new probing::PMU(getProbeManager(), "Refills"));
}

Port *
TLB::getTableWalkerPort()
{
    return &tableWalker->getTableWalkerPort();
}

} // namespace gem5
