/*
 * Copyright (c) 2010-2013, 2016-2024 Arm Limited
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

TlbEntry*
TLB::Table::accessEntry(const KeyType &key)
{
    auto entry = findEntry(key);

    if (entry && !key.functional) {
        accessEntry(entry);
    }

    return entry;
}

TlbEntry*
TLB::Table::findEntry(const KeyType &key) const
{
    auto candidates = indexingPolicy->getPossibleEntries(key);

    for (auto candidate : candidates) {
        auto entry = static_cast<TlbEntry*>(candidate);
        // We check for pageSize match outside of the Entry::match
        // as the latter is also used to match entries in TLBI invalidation
        // where we don't care about the pageSize
        if (entry->N == key.pageSize && entry->match(key)) {
            return entry;
        }
    }

    return nullptr;
}

TLB::TLB(const ArmTLBParams &p)
    : BaseTLB(p),
      table(name().c_str(), p.size, p.assoc,
            p.replacement_policy, p.indexing_policy),
      size(p.size),
      isStage2(p.is_stage2),
      _walkCache(false),
      tableWalker(nullptr),
      stats(*this), rangeMRU(1), vmid(0)
{
    for (int lvl = LookupLevel::L0;
         lvl < LookupLevel::Num_ArmLookupLevel; lvl++) {

        auto it = std::find(
            p.partial_levels.begin(),
            p.partial_levels.end(),
            lvl);

        auto lookup_lvl = static_cast<LookupLevel>(lvl);

        if (it != p.partial_levels.end()) {
            // A partial entry from of the current LookupLevel can be
            // cached within the TLB
            partialLevels[lookup_lvl] = true;

            // Make sure this is not the last level (complete translation)
            if (lvl != LookupLevel::Num_ArmLookupLevel - 1) {
                _walkCache = true;
            }
        } else {
            partialLevels[lookup_lvl] = false;
        }
    }

    table.setDebugFlag(::gem5::debug::TLB);
}

TLB::~TLB()
{
}

void
TLB::setTableWalker(TableWalker *table_walker)
{
    tableWalker = table_walker;
    tableWalker->setTlb(this);
}

TlbEntry*
TLB::lookup(Lookup lookup_data)
{
    const auto mode = lookup_data.mode;

    TlbEntry *retval = nullptr;
    // We iterate over all stored sizes, starting from the
    // smallest until the biggest. In this way we prioritize
    // complete translations over partial translations
    for (const auto &page_size : observedPageSizes) {
        lookup_data.pageSize = page_size;
        if (retval = table.accessEntry(lookup_data); retval)
            break;
    }

    DPRINTF(TLBVerbose, "Lookup %#x, asn %#x -> %s vmn 0x%x ss %s "
            "ppn %#x size: %#x pa: %#x ap:%d ns:%d ss:%s g:%d asid: %d "
            "xs: %d regime: %s\n",
            lookup_data.va, lookup_data.asn, retval ? "hit" : "miss",
            lookup_data.vmid, lookup_data.ss,
            retval ? retval->pfn       : 0, retval ? retval->size  : 0,
            retval ? retval->pAddr(lookup_data.va) : 0,
            retval ? retval->ap        : 0,
            retval ? retval->ns        : 0,
            retval ? retval->ss : SecurityState::NonSecure,
            retval ? retval->global    : 0, retval ? retval->asid  : 0,
            retval ? retval->xs : 0,
            retval ? regimeToStr(retval->regime) : "None");

    // Updating stats if this was not a functional lookup
    if (!lookup_data.functional) {
        if (!retval) {
            if (mode == BaseMMU::Execute) {
                stats.instMisses++;
            } else if (mode == BaseMMU::Write) {
                stats.writeMisses++;
            } else {
                stats.readMisses++;
            }
        } else {
            if (retval->partial) {
                stats.partialHits++;
            }

            if (mode == BaseMMU::Execute) {
                stats.instHits++;
            } else if (mode == BaseMMU::Write) {
               stats.writeHits++;
            } else {
                stats.readHits++;
            }
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
            if (te && !lookup_data.functional &&
                (!te->partial || partialLevels[te->lookupLevel])) {
                // Insert entry only if this is not a functional
                // lookup and if the translation is complete (unless this
                // TLB caches partial translations)
                insert(lookup_data, *te);
            }
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
TLB::insert(const Lookup &lookup_data, TlbEntry &entry)
{
    TlbEntry *victim = table.findVictim(lookup_data);

    *victim = entry;

    table.insertEntry(lookup_data, victim);

    observedPageSizes.insert(entry.N);
    stats.inserts++;
    ppRefills->notify(1);
}

void
TLB::multiInsert(const Lookup &lookup, TlbEntry &entry)
{
    // Insert a partial translation only if the TLB is configured
    // as a walk cache
    if (!entry.partial || partialLevels[entry.lookupLevel]) {
        insert(lookup, entry);
    }

    if (auto next_level = static_cast<TLB*>(nextLevel())) {
        next_level->multiInsert(lookup, entry);
    }
}

void
TLB::printTlb() const
{
    DPRINTF(TLB, "Current TLB contents:\n");
    for (auto& te : table) {
        if (te.valid)
            DPRINTF(TLB, " *  %s\n", te.print());
    }
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "Flushing all TLB entries\n");
    for (auto& te : table) {
        if (te.valid) {
            DPRINTF(TLB, " -  %s\n", te.print());
            table.invalidate(&te);
            stats.flushedEntries++;
        }
    }

    stats.flushTlb++;
    observedPageSizes.clear();
}

void
TLB::flush(const TLBIOp& tlbi_op)
{
    bool valid_entry = false;
    for (auto& te : table) {
        if (tlbi_op.match(&te, vmid)) {
            DPRINTF(TLB, " -  %s\n", te.print());
            table.invalidate(&te);
            stats.flushedEntries++;
        }
        valid_entry = valid_entry || te.valid;
    }

    stats.flushTlb++;
    if (!valid_entry)
        observedPageSizes.clear();
}

void
TLB::takeOverFrom(BaseTLB *_otlb)
{
}

TLB::TlbStats::TlbStats(TLB &parent)
  : statistics::Group(&parent), tlb(parent),
    ADD_STAT(partialHits, statistics::units::Count::get(),
             "partial translation hits"),
    ADD_STAT(instHits, statistics::units::Count::get(), "Inst hits"),
    ADD_STAT(instMisses, statistics::units::Count::get(), "Inst misses"),
    ADD_STAT(readHits, statistics::units::Count::get(), "Read hits"),
    ADD_STAT(readMisses, statistics::units::Count::get(),  "Read misses"),
    ADD_STAT(writeHits, statistics::units::Count::get(), "Write hits"),
    ADD_STAT(writeMisses, statistics::units::Count::get(), "Write misses"),
    ADD_STAT(inserts, statistics::units::Count::get(),
             "Number of times an entry is inserted into the TLB"),
    ADD_STAT(flushTlb, statistics::units::Count::get(),
             "Number of times a TLB invalidation was requested"),
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

    partialHits.flags(statistics::nozero);
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
