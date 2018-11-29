/*
 * Copyright (c) 2018 Inria
 * Copyright (c) 2012-2013, 2015 ARM Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
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
 * Authors: Ron Dreslinski
 *          Steve Reinhardt
 *          Daniel Carvalho
 */

/**
 * @file
 * Stride Prefetcher template instantiations.
 */

#include "mem/cache/prefetch/stride.hh"

#include <cassert>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "params/StridePrefetcher.hh"

StridePrefetcher::StrideEntry::StrideEntry()
{
    invalidate();
}

void
StridePrefetcher::StrideEntry::invalidate()
{
    instAddr = 0;
    lastAddr = 0;
    isSecure = false;
    stride = 0;
    confidence = 0;
}

StridePrefetcher::StridePrefetcher(const StridePrefetcherParams *p)
    : QueuedPrefetcher(p),
      maxConf(p->max_conf),
      threshConf(p->thresh_conf),
      minConf(p->min_conf),
      startConf(p->start_conf),
      pcTableAssoc(p->table_assoc),
      pcTableSets(p->table_sets),
      useMasterId(p->use_master_id),
      degree(p->degree),
      replacementPolicy(p->replacement_policy)
{
    assert(isPowerOf2(pcTableSets));
}

StridePrefetcher::PCTable*
StridePrefetcher::findTable(int context)
{
    // Check if table for given context exists
    auto it = pcTables.find(context);
    if (it != pcTables.end())
        return &it->second;

    // If table does not exist yet, create one
    return allocateNewContext(context);
}

StridePrefetcher::PCTable*
StridePrefetcher::allocateNewContext(int context)
{
    // Create new table
    auto insertion_result = pcTables.insert(std::make_pair(context,
        PCTable(pcTableAssoc, pcTableSets, name(), replacementPolicy)));

    DPRINTF(HWPrefetch, "Adding context %i with stride entries\n", context);

    // Get iterator to new pc table, and then return a pointer to the new table
    return &(insertion_result.first->second);
}

StridePrefetcher::PCTable::PCTable(int assoc, int sets, const std::string name,
                                   BaseReplacementPolicy* replacementPolicy)
    : pcTableSets(sets), _name(name), entries(pcTableSets),
      replacementPolicy(replacementPolicy)
{
    for (int set = 0; set < sets; set++) {
        entries[set].resize(assoc);
        for (int way = 0; way < assoc; way++) {
            // Inform the entry its position
            entries[set][way].setPosition(set, way);

            // Initialize replacement policy data
            entries[set][way].replacementData =
                replacementPolicy->instantiateEntry();
        }
    }
}

StridePrefetcher::PCTable::~PCTable()
{
}

void
StridePrefetcher::calculatePrefetch(const PrefetchInfo &pfi,
                                    std::vector<AddrPriority> &addresses)
{
    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }

    // Get required packet info
    Addr pf_addr = pfi.getAddr();
    Addr pc = pfi.getPC();
    bool is_secure = pfi.isSecure();
    MasterID master_id = useMasterId ? pfi.getMasterId() : 0;

    // Get corresponding pc table
    PCTable* pcTable = findTable(master_id);

    // Search for entry in the pc table
    StrideEntry *entry = pcTable->findEntry(pc, is_secure);

    if (entry != nullptr) {
        // Hit in table
        int new_stride = pf_addr - entry->lastAddr;
        bool stride_match = (new_stride == entry->stride);

        // Adjust confidence for stride entry
        if (stride_match && new_stride != 0) {
            if (entry->confidence < maxConf)
                entry->confidence++;
        } else {
            if (entry->confidence > minConf)
                entry->confidence--;
            // If confidence has dropped below the threshold, train new stride
            if (entry->confidence < threshConf)
                entry->stride = new_stride;
        }

        DPRINTF(HWPrefetch, "Hit: PC %x pkt_addr %x (%s) stride %d (%s), "
                "conf %d\n", pc, pf_addr, is_secure ? "s" : "ns",
                new_stride, stride_match ? "match" : "change",
                entry->confidence);

        entry->lastAddr = pf_addr;

        // Abort prefetch generation if below confidence threshold
        if (entry->confidence < threshConf)
            return;

        // Generate up to degree prefetches
        for (int d = 1; d <= degree; d++) {
            // Round strides up to atleast 1 cacheline
            int prefetch_stride = new_stride;
            if (abs(new_stride) < blkSize) {
                prefetch_stride = (new_stride < 0) ? -blkSize : blkSize;
            }

            Addr new_addr = pf_addr + d * prefetch_stride;
            addresses.push_back(AddrPriority(new_addr, 0));
        }
    } else {
        // Miss in table
        DPRINTF(HWPrefetch, "Miss: PC %x pkt_addr %x (%s)\n", pc, pf_addr,
                is_secure ? "s" : "ns");

        StrideEntry* entry = pcTable->findVictim(pc);

        // Invalidate victim
        entry->invalidate();
        replacementPolicy->invalidate(entry->replacementData);

        // Insert new entry's data
        entry->instAddr = pc;
        entry->lastAddr = pf_addr;
        entry->isSecure = is_secure;
        entry->confidence = startConf;
        replacementPolicy->reset(entry->replacementData);
    }
}

inline Addr
StridePrefetcher::PCTable::pcHash(Addr pc) const
{
    Addr hash1 = pc >> 1;
    Addr hash2 = hash1 >> floorLog2(pcTableSets);
    return (hash1 ^ hash2) & (Addr)(pcTableSets - 1);
}

inline StridePrefetcher::StrideEntry*
StridePrefetcher::PCTable::findVictim(Addr pc)
{
    // Rand replacement for now
    int set = pcHash(pc);

    // Get possible entries to be victimized
    std::vector<ReplaceableEntry*> possible_entries;
    for (auto& entry : entries[set]) {
        possible_entries.push_back(&entry);
    }

    // Choose victim based on replacement policy
    StrideEntry* victim = static_cast<StrideEntry*>(
        replacementPolicy->getVictim(possible_entries));

    DPRINTF(HWPrefetch, "Victimizing lookup table[%d][%d].\n",
            victim->getSet(), victim->getWay());

    return victim;
}

inline StridePrefetcher::StrideEntry*
StridePrefetcher::PCTable::findEntry(Addr pc, bool is_secure)
{
    int set = pcHash(pc);
    for (auto& entry : entries[set]) {
        // Search ways for match
        if ((entry.instAddr == pc) && (entry.isSecure == is_secure)) {
            DPRINTF(HWPrefetch, "Lookup hit table[%d][%d].\n", entry.getSet(),
                    entry.getWay());
            replacementPolicy->touch(entry.replacementData);
            return &entry;
        }
    }
    return nullptr;
}

StridePrefetcher*
StridePrefetcherParams::create()
{
    return new StridePrefetcher(this);
}
