/*
 * Copyright (c) 2018 Inria
 * Copyright (c) 2012-2013, 2015, 2022-2024 Arm Limited
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
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "params/StridePrefetcher.hh"

namespace gem5
{

namespace prefetch
{

Stride::StrideEntry::StrideEntry(const SatCounter8& init_confidence,
                                 TaggedIndexingPolicy *ip)
  : TaggedEntry(ip), confidence(init_confidence)
{
    invalidate();
}

void
Stride::StrideEntry::invalidate()
{
    TaggedEntry::invalidate();
    lastAddr = 0;
    stride = 0;
    confidence.reset();
}

Stride::Stride(const StridePrefetcherParams &p)
  : Queued(p),
    initConfidence(p.confidence_counter_bits, p.initial_confidence),
    threshConf(p.confidence_threshold/100.0),
    useRequestorId(p.use_requestor_id),
    degree(p.degree),
    distance(p.distance),
    pcTableInfo(p.table_assoc, p.table_entries, p.table_indexing_policy,
                p.table_replacement_policy),
    useCachelineAddr(p.use_cache_line_address)
{
}

Stride::PCTable&
Stride::findTable(int context)
{
    // Check if table for given context exists
    auto it = pcTables.find(context);
    if (it != pcTables.end())
        return *(it->second);

    // If table does not exist yet, create one
    return allocateNewContext(context);
}

Stride::PCTable&
Stride::allocateNewContext(int context)
{
    std::string table_name = name() + ".PCTable" + std::to_string(context);
    // Create new table
    pcTables[context].reset(new PCTable(
        table_name.c_str(),
        pcTableInfo.numEntries,
        pcTableInfo.assoc,
        pcTableInfo.replacementPolicy,
        pcTableInfo.indexingPolicy,
        StrideEntry(initConfidence, pcTableInfo.indexingPolicy)));

    DPRINTF(HWPrefetch, "Adding context %i with stride entries\n", context);

    // return a reference to the new table
    return *(pcTables[context]);
}

void
Stride::calculatePrefetch(const PrefetchInfo &pfi,
                                    std::vector<AddrPriority> &addresses,
                                    const CacheAccessor &cache)
{
    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }

    // Get required packet info
    Addr pf_addr = useCachelineAddr ? blockAddress(pfi.getAddr())
                                    : pfi.getAddr();
    Addr pc = pfi.getPC();
    bool is_secure = pfi.isSecure();
    RequestorID requestor_id = useRequestorId ? pfi.getRequestorId() : 0;

    // Get corresponding pc table
    PCTable& pc_table = findTable(requestor_id);

    // Search for entry in the pc table
    const StrideEntry::KeyType key{pc, is_secure};
    StrideEntry *entry = pc_table.findEntry(key);

    if (entry != nullptr) {
        pc_table.accessEntry(entry);

        // Hit in table
        int new_stride = pf_addr - entry->lastAddr;

        // Do nothing on repeated memory access
        if (useCachelineAddr && new_stride == 0)
            return;

        bool stride_match = (new_stride == entry->stride);

        // Adjust confidence for stride entry
        if (stride_match) {
            entry->confidence++;
        } else {
            entry->confidence--;
            // If confidence has dropped below the threshold, train new stride
            if (entry->confidence.calcSaturation() < threshConf) {
                entry->stride = new_stride;
            }
        }

        DPRINTF(HWPrefetch, "Hit: PC %x pkt_addr %x (%s) stride %d (%s), "
                "conf %d\n", pc, pf_addr, is_secure ? "s" : "ns",
                new_stride, stride_match ? "match" : "change",
                (int)entry->confidence);

        entry->lastAddr = pf_addr;

        // Abort prefetch generation if below confidence threshold
        if (entry->confidence.calcSaturation() < threshConf) {
            return;
        }

        // Round strides up to at least 1 cacheline
        int prefetch_stride = entry->stride;
        if (abs(prefetch_stride) < blkSize) {
            prefetch_stride = (prefetch_stride < 0) ? -blkSize : blkSize;
        }

        Addr new_addr = pf_addr + distance * prefetch_stride;
        // Generate up to degree prefetches
        for (int d = 1; d <= degree; d++) {
            new_addr += prefetch_stride;
            addresses.push_back(AddrPriority(new_addr, 0));
        }
    } else {
        // Miss in table
        DPRINTF(HWPrefetch, "Miss: PC %x pkt_addr %x (%s)\n", pc, pf_addr,
                is_secure ? "s" : "ns");

        StrideEntry* entry = pc_table.findVictim(key);

        // Insert new entry's data
        entry->lastAddr = pf_addr;
        pc_table.insertEntry(key, entry);
    }
}

uint32_t
StridePrefetcherHashedSetAssociative::extractSet(const KeyType &key) const
{
    const Addr pc = key.address;
    const Addr hash1 = pc >> 1;
    const Addr hash2 = hash1 >> tagShift;
    return (hash1 ^ hash2) & setMask;
}

Addr
StridePrefetcherHashedSetAssociative::extractTag(const Addr addr) const
{
    return addr;
}

} // namespace prefetch
} // namespace gem5
