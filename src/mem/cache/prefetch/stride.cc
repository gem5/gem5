/*
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
 */

/**
 * @file
 * Stride Prefetcher template instantiations.
 */

#include "base/random.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/stride.hh"

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
      pcTable(pcTableAssoc, pcTableSets, name())
{
    // Don't consult stride prefetcher on instruction accesses
    onInst = false;

    assert(isPowerOf2(pcTableSets));
}

StridePrefetcher::StrideEntry**
StridePrefetcher::PCTable::allocateNewContext(int context)
{
    auto res = entries.insert(std::make_pair(context,
                              new StrideEntry*[pcTableSets]));
    auto it = res.first;
    chatty_assert(res.second, "Allocating an already created context\n");
    assert(it->first == context);

    DPRINTF(HWPrefetch, "Adding context %i with stride entries at %p\n",
            context, it->second);

    StrideEntry** entry = it->second;
    for (int s = 0; s < pcTableSets; s++) {
        entry[s] = new StrideEntry[pcTableAssoc];
    }
    return entry;
}

StridePrefetcher::PCTable::~PCTable() {
    for (auto entry : entries) {
        for (int s = 0; s < pcTableSets; s++) {
            delete[] entry.second[s];
        }
        delete[] entry.second;
    }
}

void
StridePrefetcher::calculatePrefetch(const PacketPtr &pkt,
                                    std::vector<Addr> &addresses)
{
    if (!pkt->req->hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }

    // Get required packet info
    Addr pkt_addr = pkt->getAddr();
    Addr pc = pkt->req->getPC();
    bool is_secure = pkt->isSecure();
    MasterID master_id = useMasterId ? pkt->req->masterId() : 0;

    // Lookup pc-based information
    StrideEntry *entry;

    if(pcTableHit(pc, is_secure, master_id, entry)) {
        // Hit in table
        int new_stride = pkt_addr - entry->lastAddr;
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
                "conf %d\n", pc, pkt_addr, is_secure ? "s" : "ns", new_stride,
                stride_match ? "match" : "change",
                entry->confidence);

        entry->lastAddr = pkt_addr;

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

            Addr new_addr = pkt_addr + d * prefetch_stride;
            if (samePage(pkt_addr, new_addr)) {
                DPRINTF(HWPrefetch, "Queuing prefetch to %#x.\n", new_addr);
                addresses.push_back(new_addr);
            } else {
                // Record the number of page crossing prefetches generated
                pfSpanPage += degree - d + 1;
                DPRINTF(HWPrefetch, "Ignoring page crossing prefetch.\n");
                return;
            }
        }
    } else {
        // Miss in table
        DPRINTF(HWPrefetch, "Miss: PC %x pkt_addr %x (%s)\n", pc, pkt_addr,
                is_secure ? "s" : "ns");

        StrideEntry* entry = pcTableVictim(pc, master_id);
        entry->instAddr = pc;
        entry->lastAddr = pkt_addr;
        entry->isSecure= is_secure;
        entry->stride = 0;
        entry->confidence = startConf;
    }
}

inline Addr
StridePrefetcher::pcHash(Addr pc) const
{
    Addr hash1 = pc >> 1;
    Addr hash2 = hash1 >> floorLog2(pcTableSets);
    return (hash1 ^ hash2) & (Addr)(pcTableSets - 1);
}

inline StridePrefetcher::StrideEntry*
StridePrefetcher::pcTableVictim(Addr pc, int master_id)
{
    // Rand replacement for now
    int set = pcHash(pc);
    int way = random_mt.random<int>(0, pcTableAssoc - 1);

    DPRINTF(HWPrefetch, "Victimizing lookup table[%d][%d].\n", set, way);
    return &pcTable[master_id][set][way];
}

inline bool
StridePrefetcher::pcTableHit(Addr pc, bool is_secure, int master_id,
                             StrideEntry* &entry)
{
    int set = pcHash(pc);
    StrideEntry* set_entries = pcTable[master_id][set];
    for (int way = 0; way < pcTableAssoc; way++) {
        // Search ways for match
        if (set_entries[way].instAddr == pc &&
            set_entries[way].isSecure == is_secure) {
            DPRINTF(HWPrefetch, "Lookup hit table[%d][%d].\n", set, way);
            entry = &set_entries[way];
            return true;
        }
    }
    return false;
}

StridePrefetcher*
StridePrefetcherParams::create()
{
    return new StridePrefetcher(this);
}
