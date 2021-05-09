/**
 * Copyright (c) 2018 Metempsy Technology Consulting
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

#include "mem/cache/prefetch/access_map_pattern_matching.hh"

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/AMPMPrefetcher.hh"
#include "params/AccessMapPatternMatching.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

AccessMapPatternMatching::AccessMapPatternMatching(
    const AccessMapPatternMatchingParams &p)
    : ClockedObject(p), blkSize(p.block_size), limitStride(p.limit_stride),
      startDegree(p.start_degree), hotZoneSize(p.hot_zone_size),
      highCoverageThreshold(p.high_coverage_threshold),
      lowCoverageThreshold(p.low_coverage_threshold),
      highAccuracyThreshold(p.high_accuracy_threshold),
      lowAccuracyThreshold(p.low_accuracy_threshold),
      highCacheHitThreshold(p.high_cache_hit_threshold),
      lowCacheHitThreshold(p.low_cache_hit_threshold),
      epochCycles(p.epoch_cycles),
      offChipMemoryLatency(p.offchip_memory_latency),
      accessMapTable(p.access_map_table_assoc, p.access_map_table_entries,
                     p.access_map_table_indexing_policy,
                     p.access_map_table_replacement_policy,
                     AccessMapEntry(hotZoneSize / blkSize)),
      numGoodPrefetches(0), numTotalPrefetches(0), numRawCacheMisses(0),
      numRawCacheHits(0), degree(startDegree), usefulDegree(startDegree),
      epochEvent([this]{ processEpochEvent(); }, name())
{
    fatal_if(!isPowerOf2(hotZoneSize),
        "the hot zone size must be a power of 2");
}

void
AccessMapPatternMatching::startup()
{
    schedule(epochEvent, clockEdge(epochCycles));
}

void
AccessMapPatternMatching::processEpochEvent()
{
    schedule(epochEvent, clockEdge(epochCycles));
    double prefetch_accuracy =
        ((double) numGoodPrefetches) / ((double) numTotalPrefetches);
    double prefetch_coverage =
        ((double) numGoodPrefetches) / ((double) numRawCacheMisses);
    double cache_hit_ratio = ((double) numRawCacheHits) /
        ((double) (numRawCacheHits + numRawCacheMisses));
    double num_requests = (double) (numRawCacheMisses - numGoodPrefetches +
        numTotalPrefetches);
    double memory_bandwidth = num_requests * offChipMemoryLatency /
        cyclesToTicks(epochCycles);

    if (prefetch_coverage > highCoverageThreshold &&
        (prefetch_accuracy > highAccuracyThreshold ||
        cache_hit_ratio < lowCacheHitThreshold)) {
        usefulDegree += 1;
    } else if ((prefetch_coverage < lowCoverageThreshold &&
               (prefetch_accuracy < lowAccuracyThreshold ||
                cache_hit_ratio > highCacheHitThreshold)) ||
               (prefetch_accuracy < lowAccuracyThreshold &&
                cache_hit_ratio > highCacheHitThreshold)) {
        usefulDegree -= 1;
    }
    degree = std::min((unsigned) memory_bandwidth, usefulDegree);
    // reset epoch stats
    numGoodPrefetches = 0.0;
    numTotalPrefetches = 0.0;
    numRawCacheMisses = 0.0;
    numRawCacheHits = 0.0;
}

AccessMapPatternMatching::AccessMapEntry *
AccessMapPatternMatching::getAccessMapEntry(Addr am_addr,
                bool is_secure)
{
    AccessMapEntry *am_entry = accessMapTable.findEntry(am_addr, is_secure);
    if (am_entry != nullptr) {
        accessMapTable.accessEntry(am_entry);
    } else {
        am_entry = accessMapTable.findVictim(am_addr);
        assert(am_entry != nullptr);

        accessMapTable.insertEntry(am_addr, is_secure, am_entry);
    }
    return am_entry;
}

void
AccessMapPatternMatching::setEntryState(AccessMapEntry &entry,
    Addr block, enum AccessMapState state)
{
    enum AccessMapState old = entry.states[block];
    entry.states[block] = state;

    //do not update stats when initializing
    if (state == AM_INIT) return;

    switch (old) {
        case AM_INIT:
            if (state == AM_PREFETCH) {
                numTotalPrefetches += 1;
            } else if (state == AM_ACCESS) {
                numRawCacheMisses += 1;
            }
            break;
        case AM_PREFETCH:
            if (state == AM_ACCESS) {
                numGoodPrefetches += 1;
                numRawCacheMisses += 1;
            }
            break;
        case AM_ACCESS:
            if (state == AM_ACCESS) {
                numRawCacheHits += 1;
            }
            break;
        default:
            panic("Impossible path\n");
            break;
    }
}

void
AccessMapPatternMatching::calculatePrefetch(const Base::PrefetchInfo &pfi,
    std::vector<Queued::AddrPriority> &addresses)
{
    assert(addresses.empty());

    bool is_secure = pfi.isSecure();
    Addr am_addr = pfi.getAddr() / hotZoneSize;
    Addr current_block = (pfi.getAddr() % hotZoneSize) / blkSize;
    uint64_t lines_per_zone = hotZoneSize / blkSize;

    // Get the entries of the curent block (am_addr), the previous, and the
    // following ones
    AccessMapEntry *am_entry_curr = getAccessMapEntry(am_addr, is_secure);
    AccessMapEntry *am_entry_prev = (am_addr > 0) ?
        getAccessMapEntry(am_addr-1, is_secure) : nullptr;
    AccessMapEntry *am_entry_next = (am_addr < (MaxAddr/hotZoneSize)) ?
        getAccessMapEntry(am_addr+1, is_secure) : nullptr;
    assert(am_entry_curr != am_entry_prev);
    assert(am_entry_curr != am_entry_next);
    assert(am_entry_prev != am_entry_next);
    assert(am_entry_curr != nullptr);

    //Mark the current access as Accessed
    setEntryState(*am_entry_curr, current_block, AM_ACCESS);

    /**
     * Create a contiguous copy of the 3 entries states.
     * With this, we avoid doing boundaries checking in the loop that looks
     * for prefetch candidates, mark out of range positions with AM_INVALID
     */
    std::vector<AccessMapState> states(3 * lines_per_zone);
    for (unsigned idx = 0; idx < lines_per_zone; idx += 1) {
        states[idx] =
            am_entry_prev != nullptr ? am_entry_prev->states[idx] : AM_INVALID;
        states[idx + lines_per_zone] = am_entry_curr->states[idx];
        states[idx + 2 * lines_per_zone] =
            am_entry_next != nullptr ? am_entry_next->states[idx] : AM_INVALID;
    }

    /**
     * am_entry_prev->states => states[               0 ..   lines_per_zone-1]
     * am_entry_curr->states => states[  lines_per_zone .. 2*lines_per_zone-1]
     * am_entry_next->states => states[2*lines_per_zone .. 3*lines_per_zone-1]
     */

    // index of the current_block in the new vector
    Addr states_current_block = current_block + lines_per_zone;
    // consider strides 1..lines_per_zone/2
    int max_stride = limitStride == 0 ? lines_per_zone / 2 : limitStride + 1;
    for (int stride = 1; stride < max_stride; stride += 1) {
        // Test accessed positive strides
        if (checkCandidate(states, states_current_block, stride)) {
            // candidate found, current_block - stride
            Addr pf_addr;
            if (stride > current_block) {
                // The index (current_block - stride) falls in the range of
                // the previous zone (am_entry_prev), adjust the address
                // accordingly
                Addr blk = states_current_block - stride;
                pf_addr = (am_addr - 1) * hotZoneSize + blk * blkSize;
                setEntryState(*am_entry_prev, blk, AM_PREFETCH);
            } else {
                // The index (current_block - stride) falls within
                // am_entry_curr
                Addr blk = current_block - stride;
                pf_addr = am_addr * hotZoneSize + blk * blkSize;
                setEntryState(*am_entry_curr, blk, AM_PREFETCH);
            }
            addresses.push_back(Queued::AddrPriority(pf_addr, 0));
            if (addresses.size() == degree) {
                break;
            }
        }

        // Test accessed negative strides
        if (checkCandidate(states, states_current_block, -stride)) {
            // candidate found, current_block + stride
            Addr pf_addr;
            if (current_block + stride >= lines_per_zone) {
                // The index (current_block + stride) falls in the range of
                // the next zone (am_entry_next), adjust the address
                // accordingly
                Addr blk = (states_current_block + stride) % lines_per_zone;
                pf_addr = (am_addr + 1) * hotZoneSize + blk * blkSize;
                setEntryState(*am_entry_next, blk, AM_PREFETCH);
            } else {
                // The index (current_block + stride) falls within
                // am_entry_curr
                Addr blk = current_block + stride;
                pf_addr = am_addr * hotZoneSize + blk * blkSize;
                setEntryState(*am_entry_curr, blk, AM_PREFETCH);
            }
            addresses.push_back(Queued::AddrPriority(pf_addr, 0));
            if (addresses.size() == degree) {
                break;
            }
        }
    }
}

AMPM::AMPM(const AMPMPrefetcherParams &p)
  : Queued(p), ampm(*p.ampm)
{
}

void
AMPM::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses)
{
    ampm.calculatePrefetch(pfi, addresses);
}

} // namespace prefetch
} // namespace gem5
