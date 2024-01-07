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

#include "mem/cache/prefetch/delta_correlating_prediction_tables.hh"

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/DCPTPrefetcher.hh"
#include "params/DeltaCorrelatingPredictionTables.hh"

namespace gem5
{

namespace prefetch
{

DeltaCorrelatingPredictionTables::DeltaCorrelatingPredictionTables(
   const DeltaCorrelatingPredictionTablesParams &p) : SimObject(p),
   deltaBits(p.delta_bits), deltaMaskBits(p.delta_mask_bits),
   table("DCPT", p.table_entries, p.table_assoc, p.table_replacement_policy,
         p.table_indexing_policy, DCPTEntry(p.deltas_per_entry))
{
}

void
DeltaCorrelatingPredictionTables::DCPTEntry::invalidate()
{
    CacheEntry::invalidate();

    deltas.flush();
    while (!deltas.full()) {
        deltas.push_back(0);
    }
    lastAddress = 0;
}

void
DeltaCorrelatingPredictionTables::DCPTEntry::addAddress(Addr address,
    unsigned int delta_bits)
{
    if ((address - lastAddress) != 0) {
        Addr delta = address - lastAddress;
        // Account for the sign bit
        Addr max_positive_delta = (1 << (delta_bits-1)) - 1;
        if (address > lastAddress) {
            // check positive delta overflow
            if (delta > max_positive_delta) {
                delta = 0;
            }
        } else {
            // check negative delta overflow
            if (lastAddress - address > (max_positive_delta + 1)) {
                delta = 0;
            }
        }
        deltas.push_back(delta);
        lastAddress = address;
    }
}

void
DeltaCorrelatingPredictionTables::DCPTEntry::getCandidates(
    std::vector<Queued::AddrPriority> &pfs, unsigned int mask) const
{
    assert(deltas.full());

    // Get the two most recent deltas
    const int delta_penultimate = *(deltas.end() - 2);
    const int delta_last = *(deltas.end() - 1);

    // a delta 0 means that it overflowed, we can not match it
    if (delta_last == 0 || delta_penultimate == 0) {
        return;
    }

    // Try to find the two most recent deltas in a previous position on the
    // delta circular array, if found, start issuing prefetches using the
    // remaining deltas (adding each delta to the last Addr to generate the
    // prefetched address.
    auto it = deltas.begin();
    for (; it != (deltas.end() - 2); ++it) {
        const int prev_delta_penultimate = *it;
        const int prev_delta_last = *(it + 1);
        if ((prev_delta_penultimate >> mask) == (delta_penultimate >> mask) &&
            (prev_delta_last >> mask) == (delta_last >> mask)) {
            // Pattern found. Skip the matching pair and issue prefetches with
            // the remaining deltas
            it += 2;
            Addr addr = lastAddress;
            while (it != deltas.end()) {
                const int pf_delta = *(it++);
                addr += pf_delta;
                pfs.push_back(Queued::AddrPriority(addr, 0));
            }
            break;
        }
    }
}

void
DeltaCorrelatingPredictionTables::calculatePrefetch(
    const Base::PrefetchInfo &pfi,
    std::vector<Queued::AddrPriority> &addresses,
    const CacheAccessor &cache)
{
    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }
    Addr address = pfi.getAddr();
    Addr pc = pfi.getPC();
    // Look up table entry
    DCPTEntry *entry = table.findEntry(pc);
    if (entry != nullptr) {
        entry->addAddress(address, deltaBits);
        //Delta correlating
        entry->getCandidates(addresses, deltaMaskBits);
    } else {
        entry = table.findVictim(pc);

        table.insertEntry(pc, entry);

        entry->lastAddress = address;
    }
}

DCPT::DCPT(const DCPTPrefetcherParams &p)
  : Queued(p), dcpt(*p.dcpt)
{
}

void
DCPT::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses,
    const CacheAccessor &cache)
{
    dcpt.calculatePrefetch(pfi, addresses, cache);
}

} // namespace prefetch
} // namespace gem5
