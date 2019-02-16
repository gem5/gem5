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
 *
 * Authors: Javier Bueno
 */

#include "mem/cache/prefetch/delta_correlating_prediction_tables.hh"

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/DCPTPrefetcher.hh"
#include "params/DeltaCorrelatingPredictionTables.hh"

DeltaCorrelatingPredictionTables::DeltaCorrelatingPredictionTables(
   DeltaCorrelatingPredictionTablesParams *p) : SimObject(p),
   deltaBits(p->delta_bits), deltaMaskBits(p->delta_mask_bits),
   table(p->table_assoc, p->table_entries, p->table_indexing_policy,
         p->table_replacement_policy, DCPTEntry(p->deltas_per_entry))
{
}

void
DeltaCorrelatingPredictionTables::DCPTEntry::reset()
{
    for (auto &delta : deltas) {
        delta = 0;
    }
    lastAddress = 0;
    deltaPointer = 0;
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
        deltas[deltaPointer] = delta;
        deltaPointer = (deltaPointer + 1) % deltas.size();
        lastAddress = address;
    }
}

void
DeltaCorrelatingPredictionTables::DCPTEntry::getCandidates(
    std::vector<QueuedPrefetcher::AddrPriority> &pfs, unsigned int mask) const
{
    // most recent index
    unsigned int last = (deltaPointer - 1) % deltas.size();
    // second most recent index
    unsigned int last_prev = (deltaPointer - 2) % deltas.size();
    int delta_0 = deltas[last_prev];
    int delta_1 = deltas[last];

    // a delta 0 means that it overflowed, we can not match it
    if (delta_0 == 0 || delta_1 == 0) {
        return;
    }

    // Try to find the two most recent deltas in a previous position on the
    // delta circular array, if found, start issuing prefetches using the
    // remaining deltas (adding each delta to the last Addr to generate the
    // prefetched address.

    // oldest index
    int idx_0 = deltaPointer + 1;
    // second oldest index
    int idx_1 = deltaPointer + 2;
    for (int i = 0; i < deltas.size() - 2; i += 1) {
        int this_delta_0 = deltas[(idx_0 + i) % deltas.size()];
        int this_delta_1 = deltas[(idx_1 + i) % deltas.size()];
        if ((this_delta_0 >> mask) == (delta_0 >> mask) &&
            (this_delta_1 >> mask) == (delta_1 >> mask)) {
            Addr addr = lastAddress;
            // Pattern found, issue prefetches with the remaining deltas after
            // this pair
            i += 2; // skip the matching pair
            do {
                int pf_delta = deltas[(idx_0 + i) % deltas.size()];
                addr += pf_delta;
                pfs.push_back(QueuedPrefetcher::AddrPriority(addr, 0));
                i += 1;
            } while (i < deltas.size() - 2);
        }
    }
}

void
DeltaCorrelatingPredictionTables::calculatePrefetch(
    const BasePrefetcher::PrefetchInfo &pfi,
    std::vector<QueuedPrefetcher::AddrPriority> &addresses)
{
    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }
    Addr address = pfi.getAddr();
    Addr pc = pfi.getPC();
    // Look up table entry, is_secure is unused in findEntry because we
    // index using the pc
    DCPTEntry *entry = table.findEntry(pc, false /* unused */);
    if (entry != nullptr) {
        entry->addAddress(address, deltaBits);
        //Delta correlating
        entry->getCandidates(addresses, deltaMaskBits);
    } else {
        entry = table.findVictim(pc);

        table.insertEntry(pc, false /* unused */, entry);

        entry->lastAddress = address;
    }
}

DeltaCorrelatingPredictionTables *
DeltaCorrelatingPredictionTablesParams::create()
{
   return new DeltaCorrelatingPredictionTables(this);
}

DCPTPrefetcher::DCPTPrefetcher(const DCPTPrefetcherParams *p)
  : QueuedPrefetcher(p), dcpt(*p->dcpt)
{
}

void
DCPTPrefetcher::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses)
{
    dcpt.calculatePrefetch(pfi, addresses);
}

DCPTPrefetcher*
DCPTPrefetcherParams::create()
{
    return new DCPTPrefetcher(this);
}
