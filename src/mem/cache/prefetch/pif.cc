/**
 * Copyright (c) 2019 Metempsy Technology Consulting
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

#include "mem/cache/prefetch/pif.hh"

#include <utility>

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/PIFPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

PIF::PIF(const PIFPrefetcherParams &p)
    : Queued(p),
      precSize(p.prec_spatial_region_bits),
      succSize(p.succ_spatial_region_bits),
      maxCompactorEntries(p.compactor_entries),
      historyBuffer(p.history_buffer_size),
      index(p.index_assoc, p.index_entries, p.index_indexing_policy,
            p.index_replacement_policy),
      streamAddressBuffer(p.stream_address_buffer_entries),
      listenersPC()
{
}

PIF::CompactorEntry::CompactorEntry(Addr addr,
    unsigned int prec_size, unsigned int succ_size)
{
    trigger = addr;
    prec.resize(prec_size, false);
    succ.resize(succ_size, false);
}

Addr
PIF::CompactorEntry::distanceFromTrigger(Addr target,
        unsigned int log_blk_size) const
{
    const Addr target_blk = target >> log_blk_size;
    const Addr trigger_blk = trigger >> log_blk_size;

    return target_blk > trigger_blk ?
              target_blk - trigger_blk : trigger_blk - target_blk;
}

bool
PIF::CompactorEntry::inSameSpatialRegion(Addr pc,
        unsigned int log_blk_size, bool update)
{
    Addr blk_distance = distanceFromTrigger(pc, log_blk_size);

    bool hit = (pc > trigger) ?
        (succ.size() > blk_distance) : (prec.size() > blk_distance);
    if (hit && update) {
        if (pc > trigger) {
            succ[blk_distance] = true;
        } else if (pc < trigger) {
            prec[blk_distance] = true;
        }
    }
    return hit;
}

bool
PIF::CompactorEntry::hasAddress(Addr target,
                                          unsigned int log_blk_size) const
{
    Addr blk_distance = distanceFromTrigger(target, log_blk_size);
    bool hit = false;
    if (target > trigger) {
        hit = blk_distance < succ.size() && succ[blk_distance];
    } else if (target < trigger) {
        hit = blk_distance < prec.size() && prec[blk_distance];
    } else {
        hit = true;
    }
    return hit;
}

void
PIF::CompactorEntry::getPredictedAddresses(unsigned int log_blk_size,
    std::vector<AddrPriority> &addresses) const
{
    // Calculate the addresses of the instruction blocks that are encoded
    // by the bit vector and issue prefetch requests for these addresses.
    // Predictions are made by traversing the bit vector from left to right
    // as this typically predicts the accesses in the order they will be
    // issued in the core.
    const Addr trigger_blk = trigger >> log_blk_size;
    for (int i = prec.size()-1; i >= 0; i--) {
        // Address from the preceding blocks to issue a prefetch
        if (prec[i]) {
            const Addr prec_addr = (trigger_blk - (i+1)) << log_blk_size;
            addresses.push_back(AddrPriority(prec_addr, 0));
        }
    }
    for (int i = 0; i < succ.size(); i++) {
        // Address from the succeding blocks to issue a prefetch
        if (succ[i]) {
            const Addr succ_addr = (trigger_blk + (i+1)) << log_blk_size;
            addresses.push_back(AddrPriority(succ_addr, 0));
        }
    }
}

void
PIF::notifyRetiredInst(const Addr pc)
{
    // First access to the prefetcher
    if (temporalCompactor.size() == 0) {
        spatialCompactor = CompactorEntry(pc, precSize, succSize);
        temporalCompactor.push_back(spatialCompactor);
    } else {
        // If the PC of the instruction retired is in the same spatial region
        // than the last trigger address, update the bit vectors based on the
        // distance between them
        if (spatialCompactor.inSameSpatialRegion(pc, lBlkSize, true)) {
        // If the PC of the instruction retired is outside the latest spatial
        // region, check if it matches in any of the regions in the temporal
        // compactor and update it to the MRU position
        } else {
            bool is_in_temporal_compactor = false;

            // Check if the PC is in the temporal compactor
            for (auto it = temporalCompactor.begin();
                    it != temporalCompactor.end(); it++)
            {
                if (it->inSameSpatialRegion(pc, lBlkSize, false)) {
                    spatialCompactor = (*it);
                    temporalCompactor.erase(it);
                    is_in_temporal_compactor = true;
                    break;
                }
            }

            if (temporalCompactor.size() == maxCompactorEntries) {
                temporalCompactor.pop_front(); // Discard the LRU entry
            }

            temporalCompactor.push_back(spatialCompactor);

            // If the compactor entry is neither the spatial or can't be
            // found in the temporal compactor, reset the spatial compactor
            // updating the trigger address and resetting the vector bits
            if (!is_in_temporal_compactor) {
                // Insert the spatial entry into the history buffer and update
                // the 'iterator' table to point to the new entry
                historyBuffer.push_back(spatialCompactor);

                IndexEntry *idx_entry =
                    index.findEntry(spatialCompactor.trigger, false);
                if (idx_entry != nullptr) {
                    index.accessEntry(idx_entry);
                } else {
                    idx_entry = index.findVictim(spatialCompactor.trigger);
                    assert(idx_entry != nullptr);
                    index.insertEntry(spatialCompactor.trigger, false,
                                      idx_entry);
                }
                idx_entry->historyIt =
                    historyBuffer.getIterator(historyBuffer.tail());

                // Reset the spatial compactor fields with the new address
                spatialCompactor = CompactorEntry(pc, precSize, succSize);
            }
        }
    }
}

void
PIF::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses,
    const CacheAccessor &cache)
{
    if (!pfi.hasPC()) {
        return;
    }

    const Addr pc = pfi.getPC();

    // First check if the access has been prefetched, this is done by
    // comparing the access against the active Stream Address Buffers
    for (auto &sabEntry : streamAddressBuffer) {
        if (sabEntry->hasAddress(pc, lBlkSize)) {
            sabEntry++;
            sabEntry->getPredictedAddresses(lBlkSize, addresses);
            // We are done
            return;
        }
    }

    // Check if a valid entry in the 'index' table is found and allocate a new
    // active prediction stream
    IndexEntry *idx_entry = index.findEntry(pc, /* unused */ false);

    if (idx_entry != nullptr) {
        index.accessEntry(idx_entry);
        // Trigger address from the 'index' table and index to the history
        // buffer
        auto entry = idx_entry->historyIt;

        // Track the block in the Stream Address Buffer
        streamAddressBuffer.push_back(entry);

        entry->getPredictedAddresses(lBlkSize, addresses);
    }
}

void
PIF::PrefetchListenerPC::notify(const Addr& pc)
{
    parent.notifyRetiredInst(pc);
}

void
PIF::addEventProbeRetiredInsts(SimObject *obj, const char *name)
{
    ProbeManager *pm(obj->getProbeManager());
    listenersPC.push_back(new PrefetchListenerPC(*this, pm, name));
}

} // namespace prefetch
} // namespace gem5
