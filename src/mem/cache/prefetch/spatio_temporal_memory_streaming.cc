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

#include "mem/cache/prefetch/spatio_temporal_memory_streaming.hh"

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/STeMSPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

STeMS::STeMS(const STeMSPrefetcherParams &p)
  : Queued(p), spatialRegionSize(p.spatial_region_size),
    spatialRegionSizeBits(floorLog2(p.spatial_region_size)),
    reconstructionEntries(p.reconstruction_entries),
    activeGenerationTable("ActiveGenerationTable",
                          p.active_generation_table_entries,
			  p.active_generation_table_assoc,
                          p.active_generation_table_replacement_policy,
                          p.active_generation_table_indexing_policy,
                          ActiveGenerationTableEntry(
                              spatialRegionSize / blkSize)),
    patternSequenceTable("PatternSequenceTable",
                         p.pattern_sequence_table_entries,
			 p.pattern_sequence_table_assoc,
                         p.pattern_sequence_table_replacement_policy,
                         p.pattern_sequence_table_indexing_policy,
                         ActiveGenerationTableEntry(
                             spatialRegionSize / blkSize)),
    rmob(p.region_miss_order_buffer_entries),
    addDuplicateEntriesToRMOB(p.add_duplicate_entries_to_rmob),
    lastTriggerCounter(0)
{
    fatal_if(!isPowerOf2(spatialRegionSize),
        "The spatial region size must be a power of 2.");
}

void
STeMS::checkForActiveGenerationsEnd(const CacheAccessor &cache)
{
    // This prefetcher operates attached to the L1 and it observes all
    // accesses, this guarantees that no evictions are missed

    // Iterate over all entries, if any recorded cacheline has been evicted,
    // the generation finishes, move the entry to the PST
    for (auto &agt_entry : activeGenerationTable) {
        if (agt_entry.isValid()) {
            bool generation_ended = false;
            bool sr_is_secure = agt_entry.isSecure();
            Addr pst_addr = 0;
            for (auto &seq_entry : agt_entry.sequence) {
                if (seq_entry.counter > 0) {
                    Addr cache_addr =
                        agt_entry.paddress + seq_entry.offset * blkSize;
                    if (!cache.inCache(cache_addr, sr_is_secure) &&
                            !cache.inMissQueue(cache_addr, sr_is_secure)) {
                        generation_ended = true;
                        pst_addr = (agt_entry.pc << spatialRegionSizeBits)
                                    + seq_entry.offset;
                        break;
                    }
                }
            }
            if (generation_ended) {
                // PST is indexed using the PC (secure bit is unused)
                ActiveGenerationTableEntry *pst_entry =
                    patternSequenceTable.findEntry(pst_addr,
                                                   false /*unused*/);
                if (pst_entry == nullptr) {
                    // Tipically an entry will not exist
                    pst_entry = patternSequenceTable.findVictim(pst_addr);
                    assert(pst_entry != nullptr);
                    patternSequenceTable.insertEntry(pst_addr,
                            false /*unused*/, pst_entry);
                } else {
                    patternSequenceTable.accessEntry(pst_entry);
                }
                // If the entry existed, this will update the values, if not,
                // this also sets the values of the entry
                pst_entry->update(agt_entry);
                // Free the AGT entry
                activeGenerationTable.invalidate(&agt_entry);
            }
        }
    }
}

void
STeMS::addToRMOB(Addr sr_addr, Addr pst_addr, unsigned int delta)
{
    RegionMissOrderBufferEntry rmob_entry;
    rmob_entry.srAddress = sr_addr;
    rmob_entry.pstAddress = pst_addr;
    rmob_entry.delta = delta;

    if (!addDuplicateEntriesToRMOB) {
        for (const auto& entry : rmob) {
            if (entry.srAddress == sr_addr &&
                entry.pstAddress == pst_addr &&
                entry.delta == delta) {
                return;
            }
        }
    }

    rmob.push_back(rmob_entry);
}

void
STeMS::calculatePrefetch(const PrefetchInfo &pfi,
                                   std::vector<AddrPriority> &addresses,
                                   const CacheAccessor &cache)
{
    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }

    Addr pc = pfi.getPC();
    bool is_secure = pfi.isSecure();
    // Spatial region address
    Addr sr_addr = pfi.getAddr() / spatialRegionSize;
    Addr paddr = pfi.getPaddr();

    // Offset in cachelines within the spatial region
    Addr sr_offset = (pfi.getAddr() % spatialRegionSize) / blkSize;

    // Check if any active generation has ended
    checkForActiveGenerationsEnd(cache);

    ActiveGenerationTableEntry *agt_entry =
        activeGenerationTable.findEntry(sr_addr, is_secure);
    if (agt_entry != nullptr) {
        // found an entry in the AGT, entry is currently being recorded,
        // add the offset
        activeGenerationTable.accessEntry(agt_entry);
        agt_entry->addOffset(sr_offset);
        lastTriggerCounter += 1;
    } else {
        // Not found, this is the first access (Trigger access)

        // Add entry to RMOB
        Addr pst_addr = (pc << spatialRegionSizeBits) + sr_offset;
        addToRMOB(sr_addr, pst_addr, lastTriggerCounter);
        // Reset last trigger counter
        lastTriggerCounter = 0;

        // allocate a new AGT entry
        agt_entry = activeGenerationTable.findVictim(sr_addr);
        assert(agt_entry != nullptr);
        activeGenerationTable.insertEntry(sr_addr, is_secure, agt_entry);
        agt_entry->pc = pc;
        agt_entry->paddress = paddr;
        agt_entry->addOffset(sr_offset);
    }
    // increase the seq Counter for other entries
    for (auto &agt_e : activeGenerationTable) {
        if (agt_e.isValid() && agt_entry != &agt_e) {
            agt_e.seqCounter += 1;
        }
    }

    // Prefetch generation: if this is a miss, search for the most recent
    // entry in the RMOB, and reconstruct the registered access sequence
    if (pfi.isCacheMiss()) {
        auto it = rmob.end();
        while (it != rmob.begin()) {
            --it;
            if (it->srAddress == sr_addr) {
                // reconstruct the access sequence
                reconstructSequence(it, addresses);
                break;
            }
        }
    }
}

void
STeMS::reconstructSequence(
    CircularQueue<RegionMissOrderBufferEntry>::iterator rmob_it,
    std::vector<AddrPriority> &addresses)
{
    std::vector<Addr> reconstruction(reconstructionEntries, MaxAddr);
    unsigned int idx = 0;

    // Process rmob entries from rmob_it (most recent with address = sr_addr)
    // to the latest one
    for (auto it = rmob_it; it != rmob.end() && (idx < reconstructionEntries);
        it++) {
        reconstruction[idx] = it->srAddress * spatialRegionSize;
        idx += (it+1)->delta + 1;
    }

    // Now query the PST with the PC of each RMOB entry
    idx = 0;
    for (auto it = rmob_it; it != rmob.end() && (idx < reconstructionEntries);
        it++) {
        ActiveGenerationTableEntry *pst_entry =
            patternSequenceTable.findEntry(it->pstAddress, false /* unused */);
        if (pst_entry != nullptr) {
            patternSequenceTable.accessEntry(pst_entry);
            for (auto &seq_entry : pst_entry->sequence) {
                if (seq_entry.counter > 1) {
                    // 2-bit counter: high enough confidence with a
                    // value greater than 1
                    Addr rec_addr = it->srAddress * spatialRegionSize +
                        seq_entry.offset;
                    unsigned ridx = idx + seq_entry.delta;
                    // Try to use the corresponding position, if it has been
                    // already used, look the surrounding positions
                    if (ridx < reconstructionEntries &&
                        reconstruction[ridx] == MaxAddr) {
                        reconstruction[ridx] = rec_addr;
                    } else if ((ridx + 1) < reconstructionEntries &&
                        reconstruction[ridx + 1] == MaxAddr) {
                        reconstruction[ridx + 1] = rec_addr;
                    } else if ((ridx + 2) < reconstructionEntries &&
                        reconstruction[ridx + 2] == MaxAddr) {
                        reconstruction[ridx + 2] = rec_addr;
                    } else if ((ridx > 0) &&
                        ((ridx - 1) < reconstructionEntries) &&
                        reconstruction[ridx - 1] == MaxAddr) {
                        reconstruction[ridx - 1] = rec_addr;
                    } else if ((ridx > 1) &&
                        ((ridx - 2) < reconstructionEntries) &&
                        reconstruction[ridx - 2] == MaxAddr) {
                        reconstruction[ridx - 2] = rec_addr;
                    }
                }
            }
        }
        idx += (it+1)->delta + 1;
    }

    for (Addr pf_addr : reconstruction) {
        if (pf_addr != MaxAddr) {
            addresses.push_back(AddrPriority(pf_addr, 0));
        }
    }
}

} // namespace prefetch
} // namespace gem5
