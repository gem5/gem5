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

 /**
  * Implementation of the Spatio-Temporal Memory Streaming Prefetcher (STeMS)
  * Reference:
  *    Spatio-temporal memory streaming.
  *    Somogyi, S., Wenisch, T. F., Ailamaki, A., & Falsafi, B. (2009).
  *    ACM SIGARCH Computer Architecture News, 37(3), 69-80.
  *
  * Notes:
  * - The functionality described in the paper as Streamed Value Buffer (SVB)
  *   is not implemented here, as this is handled by the QueuedPrefetcher class
  */

#ifndef __MEM_CACHE_PREFETCH_SPATIO_TEMPORAL_MEMORY_STREAMING_HH__
#define __MEM_CACHE_PREFETCH_SPATIO_TEMPORAL_MEMORY_STREAMING_HH__

#include <vector>

#include "base/circular_queue.hh"
#include "base/sat_counter.hh"
#include "mem/cache/prefetch/associative_set.hh"
#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct STeMSPrefetcherParams;

namespace prefetch
{

class STeMS : public Queued
{
    /** Size of each spatial region */
    const size_t spatialRegionSize;
    /** log_2 of the spatial region size */
    const size_t spatialRegionSizeBits;
    /** Number of reconstruction entries */
    const unsigned int reconstructionEntries;

    /**
     * Entry data type for the Active Generation Table (AGT) and the Pattern
     * Sequence Table (PST)
     */
    struct ActiveGenerationTableEntry : public TaggedEntry
    {
        /** Physical address of the spatial region */
        Addr paddress;
        /** PC that started this generation */
        Addr pc;
        /** Counter to keep track of the interleaving between sequences */
        unsigned int seqCounter;

        /** Sequence entry data type */
        struct SequenceEntry
        {
            /** 2-bit confidence counter */
            SatCounter8 counter;
            /** Offset, in cache lines, within the spatial region */
            unsigned int offset;
            /** Intearleaving position on the global access sequence */
            unsigned int delta;
            SequenceEntry() : counter(2), offset(0), delta(0)
            {}
        };
        /** Sequence of accesses */
        std::vector<SequenceEntry> sequence;

        ActiveGenerationTableEntry(int num_positions)
          : TaggedEntry(), paddress(0), pc(0),
            seqCounter(0), sequence(num_positions)
        {
        }

        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            paddress = 0;
            pc = 0;
            seqCounter = 0;
            for (auto &seq_entry : sequence) {
                seq_entry.counter.reset();
                seq_entry.offset = 0;
                seq_entry.delta = 0;
            }
        }

        /**
         * Update the entry data with an entry from a generation that just
         * ended. This operation can not be done with the copy constructor,
         * becasuse the TaggedEntry component must not be copied.
         * @param e entry which generation has ended
         */
        void update(ActiveGenerationTableEntry const &e)
        {
            paddress = e.paddress;
            pc = e.pc;
            seqCounter = e.seqCounter;
            sequence = e.sequence;
        }

        /**
         * Add a new access to the sequence
         * @param offset offset in cachelines within the spatial region
         */
        void addOffset(unsigned int offset) {
            // Search for the offset in the deltas array, if it exist, update
            // the corresponding counter, if not, add the offset to the array
            for (auto &seq_entry : sequence) {
                if (seq_entry.counter > 0) {
                    if (seq_entry.offset == offset) {
                        seq_entry.counter++;
                    }
                } else {
                    // If the counter is 0 it means that this position is not
                    // being used, and we can allocate the new offset here
                    seq_entry.counter++;
                    seq_entry.offset = offset;
                    seq_entry.delta = seqCounter;
                    break;
                }
            }
            seqCounter = 0;
        }
    };

    /** Active Generation Table (AGT) */
    AssociativeSet<ActiveGenerationTableEntry> activeGenerationTable;
    /** Pattern Sequence Table (PST) */
    AssociativeCache<ActiveGenerationTableEntry> patternSequenceTable;

    /** Data type of the Region Miss Order Buffer entry */
    struct RegionMissOrderBufferEntry
    {
        /** Address of the spatial region */
        Addr srAddress;
        /**
         * Address used to index the PST table, generated using the PC and the
         * offset within the spatial region
         */
        Addr pstAddress;
        /** Delta within the global miss order sequence */
        unsigned int delta;
    };

    /** Region Miss Order Buffer (RMOB) */
    CircularQueue<RegionMissOrderBufferEntry> rmob;

    /** Add duplicate entries to RMOB  */
    bool addDuplicateEntriesToRMOB;

    /** Counter to keep the count of accesses between trigger accesses */
    unsigned int lastTriggerCounter;

    /** Checks if the active generations have ended */
    void checkForActiveGenerationsEnd(const CacheAccessor &cache);
    /**
     * Adds an entry to the RMOB
     * @param sr_addr Spatial region address
     * @param pst_addr Corresponding PST address
     * @param delta Number of entries skipped in the global miss order
     */
    void addToRMOB(Addr sr_addr, Addr pst_addr, unsigned int delta);

    /**
     * Reconstructs a sequence of accesses and generates the prefetch
     * addresses, adding them to the addresses vector
     *
     * @param rmob_it rmob position to start generating from.
     * @param addresses vector to add the addresses to be prefetched
     */
    void reconstructSequence(
        CircularQueue<RegionMissOrderBufferEntry>::iterator rmob_it,
        std::vector<AddrPriority> &addresses);

  public:
    STeMS(const STeMSPrefetcherParams &p);
    ~STeMS() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif//__MEM_CACHE_PREFETCH_SPATIO_TEMPORAL_MEMORY_STREAMING_HH__
