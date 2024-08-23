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

/** Implementation of the 'Proactive Instruction Fetch' prefetcher
 *  Reference:
 *    Ferdman, M., Kaynak, C., & Falsafi, B. (2011, December).
 *    Proactive instruction fetch.
 *    In Proceedings of the 44th Annual IEEE/ACM International Symposium
 *    on Microarchitecture (pp. 152-162). ACM.
 */

#ifndef __MEM_CACHE_PREFETCH_PIF_HH__
#define __MEM_CACHE_PREFETCH_PIF_HH__

#include <deque>
#include <vector>

#include "base/cache/associative_cache.hh"
#include "base/circular_queue.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/tags/tagged_entry.hh"

namespace gem5
{

struct PIFPrefetcherParams;

namespace prefetch
{

class PIF : public Queued
{
    private:
        /** Number of preceding and subsequent spatial addresses to compact */
        const unsigned int precSize;
        const unsigned int succSize;
        /** Number of entries used for the temporal compactor */
        const unsigned int maxCompactorEntries;

        /**
         * The compactor tracks retired instructions addresses, leveraging the
         * spatial and temporal locality among instructions for compaction. It
         *comprises the spatial and temporal compaction mechanisms.
         *
         * Taking advantage of the spatial locality across instruction blocks,
         * the spatial compactor combines instruction-block addresses that fall
         * within a 'spatial region', a group of adjacent instruction blocks.
         * When an instruction outside the current spatial region retires, the
         * existing spatial region is sent to the temporal compactor.
         *
         * The temporal compactor tracks a small number of the
         * most-recently-observed spatial region records.
         */
        struct CompactorEntry
        {
            Addr trigger;
            std::vector<bool> prec;
            std::vector<bool> succ;
            CompactorEntry() {}
            CompactorEntry(Addr, unsigned int, unsigned int);

            /**
             * Checks if a given address is in the same defined spatial region
             * as the compactor entry.
             * @param addr Address to check if it's inside the spatial region
             * @param log_blk_distance log_2(block size of the cache)
             * @param update if true, set the corresponding succ/prec entry
             * @return TRUE if they are in the same spatial region, FALSE
             *   otherwise
             */
            bool inSameSpatialRegion(Addr addr, unsigned int log_blk_size,
                                     bool update);
            /**
             * Checks if the provided address is contained in this spatial
             * region and if its corresponding bit vector entry is set
             * @param target address to check
             * @param log_blk_distance log_2(block size of the cache)
             * @return TRUE if target has its bit set
             */
            bool hasAddress(Addr target, unsigned int log_blk_size) const;

            /**
             * Fills the provided vector with the predicted addresses using the
             * recorded bit vectors of the entry
             * @param log_blk_distance log_2(block size of the cache)
             * @param addresses reference to a vector to add the generated
             * addresses
             */
            void getPredictedAddresses(unsigned int log_blk_size,
                    std::vector<AddrPriority> &addresses) const;
          private:
            /**
             * Computes the distance, in cache blocks, from an address to the
             * trigger of the entry.
             * @param addr address to compute the distance from the trigger
             * @param log_blk_distance log_2(block size of the cache)
             * @result distance in cache blocks from the address to the trigger
             */
            Addr distanceFromTrigger(Addr addr,
                                     unsigned int log_blk_size) const;
        };

        CompactorEntry spatialCompactor;
        std::deque<CompactorEntry> temporalCompactor;

        /**
         * History buffer is a circular buffer that stores the sequence of
         * retired instructions in FIFO order.
         */
        using HistoryBuffer = CircularQueue<CompactorEntry>;
        HistoryBuffer historyBuffer;

        struct IndexEntry : public TaggedEntry
        {
            IndexEntry(TagExtractor ext)
              : TaggedEntry()
            {
                registerTagExtractor(ext);
            }

            HistoryBuffer::iterator historyIt;
        };

        /**
         * The index table is a small cache-like structure that facilitates
         * fast search of the history buffer.
         */
        AssociativeCache<IndexEntry> index;

        /**
         * A Stream Address Buffer (SAB) tracks a window of consecutive
         * spatial regions. The SAB mantains a pointer to the sequence in the
         * history buffer, initiallly set to the pointer taken from the index
         * table
         */
        CircularQueue<HistoryBuffer::iterator> streamAddressBuffer;

        /**
         * Updates the prefetcher structures upon an instruction retired
         * @param pc PC of the instruction being retired
         */
        void notifyRetiredInst(const Addr pc);

        /**
         * Probe Listener to handle probe events from the CPU
         */
        class PrefetchListenerPC : public ProbeListenerArgBase<Addr>
        {
          public:
            PrefetchListenerPC(PIF &_parent, ProbeManager *pm,
                             const std::string &name)
                : ProbeListenerArgBase(pm, name),
                  parent(_parent) {}
            void notify(const Addr& pc) override;
          protected:
            PIF &parent;
        };

        /** Array of probe listeners */
        std::vector<PrefetchListenerPC *> listenersPC;


    public:
        PIF(const PIFPrefetcherParams &p);
        ~PIF() = default;

        void calculatePrefetch(const PrefetchInfo &pfi,
                               std::vector<AddrPriority> &addresses,
                               const CacheAccessor &cache);

        /**
         * Add a SimObject and a probe name to monitor the retired instructions
         * @param obj The SimObject pointer to listen from
         * @param name The probe name
         */
        void addEventProbeRetiredInsts(SimObject *obj, const char *name);
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_PIF_HH__
