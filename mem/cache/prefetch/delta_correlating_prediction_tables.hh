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

#ifndef __MEM_CACHE_PREFETCH_DELTA_CORRELATING_PREDICTION_TABLES_HH_
#define __MEM_CACHE_PREFETCH_DELTA_CORRELATING_PREDICTION_TABLES_HH_

#include "base/circular_queue.hh"
#include "mem/cache/prefetch/associative_set.hh"
#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct DeltaCorrelatingPredictionTablesParams;
struct DCPTPrefetcherParams;

namespace prefetch
{

/**
 * Delta Correlating Prediction Tables Prefetcher
 * References:
 *   Multi-level hardware prefetching using low complexity delta correlating
 *   prediction tables with partial matching.
 *   Marius Grannaes, Magnus Jahre, and Lasse Natvig. 2010.
 *   In Proceedings of the 5th international conference on High Performance
 *   Embedded Architectures and Compilers (HiPEAC'10)
 *
 * The filter feature is not implemented as gem5 already drops redundant
 * prefetches.
 * The main prefetcher logic is implemented on a separate SimObject as there
 * are other prefetcher that can rehuse this component.
 */

class DeltaCorrelatingPredictionTables : public SimObject
{
    /** Number of bits of each delta */
    const unsigned int deltaBits;
    /** Number of lower bits to ignore from the deltas */
    const unsigned int deltaMaskBits;

    /** DCPT Table entry datatype */
    struct DCPTEntry : public TaggedEntry
    {
        /** Last accessed address */
        Addr lastAddress;
        /** Stored deltas */
        CircularQueue<Addr> deltas;

        /**
         * Constructor
         * @param num_deltas number of deltas stored in the entry
         */
        DCPTEntry(unsigned int num_deltas)
          : TaggedEntry(), lastAddress(0), deltas(num_deltas)
        {
        }

        void invalidate() override;

        /**
         * Adds an address to the entry, if the entry already existed, a delta
         * will be generated
         * @param address address to add
         * @param delta_num_bits number of bits of the delta
         */
        void addAddress(Addr address, unsigned int delta_num_bits);

        /**
         * Attempt to generate prefetch candidates using the two most recent
         * deltas. Prefetch candidates are added to the provided vector.
         * @param pfs reference to a vector where candidates will be added
         * @param mask_bits the number of lower bits that should be masked
         *        (ignored) when comparing deltas
         */
        void getCandidates(std::vector<Queued::AddrPriority> &pfs,
                           unsigned int mask_bits) const;

    };
    /** The main table */
    AssociativeSet<DCPTEntry> table;

  public:
    DeltaCorrelatingPredictionTables(
        const DeltaCorrelatingPredictionTablesParams &p);
    ~DeltaCorrelatingPredictionTables() = default;

    /**
     * Computes the prefetch candidates given a prefetch event.
     * @param pfi The prefetch event information
     * @param addresses prefetch candidates generated
     */
    void calculatePrefetch(const Base::PrefetchInfo &pfi,
        std::vector<Queued::AddrPriority> &addresses);

};

/** The prefetcher object using the DCPT */
class DCPT : public Queued
{
    /** DCPT object */
    DeltaCorrelatingPredictionTables &dcpt;
  public:
    DCPT(const DCPTPrefetcherParams &p);
    ~DCPT() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
        std::vector<AddrPriority> &addresses) override;
};

} // namespace prefetch
} // namespace gem5

#endif//__MEM_CACHE_PREFETCH_DELTA_CORRELATING_PREDICTION_TABLES_HH_
