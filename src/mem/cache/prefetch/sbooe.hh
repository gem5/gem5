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

/**
 * Implementation of the 'Sandbox Based Optimal Offset Estimation'
 * Reference:
 *   Brown, N. T., & Sendag, R. Sandbox Based Optimal Offset Estimation.
*/

#ifndef __MEM_CACHE_PREFETCH_SBOOE_HH__
#define __MEM_CACHE_PREFETCH_SBOOE_HH__

#include <unordered_map>
#include <vector>

#include "base/circular_queue.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/packet.hh"

struct SBOOEPrefetcherParams;

namespace Prefetcher {

class SBOOE : public Queued
{
    private:

        /** Prefetcher parameters */
        const int sequentialPrefetchers;

        /** Threshold used to issue prefetchers */
        const unsigned int scoreThreshold;

        /**
         * Holds the current demand addresses and tick. This is later used to
         * calculate the average latency buffer when the address is filled in
         * the cache.
         */
        std::unordered_map<Addr, Tick> demandAddresses;

        /**
         * The latency buffer holds the elapsed ticks between the demand and
         * the fill in the cache for the latest N accesses which are used to
         * calculate the average access latency which is later used to
         * predict if a prefetcher would be filled on time if issued.
         */
        CircularQueue<Tick> latencyBuffer;

        /** Holds the current average access latency */
        Tick averageAccessLatency;

        /** Holds the current sum of the latency buffer latency */
        Tick latencyBufferSum;

        struct SandboxEntry {
            /** Cache line predicted by the candidate prefetcher */
            Addr line;
            /** Tick when the simulated prefetch is expected to be filled */
            Tick expectedArrivalTick;
            /** To indicate if it was initialized */
            bool valid;

            SandboxEntry()
                : valid(false)
            {}
        };

        class Sandbox
        {
          private:
            /** FIFO queue containing the sandbox entries. */
            CircularQueue<SandboxEntry> entries;

            /**
             * Accesses during the eval period that were present
             * in the sandbox
             */
            unsigned int sandboxScore;

            /** Hits in the sandbox that wouldn't have been filled on time */
            unsigned int lateScore;

          public:
            /** Sequential stride for this prefetcher */
            const int stride;

            Sandbox(unsigned int max_entries, int _stride)
              : entries(max_entries), sandboxScore(0), lateScore(0),
                stride(_stride)
            {
            }

            /**
             * Update score and insert the line address being accessed into the
             * FIFO queue of the sandbox.
             *
             * @param line Line address being accessed
             * @param tick Tick in which the access is expected to be filled
             */
            void access(Addr line, Tick tick);

            /** Calculate the useful score
             *  @return Useful score of the sandbox. Sandbox score adjusted by
             *          by the late score
             */
            unsigned int score() const { return (sandboxScore - lateScore); }
        };

        std::vector<Sandbox> sandboxes;

        /** Current best sandbox */
        const Sandbox* bestSandbox;

        /** Number of accesses notified to the prefetcher */
        unsigned int accesses;

        /**
         * Process an access to the specified line address and update the
         * sandbox counters counters.
         * @param line Address of the line being accessed
         * @return TRUE if the evaluation finished, FALSE otherwise
         */
        bool access(Addr line);

        /** Update the latency buffer after a prefetch fill */
        void notifyFill(const PacketPtr& pkt) override;

    public:
        SBOOE(const SBOOEPrefetcherParams *p);

        void calculatePrefetch(const PrefetchInfo &pfi,
                               std::vector<AddrPriority> &addresses) override;
};

} // namespace Prefetcher

#endif // __MEM_CACHE_PREFETCH_SBOOE_HH__
