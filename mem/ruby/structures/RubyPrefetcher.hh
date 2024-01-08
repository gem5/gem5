/*
 * Copyright (c) 2020 Inria
 * Copyright (c) 2020 ARM Limited
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
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_STRUCTURES_PREFETCHER_HH__
#define __MEM_RUBY_STRUCTURES_PREFETCHER_HH__

// Implements Power 4 like prefetching

#include <bitset>

#include "base/circular_queue.hh"
#include "base/statistics.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/RubyPrefetcher.hh"
#include "sim/sim_object.hh"

#define MAX_PF_INFLIGHT 8

namespace gem5
{

namespace ruby
{

class PrefetchEntry
{
    public:
        /// constructor
        PrefetchEntry()
        {
            // default: 1 cache-line stride
            m_stride   = (1 << RubySystem::getBlockSizeBits());
            m_use_time = Cycles(0);
            m_is_valid = false;
        }

        //! The base address for the stream prefetch
        Addr m_address;

        //! stride distance to get next address from
        int m_stride;

        //! the last time that any prefetched request was used
        Cycles m_use_time;

        //! valid bit for each stream
        bool m_is_valid;

        //! L1D prefetches loads and stores
        RubyRequestType m_type;

        //! Bitset for tracking prefetches for which addresses have been
        //! issued, which ones have completed.
        std::bitset<MAX_PF_INFLIGHT> requestIssued;
        std::bitset<MAX_PF_INFLIGHT> requestCompleted;
};

class RubyPrefetcher : public SimObject
{
    public:
        typedef RubyPrefetcherParams Params;
        RubyPrefetcher(const Params &p);
        ~RubyPrefetcher() = default;

        void issueNextPrefetch(Addr address, PrefetchEntry *stream);
        /**
         * Implement the prefetch hit(miss) callback interface.
         * These functions are called by the cache when it hits(misses)
         * on a line with the line's prefetch bit set. If this address
         * hits in m_array we will continue prefetching the stream.
         */
        void observePfHit(Addr address);
        void observePfMiss(Addr address);

        /**
         * Observe a memory miss from the cache.
         *
         * @param address   The physical address that missed out of the cache.
         */
        void observeMiss(Addr address, const RubyRequestType& type);

        /**
         * Print out some statistics
         */
        void print(std::ostream& out) const;
        void setController(AbstractController *_ctrl)
        { m_controller = _ctrl; }

    private:
        struct UnitFilterEntry
        {
            /** Address to which this filter entry refers. */
            Addr addr;
            /** Counter of the number of times this entry has been hit. */
            uint32_t hits;

            UnitFilterEntry(Addr _addr = 0)
              : addr(_addr), hits(0)
            {
            }
        };

        struct NonUnitFilterEntry : public UnitFilterEntry
        {
            /** Stride (in # of cache lines). */
            int stride;

            NonUnitFilterEntry(Addr _addr = 0)
              : UnitFilterEntry(_addr), stride(0)
            {
            }

            void
            clear()
            {
                addr = 0;
                stride = 0;
                hits = 0;
            }
        };

        /**
         * Returns an unused stream buffer (or if all are used, returns the
         * least recently used (accessed) stream buffer).
         * @return  The index of the least recently used stream buffer.
         */
        uint32_t getLRUindex(void);

        //! allocate a new stream buffer at a specific index
        void initializeStream(Addr address, int stride,
            uint32_t index, const RubyRequestType& type);

        //! get pointer to the matching stream entry, returns NULL if not found
        //! index holds the multiple of the stride this address is.
        PrefetchEntry* getPrefetchEntry(Addr address,
            uint32_t &index);

        /**
         * Access a unit stride filter to determine if there is a hit, and
         * update it otherwise.
         *
         * @param filter Unit filter being accessed.
         * @param line_addr Address being accessed, block aligned.
         * @param stride The stride value.
         * @param type Type of the request that generated the access.
         * @return True if a corresponding entry was found.
         */
        bool accessUnitFilter(CircularQueue<UnitFilterEntry>* const filter,
            Addr line_addr, int stride, const RubyRequestType& type);

        /**
         * Access a non-unit stride filter to determine if there is a hit, and
         * update it otherwise.
         *
         * @param line_addr Address being accessed, block aligned.
         * @param type Type of the request that generated the access.
         * @return True if a corresponding entry was found and its stride is
         *         not zero.
         */
        bool accessNonunitFilter(Addr line_addr, const RubyRequestType& type);

        /// determine the page aligned address
        Addr pageAddress(Addr addr) const;

        //! number of prefetch streams available
        uint32_t m_num_streams;
        //! an array of the active prefetch streams
        std::vector<PrefetchEntry> m_array;

        //! number of misses I must see before allocating a stream
        uint32_t m_train_misses;
        //! number of initial prefetches to startup a stream
        uint32_t m_num_startup_pfs;

        /**
         * A unit stride filter array: helps reduce BW requirement
         * of prefetching.
         */
        CircularQueue<UnitFilterEntry> unitFilter;

        /**
         * A negative unit stride filter array: helps reduce BW requirement
         * of prefetching.
         */
        CircularQueue<UnitFilterEntry> negativeFilter;

        /**
         * A non-unit stride filter array: helps reduce BW requirement of
         * prefetching.
         */
        CircularQueue<NonUnitFilterEntry> nonUnitFilter;

        /// Used for allowing prefetches across pages.
        bool m_prefetch_cross_pages;

        AbstractController *m_controller;

        const unsigned pageShift;

        struct RubyPrefetcherStats : public statistics::Group
        {
            RubyPrefetcherStats(statistics::Group *parent);

            //! Count of accesses to the prefetcher
            statistics::Scalar numMissObserved;
            //! Count of prefetch streams allocated
            statistics::Scalar numAllocatedStreams;
            //! Count of prefetch requests made
            statistics::Scalar numPrefetchRequested;
            //! Count of successful prefetches
            statistics::Scalar numHits;
            //! Count of partial successful prefetches
            statistics::Scalar numPartialHits;
            //! Count of pages crossed
            statistics::Scalar numPagesCrossed;
            //! Count of misses incurred for blocks that were prefetched
            statistics::Scalar numMissedPrefetchedBlocks;
        } rubyPrefetcherStats;
};

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_PREFETCHER_HH__
