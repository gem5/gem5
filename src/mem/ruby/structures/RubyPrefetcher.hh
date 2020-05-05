/*
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

#include "base/statistics.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/RubyPrefetcher.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

#define MAX_PF_INFLIGHT 8

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
        RubyPrefetcher(const Params *p);
        ~RubyPrefetcher();

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

        void regStats();

    private:
        /**
         * Returns an unused stream buffer (or if all are used, returns the
         * least recently used (accessed) stream buffer).
         * @return  The index of the least recently used stream buffer.
         */
        uint32_t getLRUindex(void);

        //! clear a non-unit stride prefetcher entry
        void clearNonunitEntry(uint32_t index);

        //! allocate a new stream buffer at a specific index
        void initializeStream(Addr address, int stride,
            uint32_t index, const RubyRequestType& type);

        //! get pointer to the matching stream entry, returns NULL if not found
        //! index holds the multiple of the stride this address is.
        PrefetchEntry* getPrefetchEntry(Addr address,
            uint32_t &index);

        /// access a unit stride filter to determine if there is a hit
        bool accessUnitFilter(std::vector<Addr>& filter_table,
            uint32_t *hit_table, uint32_t &index, Addr address,
            int stride, bool &alloc);

        /// access a unit stride filter to determine if there is a hit
        bool accessNonunitFilter(Addr address, int *stride,
            bool &alloc);

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
        //! number of stride filters
        uint32_t m_num_unit_filters;
        //! number of non-stride filters
        uint32_t m_num_nonunit_filters;

        /// a unit stride filter array: helps reduce BW requirement of
        /// prefetching
        std::vector<Addr> m_unit_filter;
        /// a round robin pointer into the unit filter group
        uint32_t m_unit_filter_index;
        //! An array used to count the of times particular filter entries
        //! have been hit
        uint32_t *m_unit_filter_hit;

        //! a negative unit stride filter array: helps reduce BW requirement
        //! of prefetching
        std::vector<Addr> m_negative_filter;
        /// a round robin pointer into the negative filter group
        uint32_t m_negative_filter_index;
        /// An array used to count the of times particular filter entries
        /// have been hit
        uint32_t *m_negative_filter_hit;

        /// a non-unit stride filter array: helps reduce BW requirement of
        /// prefetching
        std::vector<Addr> m_nonunit_filter;
        /// An array of strides (in # of cache lines) for the filter entries
        int *m_nonunit_stride;
        /// An array used to count the of times particular filter entries
        /// have been hit
        uint32_t *m_nonunit_hit;
        /// a round robin pointer into the unit filter group
        uint32_t m_nonunit_index;

        /// Used for allowing prefetches across pages.
        bool m_prefetch_cross_pages;

        AbstractController *m_controller;

        const Addr m_page_shift;

        //! Count of accesses to the prefetcher
        Stats::Scalar numMissObserved;
        //! Count of prefetch streams allocated
        Stats::Scalar numAllocatedStreams;
        //! Count of prefetch requests made
        Stats::Scalar numPrefetchRequested;
        //! Count of successful prefetches
        Stats::Scalar numHits;
        //! Count of partial successful prefetches
        Stats::Scalar numPartialHits;
        //! Count of pages crossed
        Stats::Scalar numPagesCrossed;
        //! Count of misses incurred for blocks that were prefetched
        Stats::Scalar numMissedPrefetchedBlocks;
};

#endif // __MEM_RUBY_STRUCTURES_PREFETCHER_HH__
