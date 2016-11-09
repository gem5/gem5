/*
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

#include "mem/ruby/structures/Prefetcher.hh"

#include "debug/RubyPrefetcher.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/system/RubySystem.hh"

Prefetcher*
PrefetcherParams::create()
{
    return new Prefetcher(this);
}

Prefetcher::Prefetcher(const Params *p)
    : SimObject(p), m_num_streams(p->num_streams),
    m_array(p->num_streams), m_train_misses(p->train_misses),
    m_num_startup_pfs(p->num_startup_pfs), m_num_unit_filters(p->unit_filter),
    m_num_nonunit_filters(p->nonunit_filter),
    m_unit_filter(p->unit_filter, 0),
    m_negative_filter(p->unit_filter, 0),
    m_nonunit_filter(p->nonunit_filter, 0),
    m_prefetch_cross_pages(p->cross_page),
    m_page_shift(p->sys->getPageShift())
{
    assert(m_num_streams > 0);
    assert(m_num_startup_pfs <= MAX_PF_INFLIGHT);

    // create +1 stride filter
    m_unit_filter_index = 0;
    m_unit_filter_hit = new uint32_t[m_num_unit_filters];
    for (uint32_t i =0; i < m_num_unit_filters; i++) {
        m_unit_filter_hit[i] = 0;
    }

    // create -1 stride filter
    m_negative_filter_index = 0;
    m_negative_filter_hit = new uint32_t[m_num_unit_filters];
    for (int i =0; i < m_num_unit_filters; i++) {
        m_negative_filter_hit[i] = 0;
    }

    // create nonunit stride filter
    m_nonunit_index = 0;
    m_nonunit_stride = new int[m_num_nonunit_filters];
    m_nonunit_hit    = new uint32_t[m_num_nonunit_filters];
    for (int i =0; i < m_num_nonunit_filters; i++) {
        m_nonunit_stride[i] = 0;
        m_nonunit_hit[i]    = 0;
    }
}

Prefetcher::~Prefetcher()
{
    delete m_unit_filter_hit;
    delete m_negative_filter_hit;
    delete m_nonunit_stride;
    delete m_nonunit_hit;
}

void
Prefetcher::regStats()
{
    SimObject::regStats();

    numMissObserved
        .name(name() + ".miss_observed")
        .desc("number of misses observed")
        ;

    numAllocatedStreams
        .name(name() + ".allocated_streams")
        .desc("number of streams allocated for prefetching")
        ;

    numPrefetchRequested
        .name(name() + ".prefetches_requested")
        .desc("number of prefetch requests made")
        ;

    numPrefetchAccepted
        .name(name() + ".prefetches_accepted")
        .desc("number of prefetch requests accepted")
        ;

    numDroppedPrefetches
        .name(name() + ".dropped_prefetches")
        .desc("number of prefetch requests dropped")
        ;

    numHits
        .name(name() + ".hits")
        .desc("number of prefetched blocks accessed")
        ;

    numPartialHits
        .name(name() + ".partial_hits")
        .desc("number of misses observed for a block being prefetched")
        ;

    numPagesCrossed
        .name(name() + ".pages_crossed")
        .desc("number of prefetches across pages")
        ;

    numMissedPrefetchedBlocks
        .name(name() + ".misses_on_prefetched_blocks")
        .desc("number of misses for blocks that were prefetched, yet missed")
        ;
}

void
Prefetcher::observeMiss(Addr address, const RubyRequestType& type)
{
    DPRINTF(RubyPrefetcher, "Observed miss for %#x\n", address);
    Addr line_addr = makeLineAddress(address);
    numMissObserved++;

    // check to see if we have already issued a prefetch for this block
    uint32_t index = 0;
    PrefetchEntry *pfEntry = getPrefetchEntry(line_addr, index);
    if (pfEntry != NULL) {
        if (pfEntry->requestIssued[index]) {
            if (pfEntry->requestCompleted[index]) {
                // We prefetched too early and now the prefetch block no
                // longer exists in the cache
                numMissedPrefetchedBlocks++;
                return;
            } else {
                // The controller has issued the prefetch request,
                // but the request for the block arrived earlier.
                numPartialHits++;
                observePfHit(line_addr);
                return;
            }
        } else {
            // The request is still in the prefetch queue of the controller.
            // Or was evicted because of other requests.
            return;
        }
    }

    // check to see if this address is in the unit stride filter
    bool alloc = false;
    bool hit = accessUnitFilter(m_unit_filter, m_unit_filter_hit,
                                m_unit_filter_index, line_addr, 1, alloc);
    if (alloc) {
        // allocate a new prefetch stream
        initializeStream(line_addr, 1, getLRUindex(), type);
    }
    if (hit) {
        DPRINTF(RubyPrefetcher, "  *** hit in unit stride buffer\n");
        return;
    }

    hit = accessUnitFilter(m_negative_filter, m_negative_filter_hit,
        m_negative_filter_index, line_addr, -1, alloc);
    if (alloc) {
        // allocate a new prefetch stream
        initializeStream(line_addr, -1, getLRUindex(), type);
    }
    if (hit) {
        DPRINTF(RubyPrefetcher, "  *** hit in unit negative unit buffer\n");
        return;
    }

    // check to see if this address is in the non-unit stride filter
    int stride = 0;  // NULL value
    hit = accessNonunitFilter(address, &stride, alloc);
    if (alloc) {
        assert(stride != 0);  // ensure non-zero stride prefetches
        initializeStream(line_addr, stride, getLRUindex(), type);
    }
    if (hit) {
        DPRINTF(RubyPrefetcher, "  *** hit in non-unit stride buffer\n");
        return;
    }
}

void
Prefetcher::observePfMiss(Addr address)
{
    numPartialHits++;
    DPRINTF(RubyPrefetcher, "Observed partial hit for %#x\n", address);
    issueNextPrefetch(address, NULL);
}

void
Prefetcher::observePfHit(Addr address)
{
    numHits++;
    DPRINTF(RubyPrefetcher, "Observed hit for %#x\n", address);
    issueNextPrefetch(address, NULL);
}

void
Prefetcher::issueNextPrefetch(Addr address, PrefetchEntry *stream)
{
    // get our corresponding stream fetcher
    if (stream == NULL) {
        uint32_t index = 0;
        stream = getPrefetchEntry(address, index);
    }

    // if (for some reason), this stream is unallocated, return.
    if (stream == NULL) {
        DPRINTF(RubyPrefetcher, "Unallocated stream, returning\n");
        return;
    }

    // extend this prefetching stream by 1 (or more)
    Addr page_addr = pageAddress(stream->m_address);
    Addr line_addr = makeNextStrideAddress(stream->m_address,
                                         stream->m_stride);

    // possibly stop prefetching at page boundaries
    if (page_addr != pageAddress(line_addr)) {
        numPagesCrossed++;
        if (!m_prefetch_cross_pages) {
            // Deallocate the stream since we are not prefetching
            // across page boundries
            stream->m_is_valid = false;
            return;
        }
    }

    // launch next prefetch
    stream->m_address = line_addr;
    stream->m_use_time = m_controller->curCycle();
    DPRINTF(RubyPrefetcher, "Requesting prefetch for %#x\n", line_addr);
    m_controller->enqueuePrefetch(line_addr, stream->m_type);
}

uint32_t
Prefetcher::getLRUindex(void)
{
    uint32_t lru_index = 0;
    Cycles lru_access = m_array[lru_index].m_use_time;

    for (uint32_t i = 0; i < m_num_streams; i++) {
        if (!m_array[i].m_is_valid) {
            return i;
        }
        if (m_array[i].m_use_time < lru_access) {
            lru_access = m_array[i].m_use_time;
            lru_index = i;
        }
    }

    return lru_index;
}

void
Prefetcher::clearNonunitEntry(uint32_t index)
{
    m_nonunit_filter[index] = 0;
    m_nonunit_stride[index] = 0;
    m_nonunit_hit[index]    = 0;
}

void
Prefetcher::initializeStream(Addr address, int stride,
     uint32_t index, const RubyRequestType& type)
{
    numAllocatedStreams++;

    // initialize the stream prefetcher
    PrefetchEntry *mystream = &(m_array[index]);
    mystream->m_address = makeLineAddress(address);
    mystream->m_stride = stride;
    mystream->m_use_time = m_controller->curCycle();
    mystream->m_is_valid = true;
    mystream->m_type = type;

    // create a number of initial prefetches for this stream
    Addr page_addr = pageAddress(mystream->m_address);
    Addr line_addr = makeLineAddress(mystream->m_address);

    // insert a number of prefetches into the prefetch table
    for (int k = 0; k < m_num_startup_pfs; k++) {
        line_addr = makeNextStrideAddress(line_addr, stride);
        // possibly stop prefetching at page boundaries
        if (page_addr != pageAddress(line_addr)) {
            numPagesCrossed++;
            if (!m_prefetch_cross_pages) {
                // deallocate this stream prefetcher
                mystream->m_is_valid = false;
                return;
            }
        }

        // launch prefetch
        numPrefetchRequested++;
        DPRINTF(RubyPrefetcher, "Requesting prefetch for %#x\n", line_addr);
        m_controller->enqueuePrefetch(line_addr, m_array[index].m_type);
    }

    // update the address to be the last address prefetched
    mystream->m_address = line_addr;
}

PrefetchEntry *
Prefetcher::getPrefetchEntry(Addr address, uint32_t &index)
{
    // search all streams for a match
    for (int i = 0; i < m_num_streams; i++) {
        // search all the outstanding prefetches for this stream
        if (m_array[i].m_is_valid) {
            for (int j = 0; j < m_num_startup_pfs; j++) {
                if (makeNextStrideAddress(m_array[i].m_address,
                    -(m_array[i].m_stride*j)) == address) {
                    return &(m_array[i]);
                }
            }
        }
    }
    return NULL;
}

bool
Prefetcher::accessUnitFilter(std::vector<Addr>& filter_table,
    uint32_t *filter_hit, uint32_t &index, Addr address,
    int stride, bool &alloc)
{
    //reset the alloc flag
    alloc = false;

    Addr line_addr = makeLineAddress(address);
    for (int i = 0; i < m_num_unit_filters; i++) {
        if (filter_table[i] == line_addr) {
            filter_table[i] = makeNextStrideAddress(filter_table[i], stride);
            filter_hit[i]++;
            if (filter_hit[i] >= m_train_misses) {
                alloc = true;
            }
            return true;
        }
    }

    // enter this address in the table
    int local_index = index;
    filter_table[local_index] = makeNextStrideAddress(line_addr, stride);
    filter_hit[local_index] = 0;
    local_index = local_index + 1;
    if (local_index >= m_num_unit_filters) {
        local_index = 0;
    }

    index = local_index;
    return false;
}

bool
Prefetcher::accessNonunitFilter(Addr address, int *stride,
    bool &alloc)
{
    //reset the alloc flag
    alloc = false;

    /// look for non-unit strides based on a (user-defined) page size
    Addr page_addr = pageAddress(address);
    Addr line_addr = makeLineAddress(address);

    for (uint32_t i = 0; i < m_num_nonunit_filters; i++) {
        if (pageAddress(m_nonunit_filter[i]) == page_addr) {
            // hit in the non-unit filter
            // compute the actual stride (for this reference)
            int delta = line_addr - m_nonunit_filter[i];

            if (delta != 0) {
                // no zero stride prefetches
                // check that the stride matches (for the last N times)
                if (delta == m_nonunit_stride[i]) {
                    // -> stride hit
                    // increment count (if > 2) allocate stream
                    m_nonunit_hit[i]++;
                    if (m_nonunit_hit[i] > m_train_misses) {
                        // This stride HAS to be the multiplicative constant of
                        // dataBlockBytes (bc makeNextStrideAddress is
                        // calculated based on this multiplicative constant!)
                        *stride = m_nonunit_stride[i] /
                                    RubySystem::getBlockSizeBytes();

                        // clear this filter entry
                        clearNonunitEntry(i);
                        alloc = true;
                    }
                } else {
                    // delta didn't match ... reset m_nonunit_hit count for
                    // this entry
                    m_nonunit_hit[i] = 0;
                }

                // update the last address seen & the stride
                m_nonunit_stride[i] = delta;
                m_nonunit_filter[i] = line_addr;
                return true;
            } else {
                return false;
            }
        }
    }

    // not found: enter this address in the table
    m_nonunit_filter[m_nonunit_index] = line_addr;
    m_nonunit_stride[m_nonunit_index] = 0;
    m_nonunit_hit[m_nonunit_index]    = 0;

    m_nonunit_index = m_nonunit_index + 1;
    if (m_nonunit_index >= m_num_nonunit_filters) {
        m_nonunit_index = 0;
    }
    return false;
}

void
Prefetcher::print(std::ostream& out) const
{
    out << name() << " Prefetcher State\n";
    // print out unit filter
    out << "unit table:\n";
    for (int i = 0; i < m_num_unit_filters; i++) {
        out << m_unit_filter[i] << std::endl;
    }

    out << "negative table:\n";
    for (int i = 0; i < m_num_unit_filters; i++) {
        out << m_negative_filter[i] << std::endl;
    }

    // print out non-unit stride filter
    out << "non-unit table:\n";
    for (int i = 0; i < m_num_nonunit_filters; i++) {
        out << m_nonunit_filter[i] << " "
            << m_nonunit_stride[i] << " "
            << m_nonunit_hit[i] << std::endl;
    }

    // print out allocated stream buffers
    out << "streams:\n";
    for (int i = 0; i < m_num_streams; i++) {
        out << m_array[i].m_address << " "
            << m_array[i].m_stride << " "
            << m_array[i].m_is_valid << " "
            << m_array[i].m_use_time << std::endl;
    }
}

Addr
Prefetcher::pageAddress(Addr addr) const
{
    return maskLowOrderBits(addr, m_page_shift);
}
