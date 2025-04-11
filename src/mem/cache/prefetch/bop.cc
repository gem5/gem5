/**
 * Copyright (c) 2025 Arm Limited
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
 * Copyright (c) 2018 Metempsy Technology Consulting
 * Copyright (c) 2024 Samsung Electronics
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

#include "mem/cache/prefetch/bop.hh"

#include "debug/HWPrefetch.hh"
#include "params/BOPPrefetcher.hh"

namespace gem5
{

namespace prefetch
{
BOP::BOP(const BOPPrefetcherParams &p)
    : Queued(p),
      scoreMax(p.score_max), roundMax(p.round_max),
      badScore(p.bad_score), rrEntries(p.rr_size),
      tagMask((1 << p.tag_bits) - 1),
      delayQueueEnabled(p.delay_queue_enable),
      delayQueueSize(p.delay_queue_size),
      delayTicks(cyclesToTicks(p.delay_queue_cycles)),
      delayQueueEvent([this]{ delayQueueEventWrapper(); }, name()),
      issuePrefetchRequests(false), bestOffset(1), phaseBestOffset(0),
      bestScore(0), round(0), degree(p.degree)
{
    if (!isPowerOf2(rrEntries)) {
        fatal("%s: number of RR entries is not power of 2\n", name());
    }
    if (!isPowerOf2(blkSize)) {
        fatal("%s: cache line size is not power of 2\n", name());
    }
    if (p.negative_offsets_enable && (p.offset_list_size % 2 != 0)) {
        fatal("%s: negative offsets enabled with odd offset list size\n",
              name());
    }
    if (p.degree <= 0) {
        fatal("%s: prefetch degree must be strictly greater than zero\n",
              name());
    }

    rrLeft.resize(rrEntries);
    rrRight.resize(rrEntries);

    /*
     * Following the paper implementation, a list with the specified number
     * of offsets which are of the form 2^i * 3^j * 5^k with i,j,k >= 0
     */
    const int factors[] = { 2, 3, 5 };
    unsigned int i = 0;
    int64_t offset_i = 1;

    while (i < p.offset_list_size)
    {
        int64_t offset = offset_i;

        for (int n : factors) {
            while ((offset % n) == 0) {
                offset /= n;
            }
        }

        if (offset == 1) {
            offsetsList.push_back(OffsetListEntry(offset_i, 0));
            i++;
            /*
             * If we want to use negative offsets, add also the negative value
             * of the offset just calculated
             */
            if (p.negative_offsets_enable)  {
                offsetsList.push_back(OffsetListEntry(-offset_i, 0));
                i++;
            }
        }

        offset_i++;
    }

    offsetsListIterator = offsetsList.begin();
}

void
BOP::delayQueueEventWrapper()
{
    while (!delayQueue.empty() &&
            delayQueue.front().processTick <= curTick())
    {
        Addr addr_x = delayQueue.front().baseAddr;
        Addr addr_tag = tag(addr_x);
        insertIntoRR(addr_x, addr_tag, RRWay::Left);
        delayQueue.pop_front();
    }

    // Schedule an event for the next element if there is one
    if (!delayQueue.empty()) {
        schedule(delayQueueEvent, delayQueue.front().processTick);
    }
}

unsigned int
BOP::index(Addr addr, unsigned int way) const
{
    /*
     * The second parameter, way, is set to 0 for indexing the left side of the
     * RR Table and, it is set to 1 for indexing the right side of the RR
     * Table. This is because we always pass the enum RRWay as the way argument
     * while calling index. This enum is defined in the bop.hh file.
     *
     * The indexing function in the author's ChampSim code, which can be found
     * here: https://comparch-conf.gatech.edu/dpc2/final_program.html, computes
     * the hash as follows:
     *
     *  1. For indexing the left side of the RR Table (way = 0), the cache line
     *     address is XORed with itself after right shifting it by the log base
     *     2 of the number of entries in the RR Table.
     *  2. For indexing the right side of the RR Table (way = 1), the cache
     *     line address is XORed with itself after right shifting it by the log
     *     base 2 of the number of entries in the RR Table, multiplied by two.
     *
     * Therefore, we if just left shift the log base 2 of the number of RR
     * entries (log_rr_entries) with the parameter way, then if we are indexing
     * the left side, we'll leave log_rr_entries as it is, but if we are
     * indexing the right side, we'll multiply it with 2. Now once we have the
     * result of this operation, we can right shift the cache line address
     * (line_addr) by this value to get the first operand of the final XOR
     * operation. The second operand of the XOR operation is line_addr itself
     */
    Addr log_rr_entries = floorLog2(rrEntries);
    Addr line_addr = addr >> lBlkSize;
    Addr hash = line_addr ^ (line_addr >> (log_rr_entries << way));
    hash &= ((1ULL << log_rr_entries) - 1);
    return hash % rrEntries;
}

void
BOP::insertIntoRR(Addr addr, Addr tag, unsigned int way)
{
    switch (way) {
        case RRWay::Left:
            rrLeft[index(addr, RRWay::Left)] = tag;
            break;
        case RRWay::Right:
            rrRight[index(addr, RRWay::Right)] = tag;
            break;
    }
}

void
BOP::insertIntoDelayQueue(Addr x)
{
    if (delayQueue.size() == delayQueueSize) {
        return;
    }

    /*
     * Add the address to the delay queue and schedule an event to process
     * it after the specified delay cycles
     */
    Tick process_tick = curTick() + delayTicks;

    delayQueue.push_back(DelayQueueEntry(x, process_tick));

    if (!delayQueueEvent.scheduled()) {
        schedule(delayQueueEvent, process_tick);
    }
}

void
BOP::resetScores()
{
    for (auto& it : offsetsList) {
        it.second = 0;
    }
}

inline Addr
BOP::tag(Addr addr) const
{
    return (addr >> lBlkSize) & tagMask;
}

bool
BOP::testRR(Addr addr_tag) const
{
    for (auto& it : rrLeft) {
        if (it == addr_tag) {
            return true;
        }
    }

    for (auto& it : rrRight) {
        if (it == addr_tag) {
            return true;
        }
    }

    return false;
}

void
BOP::bestOffsetLearning(Addr addr)
{
    Addr offset_tag = (*offsetsListIterator).first;

    /*
     * Compute the lookup tag for the RR table. As tags are generated using
     * lower 12 bits we subtract offset from the full address rather than the
     * tag to avoid integer underflow.
     */
    Addr lookup_tag = tag((addr) - (offset_tag << lBlkSize));

    // There was a hit in the RR table, increment the score for this offset
    if (testRR(lookup_tag)) {
        DPRINTF(HWPrefetch, "Address %#lx found in the RR table\n",
                lookup_tag);
        (*offsetsListIterator).second++;
        if ((*offsetsListIterator).second > bestScore) {
            bestScore = (*offsetsListIterator).second;
            phaseBestOffset = (*offsetsListIterator).first;
            DPRINTF(HWPrefetch, "New best score is %lu\n", bestScore);
        }
    }

    // Move the offset iterator forward to prepare for the next time
    offsetsListIterator++;

    /*
     * All the offsets in the list were visited meaning that a learning
     * phase finished. Check if
     */
    if (offsetsListIterator == offsetsList.end()) {
        offsetsListIterator = offsetsList.begin();
        round++;
    }

    // Check if its time to re-calculate the best offset
    if ((bestScore >= scoreMax) || (round >= roundMax)) {
        round = 0;

        /*
         * If the current best score (bestScore) has exceed the threshold to
         * enable prefetching (badScore), reset the learning structures and
         * enable prefetch generation
         */
        if (bestScore > badScore) {
            bestOffset = phaseBestOffset;
            round = 0;
            issuePrefetchRequests = true;
        } else {
            issuePrefetchRequests = false;
        }
        resetScores();
        bestScore = 0;
        phaseBestOffset = 0;
    }
}

void
BOP::calculatePrefetch(const PrefetchInfo &pfi,
        std::vector<AddrPriority> &addresses,
        const CacheAccessor &cache)
{
    Addr addr = pfi.getAddr();
    Addr tag_x = tag(addr);

    if (delayQueueEnabled) {
        insertIntoDelayQueue(addr);
    } else {
        insertIntoRR(addr, tag_x, RRWay::Left);
    }

    /*
     * Go through the nth offset and update the score, the best score and the
     * current best offset if a better one is found
     */
    bestOffsetLearning(addr);

    if (issuePrefetchRequests) {
        for (int i = 1; i <= degree; i++) {
            Addr prefetch_addr = addr + ((i * bestOffset) << lBlkSize);
            addresses.push_back(AddrPriority(prefetch_addr, 0));
            DPRINTF(HWPrefetch, "Generated prefetch %#lx\n", prefetch_addr);
        }
    }
}

void
BOP::notifyFill(const CacheAccessProbeArg &arg)
{
    const PacketPtr& pkt = arg.pkt;

    // Only insert into the RR right way if it's the pkt is a hardware prefetch
    if (!pkt->cmd.isHWPrefetch()) return;

    Addr addr = pkt->getAddr();
    Addr tag_y = tag(addr);

    if (issuePrefetchRequests) {
        insertIntoRR(addr, tag_y - bestOffset, RRWay::Right);
    }
}

} // namespace prefetch
} // namespace gem5
