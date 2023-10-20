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
      bestScore(0), round(0)
{
    if (!isPowerOf2(rrEntries)) {
        fatal("%s: number of RR entries is not power of 2\n", name());
    }
    if (!isPowerOf2(blkSize)) {
        fatal("%s: cache line size is not power of 2\n", name());
    }
    if (!(p.negative_offsets_enable && (p.offset_list_size % 2 == 0))) {
        fatal("%s: negative offsets enabled with odd offset list size\n",
              name());
    }

    rrLeft.resize(rrEntries);
    rrRight.resize(rrEntries);

    // Following the paper implementation, a list with the specified number
    // of offsets which are of the form 2^i * 3^j * 5^k with i,j,k >= 0
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
            // If we want to use negative offsets, add also the negative value
            // of the offset just calculated
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
        insertIntoRR(addr_x, RRWay::Left);
        delayQueue.pop_front();
    }

    // Schedule an event for the next element if there is one
    if (!delayQueue.empty()) {
        schedule(delayQueueEvent, delayQueue.front().processTick);
    }
}

unsigned int
BOP::hash(Addr addr, unsigned int way) const
{
    Addr hash1 = addr >> way;
    Addr hash2 = hash1 >> floorLog2(rrEntries);
    return (hash1 ^ hash2) & (Addr)(rrEntries - 1);
}

void
BOP::insertIntoRR(Addr addr, unsigned int way)
{
    switch (way) {
        case RRWay::Left:
            rrLeft[hash(addr, RRWay::Left)] = addr;
            break;
        case RRWay::Right:
            rrRight[hash(addr, RRWay::Right)] = addr;
            break;
    }
}

void
BOP::insertIntoDelayQueue(Addr x)
{
    if (delayQueue.size() == delayQueueSize) {
        return;
    }

    // Add the address to the delay queue and schedule an event to process
    // it after the specified delay cycles
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
    return (addr >> blkSize) & tagMask;
}

bool
BOP::testRR(Addr addr) const
{
    for (auto& it : rrLeft) {
        if (it == addr) {
            return true;
        }
    }

    for (auto& it : rrRight) {
        if (it == addr) {
            return true;
        }
    }

    return false;
}

void
BOP::bestOffsetLearning(Addr x)
{
    Addr offset_addr = (*offsetsListIterator).first;
    Addr lookup_addr = x - offset_addr;

    // There was a hit in the RR table, increment the score for this offset
    if (testRR(lookup_addr)) {
        DPRINTF(HWPrefetch, "Address %#lx found in the RR table\n", x);
        (*offsetsListIterator).second++;
        if ((*offsetsListIterator).second > bestScore) {
            bestScore = (*offsetsListIterator).second;
            phaseBestOffset = (*offsetsListIterator).first;
            DPRINTF(HWPrefetch, "New best score is %lu\n", bestScore);
        }
    }

    offsetsListIterator++;

    // All the offsets in the list were visited meaning that a learning
    // phase finished. Check if
    if (offsetsListIterator == offsetsList.end()) {
        offsetsListIterator = offsetsList.begin();
        round++;

        // Check if the best offset must be updated if:
        // (1) One of the scores equals SCORE_MAX
        // (2) The number of rounds equals ROUND_MAX
        if ((bestScore >= scoreMax) || (round == roundMax)) {
            bestOffset = phaseBestOffset;
            round = 0;
            bestScore = 0;
            phaseBestOffset = 0;
            resetScores();
            issuePrefetchRequests = true;
        } else if (bestScore <= badScore) {
            issuePrefetchRequests = false;
        }
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
        insertIntoDelayQueue(tag_x);
    } else {
        insertIntoRR(tag_x, RRWay::Left);
    }

    // Go through the nth offset and update the score, the best score and the
    // current best offset if a better one is found
    bestOffsetLearning(tag_x);

    // This prefetcher is a degree 1 prefetch, so it will only generate one
    // prefetch at most per access
    if (issuePrefetchRequests) {
        Addr prefetch_addr = addr + (bestOffset << lBlkSize);
        addresses.push_back(AddrPriority(prefetch_addr, 0));
        DPRINTF(HWPrefetch, "Generated prefetch %#lx\n", prefetch_addr);
    }
}

void
BOP::notifyFill(const CacheAccessProbeArg &arg)
{
    const PacketPtr& pkt = arg.pkt;

    // Only insert into the RR right way if it's the pkt is a HWP
    if (!pkt->cmd.isHWPrefetch()) return;

    Addr tag_y = tag(pkt->getAddr());

    if (issuePrefetchRequests) {
        insertIntoRR(tag_y - bestOffset, RRWay::Right);
    }
}

} // namespace prefetch
} // namespace gem5
