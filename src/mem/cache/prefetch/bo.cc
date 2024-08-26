/*
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

/**
 * @file
 * Describes a HPCA variant Best-Offset prefetcher based on template policies.
 */

#include "mem/cache/prefetch/bo.hh"

#include "params/BoPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

Bo::Bo(const BoPrefetcherParams &p)
    : Queued(p), roundMax(p.round_max), maxScore(p.max_score),
    badScore(p.bad_score), rrSize(p.rr_size)
{
    bestOffset = 0;
    for (auto offsets : offsetList) {
        scoreTable[offsets] = 0;
    }
    rrTable.clear();
    offsetIndex = 0;
    round = 0;
    maxScoreAttained = false;
}

void
Bo::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses,
    const CacheAccessor &cache)
{
    Addr blkAddr = blockAddress(pfi.getAddr());

    assert (scoreTable.size() == offsetList.size());

    // lookup into RR
    Addr lookupAddr = blkAddr - (blkSize * offsetList.at(offsetIndex));
    for (auto rrEntry: rrTable) {
        if (rrEntry == lookupAddr) {
            scoreTable[offsetList.at(offsetIndex)] += 1;
            if (scoreTable[offsetList.at(offsetIndex)] >= maxScore) {
                maxScoreAttained = true;
                break;
            }
        }
    }

    // prepare for the next learning phase
    if (offsetIndex >= (offsetList.size() - 1)) {
        //Next round
        round += 1;
        offsetIndex = 0;
    }
    else
        offsetIndex += 1;

   //Update BestOffset based on learning phase
    if ((round > roundMax) || maxScoreAttained) {
        //get max score
        int max = 0;
        bestOffset = 0;
        for (auto score: scoreTable) {
            if (max < score.second) {
                max = score.second;
                bestOffset = score.first;
            }
        }

        if (max <= badScore) bestOffset = 0;

        //Reset
        for (auto &score : scoreTable) {
             score.second = 0;
        }
        round = 0;
        offsetIndex = 0;
        maxScoreAttained = false;
    }

    //Degree 1 prefetching if bestOffset
    if (bestOffset != 0) {
        Addr newAddr = blkAddr + (bestOffset * blkSize);
        if (samePage(blkAddr, newAddr)) {
            addresses.push_back(AddrPriority(newAddr,0));
        }
    }
}

void
Bo::notifyFill(const CacheAccessProbeArg &arg)
{
    const PacketPtr& pkt = arg.pkt;
    Addr blkAddr = blockAddress(pkt->getAddr());
    //Insert in RR
    if (pkt->cmd.isHWPrefetch() || (bestOffset == 0)) {
        Addr insertAddr = blkAddr - (blkSize * bestOffset);
        if (samePage(blkAddr, insertAddr)) rrTable.push_back(insertAddr);
        //Maintain the RR size
        while (rrTable.size() > rrSize) {
                rrTable.erase(rrTable.begin());
        }
    }

}

} // namespace prefetch
} // namespace gem5
