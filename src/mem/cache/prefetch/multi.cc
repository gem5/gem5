/*
 * Copyright (c) 2014, 2019, 2023 ARM Limited
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

#include "mem/cache/prefetch/multi.hh"

#include "params/MultiPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

Multi::Multi(const MultiPrefetcherParams &p)
    : Base(p),
      prefetchers(p.prefetchers.begin(), p.prefetchers.end()),
      lastChosenPf(0)
{}

void
Multi::setParentInfo(System *sys, ProbeManager *pm, unsigned blk_size)
{
    for (auto pf : prefetchers)
        pf->setParentInfo(sys, pm, blk_size);
}

Tick
Multi::nextPrefetchReadyTime() const
{
    Tick next_ready = MaxTick;

    for (auto pf : prefetchers)
        next_ready = std::min(next_ready, pf->nextPrefetchReadyTime());

    return next_ready;
}

PacketPtr
Multi::getPacket()
{
    lastChosenPf = (lastChosenPf + 1) % prefetchers.size();
    uint8_t pf_turn = lastChosenPf;

    for (int pf = 0; pf < prefetchers.size(); pf++) {
        if (prefetchers[pf_turn]->nextPrefetchReadyTime() <= curTick()) {
            PacketPtr pkt = prefetchers[pf_turn]->getPacket();
            panic_if(!pkt, "Prefetcher is ready but didn't return a packet.");
            prefetchStats.pfIssued++;
            issuedPrefetches++;
            return pkt;
        }
        pf_turn = (pf_turn + 1) % prefetchers.size();
    }

    return nullptr;
}

} // namespace prefetch
} // namespace gem5
