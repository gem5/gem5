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

#include "mem/cache/prefetch/sbooe.hh"

#include "debug/HWPrefetch.hh"
#include "params/SBOOEPrefetcher.hh"

namespace Prefetcher {

SBOOE::SBOOE(const SBOOEPrefetcherParams *p)
    : Queued(p),
      sequentialPrefetchers(p->sequential_prefetchers),
      scoreThreshold((p->sandbox_entries*p->score_threshold_pct)/100),
      latencyBuffer(p->latency_buffer_size),
      averageAccessLatency(0), latencyBufferSum(0),
      bestSandbox(NULL),
      accesses(0)
{
    // Initialize a sandbox for every sequential prefetcher between
    // -1 and the number of sequential prefetchers defined
    for (int i = 0; i < sequentialPrefetchers; i++) {
        sandboxes.push_back(Sandbox(p->sandbox_entries, i-1));
    }
}

void
SBOOE::Sandbox::access(Addr addr, Tick tick)
{
    // Search for the address in the FIFO queue to update the score
    for (const SandboxEntry &entry: entries) {
        if (entry.valid && entry.line == addr) {
            sandboxScore++;
            if (entry.expectedArrivalTick > curTick()) {
                lateScore++;
            }
        }
    }

    // Insert new access in this sandbox
    SandboxEntry entry;
    entry.valid = true;
    entry.line = addr + stride;
    entry.expectedArrivalTick = tick;
    entries.push_back(entry);
}

bool
SBOOE::access(Addr access_line)
{
    for (Sandbox &sb : sandboxes) {
        sb.access(access_line, curTick() + averageAccessLatency);

        if (bestSandbox == NULL || sb.score() > bestSandbox->score()) {
            bestSandbox = &sb;
        }
    }

    accesses++;

    return (accesses >= sandboxes.size());
}

void
SBOOE::notifyFill(const PacketPtr& pkt)
{
    // (1) Look for the address in the demands list
    // (2) Calculate the elapsed cycles until it was filled (curTick)
    // (3) Insert the latency into the latency buffer (FIFO)
    // (4) Calculate the new average access latency

    auto it = demandAddresses.find(pkt->getAddr());

    if (it != demandAddresses.end()) {
        Tick elapsed_ticks = curTick() - it->second;

        if (latencyBuffer.full()) {
            latencyBufferSum -= latencyBuffer.front();
        }
        latencyBuffer.push_back(elapsed_ticks);
        latencyBufferSum += elapsed_ticks;

        averageAccessLatency = latencyBufferSum / latencyBuffer.size();

        demandAddresses.erase(it);
    }
}

void
SBOOE::calculatePrefetch(const PrefetchInfo &pfi,
                                   std::vector<AddrPriority> &addresses)
{
    const Addr pfi_addr = pfi.getAddr();
    const Addr pfi_line = pfi_addr >> lBlkSize;

    auto it = demandAddresses.find(pfi_addr);

    if (it == demandAddresses.end()) {
        demandAddresses.insert(std::pair<Addr, Tick>(pfi_addr, curTick()));
    }

    const bool evaluationFinished = access(pfi_line);

    if (evaluationFinished && bestSandbox->score() > scoreThreshold) {
        Addr pref_line = pfi_line + bestSandbox->stride;
        addresses.push_back(AddrPriority(pref_line << lBlkSize, 0));
    }
}

} // namespace Prefetcher

Prefetcher::SBOOE*
SBOOEPrefetcherParams::create()
{
    return new Prefetcher::SBOOE(this);
}
