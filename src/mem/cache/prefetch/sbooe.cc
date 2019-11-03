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
 *
 * Authors: Ivan Pizarro
 */

#include "mem/cache/prefetch/sbooe.hh"

#include "debug/HWPrefetch.hh"
#include "params/SBOOEPrefetcher.hh"

SBOOEPrefetcher::SBOOEPrefetcher(const SBOOEPrefetcherParams *p)
    : QueuedPrefetcher(p),
      latencyBufferSize(p->latency_buffer_size),
      sequentialPrefetchers(p->sequential_prefetchers),
      scoreThreshold((p->sandbox_entries*p->score_threshold_pct)/100),
      averageAccessLatency(0), latencyBufferSum(0),
      bestSandbox(NULL),
      accesses(0)
{
    if (!(p->score_threshold_pct >= 0 && p->score_threshold_pct <= 100)) {
        fatal("%s: the score threshold should be between 0 and 100\n", name());
    }

    // Initialize a sandbox for every sequential prefetcher between
    // -1 and the number of sequential prefetchers defined
    for (int i = 0; i < sequentialPrefetchers; i++) {
        sandboxes.push_back(Sandbox(p->sandbox_entries, i-1));
    }
}

void
SBOOEPrefetcher::Sandbox::insert(Addr addr, Tick tick)
{
    entries[index].valid = true;
    entries[index].line = addr + stride;
    entries[index].expectedArrivalTick = tick;

    index++;

    if (index == entries.size()) {
        index = 0;
    }
}

bool
SBOOEPrefetcher::access(Addr access_line)
{
    for (Sandbox &sb : sandboxes) {
        // Search for the address in the FIFO queue
        for (const SandboxEntry &entry: sb.entries) {
            if (entry.valid && entry.line == access_line) {
                sb.sandboxScore++;
                if (entry.expectedArrivalTick > curTick()) {
                    sb.lateScore++;
                }
            }
        }

        sb.insert(access_line, curTick() + averageAccessLatency);

        if (bestSandbox == NULL || sb.score() > bestSandbox->score()) {
            bestSandbox = &sb;
        }
    }

    accesses++;

    return (accesses >= sandboxes.size());
}

void
SBOOEPrefetcher::notifyFill(const PacketPtr& pkt)
{
    // (1) Look for the address in the demands list
    // (2) Calculate the elapsed cycles until it was filled (curTick)
    // (3) Insert the latency into the latency buffer (FIFO)
    // (4) Calculate the new average access latency

    auto it = demandAddresses.find(pkt->getAddr());

    if (it != demandAddresses.end()) {
        Tick elapsed_ticks = curTick() - it->second;

        latencyBuffer.push_back(elapsed_ticks);
        latencyBufferSum += elapsed_ticks;

        if (latencyBuffer.size() > latencyBufferSize) {
            latencyBufferSum -= latencyBuffer.front();
            latencyBuffer.pop_front();
        }

        averageAccessLatency = latencyBufferSum / latencyBuffer.size();

        demandAddresses.erase(it);
    }
}

void
SBOOEPrefetcher::calculatePrefetch(const PrefetchInfo &pfi,
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

SBOOEPrefetcher*
SBOOEPrefetcherParams::create()
{
    return new SBOOEPrefetcher(this);
}
