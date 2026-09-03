/*
 * Copyright (c) 2022 The Regents of the University of California.
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

#include "cpu/probes/pc_count_tracker.hh"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "params/PcCountTracker.hh"
#include "sim/probe/probe.hh"
#include "sim/probe/probe_listener_object.hh"

namespace gem5
{

PcCountTracker::PcCountTracker(const PcCountTrackerParams &p)
    : ProbeListenerObject(p),
      cpuptr(p.core),
      ptmanager(p.ptmanager),
      enablePcProfiling(p.enable_pc_profiling),
      filterRanges(p.filter_ranges)
{
    fatal_if(p.granularity >= sizeof(unsigned long long) * 8,
             "granularity %d too large for %zu bits. Granularity is the bits "
             "to mask,"
             " not the size",
             p.granularity, sizeof(unsigned long long) * 8);
    pcMask = ~((1ULL << p.granularity) - 1);
    fatal_if(!cpuptr, "CPU is NULL");
    fatal_if(!ptmanager && !enablePcProfiling,
             "PcCountTracker requires either a PcCountTrackerManager or "
             "enable_pc_profiling=True");
    for (int i = 0; i < p.targets.size(); i++) {
        // initialize the set of targeting Program Counter addresses
        targetPC.insert(p.targets[i].getPC());
    }
    if (targetPC.size() == 1) {
        singleTargetPC = *targetPC.begin();
        hasSingleTarget = true;
    }
}

void
PcCountTracker::regProbeListeners()
{
    // connect the probe listener with the probe "RetriedInstsPC" in the
    // corresponding core.
    // when "RetiredInstsPC" notifies the probe listener, then the function
    // 'check_pc' is automatically called
    typedef ProbeListenerArg<PcCountTracker, Addr> PcCountTrackerListener;
    connectListener<PcCountTrackerListener>(this, "RetiredInstsPC",
                                            &PcCountTracker::checkPc);
}

void
PcCountTracker::checkPc(const Addr& pc) {
    // Apply filter first
    bool allow = filterRanges.empty();
    for (const auto &range : filterRanges) {
        if (range.contains(pc)) {
            allow = true;
            break;
        }
    }
    if (!allow) {
        return;
    }

    Addr masked_pc = pc & pcMask;

    if (enablePcProfiling) {
        pcCounts[masked_pc]++;
    }
    if (ptmanager) {
        if (hasSingleTarget) {
            if (pc == singleTargetPC) {
                ptmanager->checkCount(pc);
            }
        } else if (targetPC.find(pc) != targetPC.end()) {
            ptmanager->checkCount(pc);
        }
    }
}

std::vector<std::pair<Addr, uint64_t>>
PcCountTracker::getHottestPcs(unsigned n) const
{
    if (n == 0 || pcCounts.empty()) {
        return {};
    }

    std::vector<std::pair<Addr, uint64_t>> sorted_pcs(pcCounts.begin(),
                                                      pcCounts.end());
    std::sort(sorted_pcs.begin(), sorted_pcs.end(),
              [](const std::pair<Addr, uint64_t> &a,
                 const std::pair<Addr, uint64_t> &b) {
                  return a.second > b.second;
              });

    if (sorted_pcs.size() > n) {
        sorted_pcs.resize(n);
    }
    return sorted_pcs;
}

void
PcCountTracker::resetStats()
{
    ProbeListenerObject::resetStats();
    pcCounts.clear();
}

} // namespace gem5
