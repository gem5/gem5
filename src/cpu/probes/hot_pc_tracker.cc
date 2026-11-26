/*
 * Copyright 2026 Google, LLC.
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

#include "cpu/probes/hot_pc_tracker.hh"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "params/HotPcTracker.hh"
#include "sim/probe/probe.hh"
#include "sim/probe/probe_listener_object.hh"

namespace gem5
{

HotPcTracker::HotPcTracker(const HotPcTrackerParams &p)
    : ProbeListenerObject(p), filterRanges(p.filter_ranges)
{
    fatal_if(p.granularity >= sizeof(unsigned long long) * 8,
             "granularity %u too large for %zu bits. "
             "Granularity is the bits to mask, not the size",
             p.granularity, sizeof(unsigned long long) * 8);
    pcMask = ~((1ULL << p.granularity) - 1);
}

void
HotPcTracker::regProbeListeners()
{
    // connect the probe listener with the probe "RetriedInstsPC" in the
    // corresponding core.  When "RetiredInstsPC" notifies the probe listener,
    // then the function 'checkPc' is automatically called
    typedef ProbeListenerArg<HotPcTracker, Addr> HotPcTrackerListener;
    connectListener<HotPcTrackerListener>(this, "RetiredInstsPC",
                                          &HotPcTracker::checkPc);
}

void
HotPcTracker::checkPc(const Addr &pc)
{
    bool allow = filterRanges.empty();
    for (const auto &range : filterRanges) {
        if (range.contains(pc)) {
            allow = true;
            break;
        }
    }
    if (allow) {
        Addr masked_pc = pc & pcMask;
        pcCounts[masked_pc]++;
    }
}

std::vector<std::pair<Addr, uint64_t>>
HotPcTracker::getHottestPcs(unsigned n) const
{
    if (n == 0 || pcCounts.empty()) {
        return {};
    }

    std::vector<std::pair<Addr, uint64_t>> sorted_pcs(pcCounts.begin(),
                                                      pcCounts.end());
    const auto cmp = [](const auto &a, const auto &b) {
        return a.second > b.second;
    };
    // filter down to N most common PCs
    if (n < sorted_pcs.size()) {
        std::nth_element(sorted_pcs.begin(), sorted_pcs.begin() + n,
                         sorted_pcs.end(), cmp);
        sorted_pcs.resize(n);
    }
    // sort the results so the user can easily work from the most
    // common (hottest) PC down.
    std::sort(sorted_pcs.begin(), sorted_pcs.end(), cmp);
    return sorted_pcs;
}

void
HotPcTracker::resetStats()
{
    ProbeListenerObject::resetStats();
    pcCounts.clear();
}

} // namespace gem5
