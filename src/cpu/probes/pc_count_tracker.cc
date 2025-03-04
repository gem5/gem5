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


namespace gem5
{

PcCountTracker::PcCountTracker(const PcCountTrackerParams &p)
    : ProbeListenerObject(p),
      cpuptr(p.core),
      manager(p.ptmanager)
{
    if (!cpuptr || !manager) {
        fatal("%s is NULL", !cpuptr ? "CPU": "PcCountTrackerManager");
    }
    for (int i = 0; i < p.targets.size(); i++) {
        // initialize the set of targeting Program Counter addresses
        targetPC.insert(p.targets[i].getPC());
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
    if (targetPC.find(pc) != targetPC.end()) {
        // if the PC is one of the target PCs, then notify the
        // PcCounterTrackerManager by calling its `check_count` function
        manager->checkCount(pc);
    }
}

} // namespace gem5
