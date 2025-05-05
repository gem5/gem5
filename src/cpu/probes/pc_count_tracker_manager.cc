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

#include "cpu/probes/pc_count_tracker_manager.hh"

namespace gem5
{

PcCountTrackerManager::PcCountTrackerManager(
    const PcCountTrackerManagerParams &p)
    : SimObject(p)
{
    currentPair = PcCountPair(0,0);
    ifListNotEmpty = true;

    for (int i = 0 ; i < p.targets.size() ; i++) {
        // initialize the counter for the inputted PC Count pair
        // unordered_map does not allow duplicate, so counter won't
        // have duplicates
        counter.insert(std::make_pair(p.targets[i].getPC(),0));
        // store all the PC Count pair into the targetPair set
        targetPair.insert(p.targets[i]);
    }
    DPRINTF(PcCountTracker,
            "total %i PCs in counter\n", counter.size());
    DPRINTF(PcCountTracker,
            "all targets: \n%s", printAllTargets());
}

void
PcCountTrackerManager::checkCount(Addr pc)
{

    if(ifListNotEmpty) {
        uint64_t count = ++counter.find(pc)->second;
        // increment the counter of the encountered PC address by 1

        currentPair = PcCountPair(pc,count);
        // update the current PC Count pair
        if(targetPair.find(currentPair) != targetPair.end()) {
            // if the current PC Count pair is one of the target pairs
            DPRINTF(PcCountTracker,
                "pc:%s encountered\n", currentPair.to_string());

            exitSimLoopNow("simpoint starting point found");
            // raise the SIMPOINT_BEGIN exit event

            targetPair.erase(currentPair);
            // erase the encountered PC Count pair from the target pairs
            DPRINTF(PcCountTracker,
                "There are %i targets remained\n", targetPair.size());
        }

        if(targetPair.empty()) {
            // if all target PC Count pairs are encountered
            DPRINTF(PcCountTracker,
                    "all targets are encountered.\n");
            ifListNotEmpty = false;
        }
    }
}

}
