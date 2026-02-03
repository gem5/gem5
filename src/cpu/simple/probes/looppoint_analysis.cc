/*
 * Copyright (c) 2024 The Regents of the University of California.
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

#include "cpu/simple/probes/looppoint_analysis.hh"

namespace gem5
{

LooppointAnalysis::LooppointAnalysis(const LooppointAnalysisParams &params)
    : ProbeListenerObject(params),
      lpaManager(params.looppoint_analysis_manager),
      bbValidAddrRange(params.bb_valid_addr_range),
      markerValidAddrRange(params.marker_valid_addr_range),
      ifListening(params.if_listening),
      bbInstCounter(0)
{
    DPRINTF(LooppointAnalysis, "Start listening from the beginning of the "
                            "simulation? %s\n", ifListening ? "Yes" : "No");

    for (int i = 0; i < params.bb_excluded_addr_ranges.size(); i++) {
        bbExcludedAddrRanges.push_back(
            AddrRange(
                params.bb_excluded_addr_ranges[i].start(),
                params.bb_excluded_addr_ranges[i].end()
            )
        );
        DPRINTF(LooppointAnalysis, "Excluding address range: (%li, %li)\n",
                params.bb_excluded_addr_ranges[i].start(),
                params.bb_excluded_addr_ranges[i].end()
        );
    }

    DPRINTF(LooppointAnalysis, "Valid address range: (%li, %li)\n",
            bbValidAddrRange.start(), bbValidAddrRange.end());
    DPRINTF(LooppointAnalysis, "Valid marker address range: (%li, %li)\n",
            markerValidAddrRange.start(), markerValidAddrRange.end());
}

void
LooppointAnalysis::updateLocalBBV(const Addr pc)
{
    if (localBBV.find(pc) == localBBV.end()) {
        localBBV.insert(std::make_pair(pc, 1));
    } else {
        localBBV.find(pc)->second++;
    }
}

void
LooppointAnalysis::checkPc(
    const std::pair<SimpleThread*,const StaticInstPtr>& inst_pair
)
{
    const StaticInstPtr &inst = inst_pair.second;

    if (inst->isMicroop() && !inst->isLastMicroop()) {
        // ignore this if it is a microop
        return;
    }

    SimpleThread* thread = inst_pair.first;

    auto &pcstate =
                thread->getTC()->pcState().as<GenericISA::PCStateWithNext>();
    Addr pc = pcstate.pc();

    if (lpaManager->ifEncountered(pc)) {
        // if we have already encountered this pc, we should already
        // categorized it
        if (lpaManager->ifValidNotControl(pc)) {
            // if it is categorized as a valid not control instruction
            bbInstCounter++;
            lpaManager->incrementGlobalInstCounter();
        } else if (lpaManager->ifValidControl(pc)) {
            // if it is categorized as a valid control instruction
            bbInstCounter ++;
            lpaManager->incrementGlobalInstCounter();
            lpaManager->updateBBInstMap(pc, bbInstCounter);
            updateLocalBBV(pc);
            lpaManager->updateGlobalBBV(pc);
            bbInstCounter = 0;
            if (lpaManager->ifBackwardBranch(pc)) {
                // if it is categorized as a backward branch
                lpaManager->countBackwardBranch(pc);
            }
        }
        return;
    }

    lpaManager->updateEncountered(pc);
    // if we have not encountered this pc before, we should now update it to
    // the corresponding category

    if (!thread->getIsaPtr()->inUserMode()) {
        // ignore this if it is not in user mode
        return;
    }

    if (bbValidAddrRange.end() > 0 && ! bbValidAddrRange.contains(pc)) {
        // ignore this if it is not in the valid address range
        return;
    }

    if (bbExcludedAddrRanges.size() > 0) {
        for (int i = 0; i < bbExcludedAddrRanges.size(); i++) {
            if (bbExcludedAddrRanges[i].contains(pc)) {
                // ignore this if it is in the excluded address range
                return;
            }
        }
    }

    bbInstCounter++;
    lpaManager->incrementGlobalInstCounter();

    if (inst->isControl()) {
        // if it is a control instruction, we see it as the end of a basic
        // block
        lpaManager->updateValidControl(pc);
        lpaManager->updateBBInstMap(pc, bbInstCounter);
        updateLocalBBV(pc);
        lpaManager->updateGlobalBBV(pc);
        bbInstCounter = 0;

        if (markerValidAddrRange.end() > 0
                && ! markerValidAddrRange.contains(pc)) {
            // if it is not in the marker valid address range, we do not
            // consider it as a possible marker used loop branch instruction
            return;
        }

        if (inst->isDirectCtrl()) {
            // We only consider direct control instructions as possible
            // loop branch instructions because it is PC-relative and it
            // excludes return instructions.
            if (pcstate.npc() < pc) {
                lpaManager->countBackwardBranch(pc);
            }

        }
    } else {
        lpaManager->updateValidNotControl(pc);
    }
}

void
LooppointAnalysis::regProbeListeners()
{
    if (ifListening) {
        if (listeners.empty()) {
            connectListener<looppointAnalysisListener>(
                this, "Commit", &LooppointAnalysis::checkPc);
            DPRINTF(LooppointAnalysis,
                                    "Start listening to the RetiredInstsPC\n");
        }
    }

}

void
LooppointAnalysis::startListening()
{
    ifListening = true;
    regProbeListeners();
}

void
LooppointAnalysis::stopListening()
{
    ifListening = false;
    listeners.clear();
    DPRINTF(LooppointAnalysis, "Stop listening to Commit\n");
}

LooppointAnalysisManager::LooppointAnalysisManager(const
                                    LooppointAnalysisManagerParams &params)
    : SimObject(params),
    regionLength(params.region_length),
    globalInstCounter(0),
    mostRecentBackwardBranchPC(0)
{
    DPRINTF(LooppointAnalysis, "regionLength = %i\n", regionLength);
}

void
LooppointAnalysisManager::countBackwardBranch(const Addr pc)
{
    if (backwardBranchCounter.find(pc) == backwardBranchCounter.end()) {
        backwardBranchCounter.insert(std::make_pair(pc, 1));
    } else {
        backwardBranchCounter.find(pc)->second++;
    }

    mostRecentBackwardBranchPC = pc;

    if (globalInstCounter >= regionLength) {
        // note that we do not reset any counter here but only raise an
        // exit event.
        // we can reset the counters through the simulation script using
        // the helper functions in the LooppointAnalysisManager class
        DPRINTF(LooppointAnalysis, "simpoint starting point found\n");
        DPRINTF(LooppointAnalysis, "globalInstCounter = %lu\n",
                globalInstCounter);
        DPRINTF(LooppointAnalysis, "regionLength = %lu\n",
                regionLength);
        exitSimLoopNow("simpoint starting point found");
    }
}

void
LooppointAnalysisManager::updateGlobalBBV(const Addr pc)
{
    if (globalBBV.find(pc) == globalBBV.end()) {
        globalBBV.insert(std::make_pair(pc, 1));
    } else {
        globalBBV.find(pc)->second++;
    }
}

}// namespace gem5
