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

#ifndef __CPU_SIMPLE_PROBES_LOOPPOINT_ANALYSIS_HH__
#define __CPU_SIMPLE_PROBES_LOOPPOINT_ANALYSIS_HH__

// C++ includes
#include <list>
#include <unordered_map>
#include <unordered_set>

// m5 includes
#include "arch/generic/pcstate.hh"
#include "cpu/simple_thread.hh"
#include "sim/probe/probe_listener_object.hh"
#include "sim/sim_exit.hh"

// class includes
#include "debug/LooppointAnalysis.hh"
#include "params/LooppointAnalysis.hh"
#include "params/LooppointAnalysisManager.hh"

namespace gem5
{

class LooppointAnalysis : public ProbeListenerObject
{
  public:
    LooppointAnalysis(const LooppointAnalysisParams &params);

    virtual void regProbeListeners();

    /**
     * This function is called when a the probe point notifies the
     * LoopPointAnalysis probe listener.
     *
     * @param inst_pair A pair that contains the SimpleThread pointer and the
     * StaticInstPtr of the commited instruction from the atomic CPU.
     */
    void checkPc(const std::pair<SimpleThread*,
        const StaticInstPtr>& inst_pair);

    /**
     * When this function is called, it sets the class variable ifListening to
     * true, then calls the regProbeListeners() function to register the probe
     * listeners to the probe points,
     */
    void startListening();

    /**
     * When this function is called, it sets the class variable ifListening to
     * false, then removes the probe listeners from the probe points.
     */
    void stopListening();

    typedef ProbeListenerArg<LooppointAnalysis, std::pair<SimpleThread*,
        const StaticInstPtr>> looppointAnalysisListener;

  private:

    /**
     * This is the pointer to the LooppointAnalysisManager SimObject that is
     * managing all the LooppointAnalysis ProbeListenerObjects.
     */
    LooppointAnalysisManager *lpaManager;

    /**
     * This is the valid address range for the basic block that the
     * LooppointAnalysis considers analyzing. Any basic block that is not in
     * this range will not be analyzed, i.e. counting.
     */
    AddrRange bbValidAddrRange;

    /**
     * We only consider the loops within this address range as candidates for
     * marking the execution points.
     */
    AddrRange markerValidAddrRange;

    /**
     * Any basic block that is in this range will not be analyzed.
     */
    std::vector<AddrRange> bbExcludedAddrRanges;

    /**
     * Only when this is set to true, the LooppointAnalysis will listen to the
     * probe points.
     */
    bool ifListening;

    /**
     * The counter for the number of instructions within the current basic
     * block.
     */
    uint64_t bbInstCounter;

    /**
     * The basic block vector for the current core that the LooppointAnalysis
     * is attached to.
     */
    std::unordered_map<Addr, uint64_t> localBBV;

    /**
     * This function updates the localBBV for the input PC's basic block.
     * Specifically, it increments the count of the input PC's basic block in
     * the localBBV.
     */
    void updateLocalBBV(const Addr pc);

  public:
    std::unordered_map<Addr, uint64_t>
    getLocalBBV() const
    {
        return localBBV;
    };

    void
    clearLocalBBV()
    {
        localBBV.clear();
    };
};

class LooppointAnalysisManager: public SimObject
{
  public:
    LooppointAnalysisManager(const LooppointAnalysisManagerParams &params);

    /**
     * This function is called by the LooppointAnalysis probe listener when it
     * finds a valid backward branch that can be used as a marker.
     * Specifically, it increments the backwardBranchCounter for the input PC.
     */
    void countBackwardBranch(const Addr pc);

    /**
     * This function is called by the LooppointAnalysis probe listener when it
     * reaches the branch instruction for a valid basic block.
     * Specifically, it increments the globalBBV for the input PC.
     */
    void updateGlobalBBV(const Addr pc);

  private:
    /**
     * This counter is for the valid backward branches that we consider as
     * candidates for marking the execution points.
     * The key is the Program Counter address of the backward branch, and the
     * value is the number of times the backward branch is executed.
     */
    std::unordered_map<Addr, uint64_t> backwardBranchCounter;

    /**
     * This is the global basic block vector that contains the count of each
     * basic block that is executed.
     * The key is the Program Counter address of the basic block's branch
     * instruction, and the value is the number of times the basic block is
     * executed.
     */
    std::unordered_map<Addr, uint64_t> globalBBV;

    /**
     * This map stores the number of instructions in each basic block.
     * The key is the Program Counter address of the basic block's branch
     * instruction, and the value is the number of instructions in the basic
     * block.
     */
    std::unordered_map<Addr, uint64_t> bbInstMap;

    /**
     * This variable stores the number of instructions that we used to define
     * a region. For example, if the regionLength is 100, then every time when
     * there are 100 valid instructions executed globally through all the
     * cores, we will consider that as a region.
     */
    uint64_t regionLength;

    /**
     * This is a counter for the globally executed instructions. We use this to
     * compare with the regionLength to determine if we reach the end of a
     * region.
     */
    uint64_t globalInstCounter;

    /**
     * This variable stores the Program Counter address of the most recent
     * valid backward branch that we consider as a marker.
     */
    Addr mostRecentBackwardBranchPC;

    /**
     * This set stores the Program Counter addresses of the valid not control
     * instructions.
     */
    std::unordered_set<Addr> validNotControlPC;
    /**
     * This set stores the Program Counter addresses of the valid control
     * instructions.
     */
    std::unordered_set<Addr> validControlPC;
    /**
     * This set stores the Program Counter addresses of the encountered
     * instructions.
     */
    std::unordered_set<Addr> encounteredPC;

  public:
    bool
    ifBackwardBranch(const Addr pc) const
    {
        return backwardBranchCounter.find(pc) != backwardBranchCounter.end();
    };

    bool
    ifValidNotControl(const Addr pc) const
    {
        return validNotControlPC.find(pc) != validNotControlPC.end();
    };

    bool
    ifValidControl(const Addr pc) const
    {
        return validControlPC.find(pc) != validControlPC.end();
    };

    bool
    ifEncountered(const Addr pc) const
    {
        return encounteredPC.find(pc) != encounteredPC.end();
    };

    void
    updateValidNotControl(const Addr pc)
    {
        validNotControlPC.insert(pc);
    };

    void
    updateValidControl(const Addr pc)
    {
        validControlPC.insert(pc);
    };

    void
    updateEncountered(const Addr pc)
    {
        encounteredPC.insert(pc);
    };

    void
    updateBBInstMap(Addr pc, uint64_t inst_ount)
    {
        if (bbInstMap.find(pc) == bbInstMap.end())
        {
            bbInstMap.insert(std::make_pair(pc, inst_ount));
        }
    };

    std::unordered_map<Addr, uint64_t> getBBInstMap() const
    {
        return bbInstMap;
    };

    std::unordered_map<Addr, uint64_t>
    getGlobalBBV() const
    {
        return globalBBV;
    };

    void
    clearGlobalBBV()
    {
        globalBBV.clear();
        DPRINTF(LooppointAnalysis,"globalBBV is cleared\n");
    };

    uint64_t
    getGlobalInstCounter() const
    {
        return globalInstCounter;
    };

    void
    clearGlobalInstCounter()
    {
        globalInstCounter = 0;
        DPRINTF(LooppointAnalysis,"globalInstCounter is cleared\n current "
            "globalInstCounter = %lu\n", globalInstCounter);
    };

    void
    incrementGlobalInstCounter()
    {
        globalInstCounter++;
    };

    Addr
    getMostRecentBackwardBranchPC() const
    {
        return mostRecentBackwardBranchPC;
    };

    std::unordered_map<Addr, uint64_t>
    getBackwardBranchCounter() const
    {
        return backwardBranchCounter;
    };

    uint64_t
    getMostRecentBackwardBranchCount() const
    {
        return backwardBranchCounter.find(mostRecentBackwardBranchPC)->second;
    };
};

} // namespace gem5




#endif // __CPU_SIMPLE_PROBES_LOOPPOINT_ANALYSIS_HH__
