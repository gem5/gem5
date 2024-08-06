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
#include "cpu/probes/pc_count_pair.hh"
#include "cpu/simple_thread.hh"
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

    void checkPc(const std::pair<SimpleThread*, StaticInstPtr>& inst_pair);

    void startListening();
    void stopListening();

    typedef ProbeListenerArg<LooppointAnalysis,
        std::pair<SimpleThread*, StaticInstPtr>> looppointAnalysisListener;

  private:

    LooppointAnalysisManager *lpaManager;
    AddrRange bbValidAddrRange;
    AddrRange markerValidAddrRange;
    std::vector<AddrRange> bbExcludedAddrRanges;
    bool ifListening;

    uint64_t bbInstCounter;

    std::unordered_map<Addr, uint64_t> localBBV;

    void updateLocalBBV(const Addr pc);

  public:
    std::unordered_map<Addr, uint64_t> getLocalBBV() const
    {
        return localBBV;
    };

    void clearLocalBBV()
    {
        localBBV.clear();
    };
};

class LooppointAnalysisManager: public SimObject
{
  public:
    LooppointAnalysisManager(const LooppointAnalysisManagerParams &params);

    void countPc(const Addr pc);
    void updateBBV(const Addr pc);

  private:
    std::unordered_map<Addr, uint64_t> loopCounter;
    std::unordered_map<Addr, uint64_t> globalBBV;
    std::unordered_map<Addr, uint64_t> bbInstMap;

    uint64_t regionLength;
    uint64_t globalInstCounter;

    Addr mostRecentLoopPC;

    std::unordered_set<Addr> backwardBranchPC;
    std::unordered_set<Addr> validNotControlPC;
    std::unordered_set<Addr> validControlPC;
    std::unordered_set<Addr> encounteredPC;

  public:
    bool ifBackwardBranch(const Addr pc) const
    {
        return backwardBranchPC.find(pc) != backwardBranchPC.end();
    };

    bool ifValidNotControl(const Addr pc) const
    {
        return validNotControlPC.find(pc) != validNotControlPC.end();
    };

    bool ifValidControl(const Addr pc) const
    {
        return validControlPC.find(pc) != validControlPC.end();
    };

    bool ifEncountered(const Addr pc) const
    {
        return encounteredPC.find(pc) != encounteredPC.end();
    };

    void updateBackwardBranch(const Addr pc)
    {
        backwardBranchPC.insert(pc);
    };

    void updateValidNotControl(const Addr pc)
    {
        validNotControlPC.insert(pc);
    };

    void updateValidControl(const Addr pc)
    {
        validControlPC.insert(pc);
    };

    void updateEncountered(const Addr pc)
    {
        encounteredPC.insert(pc);
    };

    void updateBBInstMap(Addr pc, uint64_t inst_ount)
    {
        if (bbInstMap.find(pc) == bbInstMap.end())
        {
            bbInstMap.insert(std::make_pair(pc, inst_ount));
        }
    };

    std::unordered_map<Addr, uint64_t> getGlobalBBV() const
    {
        return globalBBV;
    };

    void clearGlobalBBV()
    {
        globalBBV.clear();
    };

    uint64_t getGlobalInstCounter() const
    {
        return globalInstCounter;
    };

    void clearGlobalInstCounter()
    {
        globalInstCounter = 0;
    };

    void incrementGlobalInstCounter()
    {
        globalInstCounter++;
    };

    Addr getMostRecentLoopPC() const
    {
        return mostRecentLoopPC;
    };

    std::unordered_map<Addr, uint64_t> getLoopCounter() const
    {
        return loopCounter;
    };

    uint64_t getMostRecentLoopCount() const
    {
        return loopCounter.find(mostRecentLoopPC)->second;
    };
};

} // namespace gem5




#endif // __CPU_SIMPLE_PROBES_LOOPPOINT_ANALYSIS_HH__
