/*
 * Copyright (c) 2024-2025 REDS-HEIG-VD and ESL-EPFL
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
 */

/* @file
 * Implementation of a gshare branch predictor
 */

#ifndef __CPU_PRED_GSHARE_PRED_HH__
#define __CPU_PRED_GSHARE_PRED_HH__

#include "base/sat_counter.hh"
#include "cpu/pred/bpred_unit.hh"
#include "params/GshareBP.hh"

namespace gem5
{

namespace branch_prediction
{

/**
 * Implements a gshare branch predictor. The gshare predictor takes hash
 * of global history and the program counter to access the n-bit counter
 * to predict the branch (taken or not).This was a precursor to the bi-mode
 * branch predictor model, which handles destructive aliasing by seperating
 * taken and not taken counters.
 */

class GshareBP : public BPredUnit
{
  public:
    GshareBP(const GshareBPParams &params);
    bool lookup(ThreadID tid, Addr pc, void *&bp_history);
    void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target, const StaticInstPtr &inst,
                         void *&bp_history);
    void squash(ThreadID tid, void *&bp_history);
    void update(ThreadID tid, Addr pc, bool taken, void *&bp_history,
                bool squashed, const StaticInstPtr &inst, Addr target);

  private:
    void updateGlobalHistReg(ThreadID tid, bool taken);
    void uncondBranch(ThreadID tid, Addr pc, void *&bp_history);

    struct BPHistory
    {
        unsigned globalHistoryReg;
        bool finalPred;
    };

    std::vector<unsigned> globalHistoryReg;
    unsigned globalHistoryBits;
    unsigned historyRegisterMask;

    unsigned globalPredictorSize;
    unsigned globalCtrBits;
    unsigned globalHistoryMask;

    std::vector<SatCounter8> globalCtrs;
    unsigned takenThreshold;
};

} // namespace branch_prediction
} // namespace gem5
#endif // __CPU_PRED_GSHARE_PRED_HH__
