/*
 * Copyright (c) 2022-2023 The University of Edinburgh
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2014 The Regents of The University of Michigan
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

/* @file
 * Implementation of a bi-mode branch predictor
 */

#ifndef __CPU_PRED_BI_MODE_PRED_HH__
#define __CPU_PRED_BI_MODE_PRED_HH__

#include "base/sat_counter.hh"
#include "cpu/pred/bpred_unit.hh"
#include "params/BiModeBP.hh"

namespace gem5
{

namespace branch_prediction
{

/**
 * Implements a bi-mode branch predictor. The bi-mode predictor is a two-level
 * branch predictor that has three seprate history arrays: a taken array, a
 * not-taken array, and a choice array. The taken/not-taken arrays are indexed
 * by a hash of the PC and the global history. The choice array is indexed by
 * the PC only. Because the taken/not-taken arrays use the same index, they must
 * be the same size.
 *
 * The bi-mode branch predictor aims to eliminate the destructive aliasing that
 * occurs when two branches of opposite biases share the same global history
 * pattern. By separating the predictors into taken/not-taken arrays, and using
 * the branch's PC to choose between the two, destructive aliasing is reduced.
 */

class BiModeBP : public BPredUnit
{
  public:
    BiModeBP(const BiModeBPParams &params);
    bool lookup(ThreadID tid, Addr pc, void * &bpHistory) override;
    void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target,  void * &bpHistory) override;
    void squash(ThreadID tid, void * &bpHistory) override;
    void update(ThreadID tid, Addr pc, bool taken,
                void * &bpHistory, bool squashed,
                const StaticInstPtr &inst, Addr corrTarget) override;

  private:
    void updateGlobalHistReg(ThreadID tid, bool taken);
    void uncondBranch(ThreadID tid, Addr pc, void * &bpHistory);

    struct BPHistory
    {
        unsigned globalHistoryReg;
        // was the taken array's prediction used?
        // true: takenPred used
        // false: notPred used
        bool takenUsed;
        // prediction of the taken array
        // true: predict taken
        // false: predict not-taken
        bool takenPred;
        // prediction of the not-taken array
        // true: predict taken
        // false: predict not-taken
        bool notTakenPred;
        // the final taken/not-taken prediction
        // true: predict taken
        // false: predict not-taken
        bool finalPred;
    };

    std::vector<unsigned> globalHistoryReg;
    unsigned globalHistoryBits;
    unsigned historyRegisterMask;

    unsigned choicePredictorSize;
    unsigned choiceCtrBits;
    unsigned choiceHistoryMask;
    unsigned globalPredictorSize;
    unsigned globalCtrBits;
    unsigned globalHistoryMask;

    // choice predictors
    std::vector<SatCounter8> choiceCounters;
    // taken direction predictors
    std::vector<SatCounter8> takenCounters;
    // not-taken direction predictors
    std::vector<SatCounter8> notTakenCounters;

    unsigned choiceThreshold;
    unsigned takenThreshold;
    unsigned notTakenThreshold;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_BI_MODE_PRED_HH__
