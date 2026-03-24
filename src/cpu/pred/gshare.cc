/*
 * Copyright (c) 2024 REDS-HEIG-VD and ESL-EPFL
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
 * Implementation of a bi-mode branch predictor
 */

#include "cpu/pred/gshare.hh"

#include "base/bitfield.hh"
#include "base/intmath.hh"

namespace gem5
{

namespace branch_prediction
{

GshareBP::GshareBP(const GshareBPParams &params)
    : BPredUnit(params),
      globalHistoryReg(params.numThreads, 0),
      globalHistoryBits(ceilLog2(params.global_predictor_size)),
      globalPredictorSize(params.global_predictor_size),
      globalCtrBits(params.global_counter_bits),
      globalCtrs(globalPredictorSize, SatCounter8(globalCtrBits))
{

    if (!isPowerOf2(globalPredictorSize)) {
        fatal("Invalid global history predictor size.\n");
    }
    historyRegisterMask = mask(globalHistoryBits);
    globalHistoryMask = globalPredictorSize - 1;
    takenThreshold = (1ULL << (globalCtrBits - 1)) - 1;
}

/*
 * For an unconditional branch we set its history such that
 * everything is set to taken. I.e., its choice predictor
 * chooses the taken array and the taken array predicts taken.
 */
void
GshareBP::uncondBranch(ThreadID tid, Addr pc, void *&bp_history)
{
    BPHistory *history = new BPHistory;
    history->globalHistoryReg = globalHistoryReg[tid];
    history->finalPred = true;
    bp_history = static_cast<void *>(history);
}

void
GshareBP::updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                          Addr target, const StaticInstPtr &inst,
                          void *&bp_history)
{
    assert(uncond || bp_history);
    if (uncond) {
        uncondBranch(tid, pc, bp_history);
    }
    updateGlobalHistReg(tid, taken);
}

void
GshareBP::squash(ThreadID tid, void *&bp_history)
{
    BPHistory *history = static_cast<BPHistory *>(bp_history);
    globalHistoryReg[tid] = history->globalHistoryReg;

    delete history;
    bp_history = nullptr;
}

/*
 * Here we lookup the actual branch prediction. A hash of
 * the global history register and a branch's PC is used to
 * index into the counter, which both present a prediction.
 */
bool
GshareBP::lookup(ThreadID tid, Addr branchAddr, void *&bp_history)
{
    unsigned globalHistoryIdx =
        (((branchAddr >> instShiftAmt) ^ globalHistoryReg[tid]) &
         globalHistoryMask);

    assert(globalHistoryIdx < globalPredictorSize);

    bool final_prediction = globalCtrs[globalHistoryIdx] > takenThreshold;

    BPHistory *history = new BPHistory;
    history->globalHistoryReg = globalHistoryReg[tid];
    history->finalPred = final_prediction;
    bp_history = static_cast<void *>(history);

    return final_prediction;
}

/* Updates the counter values based on the actual branch
 * direction.
 */
void
GshareBP::update(ThreadID tid, Addr branchAddr, bool taken, void *&bp_history,
                 bool squashed, const StaticInstPtr &inst, Addr target)
{
    assert(bp_history);

    BPHistory *history = static_cast<BPHistory *>(bp_history);

    // We do not update the counters speculatively on a squash.
    // We just restore the global history register.
    if (squashed) {
        globalHistoryReg[tid] = (history->globalHistoryReg << 1) | taken;
        return;
    }

    unsigned globalHistoryIdx =
        (((branchAddr >> instShiftAmt) ^ history->globalHistoryReg) &
         globalHistoryMask);

    assert(globalHistoryIdx < globalPredictorSize);

    if (taken) {
        globalCtrs[globalHistoryIdx]++;
    } else {
        globalCtrs[globalHistoryIdx]--;
    }
    delete history;
    bp_history = nullptr;
}

void
GshareBP::updateGlobalHistReg(ThreadID tid, bool taken)
{
    globalHistoryReg[tid] = taken ? (globalHistoryReg[tid] << 1) | 1
                                  : (globalHistoryReg[tid] << 1);
    globalHistoryReg[tid] &= historyRegisterMask;
}

} // namespace branch_prediction
} // namespace gem5
