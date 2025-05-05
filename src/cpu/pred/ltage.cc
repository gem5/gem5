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
 * Copyright (c) 2014 The University of Wisconsin
 *
 * Copyright (c) 2006 INRIA (Institut National de Recherche en
 * Informatique et en Automatique  / French National Research Institute
 * for Computer Science and Applied Mathematics)
 *
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
 * Implementation of a L-TAGE branch predictor
 */

#include "cpu/pred/ltage.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"
#include "debug/LTage.hh"

namespace gem5
{

namespace branch_prediction
{

LTAGE::LTAGE(const LTAGEParams &params)
  : TAGE(params), loopPredictor(params.loop_predictor)
{
}

void
LTAGE::init()
{
    TAGE::init();
}

void
LTAGE::branchPlaceholder(ThreadID tid, Addr pc,
                         bool uncond, void * &bpHistory)
{
    LTageBranchInfo *bi = new LTageBranchInfo(*tage, *loopPredictor,
                                              pc, !uncond);
    bpHistory = (void*)(bi);
}

//prediction
bool
LTAGE::predict(ThreadID tid, Addr branch_pc, bool cond_branch, void* &b)
{
    LTageBranchInfo *bi = new LTageBranchInfo(*tage, *loopPredictor,
                                              branch_pc, cond_branch);
    b = (void*)(bi);

    bool pred_taken = tage->tagePredict(tid, branch_pc, cond_branch,
                                        bi->tageBranchInfo);

    pred_taken = loopPredictor->loopPredict(tid, branch_pc, cond_branch,
                                            bi->lpBranchInfo, pred_taken,
                                            instShiftAmt);
    if (cond_branch) {
        if (bi->lpBranchInfo->loopPredUsed) {
            bi->tageBranchInfo->provider = LOOP;
        }
        DPRINTF(LTage, "Predict for %lx: taken?:%d, loopTaken?:%d, "
                "loopValid?:%d, loopUseCounter:%d, tagePred:%d, altPred:%d\n",
                branch_pc, pred_taken, bi->lpBranchInfo->loopPred,
                bi->lpBranchInfo->loopPredValid,
                loopPredictor->getLoopUseCounter(),
                bi->tageBranchInfo->tagePred, bi->tageBranchInfo->altTaken);
    }

    // record final prediction
    bi->lpBranchInfo->predTaken = pred_taken;

    return pred_taken;
}

// PREDICTOR UPDATE
void
LTAGE::update(ThreadID tid, Addr pc, bool taken, void * &bp_history,
              bool squashed, const StaticInstPtr & inst, Addr target)
{
    assert(bp_history);

    LTageBranchInfo* bi = static_cast<LTageBranchInfo*>(bp_history);

    if (squashed) {
        if (tage->isSpeculativeUpdateEnabled()) {
            // This restores the global history, then update it
            // and recomputes the folded histories.
            tage->squash(tid, taken, target, inst, bi->tageBranchInfo);

            if (bi->tageBranchInfo->condBranch) {
                loopPredictor->squashLoop(bi->lpBranchInfo);
            }
        }
        return;
    }

    int nrand = rng->random<int>() & 3;
    if (bi->tageBranchInfo->condBranch) {
        DPRINTF(LTage, "Updating tables for branch:%lx; taken?:%d\n",
                pc, taken);
        tage->updateStats(taken, bi->tageBranchInfo);

        loopPredictor->updateStats(taken, bi->lpBranchInfo);

        loopPredictor->condBranchUpdate(tid, pc, taken,
            bi->tageBranchInfo->tagePred, bi->lpBranchInfo, instShiftAmt);

        tage->condBranchUpdate(tid, pc, taken, bi->tageBranchInfo,
            nrand, target, bi->lpBranchInfo->predTaken);
    }

    tage->updateHistories(tid, pc, false, taken, target,
                           inst, bi->tageBranchInfo);

    delete bi;
    bp_history = nullptr;
}

void
LTAGE::squash(ThreadID tid, void * &bp_history)
{
    LTageBranchInfo* bi = (LTageBranchInfo*)(bp_history);

    if (bi->tageBranchInfo->condBranch) {
        loopPredictor->squash(tid, bi->lpBranchInfo);
    }

    TAGE::squash(tid, bp_history);
}

} // namespace branch_prediction
} // namespace gem5
