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
 * Implementation of a TAGE branch predictor
 */

#include "cpu/pred/tage.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"
#include "debug/Tage.hh"

namespace gem5
{

namespace branch_prediction
{

TAGE::TAGE(const TAGEParams &params) : BPredUnit(params), tage(params.tage)
{
}

// PREDICTOR UPDATE
void
TAGE::update(ThreadID tid, Addr pc, bool taken, void * &bp_history,
              bool squashed, const StaticInstPtr & inst, Addr target)
{
    assert(bp_history);

    TageBranchInfo *bi = static_cast<TageBranchInfo*>(bp_history);
    TAGEBase::BranchInfo *tage_bi = bi->tageBranchInfo;

    if (squashed) {
        // This restores the global history, then update it
        // and recomputes the folded histories.
        tage->squash(tid, taken, target, inst, tage_bi);
        return;
    }

    int nrand = random_mt.random<int>() & 3;
    if (bi->tageBranchInfo->condBranch) {
        DPRINTF(Tage, "Updating tables for branch:%lx; taken?:%d\n",
                pc, taken);
        tage->updateStats(taken, bi->tageBranchInfo);
        tage->condBranchUpdate(tid, pc, taken, tage_bi, nrand,
                               target, bi->tageBranchInfo->tagePred);
    }

    // optional non speculative update of the histories
    tage->updateHistories(tid, pc, false, taken, target, inst, tage_bi);
    delete bi;
    bp_history = nullptr;
}

void
TAGE::squash(ThreadID tid, void * &bp_history)
{
    TageBranchInfo *bi = static_cast<TageBranchInfo*>(bp_history);
    tage->restoreHistState(tid, bi->tageBranchInfo);
    DPRINTF(Tage, "Deleting branch info: %lx\n", bi->tageBranchInfo->branchPC);
    delete bi;
    bp_history = nullptr;
}

bool
TAGE::predict(ThreadID tid, Addr pc, bool cond_branch, void* &b)
{
    TageBranchInfo *bi = new TageBranchInfo(*tage, pc, cond_branch);
    b = (void*)(bi);
    return tage->tagePredict(tid, pc, cond_branch, bi->tageBranchInfo);
}

bool
TAGE::lookup(ThreadID tid, Addr pc, void* &bp_history)
{
    bool retval = predict(tid, pc, true, bp_history);

    DPRINTF(Tage, "Lookup branch: %lx; predict:%d\n", pc, retval);

    return retval;
}

void
TAGE::updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                   Addr target, const StaticInstPtr &inst, void * &bp_history)
{
    if (bp_history == nullptr) {

        // We should only see unconditional branches
        assert(uncond);

        predict(tid, pc, false, bp_history);
    }

    // Update the global history for all branches
    TageBranchInfo *bi = static_cast<TageBranchInfo*>(bp_history);
    tage->updateHistories(tid, pc, true, taken, target, inst,
                          bi->tageBranchInfo);
}

void
TAGE::branchPlaceholder(ThreadID tid, Addr pc, bool uncond, void * &bpHistory)
{
    TageBranchInfo *bi = new TageBranchInfo(*tage, pc, !uncond);
    bpHistory = (void*)(bi);
}

} // namespace branch_prediction
} // namespace gem5
