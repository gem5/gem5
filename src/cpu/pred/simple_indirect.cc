/*
 * Copyright (c) 2014 ARM Limited
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

#include "cpu/pred/simple_indirect.hh"

#include "base/intmath.hh"
#include "debug/Indirect.hh"

namespace gem5
{

namespace branch_prediction
{

SimpleIndirectPredictor::SimpleIndirectPredictor(
    const SimpleIndirectPredictorParams &params)
    : IndirectPredictor(params),
      hashGHR(params.indirectHashGHR),
      hashTargets(params.indirectHashTargets),
      numSets(params.indirectSets),
      numWays(params.indirectWays),
      tagBits(params.indirectTagSize),
      pathLength(params.indirectPathLength),
      speculativePathLength(params.speculativePathLength),
      instShift(params.instShiftAmt),
      ghrNumBits(params.indirectGHRBits),
      ghrMask((1 << params.indirectGHRBits) - 1),
      stats(this)
{
    if (!isPowerOf2(numSets)) {
        panic("Indirect predictor requires power of 2 number of sets");
    }

    threadInfo.resize(params.numThreads);

    targetCache.resize(numSets);
    for (unsigned i = 0; i < numSets; i++) {
        targetCache[i].resize(numWays);
    }

    fatal_if(ghrNumBits > (sizeof(ThreadInfo::ghr) * 8),
             "ghr_size is too big");
}

void
SimpleIndirectPredictor::reset()
{
    DPRINTF(Indirect, "Reset Indirect predictor\n");

    for (auto &ti : threadInfo) {
        ti.ghr = 0;
        ti.pathHist.clear();
    }

    for (unsigned i = 0; i < numSets; i++) {
        for (unsigned j = 0; j < numWays; j++) {
            targetCache[i][j].tag = 0;
        }
    }
}

void
SimpleIndirectPredictor::genIndirectInfo(ThreadID tid, void *&i_history)
{
    // Record the GHR as it was before this prediction
    // It will be used to recover the history in case this prediction is
    // wrong or belongs to bad path
    IndirectHistory *history = new IndirectHistory;
    history->ghr = threadInfo[tid].ghr;
    i_history = static_cast<void *>(history);
}

void
SimpleIndirectPredictor::updateDirectionInfo(ThreadID tid, bool taken, Addr pc,
                                             Addr target)
{
    // Direction history
    threadInfo[tid].ghr <<= 1;
    threadInfo[tid].ghr |= taken;
    threadInfo[tid].ghr &= ghrMask;
}

// Interface methods ------------------------------
const PCStateBase *
SimpleIndirectPredictor::lookup(ThreadID tid, InstSeqNum sn, Addr pc,
                                void *&i_history)
{
    assert(i_history == nullptr);

    genIndirectInfo(tid, i_history);
    IndirectHistory *history = static_cast<IndirectHistory *>(i_history);

    history->pcAddr = pc;
    history->was_indirect = true;

    /** Do the prediction for indirect branches (no returns) */
    PCStateBase *target = nullptr;
    history->hit = lookup(tid, pc, target, history);
    return target;
}

bool
SimpleIndirectPredictor::lookup(ThreadID tid, Addr br_addr,
                                PCStateBase *&target,
                                IndirectHistory *&history)
{
    history->set_index = getSetIndex(br_addr, tid);
    history->tag = getTag(br_addr);
    assert(history->set_index < numSets);
    stats.lookups++;

    DPRINTF(Indirect,
            "Looking up PC:%#x, (set:%d, tag:%d), "
            "ghr:%#x, pathHist sz:%#x\n",
            history->pcAddr, history->set_index, history->tag, history->ghr,
            threadInfo[tid].pathHist.size());

    const auto &iset = targetCache[history->set_index];
    for (auto way = iset.begin(); way != iset.end(); ++way) {
        // tag may be 0 and match the default in way->tag, so we also have to
        // check that way->target has been initialized.
        if (way->tag == history->tag && way->target) {
            DPRINTF(Indirect, "Hit %x (target:%s)\n", br_addr, *way->target);
            set(target, *way->target);
            history->hit = true;
            stats.hits++;
            return history->hit;
        }
    }
    DPRINTF(Indirect, "Miss %x\n", br_addr);
    history->hit = false;
    stats.misses++;
    return history->hit;
}

void
SimpleIndirectPredictor::commit(ThreadID tid, InstSeqNum sn, void *&i_history)
{
    if (i_history == nullptr)
        return;
    // we do not need to recover the GHR, so delete the information
    IndirectHistory *history = static_cast<IndirectHistory *>(i_history);

    DPRINTF(Indirect, "Committing seq:%d, PC:%#x, ghr:%#x, pathHist sz:%lu\n",
            sn, history->pcAddr, history->ghr,
            threadInfo[tid].pathHist.size());

    delete history;
    i_history = nullptr;

    /** Delete histories if the history grows to much */
    while (threadInfo[tid].pathHist.size() >=
           (pathLength + speculativePathLength)) {
        threadInfo[tid].pathHist.pop_front();
    }
}

void
SimpleIndirectPredictor::update(ThreadID tid, InstSeqNum sn, Addr pc,
                                bool squash, bool taken,
                                const PCStateBase &target, BranchType br_type,
                                void *&i_history)
{
    // If there is no history we did not use the indirect predictor yet.
    // Create one
    if (i_history == nullptr) {
        genIndirectInfo(tid, i_history);
    }
    IndirectHistory *history = static_cast<IndirectHistory *>(i_history);
    assert(history != nullptr);

    DPRINTF(Indirect, "Update sn:%i PC:%#x, squash:%i, ghr:%#x,path sz:%i\n",
            sn, pc, squash, history->ghr, threadInfo[tid].pathHist.size());

    /** If update was called during squash we need to fix the indirect
     * path history and the global path history.
     * We restore the state before this branch incorrectly updated it
     * and perform the update afterwards again.
     */
    history->was_indirect = isIndirectNoReturn(br_type);
    if (squash) {
        /** restore global history */
        threadInfo[tid].ghr = history->ghr;

        /** For indirect branches recalculate index and tag */
        if (history->was_indirect) {
            if (!threadInfo[tid].pathHist.empty()) {
                threadInfo[tid].pathHist.pop_back();
            }

            history->set_index = getSetIndex(history->pcAddr, tid);
            history->tag = getTag(history->pcAddr);

            DPRINTF(Indirect,
                    "Record Target seq:%d, PC:%#x, TGT:%#x, "
                    "ghr:%#x, (set:%x, tag:%x)\n",
                    sn, history->pcAddr, target, history->ghr,
                    history->set_index, history->tag);
        }
    }

    // Only indirect branches are recorded in the path history
    if (history->was_indirect) {
        DPRINTF(Indirect, "Recording %x seq:%d\n", history->pcAddr, sn);
        threadInfo[tid].pathHist.emplace_back(history->pcAddr,
                                              target.instAddr(), sn);

        stats.indirectRecords++;
    }

    // All branches update the global history
    updateDirectionInfo(tid, taken, history->pcAddr, target.instAddr());

    // Finally if update is called during at squash we know the target
    // we predicted was wrong therefore we update the target.
    // We only record the target if the branch was indirect and taken
    if (squash && history->was_indirect && taken)
        recordTarget(tid, sn, target, history);
}

void
SimpleIndirectPredictor::squash(ThreadID tid, InstSeqNum sn, void *&i_history)
{
    if (i_history == nullptr)
        return;

    // we do not need to recover the GHR, so delete the information
    IndirectHistory *history = static_cast<IndirectHistory *>(i_history);

    DPRINTF(Indirect,
            "Squashing seq:%d, PC:%#x, indirect:%i, "
            "ghr:%#x, pathHist sz:%#x\n",
            sn, history->pcAddr, history->was_indirect, history->ghr,
            threadInfo[tid].pathHist.size());

    // Revert the global history register.
    threadInfo[tid].ghr = history->ghr;

    // If we record this branch as indirect branch
    // remove it from the history.
    // Restore the old head in the history.
    if (history->was_indirect) {
        // Should not be empty
        if (threadInfo[tid].pathHist.size() < pathLength) {
            stats.speculativeOverflows++;
        }

        if (!threadInfo[tid].pathHist.empty()) {
            threadInfo[tid].pathHist.pop_back();
        }
    }

    delete history;
    i_history = nullptr;
}

// Internal functions ------------------------------

void
SimpleIndirectPredictor::recordTarget(ThreadID tid, InstSeqNum sn,
                                      const PCStateBase &target,
                                      IndirectHistory *&history)
{
    // Should have just squashed so this branch should be the oldest
    // and it should be predicted as indirect.
    assert(!threadInfo[tid].pathHist.empty());
    assert(history->was_indirect);

    if (threadInfo[tid].pathHist.rbegin()->pcAddr != history->pcAddr) {
        DPRINTF(Indirect, "History seems to be corrupted. %#x != %#x\n",
                history->pcAddr, threadInfo[tid].pathHist.rbegin()->pcAddr);
    }

    DPRINTF(Indirect,
            "Record Target seq:%d, PC:%#x, TGT:%#x, "
            "ghr:%#x, (set:%x, tag:%x)\n",
            sn, history->pcAddr, target.instAddr(), history->ghr,
            history->set_index, history->tag);

    assert(history->set_index < numSets);
    stats.targetRecords++;

    // Update the target cache
    auto &iset = targetCache[history->set_index];
    for (auto way = iset.begin(); way != iset.end(); ++way) {
        if (way->tag == history->tag) {
            DPRINTF(Indirect,
                    "Updating Target (seq: %d br:%x set:%d target:%s)\n", sn,
                    history->pcAddr, history->set_index, target);
            set(way->target, target);
            return;
        }
    }

    DPRINTF(Indirect, "Allocating Target (seq: %d br:%x set:%d target:%s)\n",
            sn, history->pcAddr, history->set_index, target);

    // Did not find entry, random replacement
    auto &way = iset[rand() % numWays];
    way.tag = history->tag;
    set(way.target, target);
}

inline Addr
SimpleIndirectPredictor::getSetIndex(Addr br_addr, ThreadID tid)
{
    Addr hash = br_addr >> instShift;
    if (hashGHR) {
        hash ^= threadInfo[tid].ghr;
    }
    if (hashTargets) {
        unsigned hash_shift = floorLog2(numSets) / pathLength;
        for (int i = threadInfo[tid].pathHist.size() - 1, p = 0;
             i >= 0 && p < pathLength; i--, p++) {
            hash ^= (threadInfo[tid].pathHist[i].targetAddr >>
                     (instShift + p * hash_shift));
        }
    }
    return hash & (numSets - 1);
}

inline Addr
SimpleIndirectPredictor::getTag(Addr br_addr)
{
    return (br_addr >> instShift) & ((0x1 << tagBits) - 1);
}

SimpleIndirectPredictor::IndirectStats::IndirectStats(
    statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(lookups, statistics::units::Count::get(), "Number of lookups"),
      ADD_STAT(hits, statistics::units::Count::get(),
               "Number of hits of a tag"),
      ADD_STAT(misses, statistics::units::Count::get(), "Number of misses"),
      ADD_STAT(targetRecords, statistics::units::Count::get(),
               "Number of targets that where recorded/installed in the cache"),
      ADD_STAT(indirectRecords, statistics::units::Count::get(),
               "Number of indirect branches/calls recorded in the"
               " indirect hist"),
      ADD_STAT(
          speculativeOverflows, statistics::units::Count::get(),
          "Number of times more than the allowed capacity for speculative "
          "branches/calls where in flight and destroy the path history")
{}

} // namespace branch_prediction
} // namespace gem5
