/*
 * Copyright (c) 2014 ARM Limited
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
      instShift(params.instShiftAmt),
      ghrNumBits(params.indirectGHRBits),
      ghrMask((1 << params.indirectGHRBits)-1)
{
    if (!isPowerOf2(numSets)) {
        panic("Indirect predictor requires power of 2 number of sets");
    }

    threadInfo.resize(params.numThreads);

    targetCache.resize(numSets);
    for (unsigned i = 0; i < numSets; i++) {
        targetCache[i].resize(numWays);
    }

    fatal_if(ghrNumBits > (sizeof(ThreadInfo::ghr)*8), "ghr_size is too big");
}

void
SimpleIndirectPredictor::genIndirectInfo(ThreadID tid,
                                         void* & indirect_history)
{
    // record the GHR as it was before this prediction
    // It will be used to recover the history in case this prediction is
    // wrong or belongs to bad path
    indirect_history = new unsigned(threadInfo[tid].ghr);
}

void
SimpleIndirectPredictor::updateDirectionInfo(
    ThreadID tid, bool actually_taken)
{
    threadInfo[tid].ghr <<= 1;
    threadInfo[tid].ghr |= actually_taken;
    threadInfo[tid].ghr &= ghrMask;
}

void
SimpleIndirectPredictor::changeDirectionPrediction(ThreadID tid,
    void * indirect_history, bool actually_taken)
{
    unsigned * previousGhr = static_cast<unsigned *>(indirect_history);
    threadInfo[tid].ghr = ((*previousGhr) << 1) + actually_taken;
    threadInfo[tid].ghr &= ghrMask;
}

bool
SimpleIndirectPredictor::lookup(Addr br_addr, PCStateBase& target,
    ThreadID tid)
{
    Addr set_index = getSetIndex(br_addr, threadInfo[tid].ghr, tid);
    Addr tag = getTag(br_addr);

    assert(set_index < numSets);

    DPRINTF(Indirect, "Looking up %x (set:%d)\n", br_addr, set_index);
    const auto &iset = targetCache[set_index];
    for (auto way = iset.begin(); way != iset.end(); ++way) {
        // tag may be 0 and match the default in way->tag, so we also have to
        // check that way->target has been initialized.
        if (way->tag == tag && way->target) {
            DPRINTF(Indirect, "Hit %x (target:%s)\n", br_addr, *way->target);
            set(target, *way->target);
            return true;
        }
    }
    DPRINTF(Indirect, "Miss %x\n", br_addr);
    return false;
}

void
SimpleIndirectPredictor::recordIndirect(Addr br_addr, Addr tgt_addr,
    InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Indirect, "Recording %x seq:%d\n", br_addr, seq_num);
    HistoryEntry entry(br_addr, tgt_addr, seq_num);
    threadInfo[tid].pathHist.push_back(entry);
}

void
SimpleIndirectPredictor::commit(InstSeqNum seq_num, ThreadID tid,
                          void * indirect_history)
{
    DPRINTF(Indirect, "Committing seq:%d\n", seq_num);
    ThreadInfo &t_info = threadInfo[tid];

    // we do not need to recover the GHR, so delete the information
    unsigned * previousGhr = static_cast<unsigned *>(indirect_history);
    delete previousGhr;

    if (t_info.pathHist.empty()) return;

    if (t_info.headHistEntry < t_info.pathHist.size() &&
        t_info.pathHist[t_info.headHistEntry].seqNum <= seq_num) {
        if (t_info.headHistEntry >= pathLength) {
            t_info.pathHist.pop_front();
        } else {
             ++t_info.headHistEntry;
        }
    }
}

void
SimpleIndirectPredictor::squash(InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Indirect, "Squashing seq:%d\n", seq_num);
    ThreadInfo &t_info = threadInfo[tid];
    auto squash_itr = t_info.pathHist.begin();
    while (squash_itr != t_info.pathHist.end()) {
        if (squash_itr->seqNum > seq_num) {
           break;
        }
        ++squash_itr;
    }
    if (squash_itr != t_info.pathHist.end()) {
        DPRINTF(Indirect, "Squashing series starting with sn:%d\n",
                squash_itr->seqNum);
    }
    t_info.pathHist.erase(squash_itr, t_info.pathHist.end());
}

void
SimpleIndirectPredictor::deleteIndirectInfo(ThreadID tid,
                                            void * indirect_history)
{
    unsigned * previousGhr = static_cast<unsigned *>(indirect_history);
    threadInfo[tid].ghr = *previousGhr;

    delete previousGhr;
}

void
SimpleIndirectPredictor::recordTarget(
    InstSeqNum seq_num, void * indirect_history, const PCStateBase& target,
    ThreadID tid)
{
    ThreadInfo &t_info = threadInfo[tid];

    unsigned * ghr = static_cast<unsigned *>(indirect_history);

    // Should have just squashed so this branch should be the oldest
    auto hist_entry = *(t_info.pathHist.rbegin());
    // Temporarily pop it off the history so we can calculate the set
    t_info.pathHist.pop_back();
    Addr set_index = getSetIndex(hist_entry.pcAddr, *ghr, tid);
    Addr tag = getTag(hist_entry.pcAddr);
    hist_entry.targetAddr = target.instAddr();
    t_info.pathHist.push_back(hist_entry);

    assert(set_index < numSets);

    auto &iset = targetCache[set_index];
    for (auto way = iset.begin(); way != iset.end(); ++way) {
        if (way->tag == tag) {
            DPRINTF(Indirect, "Updating Target (seq: %d br:%x set:%d target:"
                    "%s)\n", seq_num, hist_entry.pcAddr, set_index, target);
            set(way->target, target);
            return;
        }
    }

    DPRINTF(Indirect, "Allocating Target (seq: %d br:%x set:%d target:%s)\n",
            seq_num, hist_entry.pcAddr, set_index, target);
    // Did not find entry, random replacement
    auto &way = iset[rand() % numWays];
    way.tag = tag;
    set(way.target, target);
}


inline Addr
SimpleIndirectPredictor::getSetIndex(Addr br_addr, unsigned ghr, ThreadID tid)
{
    ThreadInfo &t_info = threadInfo[tid];

    Addr hash = br_addr >> instShift;
    if (hashGHR) {
        hash ^= ghr;
    }
    if (hashTargets) {
        unsigned hash_shift = floorLog2(numSets) / pathLength;
        for (int i = t_info.pathHist.size()-1, p = 0;
             i >= 0 && p < pathLength; i--, p++) {
            hash ^= (t_info.pathHist[i].targetAddr >>
                     (instShift + p*hash_shift));
        }
    }
    return hash & (numSets-1);
}

inline Addr
SimpleIndirectPredictor::getTag(Addr br_addr)
{
    return (br_addr >> instShift) & ((0x1<<tagBits)-1);
}

} // namespace branch_prediction
} // namespace gem5
