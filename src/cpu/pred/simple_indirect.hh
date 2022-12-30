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

#ifndef __CPU_PRED_INDIRECT_HH__
#define __CPU_PRED_INDIRECT_HH__

#include <deque>

#include "cpu/inst_seq.hh"
#include "cpu/pred/indirect.hh"
#include "params/SimpleIndirectPredictor.hh"

namespace gem5
{

namespace branch_prediction
{

class SimpleIndirectPredictor : public IndirectPredictor
{
  public:
    SimpleIndirectPredictor(const SimpleIndirectPredictorParams &params);

    bool lookup(Addr br_addr, PCStateBase& br_target, ThreadID tid);
    void recordIndirect(Addr br_addr, Addr tgt_addr, InstSeqNum seq_num,
                        ThreadID tid);
    void commit(InstSeqNum seq_num, ThreadID tid, void * indirect_history);
    void squash(InstSeqNum seq_num, ThreadID tid);
    void recordTarget(InstSeqNum seq_num, void * indirect_history,
                      const PCStateBase& target, ThreadID tid);
    void genIndirectInfo(ThreadID tid, void* & indirect_history);
    void updateDirectionInfo(ThreadID tid, bool actually_taken);
    void deleteIndirectInfo(ThreadID tid, void * indirect_history);
    void changeDirectionPrediction(ThreadID tid, void * indirect_history,
                                   bool actually_taken);

  private:
    const bool hashGHR;
    const bool hashTargets;
    const unsigned numSets;
    const unsigned numWays;
    const unsigned tagBits;
    const unsigned pathLength;
    const unsigned instShift;
    const unsigned ghrNumBits;
    const unsigned ghrMask;

    struct IPredEntry
    {
        Addr tag = 0;
        std::unique_ptr<PCStateBase> target;
    };

    std::vector<std::vector<IPredEntry> > targetCache;

    Addr getSetIndex(Addr br_addr, unsigned ghr, ThreadID tid);
    Addr getTag(Addr br_addr);

    struct HistoryEntry
    {
        HistoryEntry(Addr br_addr, Addr tgt_addr, InstSeqNum seq_num)
            : pcAddr(br_addr), targetAddr(tgt_addr), seqNum(seq_num) { }
        Addr pcAddr;
        Addr targetAddr;
        InstSeqNum seqNum;
    };


    struct ThreadInfo
    {
        std::deque<HistoryEntry> pathHist;
        unsigned headHistEntry = 0;
        unsigned ghr = 0;
    };

    std::vector<ThreadInfo> threadInfo;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_INDIRECT_HH__
