/*
 * The Clear BSD License
 *
 * Copyright (c) 2026 Toru Koizumi
 * Copyright (c) 2026 Toshiki Maekawa
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CPU_PRED_RUNLTS_192KB_HH__
#define __CPU_PRED_RUNLTS_192KB_HH__

#include <memory>
#include <vector>

#include "cpu/pred/conditional.hh"
#include "params/RUNLTS.hh"

namespace gem5
{

namespace o3
{
struct InstructionEvent;
struct InstructionFetched;
struct InstructionDecoded;
struct InstructionWrittenBack;
struct InstructionSquash;
} // namespace o3

namespace branch_prediction
{

/**
 * Implementation of the RUNLTS branch predictor described in
 *
 * T. Koizumi, T. Maekawa, M. Mizuno, M. Kuroki, T. Tsumura and R. Shioya,
 * "RUNLTS: Branch Prediction with Register-Value Correlations and Hierarchical
 * Table Orchestration," 2026 ACM/IEEE 53rd Annual International Symposium on
 * Computer Architecture (ISCA), Raleigh, NC, USA, 2026, pp. 543-558,
 * doi: 10.1109/ISCA66397.2026.00051.
 *
 * The predictor supports Seq-RBias and Log-RBias variants. It currently
 * supports only the coupled frontend.
 */

class RUNLTS : public ConditionalPredictor
{
  public:
    using Params = RUNLTSParams;
    RUNLTS(const Params &params);

    Prediction lookup(ThreadID tid, Addr pc, void *&bp_history) override;
    Prediction lookup(ThreadID tid, Addr pc, InstSeqNum seq_no,
                      void *&bp_history) override;
    void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target, const StaticInstPtr &inst,
                         void *&bp_history) override;
    void updateHistories(ThreadID tid, Addr pc, InstSeqNum seq_no, bool uncond,
                         bool taken, Addr target, const StaticInstPtr &inst,
                         void *&bp_history) override;
    void update(ThreadID tid, Addr pc, bool taken, void *&bp_history,
                bool squashed, const StaticInstPtr &inst,
                Addr target) override;
    void squash(ThreadID tid, void *&bp_history) override;
    void branchPlaceholder(ThreadID tid, Addr pc, bool uncond,
                           void *&bp_history) override;
    bool
    requiresInstructionEvents() const override
    { return true; }
    void instructionEvent(const o3::InstructionEvent &event) override;

  private:
    struct ThreadState;
    ThreadState &threadState(ThreadID tid);
    void handleInstructionEvent(const o3::InstructionFetched &event);
    void handleInstructionEvent(const o3::InstructionDecoded &event);
    void handleInstructionEvent(const o3::InstructionWrittenBack &event);
    void handleInstructionEvent(const o3::InstructionSquash &event);

    const bool useLogical;
    std::vector<std::unique_ptr<ThreadState>> threadStates;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_RUNLTS_192KB_HH__
