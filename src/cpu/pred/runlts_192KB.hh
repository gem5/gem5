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

#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include "cpu/base.hh"
#include "cpu/inst_res.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/pred/conditional.hh"
#include "cpu/reg_class.hh"
#include "params/RUNLTS.hh"
#include "sim/eventq.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

namespace o3
{
class CPU;
}

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
    void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target, const StaticInstPtr &inst,
                         void *&bp_history) override;
    void update(ThreadID tid, Addr pc, bool taken, void *&bp_history,
                bool squashed, const StaticInstPtr &inst,
                Addr target) override;
    void squash(ThreadID tid, void *&bp_history) override;
    void branchPlaceholder(ThreadID tid, Addr pc, bool uncond,
                           void *&bp_history) override;
    void setProbeTarget(BaseCPU *target);
    void regProbeListeners() override;
    DrainState drain() override;

  private:
    struct DecodeEvent
    {
        Tick when;
        ThreadID tid;
        InstSeqNum seqNum;
        StaticInstPtr staticInst;
    };

    struct WritebackEvent
    {
        Tick when;
        ThreadID tid;
        InstSeqNum seqNum;
        StaticInstPtr staticInst;
        std::vector<std::pair<RegId, InstResult>> destRegValues;
    };

    struct ThreadState;
    ThreadState &threadState(ThreadID tid);
    void fetched(const o3::DynInstPtr &inst);
    void decoded(const o3::DynInstPtr &inst);
    void writtenBack(const o3::DynInstPtr &inst);
    void squashDecided(const std::pair<ThreadID, InstSeqNum> &squash);
    void processFeedback();
    void scheduleFeedback(Tick when);

    const bool useLogical;
    ProbeManager *cpuProbeManager;
    o3::CPU *cpu;
    Cycles decodeToFetchDelay;
    Cycles iewToFetchDelay;
    EventFunctionWrapper feedbackEvent;
    std::deque<DecodeEvent> decodedEvents;
    std::deque<WritebackEvent> writtenBackEvents;
    std::vector<ProbeListenerPtr<>> probeListeners;
    std::vector<std::unique_ptr<ThreadState>> threadStates;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_RUNLTS_192KB_HH__
