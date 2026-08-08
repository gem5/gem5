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

/*
 * Implementation of the RUNLTS branch predictor described in
 *
 * T. Koizumi, T. Maekawa, M. Mizuno, M. Kuroki, T. Tsumura and R. Shioya,
 * "RUNLTS: Branch Prediction with Register-Value Correlations and Hierarchical
 * Table Orchestration," 2026 ACM/IEEE 53rd Annual International Symposium on
 * Computer Architecture (ISCA), Raleigh, NC, USA, 2026, pp. 543-558,
 * doi: 10.1109/ISCA66397.2026.00051.
 *
 * The implementation of the RUNLTS contributions described in this paper is
 * licensed under the BSD 3-Clause Clear License.
 */

#include "cpu/pred/runlts_192KB.hh"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "base/compiler.hh"
#include "config/use_arm_isa.hh"
#include "config/use_riscv_isa.hh"
#include "config/use_x86_isa.hh"

#if USE_ARM_ISA
#include "arch/arm/regs/cc.hh"
#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/vec.hh"

#endif
#if USE_RISCV_ISA
#include "arch/riscv/regs/int.hh"

#endif
#if USE_X86_ISA
#include "arch/x86/regs/int.hh"

#endif
#include "cpu/o3/comm.hh"
#include "cpu/op_class.hh"
#include "cpu/reg_class.hh"
#include "cpu/static_inst.hh"
#include "debug/RUNLTS.hh"

namespace gem5
{

#if USE_ARM_ISA
static bool
is_floating_op(const StaticInstPtr &inst)
{
    return inst->opClass() == FloatAddOp || inst->opClass() == FloatCmpOp ||
           inst->opClass() == FloatCvtOp || inst->opClass() == FloatMultOp ||
           inst->opClass() == FloatMultAccOp ||
           inst->opClass() == FloatDivOp || inst->opClass() == FloatSqrtOp;
}
#endif

static std::optional<size_t>
logical_register_index(const RegId &reg, const StaticInstPtr &inst)
{
#if !USE_ARM_ISA
    (void)inst;
#endif
    const RegClass &reg_class = reg.regClass();
    const RegIndex index = reg.index();

    if (reg_class.type() == IntRegClass) {
#if USE_ARM_ISA
        if (&reg_class == &ArmISA::intRegClass) {
            if (index == ArmISA::int_reg::Spx.index()) {
                return 31;
            }
            return index < 32 ? std::optional<size_t>(index) : std::nullopt;
        }
        if (&reg_class == &ArmISA::flatIntRegClass) {
            return index < 32 ? std::optional<size_t>(index) : std::nullopt;
        }
#endif
#if USE_X86_ISA
        if (&reg_class == &X86ISA::intRegClass ||
            &reg_class == &X86ISA::flatIntRegClass) {
            return index < X86ISA::int_reg::NumArchRegs
                       ? std::optional<size_t>(index)
                       : std::nullopt;
        }
#endif
#if USE_RISCV_ISA
        if (&reg_class == &RiscvISA::intRegClass) {
            return index < RiscvISA::int_reg::NumArchRegs
                       ? std::optional<size_t>(index)
                       : std::nullopt;
        }
#endif
        fatal("RUNLTS supports Arm, X86, and RISC-V only");
    }

#if USE_ARM_ISA
    if (&reg_class == &ArmISA::ccRegClass) {
        return 64;
    }
    if (&reg_class == &ArmISA::vecRegClass && index < 32 &&
        is_floating_op(inst)) {
        return 32 + index;
    }
#endif

    return std::nullopt;
}

namespace branch_prediction
{

namespace runlts_impl
{

static constexpr bool UseSC = true;

static constexpr int HISTBUFFERLENGTH = 8192;
static constexpr int NHIST = 23;
static constexpr int PHISTWIDTH = 27;
static constexpr int MINHIST = 6;
static constexpr double HistRate = 1.19;
static constexpr int Born2 = 18;

static constexpr int LOGG_1 = 11;
static constexpr int LOGG_BORN = 11;
static constexpr int TB_1 = 9;
static constexpr int TB_BORN = 13;
static constexpr int CWIDTH = 3;
static constexpr int UWIDTH = 1;
static constexpr int NBANKLOW = 9;
static constexpr int NBANKHIGH = 25;
static constexpr int BORN = 5;
static constexpr int BORNTICK = 1024;
static constexpr int NNN = 2;

static constexpr int HYSTSHIFT = 2;
static constexpr int LOGB = 17;

// Statistical corrector: BIAS components
static constexpr int PERCWIDTH = 6;
static constexpr int WIDTHRES = 12;
static constexpr int WIDTHRESP = 8;
static constexpr int LOGSIZEUP = 6;
static constexpr int LOGSIZEUPS = (LOGSIZEUP / 2);
static constexpr int CONFWIDTH = 7;
static constexpr int EWIDTH = 6;

// Backward GEHL (global backward history) component
static constexpr bool UseBackwardGEHL = 1;
static constexpr int LOGGNB = 11;

// Forward path GEHL component
static constexpr bool UsePathGEHL = 1;
static constexpr int SeqLOGPNB = 9;
static constexpr int LogLOGPNB = 10;

static constexpr bool UseRBias = true;
static constexpr int LogRBiasSize = 10; // 10: 5.25KiB, 11: 11.5KiB, 12: 23KiB

static constexpr int MaxDistance = 64;
static constexpr size_t ST_NBuf = 256;
static constexpr size_t LogicalRegisterCount = MaxDistance + 1;

static constexpr bool UseLocalH = true;
static constexpr size_t LOGLNB = 11; // 2 2K + 2 * 1K-entry tables

static constexpr bool UseLocalS = true;
static constexpr size_t LOGSNB = 11;

static constexpr bool UseLocalT = true;
static constexpr size_t LOGTNB = 11;

static constexpr bool UseIMLI = true;
static constexpr size_t LOGSIZEUPI = 8;

static constexpr bool UseCallStackHistory = true;
static constexpr int LOGCNB = 11;

namespace sLocal1
{
static constexpr size_t FeatureSize = 256;
size_t
get_index(uint64_t PC)
{ return (PC ^ (PC >> 2)) % FeatureSize; }
} // namespace sLocal1

namespace sLocal2
{
static constexpr size_t FeatureSize = 16;
size_t
get_index(uint64_t PC)
{ return (PC ^ (PC >> 5)) % FeatureSize; }
} // namespace sLocal2

namespace sLocal3
{
static constexpr size_t FeatureSize = 16;
size_t
get_index(uint64_t PC)
{ return (PC ^ (PC >> LOGTNB)) % FeatureSize; }
} // namespace sLocal3

namespace sCallStack
{
static constexpr size_t FeatureSize = 8;
} // namespace sCallStack

static constexpr int RBiasScale = 5;
static constexpr size_t RBiasNBanks = 8;

struct FoldedHistory
{
    unsigned comp = 0;
    int c_len = 0;
    int o_len = 0;
    int out_point = 0;

    void
    init(int original_length, int compressed_length)
    {
        comp = 0;
        o_len = original_length;
        c_len = compressed_length;
        out_point = o_len % c_len;
    }

    void
    update(const std::array<uint8_t, HISTBUFFERLENGTH> &h, int pt)
    {
        comp = (comp << 1) ^ h[pt & (HISTBUFFERLENGTH - 1)];
        comp ^= h[(pt + o_len) & (HISTBUFFERLENGTH - 1)] << out_point;
        comp ^= (comp >> c_len);
        comp &= ((1u << c_len) - 1);
    }
};

using TageIndexFolds = std::array<FoldedHistory, NHIST + 1>;
using TageTagFolds = std::array<FoldedHistory, NHIST + 1>;

struct History
{
    std::array<uint8_t, HISTBUFFERLENGTH> ghist{};
    int ptghist = 0;

    TageIndexFolds ch_i{};
    std::array<TageTagFolds, 2> ch_t{};

    uint64_t GHIST = 0;
    uint64_t phist = 0;  // path history
    uint64_t fphist = 0; // forward taken path history

    std::array<long long, sLocal1::FeatureSize> L_shist;
    std::array<long long, sLocal2::FeatureSize> S_slhist;
    std::array<long long, sLocal3::FeatureSize> T_slhist;

    size_t ubhist = 0; // uniquified backward taken path history
    size_t last_backward_target = 0;
    size_t last_backward_pc = 0;
    size_t BrIMLI = 0;
    size_t TaIMLI = 0;

    std::array<uint64_t, sCallStack::FeatureSize> call_stack_history = {};
    size_t call_stack_ptr = 0;

    long long &
    local1_hist(uint64_t PC)
    { return L_shist.at(sLocal1::get_index(PC)); }
    long long &
    local2_hist(uint64_t PC)
    { return S_slhist.at(sLocal2::get_index(PC)); }
    long long &
    local3_hist(uint64_t PC)
    { return T_slhist.at(sLocal3::get_index(PC)); }
    uint64_t &
    call_stack_hist()
    { return call_stack_history.at(call_stack_ptr); }
    long long
    local1_hist(uint64_t PC) const
    { return L_shist.at(sLocal1::get_index(PC)); }
    long long
    local2_hist(uint64_t PC) const
    { return S_slhist.at(sLocal2::get_index(PC)); }
    long long
    local3_hist(uint64_t PC) const
    { return T_slhist.at(sLocal3::get_index(PC)); }
    uint64_t
    call_stack_hist() const
    { return call_stack_history.at(call_stack_ptr); }
};

struct FoldedHistorySnapshot
{
    unsigned comp = 0;
};

using TageIndexSnapshots = std::array<FoldedHistorySnapshot, NHIST + 1>;
using TageTagSnapshots = std::array<FoldedHistorySnapshot, NHIST + 1>;

struct PredictionHistory
{
    std::array<uint64_t, MaxDistance + 1> register_values{};
    std::array<int, RBiasNBanks> best_reg{};

    TageIndexSnapshots ch_i{};
    std::array<TageTagSnapshots, 2> ch_t{};

    uint64_t GHIST = 0;
    uint64_t phist = 0;
    uint64_t fphist = 0;
    long long L_shist = 0;
    long long S_slhist = 0;
    long long T_slhist = 0;
    size_t BrIMLI = 0;
    size_t TaIMLI = 0;
    uint64_t call_stack_history = 0;
    int perceptron_sum_at_prediction = 0;
    InstSeqNum seq_no = 0;
};

struct HistoryDelta
{
    // Save only the entries historyUpdate() mutates for this branch.
    bool modified = false;
    bool conditional = false;
    int ptghist = 0;
    std::array<unsigned, NHIST + 1> ch_i{};
    std::array<std::array<unsigned, NHIST + 1>, 2> ch_t{};
    std::array<uint8_t, 3> overwritten_ghist{};
    int ghist_count = 0;

    uint64_t GHIST = 0;
    uint64_t phist = 0;
    uint64_t fphist = 0;
    size_t local_index = 0;
    size_t second_local_index = 0;
    size_t third_local_index = 0;
    long long L_shist = 0;
    long long S_slhist = 0;
    long long T_slhist = 0;

    size_t ubhist = 0;
    size_t last_backward_target = 0;
    size_t last_backward_pc = 0;
    size_t BrIMLI = 0;
    size_t TaIMLI = 0;

    size_t call_stack_ptr = 0;
    uint64_t call_stack_value = 0;
    bool call_stack_next_modified = false;
    size_t call_stack_next_ptr = 0;
    uint64_t call_stack_next_value = 0;
};

struct LogicalRegStateEntry
{
    bool valid = false;
    uint64_t payload = 0;
    uint8_t age = 0;
};

using LogicalRegState = std::array<LogicalRegStateEntry, LogicalRegisterCount>;

struct SeqRegStateEntry
{
    bool valid = false;
    uint64_t payload = 0;
};

using SeqRegState = std::array<SeqRegStateEntry, ST_NBuf>;

struct PredVars
{
    int BI = 0;                 // bimodal index
    int GI[NHIST + 1]{};        // per-table indices
    unsigned GTAG[NHIST + 1]{}; // per-table partial tags
    int BIM = 0;     // bimodal combined state (pred:bit1 | hyst:bit0)
    int HitBank = 0; // longest matching table id (0 if none)
    int AltBank = 0; // alternate matching table id (0 if none)
    bool LongestMatchPred = false; // prediction of the longest match bank
    bool alttaken = false;         // alternate prediction
    bool HighConf = false;
    bool LowConf = false;
    bool MedConf = false;
    bool AltConf = false;

    // Predictions retained for training.
    bool tage_pred = false;
    bool final_pred = false;
    int sc_lsum = 0;
    int sc_threshold = 0;
};

struct BiasArgs
{
    int hitBank = 0;
    int altBank = 0;
    bool highConf = false;
    bool lowConf = false;
    bool longestMatchPred = false;
    bool altTaken = false;
    bool tagePred = false;
};

namespace sBiasNormal
{
static constexpr size_t Width = 7;    // PERCWIDTH
static constexpr size_t LogSize = 10; // LOGBIAS

size_t
index(uint64_t PC, uint64_t, const BiasArgs &args)
{
    const bool lowConf = args.lowConf;
    const bool longestMatchPred = args.longestMatchPred;
    const bool altTaken = args.altTaken;
    const bool tagePred = args.tagePred;

    const bool uncertain = lowConf & (longestMatchPred != altTaken);
    const size_t indexHash = PC ^ (PC >> 2);
    const size_t totalHash = (indexHash << 2) | (uncertain << 1) | tagePred;
    return totalHash % (1 << LogSize);
}

int8_t
init_value(size_t i)
{
    switch (i % 4) {
        case 0:
            return -32; // certain untaken
        case 1:
            return 31; // certain taken
        case 2:
            return -1; // uncertain untaken
        case 3:
            return 0; // uncertain taken
    }
    GEM5_UNREACHABLE;
}
} // namespace sBiasNormal

namespace sBiasSkew
{
static constexpr size_t Width = 7;    // PERCWIDTH
static constexpr size_t LogSize = 10; // LOGBIAS

size_t
index(uint64_t PC, uint64_t, const BiasArgs &args)
{
    const bool highConf = args.highConf;
    const bool tagePred = args.tagePred;

    const size_t indexHash = PC ^ (PC >> (LogSize - 2));
    const size_t totalHash = (indexHash << 2) | (highConf << 1) | tagePred;
    return totalHash % (1 << LogSize);
}

int8_t
init_value(size_t i)
{
    switch (i % 4) {
        case 0:
            return -8; // not high confidence untaken
        case 1:
            return 7; // not high confidence taken
        case 2:
            return -32; // high confidence untaken
        case 3:
            return 31; // high confidence taken
    }
    GEM5_UNREACHABLE;
}
} // namespace sBiasSkew

namespace sBiasBank
{
static constexpr size_t Width = 7;    // PERCWIDTH
static constexpr size_t LogSize = 10; // LOGBIAS

size_t
index(uint64_t PC, uint64_t, const BiasArgs &args)
{
    const int hitBank = args.hitBank;
    const bool highConf = args.highConf;
    const bool altTaken = args.altTaken;
    const bool tagePred = args.tagePred;

    const size_t pcHash = PC ^ (PC >> 2);
    const size_t hitBankHash = hitBank / 3;
    const size_t totalHash = (pcHash << 6) | (hitBankHash << 3) |
                             (altTaken << 2) | (highConf << 1) | tagePred;
    return totalHash % (1 << LogSize);
}

int8_t
init_value(size_t i)
{
    switch (i % 4) {
        case 0:
            return -32; // high confidence untaken
        case 1:
            return 31; // high confidence taken
        case 2:
            return -1; // not high confidence untaken
        case 3:
            return 0; // not high confidence taken
    }
    GEM5_UNREACHABLE;
}
} // namespace sBiasBank

namespace sBrIMLI
{
static constexpr size_t Width = 6;    // PERCWIDTH
static constexpr size_t LogSize = 10; // LOG_BrIMLI

size_t
index(uint64_t PC, uint64_t BrIMLI, const BiasArgs &)
{
    const size_t pcHash = (PC >> 2) ^ (PC >> 8);
    const size_t totalHash = pcHash ^ BrIMLI;
    return totalHash % (1 << LogSize);
}
} // namespace sBrIMLI

namespace sTaIMLI
{
static constexpr size_t Width = 6;    // PERCWIDTH
static constexpr size_t LogSize = 11; // LOG_TaIMLI

size_t
index(uint64_t PC, uint64_t TaIMLI, const BiasArgs &)
{
    const size_t pcHash = (PC << 1) ^ (PC >> 6);
    const size_t totalHash = pcHash ^ TaIMLI;
    return totalHash % (1 << LogSize);
}
} // namespace sTaIMLI

class SimpleComponent
{
  public:
    using IndexFunction =
        std::function<size_t(uint64_t, uint64_t, const BiasArgs &)>;
    using InitFunction = std::function<int8_t(size_t)>;

    SimpleComponent(
        size_t logSize, size_t counterWidth, IndexFunction indexFunction,
        InitFunction initFunction =
            [](size_t index) {
                return static_cast<int8_t>((index & 1) ? 0 : -1);
            })
        : counterWidth(counterWidth),
          indexFunction(std::move(indexFunction)),
          counterMin(-(1 << (counterWidth - 1))),
          counterMax(-counterMin - 1),
          counters(1ULL << logSize)
    {
        for (size_t i = 0; i < counters.size(); ++i) {
            counters[i] = initFunction(i);
        }
    }

    int
    value(uint64_t pc, uint64_t history, const BiasArgs &args = {}) const
    {
        const size_t index = indexFunction(pc, history, args);
        assert(index < counters.size());
        return 2 * counters[index] + 1;
    }

    void
    update(bool taken, uint64_t pc, uint64_t history,
           const BiasArgs &args = {})
    {
        const size_t index = indexFunction(pc, history, args);
        assert(index < counters.size());
        auto &counter = counters[index];
        if (taken && counter < counterMax) {
            ++counter;
        }
        if (!taken && counter > counterMin) {
            --counter;
        }
    }

    size_t
    storageSizeBits() const
    { return counterWidth * counters.size(); }

  private:
    size_t counterWidth;
    IndexFunction indexFunction;
    int8_t counterMin;
    int8_t counterMax;
    std::vector<int8_t> counters;
};

class WeightGroup
{
  public:
    explicit WeightGroup(int8_t initialWeight) { weights.fill(initialWeight); }

    int
    value(uint64_t pc, uint64_t history, const BiasArgs &args = {}) const
    {
        int sum = 0;
        for (const auto &component : components) {
            sum += component.value(pc, history, args);
        }
        return sum;
    }

    int
    weightedValue(uint64_t pc, uint64_t history,
                  const BiasArgs &args = {}) const
    { return (1 + extraWeight(pc)) * value(pc, history, args); }

    void
    update(int totalSum, bool taken, uint64_t pc, uint64_t history,
           const BiasArgs &args = {})
    {
        const int componentSum = value(pc, history, args);
        const int sumWithoutExtraWeight =
            totalSum - extraWeight(pc) * componentSum;
        if (((sumWithoutExtraWeight + componentSum) >= 0) !=
            (sumWithoutExtraWeight >= 0)) {
            updateWeight(pc, (componentSum >= 0) == taken);
        }
        for (auto &component : components) {
            component.update(taken, pc, history, args);
        }
    }

    int
    extraWeight(uint64_t pc) const
    { return weights[weightIndex(pc)] >= 0; }

    int8_t
    weight(uint64_t pc) const
    { return weights[weightIndex(pc)]; }

    size_t
    storageSizeBits() const
    {
        size_t bits = EWIDTH * weights.size();
        for (const auto &component : components) {
            bits += component.storageSizeBits();
        }
        return bits;
    }

  protected:
    void
    addComponent(SimpleComponent component)
    { components.push_back(std::move(component)); }

  private:
    static size_t
    weightIndex(uint64_t pc)
    { return (pc ^ (pc >> 2)) & (weightsCount - 1); }

    void
    updateWeight(uint64_t pc, bool useful)
    {
        auto &counter = weights[weightIndex(pc)];
        constexpr int8_t min = -(1 << (EWIDTH - 1));
        constexpr int8_t max = -min - 1;
        if (useful && counter < max) {
            ++counter;
        }
        if (!useful && counter > min) {
            --counter;
        }
    }

    static constexpr size_t weightsCount = 1 << LOGSIZEUPS;
    std::array<int8_t, weightsCount> weights{};
    std::vector<SimpleComponent> components;
};

class BiasComponents : public WeightGroup
{
  public:
    BiasComponents() : WeightGroup(4)
    {
        addComponent(SimpleComponent(sBiasNormal::LogSize, sBiasNormal::Width,
                                     sBiasNormal::index,
                                     sBiasNormal::init_value));
        addComponent(SimpleComponent(sBiasSkew::LogSize, sBiasSkew::Width,
                                     sBiasSkew::index, sBiasSkew::init_value));
        addComponent(SimpleComponent(sBiasBank::LogSize, sBiasBank::Width,
                                     sBiasBank::index, sBiasBank::init_value));
    }
};

class GehlComponents : public WeightGroup
{
  public:
    GehlComponents(std::initializer_list<int> historyLengths, size_t logSize,
                   int8_t initialWeight)
        : WeightGroup(initialWeight), maxHistoryLength(*historyLengths.begin())
    {
        const size_t count = historyLengths.size();
        size_t componentIndex = 0;
        for (const int historyLength : historyLengths) {
            const size_t componentLogSize =
                logSize - (componentIndex >= count - 2);
            addComponent(SimpleComponent(
                componentLogSize, PERCWIDTH,
                [componentIndex, componentLogSize, historyLength](
                    uint64_t pc, uint64_t history, const BiasArgs &) {
                    const uint64_t maskedHistory =
                        history & ((1ULL << historyLength) - 1);
                    uint64_t hash = pc ^ maskedHistory;
                    hash ^= maskedHistory >> (8 - componentIndex);
                    hash ^= maskedHistory >> (16 - 2 * componentIndex);
                    hash ^= maskedHistory >> (24 - 3 * componentIndex);
                    hash ^= maskedHistory >> (32 - 3 * componentIndex);
                    hash ^= maskedHistory >> (40 - 4 * componentIndex);
                    return hash & ((1ULL << componentLogSize) - 1);
                }));
            ++componentIndex;
        }
    }

    size_t
    historyStorageSizeBits(size_t historyCount = 1) const
    { return historyCount * maxHistoryLength; }

  private:
    size_t maxHistoryLength;
};

class ImliComponents
{
  public:
    ImliComponents()
        : branch(sBrIMLI::LogSize, sBrIMLI::Width, sBrIMLI::index),
          target(sTaIMLI::LogSize, sTaIMLI::Width, sTaIMLI::index)
    { weights.fill(7); }

    int
    value(uint64_t pc, uint64_t branchImli, uint64_t targetImli) const
    { return branch.value(pc, branchImli) + target.value(pc, targetImli); }

    int
    weightedValue(uint64_t pc, uint64_t branchImli, uint64_t targetImli) const
    { return (1 + extraWeight(pc)) * value(pc, branchImli, targetImli); }

    void
    update(int totalSum, bool taken, uint64_t pc, uint64_t branchImli,
           uint64_t targetImli)
    {
        const size_t index = weightIndex(pc);
        const bool hadExtraWeight = weights[index] >= 0;
        const int componentSum = value(pc, branchImli, targetImli);

        branch.update(taken, pc, branchImli);
        target.update(taken, pc, targetImli);
        if (hadExtraWeight) {
            for (int i = 0; i < 2; ++i) {
                branch.update(taken, pc, branchImli);
                target.update(taken, pc, targetImli);
            }
        }

        const int sumWithoutExtraWeight =
            totalSum - 2 * hadExtraWeight * componentSum;
        if (((sumWithoutExtraWeight + 2 * componentSum) >= 0) !=
            (sumWithoutExtraWeight >= 0)) {
            auto &counter = weights[index];
            constexpr int8_t min = -(1 << (EWIDTH - 1));
            constexpr int8_t max = -min - 1;
            const bool useful = (componentSum >= 0) == taken;
            if (useful && counter < max) {
                ++counter;
            }
            if (!useful && counter > min) {
                --counter;
            }
        }
    }

    int
    extraWeight(uint64_t pc) const
    { return 2 * (weights[weightIndex(pc)] >= 0); }

    size_t
    storageSizeBits() const
    {
        return branch.storageSizeBits() + target.storageSizeBits() +
               EWIDTH * weights.size();
    }

  private:
    static size_t
    weightIndex(uint64_t pc)
    { return (pc ^ (pc >> 2)) & ((1ULL << LOGSIZEUPI) - 1); }

    SimpleComponent branch;
    SimpleComponent target;
    std::array<int8_t, 1 << LOGSIZEUPI> weights{};
};

// SC component indexed by register values: 5.25 KiB.
class RBias_
{
    std::array<std::array<int8_t, (4 << LogRBiasSize) / RBiasNBanks>,
               RBiasNBanks>
        array1; // 3 KiB
    std::array<std::array<int8_t, (2 << LogRBiasSize) / RBiasNBanks>,
               RBiasNBanks>
        array2; // 1.5 KiB
    std::array<std::array<int8_t, (1 << LogRBiasSize) / RBiasNBanks>,
               RBiasNBanks>
        array3; // 0.75 KiB
    size_t
    index1(uint64_t PR, int reg_number) const
    { return (PR + reg_number * 633) % array1[0].size(); }
    size_t
    index2(uint64_t PR, int reg_number) const
    { return ((PR >> 2 ^ PR << 6) + reg_number) % array2[0].size(); }
    size_t
    index3(uint64_t PR, int reg_number) const
    { return (PR ^ reg_number << 3) % array3[0].size(); }
    void
    ctrupdate(int8_t &ctr, bool useful)
    {
        if (useful) {
            ctr < 31 && ++ctr;
        }
        if (not useful) {
            ctr > -32 && --ctr;
        }
    }

  public:
    RBias_() : array1{}, array2{}, array3{} {}
    int
    get(uint64_t PR, int reg_number) const
    {
        return array1.at(reg_number % RBiasNBanks).at(index1(PR, reg_number)) +
               array2.at(reg_number % RBiasNBanks).at(index2(PR, reg_number)) +
               array3.at(reg_number % RBiasNBanks).at(index3(PR, reg_number));
    }
    void
    train(uint64_t PR, int reg_number, bool useful)
    {
        ctrupdate(
            array1.at(reg_number % RBiasNBanks).at(index1(PR, reg_number)),
            useful);
        ctrupdate(
            array2.at(reg_number % RBiasNBanks).at(index2(PR, reg_number)),
            useful);
        ctrupdate(
            array3.at(reg_number % RBiasNBanks).at(index3(PR, reg_number)),
            useful);
    }
    int
    storage_size() const
    {
        return (array1[0].size() + array2[0].size() + array3[0].size()) * 6 *
               RBiasNBanks;
    }
};

// Weights for register-value-indexed components: 1.15 KiB.
class WR_
{
    std::array<int8_t, (MaxDistance + 1) * 8> array1;
    std::array<int8_t, (MaxDistance + 1) * 8> array2;
    std::array<int8_t, (MaxDistance + 1) * 8> array3;

    // Hashing the PC and register number across all 65 * 8 entries would
    // require bank interleaving, complicating hardware implementation. It
    // would also let registers with stable values pollute the table, so
    // avoid spreading each register's values across the table.
    size_t
    index1(uint64_t PC, int reg_number) const
    { return reg_number * 8 + (PC ^ PC >> 2) % 8; }
    size_t
    index2(uint64_t PC, int reg_number) const
    { return reg_number * 8 + (PC ^ PC >> 4) % 8; }
    size_t
    index3(uint64_t PC, int reg_number) const
    { return reg_number * 8 + (PC ^ PC >> 6) % 8; }
    void
    ctrupdate(int8_t &ctr, bool useful)
    {
        if (useful) {
            ctr < 31 && ++ctr;
        }
        if (not useful) {
            ctr > -32 && --ctr;
        }
    }

  public:
    WR_() : array1{}, array2{}, array3{}
    {
        std::fill(array1.begin(), array1.end(), -8);
        std::fill(array2.begin(), array2.end(), -8);
        std::fill(array3.begin(), array3.end(), -8);
    }
    int
    get(uint64_t PC, int reg_number) const
    {
        return array1.at(index1(PC, reg_number)) +
               array2.at(index2(PC, reg_number)) +
               array3.at(index3(PC, reg_number));
    }
    void
    train(uint64_t PC, int reg_number, bool useful)
    {
        ctrupdate(array1.at(index1(PC, reg_number)), useful);
        ctrupdate(array2.at(index2(PC, reg_number)), useful);
        ctrupdate(array3.at(index3(PC, reg_number)), useful);
    }
    int
    storage_size() const
    {
        return (MaxDistance + 1) * 8 * /* array1 ~ array3 */ 3 *
               /* counter width */ 6;
    }
};

struct GEntry
{
    int8_t ctr = 0;
    unsigned tag = 0;
    bool useful_or_newly_alloc = false;

    bool
    is_useful() const
    { return abs(2 * ctr + 1) != 1 && useful_or_newly_alloc; }
    bool
    is_newly_alloc() const
    { return abs(2 * ctr + 1) == 1 && useful_or_newly_alloc; }
    void
    set_newly_alloc()
    { useful_or_newly_alloc = true; }
    void
    set_useful()
    { useful_or_newly_alloc = true; }
    void
    unset_newly_alloc()
    {
        assert(is_newly_alloc());
        useful_or_newly_alloc = false;
    }
    void
    unset_useful()
    {
        assert(is_useful());
        useful_or_newly_alloc = false;
    }
};

struct BEntry
{
    int8_t pred = 0;
    int8_t hyst = 1;
};

class Predictor
{
  public:
    struct Snap
    {
        PredictionHistory hist;
        PredVars pv;
        HistoryDelta delta;
    };

    inline size_t
    tageTablesSizeBits()
    {
        size_t bits = 0;
        bits += NBANKHIGH * (1 << logg[BORN]) * (CWIDTH + UWIDTH + TB[BORN]);
        bits += NBANKLOW * (1 << logg[1]) * (CWIDTH + UWIDTH + TB[1]);
        bits += SIZEUSEALT * ALTWIDTH;
        bits += m[NHIST];
        bits += PHISTWIDTH;
        bits += 10; // the TICK counter
        bits += LogMaxNewlyCounters * 2;
        return bits;
    }

    inline size_t
    bimodalSizeBits()
    { return (size_t(1) << LOGB) + (size_t(1) << (LOGB - HYSTSHIFT)); }

    inline size_t
    scSizeBits()
    {
        size_t bits = 0;

        bits += WIDTHRES;                            // updatethreshold
        bits += size_t(1u << LOGSIZEUP) * WIDTHRESP; // Pupdatethreshold[]

        bits += biasComponents.storageSizeBits();

        if (UseBackwardGEHL) {
            bits += globalGehl.storageSizeBits();
            bits += globalGehl.historyStorageSizeBits();
        }

        if (UsePathGEHL) {
            bits += pathGehl.storageSizeBits();
            bits += pathGehl.historyStorageSizeBits();
        }

        if (UseRBias) {
            bits += RBias.storage_size();
            bits += WR.storage_size();
            if (!use_logical_) {
                bits += ST_NBuf * (1 + 14);
            } else {
                bits += LogicalRegisterCount * (1 + 14 + 8);
            }
        }

        if (UseLocalH) {
            bits += local1Gehl.storageSizeBits();
            bits += local1Gehl.historyStorageSizeBits(sLocal1::FeatureSize);
        }

        if (UseLocalS) {
            bits += local2Gehl.storageSizeBits();
            bits += local2Gehl.historyStorageSizeBits(sLocal2::FeatureSize);
        }

        if (UseLocalT) {
            bits += local3Gehl.storageSizeBits();
            bits += local3Gehl.historyStorageSizeBits(sLocal3::FeatureSize);
        }

        if (UseIMLI) {
            bits += imliComponents.storageSizeBits();
            bits += 64;               // last_backward_target
            bits += 64;               // last_backward_pc
            bits += sBrIMLI::LogSize; // BrIMLI
            bits += sTaIMLI::LogSize; // TaIMLI
        }

        if (UseCallStackHistory) {
            bits += callStackGehl.storageSizeBits();
            bits +=
                callStackGehl.historyStorageSizeBits(sCallStack::FeatureSize) +
                3;
        }

        bits += 2 * CONFWIDTH; // FirstH, SecondH

        return bits;
    }

    // Important: initialize table geometry before setting up folded histories.
    // initHistories() depends on m[], logg[] and TB[] configured by
    // initTage().
    explicit Predictor(bool use_logical)
        : use_logical_(use_logical),
          pathGehl({16, 9}, use_logical_ ? LogLOGPNB : SeqLOGPNB, 7)
    {
        initTage();
        initHistories(active);

        const size_t tage_bits = tageTablesSizeBits();
        const size_t bim_bits = bimodalSizeBits();
        const size_t sc_bits = scSizeBits();
        [[maybe_unused]] const size_t total_bits =
            tage_bits + bim_bits + sc_bits;

        // 192KiB
        assert(191 * 1024 * 8 < total_bits && total_bits < 192 * 1024 * 8);
    }

    uint64_t
    RP(InstSeqNum seq, uint8_t piece)
    { return seq_no_to_correct_path_no[seq % 2048]; }

    PredictionHistory
    capturePredictionHistory(uint64_t pc, InstSeqNum seq_no)
    {
        PredictionHistory history;
        history.seq_no = seq_no;
        for (size_t i = 0; i < LogicalRegisterCount; ++i) {
            if (!use_logical_) {
                const uint64_t physical = (RP(seq_no, 0) - i) % ST_NBuf;
                history.register_values.at(i) =
                    seqRegState.at(physical).valid
                        ? seqRegState.at(physical).payload
                        : -1;
            } else {
                history.register_values.at(i) =
                    logRegState.at(i).valid ? logRegState.at(i).payload : -1;
            }
        }

        for (int i = 0; i <= NHIST; ++i) {
            history.ch_i[i].comp = active.ch_i[i].comp;
            history.ch_t[0][i].comp = active.ch_t[0][i].comp;
            history.ch_t[1][i].comp = active.ch_t[1][i].comp;
        }
        history.GHIST = active.GHIST;
        history.phist = active.phist;
        history.fphist = active.fphist;
        history.L_shist = active.local1_hist(pc);
        history.S_slhist = active.local2_hist(pc);
        history.T_slhist = active.local3_hist(pc);
        history.BrIMLI = active.BrIMLI;
        history.TaIMLI = active.TaIMLI;
        history.call_stack_history = active.call_stack_hist();
        return history;
    }

    bool
    predict(uint64_t pc, InstSeqNum seq_no, Snap &out_snap)
    {
        const auto history = capturePredictionHistory(pc, seq_no);
        const bool pred = predict_using_given_hist(pc, out_snap, history);
        out_snap.hist.perceptron_sum_at_prediction = out_snap.pv.sc_lsum;
        return pred;
    }

    void
    tagePredict(uint64_t pc, Snap &snap)
    {
        const int bimodalIndex = bindex(pc);
        snap.pv.BI = bimodalIndex;
        const int hysteresisIndex = bimodalIndex >> HYSTSHIFT;
        snap.pv.BIM = (btable[bimodalIndex].pred << 1) |
                      (btable[hysteresisIndex].hyst & 1);
        const bool bimodalPred = btable[bimodalIndex].pred > 0;
        snap.pv.final_pred = snap.pv.tage_pred = snap.pv.alttaken =
            snap.pv.LongestMatchPred = bimodalPred;
        snap.pv.HighConf = snap.pv.BIM == 0 || snap.pv.BIM == 3;
        snap.pv.LowConf = !snap.pv.HighConf;
        snap.pv.AltConf = snap.pv.HighConf;

        computeIndicesAndTags(pc, snap.hist, snap.pv);
        for (int bank = NHIST; bank >= 1; --bank) {
            if (gentry_tag_match(bank, snap.pv)) {
                snap.pv.HitBank = bank;
                snap.pv.LongestMatchPred = gentry_pred(bank, snap.pv);
                break;
            }
        }
        if (snap.pv.HitBank == 0) {
            return;
        }

        for (int bank = snap.pv.HitBank - 1; bank >= 1; --bank) {
            if (gentry_tag_match(bank, snap.pv)) {
                snap.pv.AltBank = bank;
                break;
            }
        }
        if (snap.pv.AltBank > 0) {
            snap.pv.alttaken = gentry_pred(snap.pv.AltBank, snap.pv);
            snap.pv.AltConf = gentry_already_trained(snap.pv.AltBank, snap.pv);
        } else {
            snap.pv.alttaken = bimodalPred;
        }

        const int magnitude = gentry_magnitude(snap.pv.HitBank, snap.pv);
        const int useAlternateIndex = use_alt_on_na_index(snap.pv);
        const bool useAlternate = use_alt_on_na[useAlternateIndex] >= 0;
        snap.pv.tage_pred = (!useAlternate || magnitude > 1)
                                ? snap.pv.LongestMatchPred
                                : snap.pv.alttaken;
        snap.pv.final_pred = snap.pv.tage_pred;
        snap.pv.HighConf = magnitude >= (1 << CWIDTH) - 1;
        snap.pv.LowConf = magnitude == 1;
        snap.pv.MedConf = magnitude == 5;
    }

    void
    scPredict(uint64_t pc, Snap &snap)
    {
        const bool tagePred = snap.pv.tage_pred;
        const BiasArgs biasArgs{
            snap.pv.HitBank,   snap.pv.AltBank,          snap.pv.HighConf,
            snap.pv.LowConf,   snap.pv.LongestMatchPred, snap.pv.alttaken,
            snap.pv.tage_pred,
        };
        int lsum = biasComponents.weightedValue(pc, 0, biasArgs);
        int rbiasExtraWeight = 0;

        for (auto &bestReg : snap.hist.best_reg) {
            bestReg = -1;
        }
        for (int distance = 0; distance <= MaxDistance; ++distance) {
            if (snap.hist.register_values[distance] == -1 ||
                WR.get(pc, distance) < 0) {
                continue;
            }
            auto &bestReg = snap.hist.best_reg[distance % RBiasNBanks];
            if (bestReg == -1 || WR.get(pc, bestReg) < WR.get(pc, distance)) {
                bestReg = distance;
            }
        }
        for (const int distance : snap.hist.best_reg) {
            if (distance == -1) {
                continue;
            }
            assert(snap.hist.register_values[distance] != -1);
            assert(WR.get(pc, distance) >= 0);
            lsum +=
                RBias.get(rbias_index(pc, snap.hist.register_values[distance]),
                          distance) *
                RBiasScale;
            rbiasExtraWeight += RBiasScale;
        }

        lsum +=
            globalGehl.weightedValue((pc << 1) + tagePred, snap.hist.GHIST);
        lsum += pathGehl.weightedValue(pc, snap.hist.fphist);
        lsum += local1Gehl.weightedValue(pc, snap.hist.L_shist);
        lsum += local2Gehl.weightedValue(pc, snap.hist.S_slhist);
        lsum += local3Gehl.weightedValue(pc, snap.hist.T_slhist);
        lsum += callStackGehl.weightedValue(pc, snap.hist.call_stack_history);
        lsum += imliComponents.weightedValue(pc, snap.hist.BrIMLI,
                                             snap.hist.TaIMLI);

        snap.pv.sc_threshold =
            (updatethreshold >> 1) + Pupdatethreshold[indUpd(pc)] +
            12 * (biasComponents.extraWeight(pc) + globalGehl.extraWeight(pc) +
                  pathGehl.extraWeight(pc) + local1Gehl.extraWeight(pc) +
                  local2Gehl.extraWeight(pc) + local3Gehl.extraWeight(pc) +
                  callStackGehl.extraWeight(pc) +
                  imliComponents.extraWeight(pc)) +
            6 * rbiasExtraWeight;

        const bool scPred = lsum >= 0;
        bool finalPred = tagePred;
        if (tagePred != scPred) {
            finalPred = scPred;
            const int absoluteSum = std::abs(lsum);
            if (snap.pv.HighConf) {
                if (absoluteSum < snap.pv.sc_threshold / 4) {
                    finalPred = tagePred;
                } else if (absoluteSum < snap.pv.sc_threshold / 2) {
                    finalPred = SecondH < 0 ? scPred : tagePred;
                }
            }
            if (snap.pv.MedConf && absoluteSum < snap.pv.sc_threshold / 4) {
                finalPred = FirstH < 0 ? scPred : tagePred;
            }
        }
        snap.pv.final_pred = finalPred;
        snap.pv.sc_lsum = lsum;
    }

    bool
    predict_using_given_hist(uint64_t pc, Snap &out_snap,
                             const PredictionHistory &pred_hist)
    {
        out_snap.hist = pred_hist;
        out_snap.pv = {};
        tagePredict(pc, out_snap);

        if (UseSC) {
            scPredict(pc, out_snap);
        }

        return out_snap.pv.final_pred;
    }

    void
    recordHistoryDelta(uint64_t pc, bool is_conditional,
                       const StaticInstPtr &inst, int maxt, Snap &snap)
    {
        auto &delta = snap.delta;
        assert(!delta.modified);
        delta.conditional = is_conditional;
        delta.ptghist = active.ptghist;
        delta.ghist_count = maxt;
        for (int i = 0; i <= NHIST; ++i) {
            delta.ch_i[i] = active.ch_i[i].comp;
            delta.ch_t[0][i] = active.ch_t[0][i].comp;
            delta.ch_t[1][i] = active.ch_t[1][i].comp;
        }
        for (int t = 0; t < maxt; ++t) {
            const int index = (delta.ptghist - 1 - t) & (HISTBUFFERLENGTH - 1);
            delta.overwritten_ghist[t] = active.ghist[index];
        }

        delta.GHIST = active.GHIST;
        delta.phist = active.phist;
        delta.fphist = active.fphist;
        if (is_conditional) {
            delta.local_index = sLocal1::get_index(pc);
            delta.second_local_index = sLocal2::get_index(pc);
            delta.third_local_index = sLocal3::get_index(pc);
            delta.L_shist = active.L_shist.at(delta.local_index);
            delta.S_slhist = active.S_slhist.at(delta.second_local_index);
            delta.T_slhist = active.T_slhist.at(delta.third_local_index);
        }

        delta.ubhist = active.ubhist;
        delta.last_backward_target = active.last_backward_target;
        delta.last_backward_pc = active.last_backward_pc;
        delta.BrIMLI = active.BrIMLI;
        delta.TaIMLI = active.TaIMLI;
        delta.call_stack_ptr = active.call_stack_ptr;
        delta.call_stack_value =
            active.call_stack_history.at(delta.call_stack_ptr);
        delta.call_stack_next_modified = inst->isCall();
        if (delta.call_stack_next_modified) {
            delta.call_stack_next_ptr =
                (delta.call_stack_ptr + 1) % sCallStack::FeatureSize;
            delta.call_stack_next_value =
                active.call_stack_history.at(delta.call_stack_next_ptr);
        }
    }

    void
    historyUpdate(uint64_t pc, bool is_conditional, bool taken,
                  uint64_t next_pc, const StaticInstPtr &inst, Snap &snap)
    {
        int maxt = 2;
        if (!is_conditional && inst->isIndirectCtrl()) {
            maxt = 3;
        }

        if (snap.delta.modified) {
            restore(snap);
        }
        recordHistoryDelta(pc, is_conditional, inst, maxt, snap);

        auto &X = active.phist;
        auto &Y = active.ptghist;
        auto &H = active.ch_i;
        auto &G = active.ch_t[0];
        auto &J = active.ch_t[1];

        if (is_conditional) {
            active.GHIST = (active.GHIST << 1) + (taken & (next_pc < pc));
            active.local1_hist(pc) = (active.local1_hist(pc) << 1) + taken;
            active.local2_hist(pc) =
                ((active.local2_hist(pc) << 1) + taken) ^ (pc & 15);
            active.local3_hist(pc) = (active.local3_hist(pc) << 1) + taken;
        }

        int T = ((pc ^ (pc >> 2))) ^ int(taken);
        int PATH = pc ^ (pc >> 2) ^ (pc >> 4);

        for (int t = 0; t < maxt; t++) {
            bool DIR = (T & 1);
            T >>= 1;
            int PATHBIT = (PATH & 127);
            PATH >>= 1;
            // update  history
            Y--; // ptghist
            active.ghist[Y & (HISTBUFFERLENGTH - 1)] = DIR;
            X = (X << 1) ^ PATHBIT; // phist

            // folded histories
            for (int i = 1; i <= NHIST; i++) {
                H[i].update(active.ghist, Y);
                G[i].update(active.ghist, Y);
                J[i].update(active.ghist, Y);
            }
        }

        X &= ((1u << PHISTWIDTH) - 1);

        if (next_pc > pc && taken) {
            active.fphist = (active.fphist << 3) ^ (next_pc >> 2) ^ (pc >> 1);
        }

        active.call_stack_hist() <<= 1;
        active.call_stack_hist() |= taken;
        if (inst->isCall()) {
            ++active.call_stack_ptr;
            active.call_stack_ptr %= sCallStack::FeatureSize;
            active.call_stack_hist() = 0;
        }
        if (inst->isReturn()) {
            active.call_stack_ptr =
                (active.call_stack_ptr + sCallStack::FeatureSize - 1) %
                sCallStack::FeatureSize;
        }

        if (taken && next_pc < pc && not inst->isIndirectCtrl()) {
            int prime[18] = {0,    0,    3,     7,     13,    31,
                             61,   127,  251,   509,   1021,  2039,
                             4093, 8191, 16381, 32749, 65521, 131071};
            if (active.last_backward_target / 128 == next_pc / 128) {
                active.TaIMLI = (active.TaIMLI + 1) % prime[sTaIMLI::LogSize];
            } else {
                active.TaIMLI = 0;
                active.ubhist <<= 3;
                active.ubhist ^= active.last_backward_target / 64;
            }

            if (active.last_backward_pc / 128 == pc / 128) {
                active.BrIMLI =
                    (active.BrIMLI + 1) % (1ull << sBrIMLI::LogSize);
            } else {
                active.BrIMLI = 0;
                active.ubhist <<= 3;
                active.ubhist ^= active.last_backward_pc / 64;
            }
            active.last_backward_target = next_pc;
            active.last_backward_pc = pc;
        }

        snap.delta.modified = true;
    }

    void
    update(uint64_t pc, bool taken, const Snap &snap)
    {
        SC_update(pc, taken, snap);
        TAGE_allocation(taken, snap.pv.final_pred, snap.pv);
        TAGE_train(taken, snap.pv);
    }

    void
    SC_update(uint64_t pc, bool taken, const Snap &snap)
    {
        if (UseSC) {
            const bool tagePred = snap.pv.tage_pred;
            const int lsum = snap.pv.sc_lsum;
            const int threshold = snap.pv.sc_threshold;
            const size_t thresholdIndex = indUpd(pc);
            const bool scPred = lsum >= 0;
            const BiasArgs biasArgs{
                snap.pv.HitBank,   snap.pv.AltBank,          snap.pv.HighConf,
                snap.pv.LowConf,   snap.pv.LongestMatchPred, snap.pv.alttaken,
                snap.pv.tage_pred,
            };

            DPRINTF(RUNLTS,
                    "SC-train pc=%#llx taken=%d TAGE=%d SC=%d "
                    "FINAL=%d LSUM=%d THRES=%d\n",
                    (unsigned long long)pc, taken, tagePred, scPred,
                    snap.pv.final_pred, lsum, threshold);
            if (tagePred != scPred && std::abs(lsum) < threshold) {
                if (snap.pv.HighConf) {
                    if (std::abs(lsum) < threshold / 2 &&
                        std::abs(lsum) >= threshold / 4) {
                        ctrupdate(SecondH, tagePred == taken, CONFWIDTH);
                    }
                }
                if (snap.pv.MedConf) {
                    if (std::abs(lsum) < threshold / 4) {
                        ctrupdate(FirstH, tagePred == taken, CONFWIDTH);
                    }
                }
                DPRINTF(RUNLTS,
                        "SC-chooser-update pc=%#llx FirstH=%d SecondH=%d\n",
                        (unsigned long long)pc, FirstH, SecondH);
            }

            auto thresholdUpdate = [&](int amount) {
                satUpdate(Pupdatethreshold[thresholdIndex], amount, WIDTHRESP);
                satUpdate(updatethreshold, amount, WIDTHRES);
            };
            const bool predictionSumWeak =
                std::abs(snap.hist.perceptron_sum_at_prediction) < threshold;
            const bool trainingSumWeak = std::abs(lsum) < threshold;
            const bool predictionMispred =
                (snap.hist.perceptron_sum_at_prediction >= 0) != taken;
            const bool trainingMispred = scPred != taken;
            if (predictionMispred && !trainingMispred) {
                thresholdUpdate(+6);
            }
            if (predictionMispred && trainingMispred) {
                thresholdUpdate(+1);
            }
            if (!predictionMispred && predictionSumWeak && trainingSumWeak) {
                thresholdUpdate(-1);
            }

            if (trainingMispred || trainingSumWeak) {
                biasComponents.update(lsum, taken, pc, 0, biasArgs);
                if (UseBackwardGEHL) {
                    globalGehl.update(lsum, taken, (pc << 1) + tagePred,
                                      snap.hist.GHIST);
                }
                if (UsePathGEHL) {
                    pathGehl.update(lsum, taken, pc, snap.hist.fphist);
                }
                if (UseLocalH) {
                    local1Gehl.update(lsum, taken, pc, snap.hist.L_shist);
                }
                if (UseLocalS) {
                    local2Gehl.update(lsum, taken, pc, snap.hist.S_slhist);
                }
                if (UseLocalT) {
                    local3Gehl.update(lsum, taken, pc, snap.hist.T_slhist);
                }

                if (UseRBias) {
                    bool bankUsed[RBiasNBanks] = {};
                    for (int bank = 0; bank < RBiasNBanks; ++bank) {
                        const int i = snap.hist.best_reg[bank];
                        if (i == -1) {
                            continue;
                        }
                        assert(snap.hist.register_values[i] != -1);
                        assert(WR.get(pc, i) >= 0);
                        const int contribution = RBias.get(
                            rbias_index(pc, snap.hist.register_values[i]), i);
                        const int sumWithoutContribution =
                            lsum -
                            contribution * RBiasScale * (WR.get(pc, i) >= 0);
                        RBias.train(
                            rbias_index(pc, snap.hist.register_values[i]), i,
                            taken);
                        if (((sumWithoutContribution +
                              RBiasScale * contribution) >= 0) !=
                            (sumWithoutContribution >= 0)) {
                            WR.train(pc, i, (contribution >= 0) == taken);
                        }
                        bankUsed[i % RBiasNBanks] = true;
                    }

                    const uint64_t start = MYRANDOM() % (MaxDistance + 1);
                    for (int offset = 0; offset <= MaxDistance; ++offset) {
                        const uint64_t i =
                            (offset + start) % (MaxDistance + 1);
                        if (snap.hist.register_values[i] == -1 ||
                            bankUsed[i % RBiasNBanks]) {
                            continue;
                        }
                        const int contribution = RBias.get(
                            rbias_index(pc, snap.hist.register_values[i]), i);
                        const int sumWithoutContribution =
                            lsum -
                            contribution * RBiasScale * (WR.get(pc, i) >= 0);
                        RBias.train(
                            rbias_index(pc, snap.hist.register_values[i]), i,
                            taken);
                        if (((sumWithoutContribution +
                              RBiasScale * contribution) >= 0) !=
                            (sumWithoutContribution >= 0)) {
                            WR.train(pc, i, (contribution >= 0) == taken);
                        }
                        bankUsed[i % RBiasNBanks] = true;
                    }
                }

                if (UseCallStackHistory) {
                    callStackGehl.update(lsum, taken, pc,
                                         snap.hist.call_stack_history);
                }
                if (UseIMLI) {
                    imliComponents.update(lsum, taken, pc, snap.hist.BrIMLI,
                                          snap.hist.TaIMLI);
                }

                DPRINTF(RUNLTS, "SC-gehl-update pc=%#llx WG=%d WP=%d\n",
                        (unsigned long long)pc, globalGehl.weight(pc),
                        pathGehl.weight(pc));
            }
        }
    }

    void
    baseupdate(bool taken, const PredVars &pv)
    {
        int inter = pv.BIM;
        if (taken) {
            if (inter < 3) {
                ++inter;
            }
        } else if (inter > 0) {
            --inter;
        }
        btable[pv.BI].pred = inter >> 1;
        btable[pv.BI >> HYSTSHIFT].hyst = inter & 1;
    }

    void
    TICK_update(int numberOfBlocks, int numberOfAllocations)
    {
        TICK += numberOfBlocks - 2 * numberOfAllocations;
        if (TICK < 0) {
            TICK = 0;
        }
        if (TICK < BORNTICK) {
            return;
        }

        for (int bank : {1, BORN}) {
            for (int index = 0; index < SizeTable[bank]; ++index) {
                auto &entry = gtable[bank][index];
                if (entry.is_useful()) {
                    entry.unset_useful();
                }
            }
        }
        TICK = 0;
    }

    void
    TAGE_do_allocation(bool taken, const PredVars &pv)
    {
        int numberOfBlocks = 0;
        int numberOfAllocations = 0;

        const bool aggressiveAllocation = NewlyDecay <= NewlyUseful * 2;
        const bool modestAllocation = NewlyDecay > NewlyUseful * 4;
        int remainingAllocations = aggressiveAllocation
                                       ? NNN + 2
                                       : (modestAllocation ? NNN : NNN + 1);
        int lastAllocatedBank = -1;

        const auto tryAllocation = [&](int bank) {
            auto &entry = gentry(bank, pv);
            if (aggressiveAllocation && entry.is_newly_alloc()) {
                entry.unset_newly_alloc();
                ++NewlyDecay;
                if (NewlyDecay >= (1 << LogMaxNewlyCounters)) {
                    NewlyDecay >>= 1;
                    NewlyUseful >>= 1;
                }
                --remainingAllocations;
                return false;
            }

            if (entry.is_useful()) {
                ++numberOfBlocks;
                return false;
            }

            if (gentry_magnitude(bank, pv) > 3) {
                if (entry.ctr > 0) {
                    --entry.ctr;
                } else {
                    ++entry.ctr;
                }
                return false;
            }

            entry.tag = pv.GTAG[bank];
            entry.ctr = taken ? 0 : -1;
            if (entry.is_newly_alloc()) {
                entry.unset_newly_alloc();
                ++NewlyDecay;
                if (NewlyDecay >= (1 << LogMaxNewlyCounters)) {
                    NewlyDecay >>= 1;
                    NewlyUseful >>= 1;
                }
            }
            lastAllocatedBank = bank;
            ++numberOfAllocations;
            --remainingAllocations;
            return true;
        };

        const int skip = (MYRANDOM() & 127) < 32 ? 1 : 0;
        const int startBank = pv.HitBank + 1 + skip;
        for (int bank = startBank; bank <= NHIST; ++bank) {
            if (!tryAllocation(bank)) {
                continue;
            }
            if (remainingAllocations <= 0) {
                break;
            }
            bank += bank < 3 ? 1 : (bank < Born2 ? 2 : 0);
        }

        if (lastAllocatedBank != -1) {
            gentry(lastAllocatedBank, pv).set_newly_alloc();
        }

        TICK_update(numberOfBlocks, numberOfAllocations);
    }

    void
    TAGE_allocation(bool taken, bool predTaken, const PredVars &pv)
    {
        bool allocate = pv.tage_pred != taken && pv.HitBank < NHIST;

        if (pv.HitBank > 0) {
            assert(gentry_tag_match(pv.HitBank, pv));
            if (gentry_newly_allocated(pv.HitBank, pv)) {
                if (pv.LongestMatchPred == taken) {
                    allocate = false;
                }
                if (pv.LongestMatchPred != pv.alttaken) {
                    ctrupdate(use_alt_on_na[use_alt_on_na_index(pv)],
                              pv.alttaken == taken, ALTWIDTH);
                }
            }
        }

        if (predTaken == taken && (MYRANDOM() & 31) != 0) {
            allocate = false;
        }

        if (allocate) {
            TAGE_do_allocation(taken, pv);
        }
    }

    void
    TAGE_train(bool taken, const PredVars &pv)
    {
        if (pv.HitBank > 0) {
            auto &hit = gentry(pv.HitBank, pv);

            if (gentry_newly_allocated(pv.HitBank, pv) &&
                pv.LongestMatchPred != taken) {
                if (pv.AltBank > 0) {
                    auto &alternate = gentry(pv.AltBank, pv);
                    if (alternate.is_newly_alloc()) {
                        alternate.unset_newly_alloc();
                    }
                    ctrupdate(gentry_ctr(pv.AltBank, pv), taken, CWIDTH);
                    if (gentry_newly_allocated(pv.AltBank, pv)) {
                        alternate.useful_or_newly_alloc = false;
                    }
                } else {
                    baseupdate(taken, pv);
                }
            }

            if (hit.is_newly_alloc()) {
                if (pv.LongestMatchPred == taken) {
                    ++NewlyUseful;
                }
                hit.unset_newly_alloc();
                if (NewlyUseful >= (1 << LogMaxNewlyCounters)) {
                    NewlyDecay >>= 1;
                    NewlyUseful >>= 1;
                }
            }

            ctrupdate(gentry_ctr(pv.HitBank, pv), taken, CWIDTH);
            if (gentry_newly_allocated(pv.HitBank, pv)) {
                hit.useful_or_newly_alloc = false;
            }

            if (pv.alttaken == taken && pv.AltBank > 0 &&
                gentry_magnitude(pv.AltBank, pv) == 7 &&
                pv.LongestMatchPred == taken && hit.is_useful()) {
                hit.unset_useful();
            }
        } else {
            baseupdate(taken, pv);
        }

        if (pv.LongestMatchPred != pv.alttaken &&
            pv.LongestMatchPred == taken) {
            assert(!gentry_newly_allocated(pv.HitBank, pv));
            gentry(pv.HitBank, pv).set_useful();
        }
    }

    void
    restore(Snap &snap)
    {
        auto &delta = snap.delta;
        if (!delta.modified) {
            return;
        }

        active.ptghist = delta.ptghist;
        for (int t = 0; t < delta.ghist_count; ++t) {
            const int index = (delta.ptghist - 1 - t) & (HISTBUFFERLENGTH - 1);
            active.ghist[index] = delta.overwritten_ghist[t];
        }
        for (int i = 0; i <= NHIST; ++i) {
            active.ch_i[i].comp = delta.ch_i[i];
            active.ch_t[0][i].comp = delta.ch_t[0][i];
            active.ch_t[1][i].comp = delta.ch_t[1][i];
        }

        active.GHIST = delta.GHIST;
        active.phist = delta.phist;
        active.fphist = delta.fphist;
        if (delta.conditional) {
            active.L_shist.at(delta.local_index) = delta.L_shist;
            active.S_slhist.at(delta.second_local_index) = delta.S_slhist;
            active.T_slhist.at(delta.third_local_index) = delta.T_slhist;
        }

        active.ubhist = delta.ubhist;
        active.last_backward_target = delta.last_backward_target;
        active.last_backward_pc = delta.last_backward_pc;
        active.BrIMLI = delta.BrIMLI;
        active.TaIMLI = delta.TaIMLI;
        active.call_stack_ptr = delta.call_stack_ptr;
        active.call_stack_history.at(delta.call_stack_ptr) =
            delta.call_stack_value;
        if (delta.call_stack_next_modified) {
            active.call_stack_history.at(delta.call_stack_next_ptr) =
                delta.call_stack_next_value;
        }
        delta.modified = false;
    }

    void
    decode(InstSeqNum seq_num, const StaticInstPtr &inst)
    {
        if (!use_logical_) {
            seqRegState.at(RP(seq_num, 0) % ST_NBuf).valid = false;
            return;
        }
        if (isSquashedDecode(seq_num)) {
            return;
        }

        for (auto &entry : logRegState) {
            if (!entry.valid) {
                continue;
            }
            if (entry.age == 255) {
                entry.valid = false;
                entry.age = 0;
            } else {
                ++entry.age;
            }
        }

        for (size_t i = 0; i < inst->numDestRegs(); ++i) {
            const auto index =
                logical_register_index(inst->destRegIdx(i), inst);
            if (!index) {
                continue;
            }

            auto &entry = logRegState.at(*index);
            entry.valid = false;
            entry.payload = seq_num;
            entry.age = 0;
        }

        assert(logicalRegCheckpoints.empty() ||
               logicalRegCheckpoints.back().seqNum < seq_num);
        logicalRegCheckpoints.push_back({seq_num, logRegState});
        compactCommittedState();
    }

    uint64_t
    make_reg_digest(size_t reg_num, uint64_t value)
    {
        constexpr size_t W = 12;
        assert(W < 16);
        uint64_t hash = 0;
        if (32 <= reg_num && reg_num < 64) { // FP
            if (value >> 16 == 0) {
                hash = value >> (16 - 3);
            } else if (value >> 32 == 0) {
                hash = value >> (32 - 6);
            } else {
                hash = value >> (64 - 9);
            }
        } else if (reg_num == 64) { // Flag
            hash = value << 8 ^ value << 4 ^ value;
        } else { // INT
            // Count consecutive zeros and ones from the MSB and LSB, then mix
            // the counts into the hash.
            int msb_one_pos = 0, msb_zero_pos = 0;
            int lsb_one_pos = 0, lsb_zero_pos = 0;
            for (int i = 0; i < 64; ++i) {
                if (!((value >> i) & 1)) {
                    lsb_one_pos = i;
                    break;
                }
            }
            for (int i = 0; i < 64; ++i) {
                if ((value >> i) & 1) {
                    lsb_zero_pos = i;
                    break;
                }
            }
            // Consecutive one bits starting at the MSB are unlikely, so
            // subtract from 63 to make common case small.
            for (int i = 63; i >= 0; --i) {
                if (!((value >> i) & 1)) {
                    msb_one_pos = 63 - i;
                    break;
                }
            }
            for (int i = 63; i >= 0; --i) {
                if ((value >> i) & 1) {
                    msb_zero_pos = 63 - i;
                    break;
                }
            }
            // Important to mix the low bits of value into the hash.
            hash = ((lsb_one_pos ^ lsb_zero_pos)) ^
                   ((msb_one_pos ^ msb_zero_pos) << 3) ^ (value << 6);
        }

        // Mix the logical register number to distinguish path-dependent
        // distances.
        if (!use_logical_) {
            hash ^= reg_num ^ reg_num << 6;
        }

        return hash % (1 << W);
    }

    void
    writeback(const o3::InstructionWrittenBack &writeback)
    {
        const InstSeqNum seq_num = writeback.seqNum;
        auto set_digest = [&](size_t reg, uint64_t value) {
            const uint64_t digest = make_reg_digest(reg, value);
            if (!use_logical_) {
                const size_t index = RP(seq_num, 0) % ST_NBuf;
                seqRegState.at(index).valid = true;
                seqRegState.at(index).payload = digest;
                return;
            }

            auto update = [&](LogicalRegState &state) {
                auto &entry = state.at(reg);
                if (!entry.valid && entry.payload == seq_num) {
                    entry.valid = true;
                    entry.payload = digest;
                }
            };
            update(logRegState);
            for (auto &checkpoint : logicalRegCheckpoints) {
                update(checkpoint.state);
            }
        };
        bool has_cc = false;
        uint64_t cc_packed = 0; // [0]=V, [1]=C, [2]=N, [3]=Z

        for (const auto &[dest_reg, dest_value] : writeback.destRegValues) {
            uint64_t value = 0;
            const RegClass &reg_class = dest_reg.regClass();
#if USE_ARM_ISA
            if (&reg_class == &ArmISA::ccRegClass) {
                has_cc = true;
                switch (dest_reg.index()) {
                    case 0:
                        cc_packed |= dest_value.asRegVal() << 2;
                        break; // NZ
                    case 1:
                        cc_packed |= dest_value.asRegVal() << 1;
                        break; // C
                    case 2:
                        cc_packed |= dest_value.asRegVal();
                        break; // V
                }
                continue;
            }
#endif

            const auto dst_reg =
                logical_register_index(dest_reg, writeback.staticInst);
            if (!dst_reg) {
                continue;
            }

            if (reg_class.type() == IntRegClass) {
                value = dest_value.asRegVal();
#if USE_ARM_ISA
            } else if (&reg_class == &ArmISA::vecRegClass) {
                // Read the first 8 bytes only
                std::memcpy(&value, dest_value.asBlob(), sizeof(value));
#endif
            } else {
                continue;
            }

            set_digest(*dst_reg, value);
        }

        if (has_cc) {
            set_digest(64, cc_packed);
        }
    }

    void
    fetch(InstSeqNum seq_num)
    {
        seq_no_to_correct_path_no[seq_num % 2048] = correct_path_no++;
        lastFetchedSeqNum = seq_num;
    }

    void
    commit(InstSeqNum seq_num)
    {
        lastCommittedSeqNum = std::max(lastCommittedSeqNum, seq_num);
        compactCommittedState();
    }

    void
    squashInstructionState(InstSeqNum seq_num)
    {
        correct_path_no = seq_no_to_correct_path_no[seq_num % 2048] + 1;

        if (use_logical_) {
            if (lastFetchedSeqNum > seq_num) {
                SquashedDecodeRange range{seq_num + 1, lastFetchedSeqNum};
                while (!squashedDecodeRanges.empty() &&
                       range.first <= squashedDecodeRanges.back().last + 1) {
                    range.first = std::min(range.first,
                                           squashedDecodeRanges.back().first);
                    range.last =
                        std::max(range.last, squashedDecodeRanges.back().last);
                    squashedDecodeRanges.pop_back();
                }
                squashedDecodeRanges.push_back(range);
            }

            const auto log_it = std::find_if(
                logicalRegCheckpoints.rbegin(), logicalRegCheckpoints.rend(),
                [seq_num](const auto &checkpoint) {
                    return checkpoint.seqNum <= seq_num;
                });
            logRegState = log_it == logicalRegCheckpoints.rend()
                              ? LogicalRegState{}
                              : log_it->state;
            while (!logicalRegCheckpoints.empty() &&
                   logicalRegCheckpoints.back().seqNum > seq_num) {
                logicalRegCheckpoints.pop_back();
            }
        }
    }

    bool
    isSquashedDecode(InstSeqNum seq_num)
    {
        // Decode events arrive in sequence order, so this is also a watermark.
        while (!squashedDecodeRanges.empty() &&
               squashedDecodeRanges.front().last < seq_num) {
            squashedDecodeRanges.pop_front();
        }
        return !squashedDecodeRanges.empty() &&
               squashedDecodeRanges.front().first <= seq_num;
    }

  private:
    void
    compactCommittedState()
    {
        while (logicalRegCheckpoints.size() > 1 &&
               logicalRegCheckpoints[1].seqNum <= lastCommittedSeqNum) {
            logicalRegCheckpoints.pop_front();
        }
    }

    const bool use_logical_;
    History active;
    uint64_t Seed = 0;

    BiasComponents biasComponents;
    GehlComponents globalGehl{{40, 24, 10}, LOGGNB, 7};
    GehlComponents pathGehl;
    GehlComponents local1Gehl{{18, 11, 6, 3}, LOGLNB, 7};
    GehlComponents local2Gehl{{21, 16, 11, 6}, LOGSNB, 7};
    GehlComponents local3Gehl{{19, 14, 9, 4}, LOGTNB, 7};
    GehlComponents callStackGehl{{47, 31, 18, 10, 5}, LOGCNB, 7};
    ImliComponents imliComponents;

    int updatethreshold = 35;
    std::array<int, (1 << LOGSIZEUP)> Pupdatethreshold{};
    int8_t FirstH = 0, SecondH = 0;

    uint64_t seq_no_to_correct_path_no[2048] = {};
    uint64_t correct_path_no = 0;

    RBias_ RBias;
    WR_ WR;

    struct LogicalRegCheckpoint
    {
        InstSeqNum seqNum;
        LogicalRegState state;
    };

    struct SquashedDecodeRange
    {
        InstSeqNum first;
        InstSeqNum last;
    };

    SeqRegState seqRegState = {};
    LogicalRegState logRegState = {};
    std::deque<LogicalRegCheckpoint> logicalRegCheckpoints;
    std::deque<SquashedDecodeRange> squashedDecodeRanges;
    InstSeqNum lastFetchedSeqNum = 0;
    InstSeqNum lastCommittedSeqNum = 0;

    std::vector<GEntry> tageLowBank;
    std::vector<GEntry> tageHighBank;
    GEntry *gtable[NHIST + 1]{};
    int TB[NHIST + 1]{};
    int logg[NHIST + 1]{};
    int m[NHIST + 1]{};
    int SizeTable[NHIST + 1]{};
    int TICK = 0;

    std::vector<BEntry> btable;
    inline int
    bindex(uint64_t pc) const
    { return (pc ^ (pc >> 2)) & ((1 << LOGB) - 1); }

    static constexpr int LogMaxNewlyCounters = 16;
    int NewlyDecay = 0;
    int NewlyUseful = 4;

    static constexpr int LOGSIZEUSEALT = 4;
    static constexpr int ALTWIDTH = 5;
    static constexpr int SIZEUSEALT = (1 << LOGSIZEUSEALT);
    int8_t use_alt_on_na[SIZEUSEALT] = {0};

    void
    ctrupdate(int8_t &ctr, bool up, int width) const
    {
        const int8_t cmin = -(1 << (width - 1));
        const int8_t cmax = -cmin - 1;
        if (up) {
            if (ctr < cmax) {
                ++ctr;
            }
        } else {
            if (ctr > cmin) {
                --ctr;
            }
        }
    }

    int
    F(uint64_t hist, int size, int bank) const
    {
        const int lg = logg[bank];
        const uint32_t mask_lg = (lg > 0) ? ((1u << lg) - 1u) : 0u;
        const uint64_t mask_sz =
            (size >= 64) ? ~0ull : ((1ull << size) - 1ull);
        uint64_t A = hist & mask_sz;
        uint32_t A1 = static_cast<uint32_t>(A) & mask_lg;
        uint32_t A2 = static_cast<uint32_t>(A >> lg);
        if (bank < lg && lg > 0) {
            const int sh = bank % lg;
            A2 = ((A2 << sh) & mask_lg) | (A2 >> (lg - sh));
        }
        uint32_t X = A1 ^ A2;
        if (bank < lg && lg > 0) {
            const int sh = bank % lg;
            X = ((X << sh) & mask_lg) | (X >> (lg - sh));
        }
        return static_cast<int>(X & mask_lg);
    }

    int
    gindex(uint64_t PC, int bank, uint64_t phist_val,
           const TageIndexSnapshots &ch_i_arr) const
    {
        const int lg = logg[bank];
        const int M = (m[bank] > PHISTWIDTH) ? PHISTWIDTH : m[bank];
        const int shift = std::abs(lg - bank) + 1;
        const uint32_t mask = (lg > 0) ? ((1u << lg) - 1u) : 0u;
        uint32_t idx = static_cast<uint32_t>(PC ^ (PC >> shift)) ^
                       ch_i_arr[bank].comp ^ F(phist_val, M, bank);
        return static_cast<int>(idx & mask);
    }

    unsigned
    gtag(uint64_t PC, int bank, const TageTagSnapshots &t0,
         const TageTagSnapshots &t1) const
    {
        const unsigned mask = (TB[bank] > 0) ? ((1u << TB[bank]) - 1u) : 0u;
        unsigned tag =
            static_cast<unsigned>(PC) ^ t0[bank].comp ^ (t1[bank].comp << 1);
        return tag & mask;
    }

    inline void
    computeIndicesAndTags(uint64_t PC, const PredictionHistory &hist_to_use,
                          PredVars &pv)
    {
        // Index and tag computation
        for (int i = 1; i <= NHIST; ++i) {
            pv.GI[i] = gindex(PC, i, hist_to_use.phist, hist_to_use.ch_i);
            pv.GTAG[i] = gtag(PC, i, hist_to_use.ch_t[0], hist_to_use.ch_t[1]);
        }

        // High bank
        int Th = (PC >> 2 ^ (hist_to_use.phist & ((1ull << m[BORN]) - 1))) %
                 NBANKHIGH;
        for (int i = BORN; i <= NHIST; i++) {
            pv.GI[i] += (Th << LOGG_BORN);
            Th = (Th + 1) % NBANKHIGH;
        }

        // Low bank
        int Tl =
            (PC >> 2 ^ (hist_to_use.phist & ((1ull << m[1]) - 1))) % NBANKLOW;
        for (int i = 1; i <= BORN - 1; i++) {
            pv.GI[i] += (Tl << LOGG_1);
            Tl = (Tl + 1) % NBANKLOW;
        }
    }

    uint32_t
    MYRANDOM()
    {
        Seed++;
        Seed ^= active.phist;
        Seed = (Seed >> 21) + (Seed << 11);
        Seed ^= (uint64_t)active.ptghist;
        Seed = (Seed >> 10) + (Seed << 22);
        return (uint32_t)(Seed);
    }

    GEntry &
    gentry(int bank, const PredVars &pv)
    { return gtable[bank][pv.GI[bank]]; }
    const GEntry &
    gentry(int bank, const PredVars &pv) const
    { return gtable[bank][pv.GI[bank]]; }
    bool
    gentry_tag_match(int bank, const PredVars &pv) const
    { return gentry(bank, pv).tag == pv.GTAG[bank]; }
    int8_t &
    gentry_ctr(int bank, const PredVars &pv)
    { return gentry(bank, pv).ctr; }
    bool
    gentry_pred(int bank, const PredVars &pv) const
    { return gentry(bank, pv).ctr >= 0; }
    int
    gentry_magnitude(int bank, const PredVars &pv) const
    { return std::abs(2 * gentry(bank, pv).ctr + 1); }
    bool
    gentry_already_trained(int bank, const PredVars &pv) const
    { return gentry_magnitude(bank, pv) > 1; }
    bool
    gentry_newly_allocated(int bank, const PredVars &pv) const
    { return gentry_magnitude(bank, pv) == 1; }
    int
    use_alt_on_na_index(const PredVars &pv) const
    { return ((((pv.HitBank - 1) / 8) << 1) + pv.AltConf) % (SIZEUSEALT - 1); }

    uint64_t
    rbias_index(uint64_t PC, uint64_t reg_value) const
    { return (PC ^ PC >> 8 ^ reg_value) % 4096; }
    uint64_t
    indUpd(uint64_t PC) const
    {
        return (static_cast<uint64_t>((PC ^ (PC >> 2))) &
                ((1ull << LOGSIZEUP) - 1));
    }
    void
    satUpdate(int &v, int up, int width)
    {
        const int vmin = -(1 << (width - 1));
        const int vmax = -vmin - 1;
        long nv = static_cast<long>(v) + up;
        if (nv < vmin) {
            nv = vmin;
        }
        if (nv > vmax) {
            nv = vmax;
        }
        v = static_cast<int>(nv);
    }

    void
    initHistories(History &h)
    {
        h.ghist.fill(0);
        h.ptghist = 0;
        h.phist = 0;
        h.fphist = 0;
        h.GHIST = 0;
        h.L_shist.fill(0);
        h.S_slhist.fill(0);
        h.T_slhist.fill(0);
        // Initialize folded histories with geometric lengths and widths
        for (int i = 0; i <= NHIST; i++) {
            h.ch_i[i].init(m[i], logg[i] > 0 ? logg[i] : 1);
            h.ch_t[0][i].init(m[i], TB[i] > 0 ? TB[i] : 1);
            h.ch_t[1][i].init(m[i], (TB[i] > 1 ? TB[i] - 1 : 1));
        }
    }

    void
    initTage()
    {
        for (int i = 1; i <= BORN - 1; i++) {
            TB[i] = TB_1;
            logg[i] = LOGG_1;
        }
        for (int i = BORN; i <= NHIST; i++) {
            TB[i] = TB_BORN;
            logg[i] = LOGG_BORN;
        }

        m[1] = MINHIST;
        for (int i = 2; i <= NHIST; ++i) {
            double rate = std::max(HistRate, HistRate + 0.1 * (i - Born2));
            int before = m[i - 2];
            int current = m[i - 1];
            int a = current + (current - before + 2);
            int b = int(current * rate / 2.0 + 0.5) * 2;
            m[i] = std::max(a, b);
        }

        const size_t low_entries = size_t(NBANKLOW) * (1u << LOGG_1);
        const size_t high_entries = size_t(NBANKHIGH) * (1u << LOGG_BORN);
        tageLowBank.assign(low_entries, GEntry{});
        tageHighBank.assign(high_entries, GEntry{});

        gtable[1] = tageLowBank.data();
        SizeTable[1] = static_cast<int>(low_entries);
        gtable[BORN] = tageHighBank.data();
        SizeTable[BORN] = static_cast<int>(high_entries);

        for (int i = 2; i <= BORN - 1; ++i) {
            gtable[i] = gtable[1];
        }
        for (int i = BORN + 1; i <= NHIST; ++i) {
            gtable[i] = gtable[BORN];
        }

        btable.assign(1u << LOGB, BEntry{});
    }
};

} // namespace runlts_impl

// Per-thread state
struct RUNLTS::ThreadState
{
    explicit ThreadState(bool use_logical) : pred(use_logical) {}

    runlts_impl::Predictor pred;
};

struct RUNLTSHistory
{
    InstSeqNum seqNum = 0;
    runlts_impl::Predictor::Snap snap;
};

RUNLTS::RUNLTS(const Params &params)
    : ConditionalPredictor(params), useLogical(params.use_logical)
{
    fatal_if(params.numThreads != 1,
             "RUNLTS currently supports exactly one hardware thread");
    fatal_if(!params.speculativeHistUpdate,
             "RUNLTS requires speculative history updates");
}

RUNLTS::ThreadState &
RUNLTS::threadState(ThreadID tid)
{
    if (threadStates.size() <= tid) {
        threadStates.resize(tid + 1);
    }
    if (!threadStates[tid]) {
        threadStates[tid] = std::make_unique<ThreadState>(useLogical);
    }
    return *threadStates[tid];
}

void
RUNLTS::instructionEvent(const o3::InstructionEvent &event)
{
    std::visit(
        [this](const auto &typed_event) {
            handleInstructionEvent(typed_event);
        },
        event.data);
}

void
RUNLTS::handleInstructionEvent(const o3::InstructionFetched &event)
{ threadState(event.tid).pred.fetch(event.seqNum); }

void
RUNLTS::handleInstructionEvent(const o3::InstructionDecoded &event)
{ threadState(event.tid).pred.decode(event.seqNum, event.staticInst); }

void
RUNLTS::handleInstructionEvent(const o3::InstructionWrittenBack &event)
{ threadState(event.tid).pred.writeback(event); }

void
RUNLTS::handleInstructionEvent(const o3::InstructionSquash &event)
{ threadState(event.tid).pred.squashInstructionState(event.seqNum); }

Prediction
RUNLTS::lookup(ThreadID tid, Addr pc, void *&bp_history)
{
    fatal("RUNLTS requires the sequence-number-aware O3 predictor "
          "interface");
}

Prediction
RUNLTS::lookup(ThreadID tid, Addr pc, InstSeqNum seq_no, void *&bp_history)
{
    ThreadState &state = threadState(tid);

    auto *h = new RUNLTSHistory();
    h->seqNum = seq_no;
    const bool pred = state.pred.predict(pc, seq_no, h->snap);
    bp_history = static_cast<void *>(h);
    return predictWithDefaultLatency(pred);
}

void
RUNLTS::updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                        Addr target, const StaticInstPtr &inst,
                        void *&bp_history)
{
    fatal("RUNLTS requires the sequence-number-aware O3 predictor "
          "interface");
}

void
RUNLTS::updateHistories(ThreadID tid, Addr pc, InstSeqNum seq_no, bool uncond,
                        bool taken, Addr target, const StaticInstPtr &inst,
                        void *&bp_history)
{
    ThreadState &state = threadState(tid);

    if (!bp_history) {
        assert(uncond);
        auto *h = new RUNLTSHistory();
        h->seqNum = seq_no;
        state.pred.predict(pc, seq_no, h->snap);
        bp_history = static_cast<void *>(h);
    }

    const bool is_cond = !uncond;
    auto *h = static_cast<RUNLTSHistory *>(bp_history);
    state.pred.historyUpdate(pc, is_cond, taken, target, inst, h->snap);
}

void
RUNLTS::branchPlaceholder(ThreadID tid, Addr pc, bool uncond,
                          void *&bp_history)
{ fatal("RUNLTS does not currently support decoupled frontend."); }

void
RUNLTS::update(ThreadID tid, Addr pc, bool taken, void *&bp_history,
               bool squashed, const StaticInstPtr &inst, Addr target)
{
    if (squashed) {
        if (bp_history && threadStates.size() > tid && threadStates[tid]) {
            auto *h = static_cast<RUNLTSHistory *>(bp_history);
            threadStates[tid]->pred.restore(h->snap);
            // Correct history
            threadStates[tid]->pred.historyUpdate(
                pc, inst->isCondCtrl(), taken, target, inst, h->snap);
        }
        return;
    }

    if (bp_history && inst->isCondCtrl()) {
        auto *h = static_cast<RUNLTSHistory *>(bp_history);
        threadStates[tid]->pred.predict_using_given_hist(pc, h->snap,
                                                         h->snap.hist);
        threadStates[tid]->pred.update(pc, taken, h->snap);
    }

    if (bp_history) {
        auto *h = static_cast<RUNLTSHistory *>(bp_history);
        threadStates[tid]->pred.commit(h->seqNum);
        delete h;
        bp_history = nullptr;
    }
}

void
RUNLTS::squash(ThreadID tid, void *&bp_history)
{
    if (threadStates.size() > tid && threadStates[tid]) {
        auto *h = static_cast<RUNLTSHistory *>(bp_history);
        // Restore the predictor history to the prediction-time snapshot
        threadStates[tid]->pred.restore(h->snap);
    }

    delete static_cast<RUNLTSHistory *>(bp_history);
    bp_history = nullptr;
}

} // namespace branch_prediction
} // namespace gem5
