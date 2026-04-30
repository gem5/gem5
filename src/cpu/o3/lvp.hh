/*
 * Copyright (c) 2026 NITC Calicut Research
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

/**
 * @file lvp.hh
 * @brief Load Value Predictor (LVP) Module for the gem5 O3 Pipeline.
 *
 * This module replicates the Load Value Prediction mechanism identified
 * in the FLOP paper: "FLOP: Breaking Apple M3 CPUs with Load Value
 * Prediction" (Kim et al., USENIX Security 2025).
 *
 * The predictor works as follows:
 *   - A PC-tagged table stores the most recently observed value returned
 *     by a specific load instruction.
 *   - A saturating counter tracks how many consecutive times that same
 *     value has been returned.
 *   - Once the counter reaches the confidenceThreshold (default: 250),
 *     the predictor is "confident" and will speculatively forward the
 *     predicted value to dependent instructions at the Rename stage.
 *   - At the IEW/Execute stage, the real memory value is compared against
 *     the prediction. A mismatch triggers a pipeline squash.
 *
 * Heuristic Filter:
 *   - 8-byte (64-bit) loads are intentionally ignored by the predictor.
 *     This mirrors real hardware behaviour and prevents pointer values
 *     from being predicted, ensuring the vulnerability surface matches
 *     the FLOP paper's findings (integer "type" fields, not addresses).
 */

#ifndef __CPU_O3_LVP_HH__
#define __CPU_O3_LVP_HH__

#include <cstdint>
#include <string>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"

namespace gem5
{

namespace o3
{

/**
 * @brief A single entry in the LVP table.
 *
 * Each entry is keyed by the PC (tag) of the load instruction.
 * The saturating counter accumulates confidence. When it reaches
 * confidenceThreshold, the predictedValue is forwarded speculatively
 * to any instruction that depends on this load's result.
 */
struct LVPEntry
{
    /** PC tag of the load instruction that owns this entry. */
    Addr tag;

    /**
     * The most recently observed 64-bit value returned by memory for
     * the load instruction at 'tag'. Only the low bits matching the
     * load width are meaningful; the rest are zero-extended.
     */
    uint64_t predictedValue;

    /**
     * Saturating confidence counter (8-bit, range 0–255).
     * Incremented on each correct observation; reset to 0 on a mismatch.
     * The predictor is "confident" only when this reaches confidenceThreshold.
     */
    uint8_t counter;

    /** True if this entry has been written at least once. */
    bool valid;

    /** Construct a zeroed, invalid entry. */
    LVPEntry() : tag(0), predictedValue(0), counter(0), valid(false) {}
};

/**
 * @brief Load Value Predictor: PC-tagged speculative value forwarding table.
 *
 * Instantiated once per CPU and consulted at two pipeline stages:
 *
 *   Rename  — lookup(pc, tid) is called for every load instruction.
 *             If the predictor is confident, the returned value is
 *             injected into the destination physical register immediately,
 *             resolving the data dependency without waiting for the cache.
 *
 *   IEW     — update(pc, actualValue, dataSize, tid) is called when the
 *             LSQ completes the real memory access.
 *             - Match   → counter increments (capped at 255).
 *             - Mismatch → counter resets to 0, misprediction is flagged,
 *               and the caller must trigger a pipeline squash.
 */
class LoadValuePredictor
{
  public:
    /**
     * @brief Constructor.
     *
     * @param name_             Object name (used in stats output).
     * @param numEntries_       Table size (must be power of two, default 1024).
     * @param threshold_        Confidence counter threshold (default 250).
     * @param shiftAmt_         PC right-shift before indexing (default 2).
     */
    LoadValuePredictor(const std::string &name_,
                       unsigned numEntries_ = 1024,
                       unsigned threshold_  = 250,
                       unsigned shiftAmt_   = 2);

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /**
     * @brief Consult the predictor at the Rename stage.
     *
     * Looks up the entry for 'pc'. Returns a confident prediction only when
     * the entry is valid and its counter has reached confidenceThreshold.
     * All lookups (even non-confident ones) are counted in totalLookups.
     *
     * @param pc             Program counter of the load instruction.
     * @param tid            Hardware thread issuing the instruction.
     * @param predictedValue [out] Set to the predicted value on success.
     * @return               true  → confident prediction available;
     *                       false → no prediction (let the load proceed normally).
     */
    bool lookup(Addr pc, ThreadID tid, uint64_t &predictedValue);

    /**
     * @brief Train / verify the predictor at the IEW stage.
     *
     * Called after the LSQ retrieves the actual value from cache/memory.
     *
     * Heuristic filter: if dataSize == 8 (64-bit load), the function
     * returns immediately without updating the table. This prevents the
     * predictor from learning pointer values, which mirrors the selective
     * behaviour documented in the FLOP paper.
     *
     * If the entry for 'pc' already exists:
     *   - actualValue matches predictedValue → saturate-increment counter.
     *   - actualValue differs               → reset counter to 0 (strong-bias
     *     reset policy), overwrite stored value with the new observation.
     *
     * If no entry exists yet, allocate one (evicting the slot via direct-map
     * hashing) and initialise the counter to 1.
     *
     * @param pc          Program counter of the load instruction.
     * @param actualValue Real 64-bit value returned by the memory subsystem.
     * @param dataSize    Width of the load in bytes (1, 2, 4, or 8).
     * @param tid         Hardware thread issuing the instruction.
     * @return            true if a misprediction occurred (predicted ≠ actual
     *                    AND the entry was previously confident); the caller
     *                    (IEW stage) must squash the pipeline on true.
     */
    bool update(Addr pc, uint64_t actualValue,
                unsigned dataSize, ThreadID tid);

    /** Reset all table entries and statistics (e.g., after a context switch). */
    void clear();

  private:
    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /**
     * @brief Map a PC to a table index.
     *
     * Shifts the PC right by instShiftAmount (2 for 32-bit ARM instructions)
     * to remove the always-zero low bits, then masks to numEntries.
     */
    inline unsigned getIndex(Addr pc) const
    {
        return (pc >> instShiftAmount) & (numEntries - 1);
    }

    // -----------------------------------------------------------------------
    // Parameters (set once at construction, never modified)
    // -----------------------------------------------------------------------

    /** Number of entries in the prediction table (must be a power of two). */
    const unsigned numEntries;

    /**
     * Minimum counter value required before a prediction is issued.
     * Default 250 per the FLOP paper's empirical reverse-engineering result.
     */
    const unsigned confidenceThreshold;

    /**
     * Right-shift applied to PC before indexing.
     * 2 for ARM (all instructions are 4-byte aligned → low 2 bits always 0).
     */
    const unsigned instShiftAmount;

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    /** The flat prediction table, indexed by getIndex(pc). */
    std::vector<LVPEntry> table;

    // -----------------------------------------------------------------------
    // gem5 Statistics
    // -----------------------------------------------------------------------

    /**
     * @brief Statistics group for the LVP module.
     *
     * Exposed in m5out/stats.txt under the parent CPU's namespace so the
     * evaluation phase can directly compare lookup/hit/miss rates across
     * the three configurations (No LVP / Vulnerable LVP / Managed LVP).
     */
    struct LVPStats : public statistics::Group
    {
        LVPStats(statistics::Group *parent);

        /** Total number of times lookup() was called (one per renamed load). */
        statistics::Scalar totalLookups;

        /** Number of lookups that returned a confident prediction. */
        statistics::Scalar totalPredictions;

        /** Predictions where the real value matched (counter incremented). */
        statistics::Scalar predictionHits;

        /**
         * Predictions where the real value differed (counter reset to 0).
         * A non-zero count here means the pipeline squashed at least once.
         */
        statistics::Scalar predictionMisses;
    } lvpStats;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_LVP_HH__
