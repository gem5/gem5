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
 * @file lvp.cc
 * @brief Implementation of the Load Value Predictor (LVP) for the gem5 O3 CPU.
 *
 * See lvp.hh for the full architectural description and the connection to
 * the FLOP speculative type-confusion vulnerability.
 */

#include "cpu/o3/lvp.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/LVP.hh"

namespace gem5
{

namespace o3
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

LoadValuePredictor::LoadValuePredictor(const LoadValuePredictorParams &params)
    : SimObject(params),
      numEntries(params.numEntries),
      confidenceThreshold(params.confidenceThreshold),
      instShiftAmount(params.instShiftAmount),
      table(params.numEntries),   // zero-initialises all LVPEntry fields
      lvpStats(this)
{
    // numEntries must be a power of two so the bitmask index calculation works.
    fatal_if(!isPowerOf2(numEntries),
             "LVP: numEntries (%u) must be a power of two.", numEntries);

    DPRINTF(LVP, "LoadValuePredictor created: numEntries=%u, "
                 "confidenceThreshold=%u, instShiftAmount=%u\n",
            numEntries, confidenceThreshold, instShiftAmount);
}

// ---------------------------------------------------------------------------
// Statistics initialisation
// ---------------------------------------------------------------------------

LoadValuePredictor::LVPStats::LVPStats(statistics::Group *parent)
    : statistics::Group(parent, "lvp"),
      ADD_STAT(totalLookups, statistics::units::Count::get(),
               "Total load instructions queried at Rename"),
      ADD_STAT(totalPredictions, statistics::units::Count::get(),
               "Loads for which a confident prediction was returned"),
      ADD_STAT(predictionHits, statistics::units::Count::get(),
               "Correct predictions (real value matched the prediction)"),
      ADD_STAT(predictionMisses, statistics::units::Count::get(),
               "Mispredictions (real value differed; pipeline squashed)")
{
}

// ---------------------------------------------------------------------------
// lookup() — called at the Rename stage
// ---------------------------------------------------------------------------

bool
LoadValuePredictor::lookup(Addr pc, ThreadID tid, uint64_t &predictedValue)
{
    // Count every rename-stage load query regardless of the outcome.
    ++lvpStats.totalLookups;

    const unsigned idx = getIndex(pc);
    const LVPEntry &entry = table[idx];

    // A hit requires: (1) the entry is valid, (2) the PC tag matches (guards
    // against aliasing from a different instruction mapped to the same index),
    // and (3) the counter has accumulated enough confidence.
    if (!entry.valid || entry.tag != pc ||
        entry.counter < confidenceThreshold) {
        DPRINTF(LVP, "[tid:%d] Lookup PC=%#x → no confident prediction "
                     "(valid=%d, tag_match=%d, counter=%u/%u)\n",
                tid, pc,
                entry.valid,
                entry.valid && (entry.tag == pc),
                entry.valid ? entry.counter : 0,
                confidenceThreshold);
        return false;
    }

    // We have a confident prediction — forward the stored value.
    predictedValue = entry.predictedValue;
    ++lvpStats.totalPredictions;

    DPRINTF(LVP, "[tid:%d] Lookup PC=%#x → confident prediction 0x%llx "
                 "(counter=%u)\n",
            tid, pc, predictedValue, entry.counter);
    return true;
}

// ---------------------------------------------------------------------------
// update() — called at the IEW stage after the LSQ resolves the real value
// ---------------------------------------------------------------------------

bool
LoadValuePredictor::update(Addr pc, uint64_t actualValue,
                           unsigned dataSize, ThreadID tid)
{
    // ---- Heuristic filter -------------------------------------------------
    // Ignore 8-byte (64-bit) loads.  Pointer-sized values change too
    // frequently to be useful predictions and, more importantly, predicting
    // them would allow an attacker to redirect arbitrary dereferences —
    // something real hardware explicitly avoids.  The FLOP paper confirms
    // that Apple's LVP does not predict 8-byte loads.
    if (dataSize == 8) {
        DPRINTF(LVP, "[tid:%d] PC=%#x: skipping 8-byte load (pointer heuristic)\n",
                tid, pc);
        return false;   // no misprediction possible; entry unchanged
    }
    // -----------------------------------------------------------------------

    const unsigned idx = getIndex(pc);
    LVPEntry &entry = table[idx];
    bool mispredicted = false;

    if (!entry.valid || entry.tag != pc) {
        // ---- Cold miss / conflict miss: allocate a new entry ---------------
        // Direct-mapped: the incoming PC simply evicts whoever was here.
        DPRINTF(LVP, "[tid:%d] PC=%#x: allocating entry[%u] "
                     "(evicting tag=%#x)\n",
                tid, pc, idx, entry.tag);
        entry.tag           = pc;
        entry.predictedValue = actualValue;
        entry.counter       = 1;
        entry.valid         = true;

    } else if (actualValue == entry.predictedValue) {
        // ---- Correct observation: saturate-increment the counter -----------
        // Cap at 255 (uint8_t max) so the counter never wraps around.
        if (entry.counter < 255) {
            ++entry.counter;
        }
        DPRINTF(LVP, "[tid:%d] PC=%#x: hit, counter now %u\n",
                tid, pc, entry.counter);

        // Check whether this was a confident prediction that came true.
        // (counter was already >= threshold before the increment, meaning
        //  we would have issued a prediction at Rename for this load.)
        if (entry.counter >= confidenceThreshold) {
            ++lvpStats.predictionHits;
        }

    } else {
        // ---- Mismatch: strong-bias reset -----------------------------------
        // The FLOP paper documents that Apple's LVP uses a strong-bias reset:
        // a single disagreement immediately zeros the counter, forcing 250
        // clean observations before the predictor is trusted again.
        DPRINTF(LVP, "[tid:%d] PC=%#x: MISPREDICTION "
                     "(predicted=0x%llx, actual=0x%llx), resetting counter\n",
                tid, pc, entry.predictedValue, actualValue);

        // A misprediction only matters if we were actually confident enough
        // to have issued a prediction at Rename.
        if (entry.counter >= confidenceThreshold) {
            ++lvpStats.predictionMisses;
            mispredicted = true;   // signal to IEW: trigger a squash
        }

        // Update stored value to the new observation and reset counter.
        entry.predictedValue = actualValue;
        entry.counter        = 0;
    }

    return mispredicted;
}

// ---------------------------------------------------------------------------
// clear() — called on context switch or pipeline drain
// ---------------------------------------------------------------------------

void
LoadValuePredictor::clear()
{
    DPRINTF(LVP, "Clearing all LVP table entries.\n");
    for (auto &entry : table) {
        entry.valid   = false;
        entry.counter = 0;
    }
}

} // namespace o3
} // namespace gem5
