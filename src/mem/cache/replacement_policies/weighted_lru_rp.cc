/*
 * Copyright (c) 2013-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/cache/replacement_policies/weighted_lru_rp.hh"

#include <cassert>

#include "params/WeightedLRURP.hh"

WeightedLRUPolicy::WeightedLRUPolicy(const Params* p)
    : BaseReplacementPolicy(p)
{
}

WeightedLRUPolicy *
WeightedLRURPParams::create()
{
    return new WeightedLRUPolicy(this);
}

void
WeightedLRUPolicy::touch(const std::shared_ptr<ReplacementData>&
                                                  replacement_data) const
{
    std::static_pointer_cast<WeightedLRUReplData>(replacement_data)->
                                                 last_touch_tick = curTick();
}

void
WeightedLRUPolicy::touch(const std::shared_ptr<ReplacementData>&
                        replacement_data, int occupancy) const
{
    std::static_pointer_cast<WeightedLRUReplData>(replacement_data)->
                                                  last_touch_tick = curTick();
    std::static_pointer_cast<WeightedLRUReplData>(replacement_data)->
                                                  last_occ_ptr = occupancy;
}

ReplaceableEntry*
WeightedLRUPolicy::getVictim(const ReplacementCandidates& candidates) const
{
    assert(candidates.size() > 0);

    ReplaceableEntry* victim = candidates[0];
    // Use weight (last_occ_ptr) to find victim.
    // Evict the block that has the smallest weight.
    // If two blocks have the same weight, evict the oldest one.
    for (const auto& candidate : candidates) {
        // candidate's replacement_data
        std::shared_ptr<WeightedLRUReplData> candidate_replacement_data =
            std::static_pointer_cast<WeightedLRUReplData>(
                                             candidate->replacementData);
        // victim's replacement_data
        std::shared_ptr<WeightedLRUReplData> victim_replacement_data =
            std::static_pointer_cast<WeightedLRUReplData>(
                                             victim->replacementData);

        if (candidate_replacement_data->last_occ_ptr <
                    victim_replacement_data->last_occ_ptr) {
            victim = candidate;
        } else if (candidate_replacement_data->last_occ_ptr ==
                    victim_replacement_data->last_occ_ptr) {
            // Evict the block with a smaller tick.
            Tick time = candidate_replacement_data->last_touch_tick;
            if (time < victim_replacement_data->last_touch_tick) {
                victim = candidate;
            }
        }
    }
    return victim;
}

std::shared_ptr<ReplacementData>
WeightedLRUPolicy::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new WeightedLRUReplData);
}

void
WeightedLRUPolicy::reset(const std::shared_ptr<ReplacementData>&
                                                    replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<WeightedLRUReplData>(
        replacement_data)->last_touch_tick = curTick();
}

void
WeightedLRUPolicy::invalidate(const std::shared_ptr<ReplacementData>&
                                                    replacement_data) const
{
    // Reset last touch timestamp
    std::static_pointer_cast<WeightedLRUReplData>(
        replacement_data)->last_touch_tick = Tick(0);
}
