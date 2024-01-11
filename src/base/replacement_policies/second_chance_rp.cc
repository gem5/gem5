/**
 * Copyright (c) 2018-2020 Inria
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

#include "base/replacement_policies/second_chance_rp.hh"

#include <cassert>

#include "params/SecondChanceRP.hh"

namespace gem5
{

namespace replacement_policy
{

SecondChance::SecondChance(const Params &p)
  : FIFO(p)
{
}

void
SecondChance::useSecondChance(
    const std::shared_ptr<SecondChanceReplData>& replacement_data) const
{
    // Reset FIFO data
    FIFO::reset(replacement_data);

    // Use second chance
    replacement_data->hasSecondChance = false;
}

void
SecondChance::invalidate(
    const std::shared_ptr<ReplacementData>& replacement_data)
{
    FIFO::invalidate(replacement_data);

    // Do not give a second chance to invalid entries
    std::static_pointer_cast<SecondChanceReplData>(
        replacement_data)->hasSecondChance = false;
}

void
SecondChance::touch(
    const std::shared_ptr<ReplacementData>& replacement_data) const
{
    FIFO::touch(replacement_data);

    // Whenever an entry is touched, it is given a second chance
    std::static_pointer_cast<SecondChanceReplData>(
        replacement_data)->hasSecondChance = true;
}

void
SecondChance::reset(
    const std::shared_ptr<ReplacementData>& replacement_data) const
{
    FIFO::reset(replacement_data);

    // Entries are inserted with a second chance
    std::static_pointer_cast<SecondChanceReplData>(
        replacement_data)->hasSecondChance = false;
}

ReplaceableEntry*
SecondChance::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Search for invalid entries, as they have the eviction priority
    for (const auto& candidate : candidates) {
        // Cast candidate's replacement data
        std::shared_ptr<SecondChanceReplData> candidate_replacement_data =
            std::static_pointer_cast<SecondChanceReplData>(
                candidate->replacementData);

        // Stop iteration if found an invalid entry
        if ((candidate_replacement_data->tickInserted == Tick(0)) &&
            !candidate_replacement_data->hasSecondChance) {
            return candidate;
        }
    }

    // Visit all candidates to find victim
    ReplaceableEntry* victim = candidates[0];
    bool search_victim = true;
    while (search_victim) {
        // Do a FIFO victim search
        victim = FIFO::getVictim(candidates);

        // Cast victim's replacement data for code readability
        std::shared_ptr<SecondChanceReplData> victim_replacement_data =
            std::static_pointer_cast<SecondChanceReplData>(
                victim->replacementData);

        // If victim has a second chance, use it and repeat search
        if (victim_replacement_data->hasSecondChance) {
            useSecondChance(victim_replacement_data);
        } else {
            // Found victim
            search_victim = false;
        }
    }

    return victim;
}

std::shared_ptr<ReplacementData>
SecondChance::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new SecondChanceReplData());
}

} // namespace replacement_policy
} // namespace gem5
