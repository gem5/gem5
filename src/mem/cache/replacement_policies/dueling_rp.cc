/**
 * Copyright (c) 2019, 2020 Inria
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

#include "mem/cache/replacement_policies/dueling_rp.hh"

#include "base/logging.hh"
#include "params/DuelingRP.hh"

namespace gem5
{

Dueling::Dueling(const Params &p)
  : BaseReplacementPolicy(p), replPolicyA(p.replacement_policy_a),
    replPolicyB(p.replacement_policy_b),
    duelingMonitor(p.constituency_size, p.team_size),
    duelingStats(this)
{
    fatal_if((replPolicyA == nullptr) || (replPolicyB == nullptr),
        "All replacement policies must be instantiated");
}

void
Dueling::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    std::shared_ptr<DuelerReplData> casted_replacement_data =
        std::static_pointer_cast<DuelerReplData>(replacement_data);
    replPolicyA->invalidate(casted_replacement_data->replDataA);
    replPolicyB->invalidate(casted_replacement_data->replDataB);
}

void
Dueling::touch(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    std::shared_ptr<DuelerReplData> casted_replacement_data =
        std::static_pointer_cast<DuelerReplData>(replacement_data);
    replPolicyA->touch(casted_replacement_data->replDataA, pkt);
    replPolicyB->touch(casted_replacement_data->replDataB, pkt);
}

void
Dueling::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    std::shared_ptr<DuelerReplData> casted_replacement_data =
        std::static_pointer_cast<DuelerReplData>(replacement_data);
    replPolicyA->touch(casted_replacement_data->replDataA);
    replPolicyB->touch(casted_replacement_data->replDataB);
}

void
Dueling::reset(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    std::shared_ptr<DuelerReplData> casted_replacement_data =
        std::static_pointer_cast<DuelerReplData>(replacement_data);
    replPolicyA->reset(casted_replacement_data->replDataA, pkt);
    replPolicyB->reset(casted_replacement_data->replDataB, pkt);

    // A miss in a set is a sample to the duel. A call to this function
    // implies in the replacement of an entry, which was either caused by
    // a miss, an external invalidation, or the initialization of the table
    // entry (when warming up)
    duelingMonitor.sample(static_cast<Dueler*>(casted_replacement_data.get()));
}

void
Dueling::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    std::shared_ptr<DuelerReplData> casted_replacement_data =
        std::static_pointer_cast<DuelerReplData>(replacement_data);
    replPolicyA->reset(casted_replacement_data->replDataA);
    replPolicyB->reset(casted_replacement_data->replDataB);

    // A miss in a set is a sample to the duel. A call to this function
    // implies in the replacement of an entry, which was either caused by
    // a miss, an external invalidation, or the initialization of the table
    // entry (when warming up)
    duelingMonitor.sample(static_cast<Dueler*>(casted_replacement_data.get()));
}

ReplaceableEntry*
Dueling::getVictim(const ReplacementCandidates& candidates) const
{
    // This function assumes that all candidates are either part of the same
    // sampled set, or are not samples.
    // @todo This should be improved at some point.
    panic_if(candidates.size() != params().team_size, "We currently only "
        "support team sizes that match the number of replacement candidates");

    // The team with the most misses loses
    bool winner = !duelingMonitor.getWinner();

    // If the entry is a sample, it can only be used with a certain policy.
    bool team;
    bool is_sample = duelingMonitor.isSample(static_cast<Dueler*>(
        std::static_pointer_cast<DuelerReplData>(
            candidates[0]->replacementData).get()), team);

    // All replacement candidates must be set appropriately, so that the
    // proper replacement data is used. A replacement policy X must be used
    // if the candidates are its samples - in which case they must always
    // use X - or if it is not a sample, and X is currently the best RP.
    // This assumes that A's team is "false", and B's team is "true".
    bool team_a;
    if ((is_sample && !team) || (!is_sample && !winner)) {
        duelingStats.selectedA++;
        team_a = true;
    } else {
        duelingStats.selectedB++;
        team_a = false;
    }

    // Create a temporary list of replacement candidates which re-routes the
    // replacement data of the selected team
    std::vector<std::shared_ptr<ReplacementData>> dueling_replacement_data;
    for (auto& candidate : candidates) {
        std::shared_ptr<DuelerReplData> dueler_repl_data =
            std::static_pointer_cast<DuelerReplData>(
            candidate->replacementData);

        // As of now we assume that all candidates are either part of
        // the same sampled team, or are not samples.
        bool candidate_team;
        panic_if(
            duelingMonitor.isSample(dueler_repl_data.get(), candidate_team) &&
            (team != candidate_team),
            "Not all sampled candidates belong to the same team");

        // Copy the original entry's data, re-routing its replacement data
        // to the selected one
        dueling_replacement_data.push_back(dueler_repl_data);
        candidate->replacementData = team_a ? dueler_repl_data->replDataA :
            dueler_repl_data->replDataB;
    }

    // Use the selected replacement policy to find the victim
    ReplaceableEntry* victim = team_a ? replPolicyA->getVictim(candidates) :
        replPolicyB->getVictim(candidates);

    // Search for entry within the original candidates and clean-up duplicates
    for (int i = 0; i < candidates.size(); i++) {
        candidates[i]->replacementData = dueling_replacement_data[i];
    }

    return victim;
}

std::shared_ptr<ReplacementData>
Dueling::instantiateEntry()
{
    DuelerReplData* replacement_data = new DuelerReplData(
        replPolicyA->instantiateEntry(), replPolicyB->instantiateEntry());
    duelingMonitor.initEntry(static_cast<Dueler*>(replacement_data));
    return std::shared_ptr<DuelerReplData>(replacement_data);
}

Dueling::DuelingStats::DuelingStats(statistics::Group* parent)
  : statistics::Group(parent),
    ADD_STAT(selectedA, "Number of times A was selected to victimize"),
    ADD_STAT(selectedB, "Number of times B was selected to victimize")
{
}

} // namespace gem5
