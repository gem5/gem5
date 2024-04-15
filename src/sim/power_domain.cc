/*
 * Copyright (c) 2017, 2019-2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "sim/power_domain.hh"

#include <unordered_map>

#include "base/trace.hh"
#include "debug/PowerDomain.hh"

namespace gem5
{

PowerDomain::PowerDomain(const PowerDomainParams &p)
    : PowerState(p),
      leaders(p.leaders),
      pwrStateUpdateEvent(*this),
      stats(*this)
{
    // Check if there is at least one leader
    fatal_if(leaders.empty(), "No leaders registered in %s!)", name());

    // Go over the leaders and register this power domain with them
    for (auto leader : leaders) {
        leader->setControlledDomain(this);
    }

    // We will assume a power domain to start in the most performant p-state
    // This will be corrected during startup()
    leaderTargetState = enums::PwrState::ON;
    _currState = enums::PwrState::ON;
}

void
PowerDomain::addFollower(PowerState *pwr_obj)
{
    DPRINTF(PowerDomain, "%s is a follower in %s\n", pwr_obj->name(), name());
    followers.push_back(pwr_obj);
}

void
PowerDomain::startup()
{
    DPRINTF(PowerDomain, "Checks at startup\n");
    // Check if the leaders and followers have the correct power states.
    DPRINTF(PowerDomain, "Checking power state of leaders & followers\n");
    for (const auto &objs : { leaders, followers }) {
        for (const auto &obj : objs) {
            const auto &states = obj->getPossibleStates();
            auto it = states.find(enums::PwrState::ON);
            fatal_if(it == states.end(),
                     "%s in %s does not have the required power states to be "
                     "part of a PowerDomain i.e. the ON state!",
                     obj->name(), name());
        }
    }

    // Now all objects have been checked for the minimally required power
    // states, calculate the possible power states for the domain. This is the
    // intersection between the possible power states of the followers and
    // leaders.
    calculatePossiblePwrStates();

    // Check that there is no objects which is both a leader and a
    // follower.
    DPRINTF(PowerDomain, "Checking for double entries\n");
    for (auto follower : followers) {
        for (auto leader : leaders) {
            fatal_if(leader == follower,
                     "%s is both a leader and follower"
                     " in %s\n!",
                     leader->name(), name());
        }
    }
    // Record the power states of the leaders and followers
    DPRINTF(PowerDomain, "Recording the current power states in domain\n");
    for (auto leader : leaders) {
        enums::PwrState pws = leader->get();
        fatal_if(pws == enums::PwrState::UNDEFINED,
                 "%s is in the UNDEFINED power state, not acceptable as "
                 "leader!",
                 leader->name());
    }

    // Calculate the power state of the domain, only looking at leader
    leaderTargetState = calculatePowerDomainState();
    // Set the power states of the followers, based upon leaderTargetState.
    setFollowerPowerStates();
}

bool
PowerDomain::isPossiblePwrState(enums::PwrState p_state)
{
    for (const auto &objs : { leaders, followers }) {
        for (const auto &obj : objs) {
            const auto &obj_states = obj->getPossibleStates();
            if (obj_states.find(p_state) == obj_states.end()) {
                return false;
            }
        }
    }
    return true;
}

void
PowerDomain::calculatePossiblePwrStates()
{
    assert(possibleStates.empty());
    for (auto p_state : leaders[0]->getPossibleStates()) {
        if (isPossiblePwrState(p_state)) {
            possibleStates.emplace(p_state);
            DPRINTF(PowerDomain, "%u/%s is a p-state\n", p_state,
                    enums::PwrStateStrings[p_state]);
        }
    }
}

enums::PwrState
PowerDomain::calculatePowerDomainState(
    const std::vector<enums::PwrState> &f_states)
{
    DPRINTF(PowerDomain, "Calculating the power state\n");
    enums::PwrState most_perf_state = enums::PwrState::Num_PwrState;
    std::string most_perf_leader;
    for (auto leader : leaders) {
        enums::PwrState pw = leader->get();
        if (pw < most_perf_state) {
            most_perf_state = pw;
            most_perf_leader = leader->name();
        }
    }
    assert(most_perf_state != enums::PwrState::Num_PwrState);
    DPRINTF(PowerDomain, "Most performant leader is %s, at %u\n",
            most_perf_leader, most_perf_state);

    // If asked to check the power states of the followers (f_states contains
    // the power states of the followers)
    if (!f_states.empty()) {
        for (enums::PwrState f_pw : f_states) {
            // Ignore UNDEFINED state of follower, at startup the followers
            // might be in the UNDEFINED state, PowerDomain will pull them up
            if ((f_pw != enums::PwrState::UNDEFINED) &&
                (f_pw < most_perf_state)) {
                most_perf_state = f_pw;
            }
        }
        DPRINTF(PowerDomain,
                "Most performant state, including followers "
                "is %u\n",
                most_perf_state);
    }
    return most_perf_state;
}

void
PowerDomain::setFollowerPowerStates()
{
    // Loop over all followers and tell them to change their power state so
    // they match that of the power domain (or a more performant power state)
    std::vector<enums::PwrState> matched_states;
    for (auto follower : followers) {
        enums::PwrState actual_pws =
            follower->matchPwrState(leaderTargetState);
        matched_states.push_back(actual_pws);
        assert(actual_pws <= leaderTargetState);
        DPRINTF(PowerDomain, "%u matched domain power state (%u) with %u\n",
                follower->name(), leaderTargetState, actual_pws);
    }
    // Now the power states of the follower have been changed recalculate the
    // power state of the domain as a whole, including followers
    enums::PwrState new_power_state =
        calculatePowerDomainState(matched_states);
    if (new_power_state != _currState) {
        // Change in power state of the domain, so update. Updates in power
        // state need to happen via set() so it can propagate to
        // overarching power domains (if there are any).
        DPRINTF(PowerDomain, "Updated power domain state to %u\n",
                new_power_state);
        set(new_power_state);
    }
}

void
PowerDomain::pwrStateChangeCallback(enums::PwrState new_pwr_state,
                                    PowerState *leader)
{
    DPRINTF(PowerDomain, "PwrState update to %u by %s\n", new_pwr_state,
            leader->name());

    enums::PwrState old_target_state = leaderTargetState;
    // Calculate the power state of the domain, based on the leaders
    if (new_pwr_state < _currState) {
        // The power state of the power domain always needs to match that of
        // the most performant leader so no need to go over the other leaders
        // The power state need to be changed via a the PwrStateCall() so any
        // overarching power domains get informed
        leaderTargetState = new_pwr_state;
    } else {
        // Need to calculate the newly required power state, based on the
        // leaders only and change to that state.
        leaderTargetState = calculatePowerDomainState();
    }
    if (old_target_state != leaderTargetState) {
        // The followers will try to match that power state requested by the
        // leaders in in the update event, based upon the actual power state,
        // we will 'officially' change the power state of the domain by calling
        // set()
        schedule(pwrStateUpdateEvent, curTick() + updateLatency);
        DPRINTF(PowerDomain,
                "TargetState change from %u to %u, followers will"
                "be updated in %u ticks\n",
                old_target_state, leaderTargetState, updateLatency);
        stats.numLeaderCallsChangingState++;
    }
    stats.numLeaderCalls++;
}

PowerDomain::PowerDomainStats::PowerDomainStats(PowerDomain &pd)
    : statistics::Group(&pd),
      ADD_STAT(numLeaderCalls, statistics::units::Count::get(),
               "Number of calls by leaders to change power domain state"),
      ADD_STAT(
          numLeaderCallsChangingState, statistics::units::Count::get(),
          "Number of calls by leader to change power domain state actually "
          "resulting in a power state change")
{}

void
PowerDomain::PowerDomainStats::regStats()
{
    statistics::Group::regStats();

    numLeaderCalls.flags(statistics::nozero);
    numLeaderCallsChangingState.flags(statistics::nozero);
}

} // namespace gem5
