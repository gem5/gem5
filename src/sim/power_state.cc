/*
 * Copyright (c) 2015-2017, 2019-2020 ARM Limited
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

#include "sim/power_state.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PowerDomain.hh"
#include "sim/power_domain.hh"

PowerState::PowerState(const PowerStateParams *p) :
    SimObject(p), _currState(p->default_state),
    possibleStates(p->possible_states.begin(),
                   p->possible_states.end()),
    stats(*this)
{
    for (auto &pm: p->leaders) {
        // Register this object as a follower. This object is
        // dependent on pm for power state transitions
        pm->addFollower(this);
    }
}

void
PowerState::setControlledDomain(PowerDomain* pwr_dom)
{
    // Only a power domain can register as dependant of a power stated
    // object
    controlledDomain = pwr_dom;
    DPRINTF(PowerDomain, "%s is registered as controlled by %s \n",
                         pwr_dom->name(), name());
}

void
PowerState::serialize(CheckpointOut &cp) const
{
    unsigned int currState = (unsigned int)_currState;

    SERIALIZE_SCALAR(currState);
    SERIALIZE_SCALAR(prvEvalTick);
}

void
PowerState::unserialize(CheckpointIn &cp)
{
    unsigned int currState;

    UNSERIALIZE_SCALAR(currState);
    UNSERIALIZE_SCALAR(prvEvalTick);

    _currState = Enums::PwrState(currState);
}

void
PowerState::set(Enums::PwrState p)
{
    // Check if this power state is actually allowed by checking whether it is
    // present in pwrStateToIndex-dictionary
    panic_if(possibleStates.find(p) == possibleStates.end(),
             "Cannot go to %s in %s \n", Enums::PwrStateStrings[p], name());

    // Function should ideally be called only when there is a state change
    if (_currState == p) {
        warn_once("PowerState: Already in the requested power state, "
                  "request ignored");
        return;
    }

    // No need to compute stats if in the same tick, update state though. This
    // can happen in cases like a) during start of the simulation multiple
    // state changes happens in init/startup phase, b) one takes a decision to
    // migrate state but decides to reverts back to the original state in the
    // same tick if other conditions are not met elsewhere.
    // Any state change related stats would have been recorded on previous call
    // to this function.
    if (prvEvalTick == curTick() && curTick() != 0) {
        warn("PowerState: More than one power state change request "
             "encountered within the same simulation tick");
        _currState = p;
        return;
    }

    // Record stats for previous state.
    computeStats();

    _currState = p;

    stats.numTransitions++;

    // Update the domain this object controls, if there is one
    if (controlledDomain) {
        controlledDomain->pwrStateChangeCallback(p, this);
    }

}

Enums::PwrState
PowerState::matchPwrState(Enums::PwrState p)
{
    // If the object is asked to match a power state, it has to be a follower
    // and hence should not have a pointer to a powerDomain
    assert(controlledDomain == nullptr);

    // If we are already in this power state, ignore request
    if (_currState == p) {
        DPRINTF(PowerDomain, "Already in p-state %s requested to match \n",
                Enums::PwrStateStrings[p]);
        return _currState;
    }

    Enums::PwrState old_state = _currState;
    if (possibleStates.find(p) != possibleStates.end()) {
        // If this power state is allowed in this object, just go there
        set(p);
    } else {
        // Loop over all power states in this object and find a power state
        // which is more performant than the requested one (considering we
        // cannot match it exactly)
        for (auto rev_it = possibleStates.crbegin();
             rev_it != possibleStates.crend(); rev_it++) {
            if (*(rev_it) <= p) {
                // This power state is the least performant power state that is
                // still more performant than the requested one
                DPRINTF(PowerDomain, "Best match for %s is %s \n",
                        Enums::PwrStateStrings[p],
                        Enums::PwrStateStrings[*(rev_it)]);
                set(*(rev_it));
                break;
            }
        }
    }
    // Check if the transition happened
    // The only case in which the power state cannot change is if the
    // object is already at in its most performant state.
    warn_if((_currState == old_state) &&
            possibleStates.find(_currState) != possibleStates.begin(),
            "Transition to power state %s was not possible, SimObject already"
            " in the most performance state %s",
            Enums::PwrStateStrings[p], Enums::PwrStateStrings[_currState]);

    stats.numPwrMatchStateTransitions++;
    return _currState;
}

void
PowerState::computeStats()
{
    // Calculate time elapsed from last (valid) state change
    Tick elapsed_time = curTick() - prvEvalTick;

    stats.pwrStateResidencyTicks[_currState] += elapsed_time;

    // Time spent in CLK_GATED state, this might change depending on
    // transition to other low power states in respective simulation
    // objects.
    if (_currState == Enums::PwrState::CLK_GATED) {
        stats.ticksClkGated.sample(elapsed_time);
    }

    prvEvalTick = curTick();
}

std::vector<double>
PowerState::getWeights() const
{
    // Get residency stats
    std::vector<double> ret;
    Stats::VCounter residencies;
    stats.pwrStateResidencyTicks.value(residencies);

    // Account for current state too!
    Tick elapsed_time = curTick() - prvEvalTick;
    residencies[_currState] += elapsed_time;

    ret.resize(Enums::PwrState::Num_PwrState);
    for (unsigned i = 0; i < Enums::PwrState::Num_PwrState; i++)
        ret[i] = residencies[i] / \
                     (stats.pwrStateResidencyTicks.total() + elapsed_time);

    return ret;
}

PowerState::PowerStateStats::PowerStateStats(PowerState &co)
    : Stats::Group(&co),
    powerState(co),
    ADD_STAT(numTransitions,
             "Number of power state transitions"),
    ADD_STAT(numPwrMatchStateTransitions,
             "Number of power state transitions due match request"),
    ADD_STAT(ticksClkGated,
             "Distribution of time spent in the clock gated state"),
    ADD_STAT(pwrStateResidencyTicks,
             "Cumulative time (in ticks) in various power states")
{
}

void
PowerState::PowerStateStats::regStats()
{
    Stats::Group::regStats();

    using namespace Stats;

    const PowerStateParams *p = powerState.params();

    numTransitions.flags(nozero);
    numPwrMatchStateTransitions.flags(nozero);

    // Each sample is time in ticks
    unsigned num_bins = std::max(p->clk_gate_bins, 10U);
    ticksClkGated
        .init(p->clk_gate_min, p->clk_gate_max,
              (p->clk_gate_max / num_bins))
        .flags(pdf | nozero | nonan)
        ;

    pwrStateResidencyTicks
        .init(Enums::PwrState::Num_PwrState)
        .flags(nozero)
        ;
    for (int i = 0; i < Enums::PwrState::Num_PwrState; i++) {
        pwrStateResidencyTicks.subname(i, Enums::PwrStateStrings[i]);
    }

    numTransitions = 0;
}

void
PowerState::PowerStateStats::preDumpStats()
{
    Stats::Group::preDumpStats();

    /**
     * For every stats dump, the power state residency and other distribution
     * stats should be computed just before the dump to ensure correct stats
     * value being reported for current dump window. It avoids things like
     * having any unreported time spent in a power state to be forwarded to the
     * next dump window which might have rather unpleasant effects (like
     * perturbing the distribution stats).
     */
    powerState.computeStats();
}

PowerState*
PowerStateParams::create()
{
    return new PowerState(this);
}
