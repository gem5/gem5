/*
 * Copyright (c) 2015-2016 ARM Limited
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
 *
 * Authors: Akash Bagdia
 *          David Guillen Fandos
 */

#include "sim/clocked_object.hh"

#include "base/misc.hh"
#include "sim/power/power_model.hh"

ClockedObject::ClockedObject(const ClockedObjectParams *p) :
    SimObject(p), Clocked(*p->clk_domain),
    _currPwrState(p->default_p_state),
    prvEvalTick(0)
{
    // Register the power_model with the object
    if (p->power_model)
        p->power_model->setClockedObject(this);
}

void
ClockedObject::serialize(CheckpointOut &cp) const
{
    unsigned int currPwrState = (unsigned int)_currPwrState;

    SERIALIZE_SCALAR(currPwrState);
    SERIALIZE_SCALAR(prvEvalTick);
}

void
ClockedObject::unserialize(CheckpointIn &cp)
{
    unsigned int currPwrState;

    UNSERIALIZE_SCALAR(currPwrState);
    UNSERIALIZE_SCALAR(prvEvalTick);

    _currPwrState = Enums::PwrState(currPwrState);
}

void
ClockedObject::pwrState(Enums::PwrState p)
{
    // Function should ideally be called only when there is a state change
    if (_currPwrState == p) {
        warn_once("ClockedObject: Already in the requested power state, " \
                  "request ignored");
        return;
    }

    // No need to compute stats if in the same tick, update state though. This
    // can happen in cases like a) during start of the simulation multiple
    // state changes happens in init/startup phase, b) one takes a decision to
    // migrate state but decides to reverts back to the original state in the
    // same tick if other conditions are not met elsewhere.
    // Any state change related stats would have been recorded on previous call
    // to the pwrState() function.
    if (prvEvalTick == curTick()) {
        warn("ClockedObject: More than one power state change request "\
             "encountered within the same simulation tick");
        _currPwrState = p;
        return;
    }

    // Record stats for previous state.
    computeStats();

    _currPwrState = p;

    numPwrStateTransitions++;
}

void
ClockedObject::computeStats()
{
    // Calculate time elapsed from last (valid) state change
    Tick elapsed_time = curTick() - prvEvalTick;

    pwrStateResidencyTicks[_currPwrState] += elapsed_time;

    // Time spent in CLK_GATED state, this might change depending on
    // transition to other low power states in respective simulation
    // objects.
    if (_currPwrState == Enums::PwrState::CLK_GATED) {
        pwrStateClkGateDist.sample(elapsed_time);
    }

    prvEvalTick = curTick();
}

std::vector<double>
ClockedObject::pwrStateWeights() const
{
    // Get residency stats
    std::vector<double> ret;
    Stats::VCounter residencies;
    pwrStateResidencyTicks.value(residencies);

    // Account for current state too!
    Tick elapsed_time = curTick() - prvEvalTick;
    residencies[_currPwrState] += elapsed_time;

    ret.resize(Enums::PwrState::Num_PwrState);
    for (unsigned i = 0; i < Enums::PwrState::Num_PwrState; i++)
        ret[i] = residencies[i] / \
                     (pwrStateResidencyTicks.total() + elapsed_time);

    return ret;
}

void
ClockedObject::regStats()
{
    SimObject::regStats();

    using namespace Stats;

    numPwrStateTransitions
        .name(params()->name + ".numPwrStateTransitions")
        .desc("Number of power state transitions")
        .flags(nozero)
        ;

    // Each sample is time in ticks
    unsigned num_bins = std::max(params()->p_state_clk_gate_bins, 10U);
    pwrStateClkGateDist
        .init(params()->p_state_clk_gate_min, params()->p_state_clk_gate_max,
          (params()->p_state_clk_gate_max / num_bins))
        .name(params()->name + ".pwrStateClkGateDist")
        .desc("Distribution of time spent in the clock gated state")
        .flags(pdf | nozero | nonan)
        ;

    pwrStateResidencyTicks
        .init(Enums::PwrState::Num_PwrState)
        .name(params()->name + ".pwrStateResidencyTicks")
        .desc("Cumulative time (in ticks) in various power states")
        .flags(nozero)
        ;
    for (int i = 0; i < Enums::PwrState::Num_PwrState; i++) {
        pwrStateResidencyTicks.subname(i, Enums::PwrStateStrings[i]);
    }

    numPwrStateTransitions = 0;

    /**
     * For every stats dump, the power state residency and other distribution
     * stats should be computed just before the dump to ensure correct stats
     * value being reported for current dump window. It avoids things like
     * having any unreported time spent in a power state to be forwarded to the
     * next dump window which might have rather unpleasant effects (like
     * perturbing the distribution stats).
     */
    registerDumpCallback(new ClockedObjectDumpCallback(this));
}
