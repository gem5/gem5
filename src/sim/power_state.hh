/*
 * Copyright (c) 2015-2017, 2020 ARM Limited
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

/**
 * @file
 * PowerState declaration and implementation.
 */

#ifndef __SIM_POWER_STATE_HH__
#define __SIM_POWER_STATE_HH__

#include <set>

#include "base/callback.hh"
#include "base/statistics.hh"
#include "enums/PwrState.hh"
#include "params/PowerState.hh"
#include "sim/core.hh"
#include "sim/sim_object.hh"

class PowerDomain;

/**
 * Helper class for objects that have power states. This class provides the
 * basic functionality to change between power states.
 */
class PowerState : public SimObject
{
  public:
    PowerState(const PowerStateParams *p);

    /** Parameters of PowerState object */
    typedef PowerStateParams Params;
    const Params* params() const
    {
        return reinterpret_cast<const Params*>(_params);
    }

    virtual void addFollower(PowerState* pwr_obj) {};
    void setControlledDomain(PowerDomain* pwr_dom);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Change the power state of this object to the power state p
     */
    void set(Enums::PwrState p);


    inline Enums::PwrState get() const
    {
        return _currState;
    }

    inline std::string getName() const
    {
        return Enums::PwrStateStrings[_currState];
    }

    /** Returns the percentage residency for each power state */
    std::vector<double> getWeights() const;

    /**
     * Record stats values like state residency by computing the time
     * difference from previous update. Also, updates the previous evaluation
     * tick once all stats are recorded.
     * Usually called on power state change and stats dump callback.
     */
    void computeStats();

    /**
     * Change the power state of this object to a power state equal to OR more
     * performant than p. Returns the power state the object actually went to.
     */
    Enums::PwrState matchPwrState(Enums::PwrState p);

    /**
     * Return the power states this object can be in
     */
    std::set<Enums::PwrState> getPossibleStates() const
    {
        return possibleStates;
    }

  protected:

    /** To keep track of the current power state */
    Enums::PwrState _currState;

    /** The possible power states this object can be in */
    std::set<Enums::PwrState> possibleStates;

    /** Last tick the power stats were calculated */
    Tick prvEvalTick = 0;

    /**
     * The power domain that this power state leads, nullptr if it
     * doesn't lead any.
     */
    PowerDomain* controlledDomain = nullptr;

    struct PowerStateStats : public Stats::Group
    {
        PowerStateStats(PowerState &ps);

        void regStats() override;
        void preDumpStats() override;

        PowerState &powerState;

        Stats::Scalar numTransitions;
        Stats::Scalar numPwrMatchStateTransitions;
        Stats::Distribution ticksClkGated;
        /** Tracks the time spent in each of the power states */
        Stats::Vector pwrStateResidencyTicks;
    } stats;
};

class PowerStateDumpCallback : public Callback
{
    PowerState *co;
  public:
    PowerStateDumpCallback(PowerState *co_t) : co(co_t) {}
    virtual void process() { co->computeStats(); };
};

#endif //__SIM_POWER_STATE_HH__
