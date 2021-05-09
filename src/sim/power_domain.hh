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
 *
 */

#ifndef __SIM_POWER_DOMAIN_HH__
#define __SIM_POWER_DOMAIN_HH__

#include <string>
#include <vector>

#include "base/statistics.hh"
#include "params/PowerDomain.hh"
#include "sim/clocked_object.hh"
#include "sim/power_state.hh"

namespace gem5
{

/**
 * The PowerDomain groups PowerState objects together to regulate their
 * power states. As the PowerDomain itself is a PowerState object, you can
 * create hierarchies of PowerDomains. All objects in a power domain will be in
 * the power state of the domain OR a more performant one.
 */
class PowerDomain : public PowerState
{
  public:
    PowerDomain(const PowerDomainParams &p);
    typedef PowerDomainParams Params;
    ~PowerDomain() override {};

    /**
     * During startup, the list of possible power states the
     * PowerDomain can be in is populated, the power state of the
     * PowerDomain is set and some assertions about the PowerState objects
     * in the Domain are checked.
     */
    void startup() override;

    /**
     * Register the change in power state in one of the leader. The power
     * domain will change its own power state if required and if there is a
     * power state, it will schedule an event to update its followers
     */
    void pwrStateChangeCallback(enums::PwrState new_pwr_state,
                                PowerState* leader);

    /**
     * Function called by a follower to register itself as
     * a dependant of this power domain
     */
    void addFollower(PowerState* pwr_obj) override;

  private:
    /**
     * Calculate the power state of the power domain, based upon the power
     * states of the leaders. This will be called if one the leaders
     * changes its power states.
     * If no inputs are given, only the leaders will be polled on their
     * power state. You can also pass a vector containing the power states
     * which the followers returned when asked to match a certain power
     * state (called from setFollowerPowerStates)
     */
    enums::PwrState calculatePowerDomainState(
          const std::vector<enums::PwrState> &f_states={});

    /**
     * Check if a given p_state is available across all leaders and
     * followers in this domain.
     */
    bool isPossiblePwrState(enums::PwrState p_state);

    /**
     * Calculate the possible power states of the domain based upon the
     * intersection of the power states of the individual objects within
     * the domain. Done at startup.
     */
    void calculatePossiblePwrStates();

    /**
     * Update the followers of the newly updated power state. They are
     * required to match the power state of the power domain i.e. go to the
     * same power state or a more performant one
     */
    void setFollowerPowerStates();

  private: /* Power domain attributes */
     /**
     * List of all leaders in the PowerDomain. A leader can
     * independently change its power state and does not depend on the
     * PowerDomain to change its power state. A leader needs to be a
     * PowerState object and can also be another PowerDomain. Each
     * PowerDomain needs to have at least one leader.
     */
    std::vector<PowerState*> leaders;

    /**
     * Power state requested by the leader. This is not necessarily the
     * power state of the domain as whole (as that one depends on the
     * matched power states of the followers
     */
    enums::PwrState leaderTargetState;

    /**
     * List of all followers in the PowerDomain. The power state of the
     * domain will determine the power state of the followers. A follower
     * cannot change its power state independently.
     */
    std::vector<PowerState*> followers;

    /**
     * Latency with which power state changes of the leaders will ripple
     * through to the followers.
     */
    const Tick updateLatency = 1;

    /**
     * Event to update the power states of the followers
     */
    EventWrapper<PowerDomain, &PowerDomain::setFollowerPowerStates>
                pwrStateUpdateEvent;

  protected:
    struct PowerDomainStats : public statistics::Group
    {
        PowerDomainStats(PowerDomain &pd);

        void regStats() override;

        statistics::Scalar numLeaderCalls;
        statistics::Scalar numLeaderCallsChangingState;
    } stats;
};

} // namespace gem5

#endif // __SIM_POWER_DOMAIN_HH__
