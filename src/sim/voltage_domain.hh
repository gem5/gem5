/*
 * Copyright (c) 2012, 2019 ARM Limited
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

#ifndef __SIM_VOLTAGE_DOMAIN_HH__
#define __SIM_VOLTAGE_DOMAIN_HH__

#include <vector>

#include "base/statistics.hh"
#include "params/VoltageDomain.hh"
#include "sim/clock_domain.hh"
#include "sim/sim_object.hh"

/**
 * A VoltageDomain is used to group clock domains that operate under
 * the same voltage. The class provides methods for setting and
 * getting the voltage.
 */
class VoltageDomain : public SimObject
{
  public:

    typedef VoltageDomainParams Params;
    VoltageDomain(const Params *p);

    typedef SrcClockDomain::PerfLevel PerfLevel;

    /**
     * Get the current voltage.
     *
     * @return Voltage of the domain
     */
    double voltage() const { return voltageOpPoints[_perfLevel]; }

    /**
     * Get the voltage at specified performance level.
     *
     * @param perf_level Performance level for which the voltage is requested
     * @return Voltage of the domain at specified performance level
     */
    double voltage(PerfLevel perf_level) const
    {
        chatty_assert(perf_level < numVoltages(), "VoltageDomain %s "\
                      "request for voltage perf level %u is outside "\
                      "of numVoltages %u", name(), perf_level,
                      numVoltages());
        return voltageOpPoints[perf_level];
    }

    uint32_t numVoltages() const { return (uint32_t)voltageOpPoints.size(); }

    /**
     * Set the voltage point of the domain.
     * @param Voltage value to be set
     */
    void perfLevel(PerfLevel perf_level);

    /**
     * Get the voltage point of the domain.
     * @param Voltage value to be set
     */
    PerfLevel perfLevel() const { return _perfLevel; }

    /**
     * Register a SrcClockDomain with this voltage domain.
     * @param src_clock_domain The SrcClockDomain to register.
     */
    void registerSrcClockDom(SrcClockDomain *src_clock_dom) {
        assert(src_clock_dom->voltageDomain() == this);
        srcClockChildren.push_back(src_clock_dom);
    }

    /**
     * Startup has all SrcClockDomains registered with this voltage domain, so
     * try to make sure that all perf level requests from them are met.
     */
    void startup() override;

    /**
     * Recomputes the highest (fastest, i.e., numerically lowest) requested
     * performance level of all associated clock domains, and updates the
     * performance level of this voltage domain to match.  This means that for
     * two connected clock domains, one fast and one slow, the voltage domain
     * will provide the voltage associated with the fast DVFS operation point.
     * Must be called whenever a clock domain decides to swtich its performance
     * level.
     *
     * @return True, if the voltage was actually changed.
     */
    bool sanitiseVoltages();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    typedef std::vector<double> Voltages;
    /**
     * List of possible minimum voltage at each of the frequency operational
     * points, should be in descending order and same size as freqOpPoints.
     * An empty list corresponds to default voltage specified for the voltage
     * domain its clock domain belongs. The same voltage is applied for each
     * freqOpPoints, overall implying NO DVS
     */
    const Voltages voltageOpPoints;
    PerfLevel _perfLevel;

    struct VoltageDomainStats : public Stats::Group
    {
        VoltageDomainStats(VoltageDomain &vd);

        /**
         * Stat for reporting voltage of the domain
         */
        Stats::Value voltage;
    } stats;

    /**
     * List of associated SrcClockDomains that are connected to this voltage
     * domain.
     */
    typedef std::vector<SrcClockDomain *> SrcClockChildren;
    SrcClockChildren srcClockChildren;
};

#endif
