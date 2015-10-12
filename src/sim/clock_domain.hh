/*
 * Copyright (c) 2013-2014 ARM Limited
 * Copyright (c) 2013 Cornell University
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
 * Authors: Vasileios Spiliopoulos
 *          Akash Bagdia
 *          Christopher Torng
 *          Stephan Diestelhorst
 */

/**
 * @file
 * ClockDomain declarations.
 */

#ifndef __SIM_CLOCK_DOMAIN_HH__
#define __SIM_CLOCK_DOMAIN_HH__

#include <algorithm>

#include "base/statistics.hh"
#include "params/ClockDomain.hh"
#include "params/DerivedClockDomain.hh"
#include "params/SrcClockDomain.hh"
#include "sim/sim_object.hh"

/**
 * Forward declaration
 */
class DerivedClockDomain;
class VoltageDomain;
class Clocked;

/**
 * The ClockDomain provides clock to group of clocked objects bundled
 * under the same clock domain. The clock domains, in turn, are
 * grouped into voltage domains. The clock domains provide support for
 * a hierarchial structure with source and derived domains.
 */
class ClockDomain : public SimObject
{

  private:

    /**
     * Stat to report clock period of clock domain
     */
    Stats::Value currentClock;

  protected:

    /**
     * Pre-computed clock period in ticks. This is populated by the
     * inheriting classes based on how their period is determined.
     */
    Tick _clockPeriod;

    /**
     * Voltage domain this clock domain belongs to
     */
    VoltageDomain *_voltageDomain;

    /**
     * Pointers to potential derived clock domains so we can propagate
     * changes.
     */
    std::vector<DerivedClockDomain*> children;

    /**
     * Pointers to members of this clock domain, so that when the clock
     * period changes, we can update each member's tick.
     */
    std::vector<Clocked *> members;

  public:

    typedef ClockDomainParams Params;
    ClockDomain(const Params *p, VoltageDomain *voltage_domain) :
        SimObject(p),
        _clockPeriod(0),
        _voltageDomain(voltage_domain) {}

    void regStats();

    /**
     * Get the clock period.
     *
     * @return Clock period in ticks
     */
    Tick clockPeriod() const { return _clockPeriod; }

    /**
     * Register a Clocked object with this ClockDomain.
     *
     * @param Clocked to add as a member
     */
    void registerWithClockDomain(Clocked *c)
    {
        assert(c != NULL);
        assert(std::find(members.begin(), members.end(), c) == members.end());
        members.push_back(c);
    }

    /**
     * Get the voltage domain.
     *
     * @return Voltage domain this clock domain belongs to
     */
    inline VoltageDomain *voltageDomain() const { return _voltageDomain; }


    /**
     * Get the current voltage this clock domain operates at.
     *
     * @return Voltage applied to the clock domain
     */
    double voltage() const;

    /**
     * Add a derived domain.
     *
     * @param Derived domain to add as a child
     */
    void addDerivedDomain(DerivedClockDomain *clock_domain)
    { children.push_back(clock_domain); }

};

/**
 * The source clock domains provides the notion of a clock domain that is
 * connected to a tunable clock source. It maintains the clock period and
 * provides methods for setting/getting the clock and  configuration parameters
 * for clock domain that handler is going to manage. This includes frequency
 * values at various performance levels, domain id, and current performance
 * level. Note that a performance level as requested by the software corresponds
 * to one of the frequency operational points the domain can operate at.
 */
class SrcClockDomain : public ClockDomain
{

  public:

    typedef SrcClockDomainParams Params;
    SrcClockDomain(const Params *p);

    /**
     * Set new clock value
     * @param clock The new clock period in ticks
     */
    void clockPeriod(Tick clock_period);

    // Explicitly import the otherwise hidden clockPeriod
    using ClockDomain::clockPeriod;

    typedef int32_t DomainID;
    static const DomainID emptyDomainID = -1;

    /**
     * @return the domainID of the domain
     */
    uint32_t domainID() const { return _domainID; }

    typedef uint32_t PerfLevel;
    /**
     * Checks whether the performance level requested exists in the current
     * domain configuration
     *
     * @param the target performance level of the domain
     *
     * @return validity status of the given performance level
     */
    bool validPerfLevel(PerfLevel perf_level) const {
        return perf_level < numPerfLevels();
    }

    /**
     * Sets the current performance level of the domain
     *
     * @param perf_level the target performance level
     */
    void perfLevel(PerfLevel perf_level);

    /**
     * @return the current performance level of the domain
     */
    PerfLevel perfLevel() const { return _perfLevel; }

    /**
     * Get the number of available performance levels for this clock domain.
     *
     * @return Number of perf levels configured for this domain.
     */
    PerfLevel numPerfLevels() const {return freqOpPoints.size();}

    /**
     * @returns the clock period (expressed in ticks) for the current
     * performance level
     */
    Tick clkPeriodAtPerfLevel() const { return freqOpPoints[perfLevel()]; }

    Tick clkPeriodAtPerfLevel(PerfLevel perf_level) const
    {
        assert(validPerfLevel(perf_level));
        return freqOpPoints[perf_level];
    }

    void startup() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    /**
      * List of possible frequency operational points, should be in
      * descending order
      * An empty list corresponds to default frequency specified for its
      * clock domain, overall implying NO DVFS
      */
    const std::vector<Tick> freqOpPoints;

    /**
      * Software recognizable id number for the domain, should be unique for
      * each domain
      */
    const uint32_t _domainID;

    /**
      * Current performance level the domain is set to.
      * The performance level corresponds to one selected frequency (and related
      * voltage) from the supplied list of frequencies, with perfLevel = 0 being
      * the fastest performance state.
      */
    PerfLevel _perfLevel;
};

/**
 * The derived clock domains provides the notion of a clock domain
 * that is connected to a parent clock domain that can either be a
 * source clock domain or a derived clock domain. It maintains the
 * clock divider and provides methods for getting the clock.
 */
class DerivedClockDomain: public ClockDomain
{

  public:

    typedef DerivedClockDomainParams Params;
    DerivedClockDomain(const Params *p);

    /**
     * Called by the parent clock domain to propagate changes. This
     * also involves propagating the change further to any children of
     * the derived domain itself.
     */
    void updateClockPeriod();

  private:

    /**
     * Reference to the parent clock domain this clock domain derives
     * its clock period from
     */
    ClockDomain &parent;

    /**
     * Local clock divider of the domain
     */
    const uint64_t clockDivider;
};

#endif
