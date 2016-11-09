/*
 * Copyright (c) 2013-2014 ARM Limited
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
 *          Stephan Diestelhorst
 */

/**
 * @file
 * DVFSHandler and DomainConfig class declaration used for managing voltage
 * and frequency scaling of the various DVFS domains in the system (with each
 * domain having their independent domain configuration information)
 */


#ifndef __SIM_DVFS_HANDLER_HH__
#define __SIM_DVFS_HANDLER_HH__

#include <vector>

#include "debug/DVFS.hh"
#include "params/DVFSHandler.hh"
#include "sim/clock_domain.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

/**
 * DVFS Handler class, maintains a list of all the domains it can handle.
 * Each entry of that list is an object of the DomainConfig class, and the
 * handler uses the methods provided by that class to get access to the
 * configuration of each domain. The handler is responsible for setting/getting
 * clock periods and voltages from clock/voltage domains.
 * The handler acts the bridge between software configurable information
 * for each domain as provided to the controller and the hardware
 * implementation details for those domains.
 */
class DVFSHandler : public SimObject
{
  public:
    typedef DVFSHandlerParams Params;
    DVFSHandler(const Params *p);

    typedef SrcClockDomain::DomainID DomainID;
    typedef SrcClockDomain::PerfLevel PerfLevel;

    /**
     * Get the number of domains assigned to this DVFS handler.
     * @return Number of domains
     */
    uint32_t numDomains() const { return domainIDList.size(); }

    /**
     * Get the n-th domain ID, from the domains managed by this handler.
     * @return Domain ID
     */
    DomainID domainID(uint32_t index) const;

    /**
     * Check whether a domain ID is known to the handler or not.
     * @param domain_id Domain ID to check
     * @return Domain ID known to handler?
     */
    bool validDomainID(DomainID domain_id) const;

    /**
     * Get transition latency to switch between performance levels.
     * @return Transition latency
     */
    Tick transLatency() const { return _transLatency; }

    /**
     * Set a new performance level for the specified domain.  The actual update
     * will be delayed by transLatency().
     *
     * @param domain_id Software visible ID of the domain to be configured
     * @param perf_level Requested performance level (0 - fast, >0 slower)
     * @return status whether the setting was successful
     */
    bool perfLevel(DomainID domain_id, PerfLevel perf_level);

    /**
     * Get the current performance level of a domain.  While a change request is
     * in-flight, will return the current (i.e. old, unmodified) value.
     *
     * @param domain_id Domain ID to query
     * @return Current performance level of the specified domain
     */
    PerfLevel perfLevel(DomainID domain_id) const {
         assert(isEnabled());
         return findDomain(domain_id)->perfLevel();
    }

    /**
     * Read the clock period of the specified domain at the specified
     * performance level.
     * @param domain_id Domain ID to query
     * @param perf_level Performance level of interest
     * @return Clock period in ticks for the requested performance level of
     * the respective domain
     */
    Tick clkPeriodAtPerfLevel(DomainID domain_id, PerfLevel perf_level) const
    {
        SrcClockDomain *d = findDomain(domain_id);
        assert(d);
        PerfLevel n = d->numPerfLevels();
        if (perf_level < n)
            return d->clkPeriodAtPerfLevel(perf_level);

        warn("DVFSHandler %s reads illegal frequency level %u from "\
             "SrcClockDomain %s. Returning 0\n", name(), perf_level, d->name());
        return Tick(0);
    }

    /**
     * Read the voltage of the specified domain at the specified
     * performance level.
     * @param domain_id Domain ID to query
     * @param perf_level Performance level of interest
     * @return Voltage for the requested performance level of the respective
     * domain
     */
    double voltageAtPerfLevel(DomainID domain_id, PerfLevel perf_level) const;

    /**
     * Get the total number of available performance levels.
     *
     * @param domain_id Domain ID to query
     * @return Number of performance levels that where configured for the
     * respective domain
     */
    PerfLevel numPerfLevels(PerfLevel domain_id) const
    {
        return findDomain(domain_id)->numPerfLevels();
    }

    /**
     * Check enable status of the DVFS handler, when the handler is disabled, no
     * request should be sent to the handler.
     * @return True, if the handler is enabled
     */
    bool isEnabled() const { return enableHandler; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    typedef std::map<DomainID, SrcClockDomain*> Domains;
    Domains domains;

    /**
      * List of IDs avaiable in the domain list
      */
    std::vector<DomainID> domainIDList;

    /**
      * Clock domain of the system the handler is instantiated.
      */
    SrcClockDomain* sysClkDomain;

    /**
     * Search for a domain based on the domain ID.
     *
     * @param domain_id Domain ID to search for
     * @return Pointer to the source clock domain with matching ID.
     */
    SrcClockDomain *findDomain(DomainID domain_id) const {
        auto it = domains.find(domain_id);
        panic_if(it == domains.end(),
                 "DVFS: Could not find a domain for ID %d.\n",domain_id );
        return domains.find(domain_id)->second;
    }

    /**
     * Disabling the DVFS handler ensures that all the DVFS migration requests
     * are ignored. Domains remain at their default frequency and voltage.
     */
    bool enableHandler;


    /**
     * This corresponds to the maximum transition latency associated with the
     * hardware transitioning from a particular performance level to the other
     */
    const Tick _transLatency;



    /**
     * Update performance level event, encapsulates all the required information
     * for a future call to change a domain's performance level.
     */
    struct UpdateEvent : public Event {
        UpdateEvent() : Event(DVFS_Update_Pri), domainIDToSet(0),
                        perfLevelToSet(0) {}

        /**
         * Static pointer to the single DVFS hander for all the update events
         */
        static DVFSHandler *dvfsHandler;

        /**
         * ID of the domain that will be changed by the in-flight event
         */
        DomainID domainIDToSet;

        /**
         * Target performance level of the in-flight event
         */
        PerfLevel perfLevelToSet;

        /**
         * Updates the performance level by modifying the clock and the voltage
         * of the associated clocked objects.  Gets information from
         * domainIDToSet and perfLevelToSet for easier calling through an
         * event.
         */
        void updatePerfLevel();

        void process() { updatePerfLevel(); }
    };

    typedef std::map<DomainID, UpdateEvent> UpdatePerfLevelEvents;
    /**
     * Map from domain IDs -> perf level update events, records in-flight change
     * requests per domain ID.
     */
    UpdatePerfLevelEvents updatePerfLevelEvents;
};

#endif // __SIM_DVFS_HANDLER_HH__
