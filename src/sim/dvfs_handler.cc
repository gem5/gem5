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
 */

#include "sim/dvfs_handler.hh"

#include <set>
#include <utility>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/DVFS.hh"
#include "params/DVFSHandler.hh"
#include "sim/clock_domain.hh"
#include "sim/eventq.hh"
#include "sim/stat_control.hh"
#include "sim/voltage_domain.hh"

//
//
// DVFSHandler methods implementation
//

DVFSHandler::DVFSHandler(const Params *p)
    : SimObject(p),
      sysClkDomain(p->sys_clk_domain),
      enableHandler(p->enable),
      _transLatency(p->transition_latency)
{
    // Check supplied list of domains for sanity and add them to the
    // domain ID -> domain* hash
    for (auto dit = p->domains.begin(); dit != p->domains.end(); ++dit) {
        SrcClockDomain *d = *dit;
        DomainID domain_id = d->domainID();

        fatal_if(sysClkDomain == d, "DVFS: Domain config list has a "\
                 "system clk domain entry");
        fatal_if(domain_id == SrcClockDomain::emptyDomainID,
                 "DVFS: Controlled domain %s needs to have a properly "\
                 " assigned ID.\n", d->name());

        auto entry = std::make_pair(domain_id, d);
        bool new_elem = domains.insert(entry).second;
        fatal_if(!new_elem, "DVFS: Domain %s with ID %d does not have a "\
                 "unique ID.\n", d->name(), domain_id);

        // Create a dedicated event slot per known domain ID
        UpdateEvent *event = &updatePerfLevelEvents[domain_id];
        event->domainIDToSet = d->domainID();

        // Add domain ID to the list of domains
        domainIDList.push_back(d->domainID());
    }
    UpdateEvent::dvfsHandler = this;
}

DVFSHandler *DVFSHandler::UpdateEvent::dvfsHandler;

DVFSHandler::DomainID
DVFSHandler::domainID(uint32_t index) const
{
    fatal_if(index >= numDomains(), "DVFS: Requested index out of "\
             "bound, max value %d\n", (domainIDList.size() - 1));

    assert(domains.find(domainIDList[index]) != domains.end());

    return domainIDList[index];
}

bool
DVFSHandler::validDomainID(DomainID domain_id) const
{
    assert(isEnabled());
    // This is ensure that the domain id as requested by the software is
    // availabe in the handler.
    if (domains.find(domain_id) != domains.end())
        return true;
    warn("DVFS: invalid domain ID %d, the DVFS handler does not handle this "\
         "domain\n", domain_id);
    return false;
}

bool
DVFSHandler::perfLevel(DomainID domain_id, PerfLevel perf_level)
{
    assert(isEnabled());

    DPRINTF(DVFS, "DVFS: setPerfLevel domain %d -> %d\n", domain_id, perf_level);

    auto d = findDomain(domain_id);
    if (!d->validPerfLevel(perf_level)) {
        warn("DVFS: invalid performance level %d for domain ID %d, request "\
             "ignored\n", perf_level, domain_id);
        return false;
    }

    UpdateEvent *update_event = &updatePerfLevelEvents[domain_id];
    // Drop an old DVFS change request once we have established that this is a
    // reasonable request
    if (update_event->scheduled()) {
        DPRINTF(DVFS, "DVFS: Overwriting the previous DVFS event.\n");
        deschedule(update_event);
    }

    update_event->perfLevelToSet = perf_level;

    // State changes that restore to the current state (and / or overwrite a not
    // yet completed in-flight request) will be squashed
    if (d->perfLevel() == perf_level) {
        DPRINTF(DVFS, "DVFS: Ignoring ineffective performance level change "\
                "%d -> %d\n", d->perfLevel(), perf_level);
        return false;
    }

    // At this point, a new transition will certainly take place -> schedule
    Tick when = curTick() + _transLatency;
    DPRINTF(DVFS, "DVFS: Update for perf event scheduled for %ld\n", when);

    schedule(update_event, when);
    return true;
}

void
DVFSHandler::UpdateEvent::updatePerfLevel()
{
    // Perform explicit stats dump for power estimation before performance
    // level migration
    Stats::dump();
    Stats::reset();

    // Update the performance level in the clock domain
    auto d = dvfsHandler->findDomain(domainIDToSet);
    assert(d->perfLevel() != perfLevelToSet);

    d->perfLevel(perfLevelToSet);
}

double
DVFSHandler::voltageAtPerfLevel(DomainID domain_id, PerfLevel perf_level) const
{
    VoltageDomain *d = findDomain(domain_id)->voltageDomain();
    assert(d);
    PerfLevel n = d->numVoltages();
    if (perf_level < n)
        return d->voltage(perf_level);

    // Request outside of the range of the voltage domain
    if (n == 1) {
        DPRINTF(DVFS, "DVFS: Request for perf-level %i for single-point "\
                "voltage domain %s.  Returning voltage at level 0: %.2f "\
                "V\n", perf_level, d->name(), d->voltage(0));
        // Special case for single point voltage domain -> same voltage for
        // all points
        return d->voltage(0);
    }

    warn("DVFSHandler %s reads illegal voltage level %u from "\
         "VoltageDomain %s. Returning 0 V\n", name(), perf_level, d->name());
    return 0.;
}

void
DVFSHandler::serialize(CheckpointOut &cp) const
{
    //This is to ensure that the handler status is maintained during the
    //entire simulation run and not changed from command line during checkpoint
    //and restore
    SERIALIZE_SCALAR(enableHandler);

    // Pull out the hashed data structure into easy-to-serialise arrays;
    // ensuring that the data associated with any pending update event is saved
    std::vector<DomainID> domain_ids;
    std::vector<PerfLevel> perf_levels;
    std::vector<Tick> whens;
    for (const auto &ev_pair : updatePerfLevelEvents) {
        DomainID id = ev_pair.first;
        const UpdateEvent *event = &ev_pair.second;

        assert(id == event->domainIDToSet);
        domain_ids.push_back(id);
        perf_levels.push_back(event->perfLevelToSet);
        whens.push_back(event->scheduled() ? event->when() : 0);
    }
    SERIALIZE_CONTAINER(domain_ids);
    SERIALIZE_CONTAINER(perf_levels);
    SERIALIZE_CONTAINER(whens);
}

void
DVFSHandler::unserialize(CheckpointIn &cp)
{
    bool temp = enableHandler;

    UNSERIALIZE_SCALAR(enableHandler);

    if (temp != enableHandler) {
        warn("DVFS: Forcing enable handler status to unserialized value of %d",
             enableHandler);
    }

    // Reconstruct the map of domain IDs and their scheduled events
    std::vector<DomainID> domain_ids;
    std::vector<PerfLevel> perf_levels;
    std::vector<Tick> whens;
    UNSERIALIZE_CONTAINER(domain_ids);
    UNSERIALIZE_CONTAINER(perf_levels);
    UNSERIALIZE_CONTAINER(whens);

    for (size_t i = 0; i < domain_ids.size(); ++i) {;
        UpdateEvent *event = &updatePerfLevelEvents[domain_ids[i]];

        event->domainIDToSet = domain_ids[i];
        event->perfLevelToSet = perf_levels[i];

        // Schedule all previously scheduled events
        if (whens[i])
            schedule(event, whens[i]);
    }
    UpdateEvent::dvfsHandler = this;
}

DVFSHandler*
DVFSHandlerParams::create()
{
    return new DVFSHandler(this);
}
