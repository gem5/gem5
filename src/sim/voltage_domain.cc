/*
 * Copyright (c) 2012-2014 ARM Limited
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
 */

#include <algorithm>

#include "base/statistics.hh"
#include "debug/VoltageDomain.hh"
#include "params/VoltageDomain.hh"
#include "sim/sim_object.hh"
#include "sim/voltage_domain.hh"

VoltageDomain::VoltageDomain(const Params *p)
    : SimObject(p), voltageOpPoints(p->voltage), _perfLevel(0)
{
    fatal_if(voltageOpPoints.empty(), "DVFS: Empty set of voltages for "\
             "voltage domain %s\n", name());

    // Voltages must be sorted in descending order.
    fatal_if(!std::is_sorted(voltageOpPoints.begin(), voltageOpPoints.end(),
             std::greater<Voltages::value_type>()), "DVFS: Voltage operation "\
             "points not in descending order for voltage domain %s\n",
             name());
}

void
VoltageDomain::perfLevel(PerfLevel perf_level)
{
    chatty_assert(perf_level < voltageOpPoints.size(),
                  "DVFS: Requested voltage ID %d is outside the known "\
                  "range for domain %s.\n", perf_level, name());

    if (perf_level == _perfLevel) {
        // Silently ignore identical overwrites
        return;
    }

    _perfLevel = perf_level;

    DPRINTF(VoltageDomain, "Setting voltage to %.3fV idx: %d for domain %s\n",
            voltage(), perf_level, name());
}

bool
VoltageDomain::sanitiseVoltages()
{
    if (numVoltages() == 1)
        return false;

    // Find the highest requested performance level and update the voltage
    // domain with it
    PerfLevel perf_max = (PerfLevel)-1;
    for (auto dit = srcClockChildren.begin(); dit != srcClockChildren.end(); ++dit) {
        SrcClockDomain* d = *dit;
        chatty_assert(d->voltageDomain() == this, "DVFS: Clock domain %s "\
                      "(id: %d) should not be registered with voltage domain "\
                      "%s\n", d->name(), d->domainID(), name());

        PerfLevel perf = d->perfLevel();

        DPRINTF(VoltageDomain, "DVFS: Clock domain %s (id: %d) requests perf "\
                "level %d\n", d->name(), d->domainID(), perf);

        // NOTE: Descending sort of performance levels: 0 - fast, 5 - slow
        if (perf < perf_max) {
            DPRINTF(VoltageDomain, "DVFS: Updating max perf level %d -> %d\n",
                    perf_max, perf);
            perf_max = perf;
        }
    }
    DPRINTF(VoltageDomain, "DVFS: Setting perf level of voltage domain %s "\
            "from %d to %d.\n", name(), perfLevel(), perf_max);

    // Set the performance level
    if (perf_max != perfLevel()) {
        perfLevel(perf_max);
        return true;
    } else {
        return false;
    }
}

void
VoltageDomain::startup() {
    bool changed = sanitiseVoltages();
    if (changed) {
        warn("DVFS: Perf level for voltage domain %s adapted to "\
             "requested perf levels from source clock domains.\n", name());
    }
}

void
VoltageDomain::regStats()
{
    currentVoltage
        .method(this, &VoltageDomain::voltage)
        .name(params()->name + ".voltage")
        .desc("Voltage in Volts")
        ;
}

VoltageDomain *
VoltageDomainParams::create()
{
    return new VoltageDomain(this);
}

void
VoltageDomain::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_perfLevel);
}

void
VoltageDomain::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_perfLevel);
    perfLevel(_perfLevel);
}
