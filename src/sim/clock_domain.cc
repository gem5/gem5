/*
 * Copyright (c) 2013 ARM Limited
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
 *          Andreas Hansson
 *          Christopher Torng
 */

#include "debug/ClockDomain.hh"
#include "params/ClockDomain.hh"
#include "params/DerivedClockDomain.hh"
#include "params/SrcClockDomain.hh"
#include "sim/clock_domain.hh"
#include "sim/voltage_domain.hh"
#include "sim/clocked_object.hh"

double
ClockDomain::voltage() const
{
    return _voltageDomain->voltage();
}

SrcClockDomain::SrcClockDomain(const Params *p) :
    ClockDomain(p, p->voltage_domain)
{
    clockPeriod(p->clock);
}

void
SrcClockDomain::clockPeriod(Tick clock_period)
{
    if (clock_period == 0) {
        fatal("%s has a clock period of zero\n", name());
    }

    // Align all members to the current tick
    for (auto m = members.begin(); m != members.end(); ++m) {
        (*m)->updateClockPeriod();
    }

    _clockPeriod = clock_period;

    DPRINTF(ClockDomain,
            "Setting clock period to %d ticks for source clock %s\n",
            _clockPeriod, name());

    // inform any derived clocks they need to updated their period
    for (auto c = children.begin(); c != children.end(); ++c) {
        (*c)->updateClockPeriod();
    }
}

SrcClockDomain *
SrcClockDomainParams::create()
{
    return new SrcClockDomain(this);
}

DerivedClockDomain::DerivedClockDomain(const Params *p) :
    ClockDomain(p, p->clk_domain->voltageDomain()),
    parent(*p->clk_domain),
    clockDivider(p->clk_divider)
{
    // Ensure that clock divider setting works as frequency divider and never
    // work as frequency multiplier
    if (clockDivider < 1) {
       fatal("Clock divider param cannot be less than 1");
    }

    // let the parent keep track of this derived domain so that it can
    // propagate changes
    parent.addDerivedDomain(this);

    // update our clock period based on the parents clock
    updateClockPeriod();
}

void
DerivedClockDomain::updateClockPeriod()
{
    // Align all members to the current tick
    for (auto m = members.begin(); m != members.end(); ++m) {
        (*m)->updateClockPeriod();
    }

    // recalculate the clock period, relying on the fact that changes
    // propagate downwards in the tree
    _clockPeriod = parent.clockPeriod() * clockDivider;

    DPRINTF(ClockDomain,
            "Setting clock period to %d ticks for derived clock %s\n",
            _clockPeriod, name());

    // inform any derived clocks
    for (auto c = children.begin(); c != children.end(); ++c) {
        (*c)->updateClockPeriod();
    }
}

DerivedClockDomain *
DerivedClockDomainParams::create()
{
    return new DerivedClockDomain(this);
}
