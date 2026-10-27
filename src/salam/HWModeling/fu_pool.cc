/*
 * Copyright (c) 2026 Akanksha Chaudhari
 * All rights reserved.
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

#include "salam/HWModeling/fu_pool.hh"

#include <algorithm>

#include "base/logging.hh"

namespace SALAM
{

void
FUPool::configure(const std::map<uint32_t, UnitConfig> &config,
                  gem5::Tick compute_period)
{
    panic_if(compute_period == 0, "Functional unit compute period is zero");
    panic_if(!live.empty(),
             "Reconfiguring functional units with %d operations in flight",
             live.size());
    computePeriod = compute_period;
    units.clear();
    for (const auto &entry : config) {
        Unit unit;
        unit.interval = entry.second.interval;
        unit.instances.resize(entry.second.instances);
        units.emplace(entry.first, unit);
    }
}

bool
FUPool::hasUnit(uint32_t unit) const
{
    return units.count(unit) != 0;
}

const FUPool::Unit &
FUPool::unitFor(uint32_t unit) const
{
    auto it = units.find(unit);
    panic_if(it == units.end(), "Unknown functional unit %d", unit);
    return it->second;
}

uint32_t
FUPool::instances(uint32_t unit) const
{
    return unitFor(unit).instances.size();
}

bool
FUPool::admits(const Unit &unit, const Instance &instance,
               gem5::Tick now) const
{
    return now >= instance.nextIssue &&
           (unit.interval > 0 || instance.inFlight == 0);
}

bool
FUPool::canAccept(uint32_t unit, gem5::Tick now) const
{
    const Unit &u = unitFor(unit);
    return std::any_of(
        u.instances.begin(), u.instances.end(),
        [&](const Instance &instance) { return admits(u, instance, now); });
}

gem5::Tick
FUPool::nextAdmission(uint32_t unit, gem5::Tick now) const
{
    const Unit &u = unitFor(unit);
    gem5::Tick next = gem5::MaxTick;
    for (const Instance &instance : u.instances) {
        if (u.interval > 0 || instance.inFlight == 0) {
            next = std::min(next, std::max(now, instance.nextIssue));
        }
    }
    return next;
}

FUPool::Token
FUPool::acquire(uint32_t unit, gem5::Tick now)
{
    auto it = units.find(unit);
    panic_if(it == units.end(), "Unknown functional unit %d", unit);
    Unit &u = it->second;
    for (uint32_t i = 0; i < u.instances.size(); i++) {
        Instance &instance = u.instances[i];
        if (admits(u, instance, now)) {
            instance.inFlight++;
            instance.nextIssue = now + std::max(computePeriod, u.interval);
            Token token{unit, i, ++lastId};
            live.emplace(token.id, token);
            return token;
        }
    }
    panic("Functional unit %d has no instance free at tick %d", unit, now);
}

uint32_t
FUPool::inFlight(uint32_t unit) const
{
    uint32_t count = 0;
    for (const Instance &instance : unitFor(unit).instances) {
        count += instance.inFlight;
    }
    return count;
}

uint32_t
FUPool::busyInstances(uint32_t unit) const
{
    uint32_t count = 0;
    for (const Instance &instance : unitFor(unit).instances) {
        count += instance.inFlight > 0;
    }
    return count;
}

void
FUPool::release(const Token &token)
{
    auto it = live.find(token.id);
    panic_if(it == live.end(),
             "Functional unit token %d released twice or never acquired",
             token.id);
    units.at(token.unit).instances.at(token.instance).inFlight--;
    live.erase(it);
}

} // namespace SALAM
