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

#ifndef __SALAM_HWMODELING_FU_POOL_HH__
#define __SALAM_HWMODELING_FU_POOL_HH__

#include <cstdint>
#include <map>
#include <vector>

#include "base/types.hh"

namespace SALAM
{

// Admission of operations to functional units. Each unit, keyed by its
// enum_value, has a fixed number of instances. An instance of a
// pipelined unit (interval > 0) admits an operation once interval
// ticks have passed since its last admission, however many operations
// it has in flight. An instance of an unpipelined unit admits one
// operation at a time. Every instance admits at most once per compute
// period, even when an operation completes immediately.
class FUPool
{
  public:
    struct UnitConfig
    {
        uint32_t instances;
        // Ticks between admissions to one instance; 0: not pipelined.
        gem5::Tick interval;
    };

    // An admitted operation, returned to release() when it completes.
    struct Token
    {
        uint32_t unit = 0;
        uint32_t instance = 0;
        uint64_t id = 0;
    };

    // Replace the units. No operation may be in flight.
    void configure(const std::map<uint32_t, UnitConfig> &units,
                   gem5::Tick compute_period);

    bool hasUnit(uint32_t unit) const;
    uint32_t instances(uint32_t unit) const;

    bool canAccept(uint32_t unit, gem5::Tick now) const;
    // The first tick at or after now when an instance could admit an
    // operation without any release; MaxTick if only a release can
    // make one available.
    gem5::Tick nextAdmission(uint32_t unit, gem5::Tick now) const;

    Token acquire(uint32_t unit, gem5::Tick now);
    void release(const Token &token);

    // Operations admitted and not yet released, in all units or one.
    size_t
    inFlight() const
    {
        return live.size();
    }
    uint32_t inFlight(uint32_t unit) const;
    // Instances of a unit with at least one operation in flight.
    uint32_t busyInstances(uint32_t unit) const;

  private:
    struct Instance
    {
        uint32_t inFlight = 0;
        gem5::Tick nextIssue = 0;
    };

    struct Unit
    {
        gem5::Tick interval = 0;
        std::vector<Instance> instances;
    };

    const Unit &unitFor(uint32_t unit) const;
    bool admits(const Unit &unit, const Instance &instance,
                gem5::Tick now) const;

    std::map<uint32_t, Unit> units;
    std::map<uint64_t, Token> live;
    uint64_t lastId = 0;
    gem5::Tick computePeriod = 0;
};

} // namespace SALAM

#endif // __SALAM_HWMODELING_FU_POOL_HH__
