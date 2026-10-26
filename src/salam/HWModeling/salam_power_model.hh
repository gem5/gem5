/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef __HWMODEL_SALAM_POWER_MODEL_HH__
#define __HWMODEL_SALAM_POWER_MODEL_HH__

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include "base/statistics.hh"
#include "params/SALAMPowerModel.hh"
#include "sim/sim_object.hh"

namespace gem5
{
class Packet;
}

namespace SALAM
{
namespace PPA
{

struct Quantity
{
    double value = 0;
    bool available = false;
    bool proxy = false;
};

struct TimingCompatibility
{
    bool available = false;
    bool latency = true;
    bool interval = true;
    bool pipeline = true;
    bool
    compatible() const
    {
        return available && latency && interval && pipeline;
    }
};

struct TimingConstraints
{
    bool available = false;
    double minimumLatency = 0;
    double minimumIssueInterval = 0;
    bool pipelined = false;

    TimingCompatibility
    check(double latency, double interval, double compute_period) const
    {
        if (!available) {
            return {};
        }
        const bool configured_pipeline = interval > 0;
        // Admission is edge-limited even when completion is immediate.
        const double effective_interval =
            std::max(compute_period, configured_pipeline ? interval : latency);
        return {true, latency >= minimumLatency,
                effective_interval >= minimumIssueInterval,
                configured_pipeline == pipelined};
    }
};

struct Result
{
    double area = 0;
    double dynamic = 0;
    double leakage = 0;
    double duration = 0;
    bool areaComplete = true;
    bool dynamicComplete = true;
    bool leakageComplete = true;
    bool approximate = false;

    double
    total() const
    {
        return dynamic + leakage;
    }
    bool
    powerAvailable() const
    {
        return duration > 0;
    }
    double
    power() const
    {
        return powerAvailable() ? total() / duration
                                : std::numeric_limits<double>::quiet_NaN();
    }
};

// The same arithmetic serves invocation summaries and resettable windows.
// Snapshots are pure: repeated dumps cannot integrate leakage twice.
class Ledger
{
  public:
    struct Physical
    {
        uint64_t count;
        Quantity area;
        Quantity leakage;
    };
    struct Scope
    {
        uint64_t start = 0;
        uint64_t end = 0;
        double dynamic = 0;
        uint64_t missingEvents = 0;
        std::set<std::string> missingEventKeys = {};
        bool wordCountsAvailable = true;
        bool approximate = false;
        bool finished = false;
        uint64_t operandReads = 0;
        uint64_t resultWrites = 0;
        uint64_t operandReadBits = 0;
        uint64_t resultWriteBits = 0;
        uint64_t readWords = 0;
        uint64_t writeWords = 0;
    };

    std::map<std::string, Physical> physical;
    Scope window;
    Scope invocation;
    bool active = false;
    bool activityCoverage = true;
    bool staticCoverage = true;

    void
    reset(uint64_t tick)
    {
        window = Scope{tick};
    }
    void
    begin(uint64_t tick)
    {
        invocation = Scope{tick};
        active = true;
    }
    void
    finish(uint64_t tick)
    {
        if (active) {
            invocation.end = tick;
            invocation.finished = true;
            active = false;
        }
    }
    void
    observe(uint64_t Scope::*counter, uint64_t count)
    {
        window.*counter += count;
        if (active) {
            invocation.*counter += count;
        }
    }
    void
    event(Quantity energy, uint64_t count, const std::string &missing_key = "")
    {
        auto add = [energy, count, &missing_key](Scope &scope) {
            if (energy.available) {
                scope.dynamic += energy.value * count;
                scope.approximate |= energy.proxy && count;
            } else {
                scope.missingEvents += count;
                if (count && !missing_key.empty()) {
                    scope.missingEventKeys.insert(missing_key);
                }
            }
        };
        add(window);
        if (active) {
            add(invocation);
        }
    }
    void
    unknownStorageAccess(const std::string &reason)
    {
        event({}, 1, reason);
        window.wordCountsAvailable = false;
        if (active) {
            invocation.wordCountsAvailable = false;
        }
    }

    Result
    result(const Scope &scope, uint64_t now, double ticksPerSecond) const
    {
        Result out;
        out.areaComplete = staticCoverage;
        out.leakageComplete = staticCoverage;
        uint64_t end = scope.finished ? scope.end : now;
        out.duration = double(end - scope.start) / ticksPerSecond;
        out.dynamic = scope.dynamic;
        out.dynamicComplete = activityCoverage && !scope.missingEvents;
        out.approximate = scope.approximate;
        for (const auto &entry : physical) {
            const auto &p = entry.second;
            if (!p.count) {
                continue;
            }
            if (p.area.available) {
                out.area += p.count * p.area.value;
                out.approximate |= p.area.proxy;
            } else {
                out.areaComplete = false;
            }
            if (p.leakage.available) {
                out.leakage += p.count * p.leakage.value * out.duration;
                out.approximate |= p.leakage.proxy;
            } else {
                out.leakageComplete = false;
            }
        }
        return out;
    }
};

} // namespace PPA
} // namespace SALAM

class SALAMPowerModel : public gem5::SimObject
{
  public:
    using SALAMPowerModelParams = gem5::SALAMPowerModelParams;
    using Quantity = SALAM::PPA::Quantity;
    using Result = SALAM::PPA::Result;
    struct Operation
    {
        std::string opcode;
        std::string signature;
        double latencySeconds;
    };
    using Operations = std::vector<Operation>;

    explicit SALAMPowerModel(const gem5::SALAMPowerModelParams &params);
    void startup() override;
    void resetStats() override;
    void preDumpStats() override;
    void configureFunctionalUnit(const std::string &alias, uint64_t count,
                                 const Operations &operations,
                                 double intervalSeconds,
                                 double computePeriodSeconds);
    void configureRegisters(const std::map<std::string, uint64_t> &slots);
    void configureStorage(const std::string &identity, uint64_t bytes,
                          unsigned rw);
    void beginInvocation(gem5::Tick tick);
    void finishInvocation(gem5::Tick tick);
    void operation(const std::string &alias, const std::string &opcode,
                   const std::string &signature);
    void registerRead(const std::string &type);
    void registerWrite(const std::string &type);
    void accountStorageAccess(const gem5::Packet &packet,
                              gem5::Addr memory_start);
    void storageRead(uint64_t words, uint64_t bits);
    void storageWrite(uint64_t words, uint64_t bits, uint64_t rmwWords = 0);
    void unknownStorageAccess(
        bool write, const std::string &reason = "memory:unknown_word_width");
    void markIncomplete(const std::string &reason);
    void markStaticIncomplete(const std::string &reason);
    void unboundOperation(const std::string &opcode,
                          const std::string &signature);
    void printInvocation(std::ostream &stream) const;
    void printWindow(std::ostream &stream) const;
    Result windowResult() const;
    Result invocationResult() const;
    bool
    isEnabled() const
    {
        return enabled;
    }
    unsigned
    wordBytes() const
    {
        return params().word_bytes;
    }
    const std::string &
    maskModel() const
    {
        return params().mask_model;
    }
    bool
    memoryTimingAvailable() const
    {
        return params().memory_timing_available;
    }
    double
    minimumAccessInterval() const
    {
        return params().minimum_access_interval_s;
    }
    double
    minimumAccessLatency() const
    {
        return params().minimum_access_latency_s;
    }
    const std::string &
    storagePhysical() const
    {
        return params().storage_physical;
    }
    const std::string &
    storageScope() const
    {
        return params().storage_scope;
    }
    PARAMS(SALAMPowerModel);

  private:
    struct Record
    {
        std::string id;
        std::string source;
        std::string signatures;
        Quantity area;
        Quantity leakage;
        Quantity event;
        Quantity read;
        Quantity write;
        SALAM::PPA::TimingConstraints timing;
    };
    const bool enabled;
    const bool strict;
    std::map<std::string, Record> records;
    SALAM::PPA::Ledger ledger;
    std::set<std::string> missing;
    std::map<std::string, uint64_t> registerSlots;
    bool registersConfigured = false;
    bool inventoryConfigured = false;
    std::set<std::string> incompatibleUnits;
    bool storageConfigured = false;
    bool memoryWordGeometryKnown = true;
    bool invocationStarted = false;
    bool timingCompatibilityAvailable = true;
    bool timingCompatible = true;
    const Record *find(const std::string &key) const;
    void physical(const std::string &key, uint64_t count,
                  const Record *record);
    const Record *registerRecord(const std::string &type,
                                 unsigned &scale) const;
    void charge(const std::string &key, Quantity energy, uint64_t count);
    void checkComplete() const;
    void printResult(std::ostream &stream, const Result &result,
                     const SALAM::PPA::Ledger::Scope &scope,
                     const std::string &label) const;

    struct ModelStats : public gem5::statistics::Group
    {
        explicit ModelStats(SALAMPowerModel *owner);
        gem5::statistics::Value knownAreaUm2;
        gem5::statistics::Value knownDynamicEnergyJ;
        gem5::statistics::Value knownLeakageEnergyJ;
        gem5::statistics::Value knownTotalEnergyJ;
        gem5::statistics::Value knownAveragePowerW;
        gem5::statistics::Value elapsedSeconds;
        gem5::statistics::Value areaComplete;
        gem5::statistics::Value dynamicComplete;
        gem5::statistics::Value leakageComplete;
        gem5::statistics::Value powerAvailable;
        gem5::statistics::Value approximate;
        gem5::statistics::Value timingCompatibilityAvailable;
        gem5::statistics::Value timingCompatible;
        gem5::statistics::Value missingEventCount;
        gem5::statistics::Value operandReads;
        gem5::statistics::Value resultWrites;
        gem5::statistics::Value operandReadBits;
        gem5::statistics::Value resultWriteBits;
        gem5::statistics::Value readWords;
        gem5::statistics::Value writeWords;
        gem5::statistics::Value memoryWordCountsAvailable;
    } modelStats;
};

#endif // __HWMODEL_SALAM_POWER_MODEL_HH__
