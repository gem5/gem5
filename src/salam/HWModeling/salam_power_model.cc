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

#include "salam/HWModeling/salam_power_model.hh"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "base/logging.hh"
#include "salam/storage_access.hh"
#include "sim/core.hh"
#include "sim/cur_tick.hh"

using namespace gem5;

namespace
{

SALAM::PPA::Quantity
quantity(double value, const std::string &status)
{
    fatal_if(!std::isfinite(value) || value < 0,
             "SALAM characterization coefficients must be finite and "
             "nonnegative");
    fatal_if(status != "unavailable" && status != "characterized" &&
                 status != "derived" && status != "proxy",
             "Unknown SALAM characterization status '%s'", status);
    return {value, status != "unavailable", status == "proxy"};
}

bool
allows(const std::string &list, const std::string &signature)
{
    std::istringstream input(list);
    std::string entry;
    while (std::getline(input, entry, ';')) {
        if (entry == signature) {
            return true;
        }
    }
    return false;
}

unsigned
typeBits(const std::string &type)
{
    fatal_if(type.size() < 2 ||
                 (type[0] != 'i' && type[0] != 'f' && type[0] != 'p'),
             "Invalid SALAM storage type '%s'", type);
    unsigned bits = 0;
    for (size_t i = 1; i < type.size(); ++i) {
        fatal_if(type[i] < '0' || type[i] > '9',
                 "Invalid SALAM storage type '%s'", type);
        bits = bits * 10 + type[i] - '0';
    }
    fatal_if(!bits || bits > 64, "Unsupported SALAM storage width %u", bits);
    return bits;
}

} // anonymous namespace

SALAMPowerModel::SALAMPowerModel(const SALAMPowerModelParams &p)
    : SimObject(p), enabled(p.enabled), strict(p.strict), modelStats(this)
{
    const auto n = p.keys.size();
    fatal_if(
        p.ids.size() != n || p.sources.size() != n ||
            p.signatures.size() != n || p.area_um2.size() != n ||
            p.leakage_w.size() != n || p.event_j.size() != n ||
            p.read_j.size() != n || p.write_j.size() != n ||
            p.area_status.size() != n || p.leakage_status.size() != n ||
            p.event_status.size() != n || p.read_status.size() != n ||
            p.write_status.size() != n || p.timing_available.size() != n ||
            p.minimum_latency_s.size() != n ||
            p.minimum_issue_interval_s.size() != n || p.pipelined.size() != n,
        "%s: normalized characterization vectors have unequal sizes", name());
    for (size_t i = 0; i < n; ++i) {
        Record record{p.ids[i],
                      p.sources[i],
                      p.signatures[i],
                      quantity(p.area_um2[i], p.area_status[i]),
                      quantity(p.leakage_w[i], p.leakage_status[i]),
                      quantity(p.event_j[i], p.event_status[i]),
                      quantity(p.read_j[i], p.read_status[i]),
                      quantity(p.write_j[i], p.write_status[i]),
                      {p.timing_available[i], p.minimum_latency_s[i],
                       p.minimum_issue_interval_s[i], p.pipelined[i]}};
        fatal_if(!std::isfinite(record.timing.minimumLatency) ||
                     record.timing.minimumLatency < 0 ||
                     !std::isfinite(record.timing.minimumIssueInterval) ||
                     record.timing.minimumIssueInterval < 0,
                 "%s: invalid functional-unit timing constraints", name());
        fatal_if(record.timing.available && p.keys[i].rfind("fu:", 0) != 0,
                 "%s: timing constraints require a physical FU record",
                 name());
        fatal_if(
            p.keys[i].empty() || record.id.empty() || record.source.empty(),
            "%s: characterization needs a key, identity and source", name());
        fatal_if(!records.emplace(p.keys[i], record).second,
                 "%s: duplicate characterization binding '%s'", name(),
                 p.keys[i]);
    }
}

void
SALAMPowerModel::startup()
{
    ledger.reset(curTick());
}

void
SALAMPowerModel::resetStats()
{
    SimObject::resetStats();
    ledger.reset(curTick());
}

void
SALAMPowerModel::preDumpStats()
{
    checkComplete();
    printWindow(std::cout);
    SimObject::preDumpStats();
}

const SALAMPowerModel::Record *
SALAMPowerModel::find(const std::string &key) const
{
    auto it = records.find(key);
    return it == records.end() ? nullptr : &it->second;
}

void
SALAMPowerModel::physical(const std::string &key, uint64_t count,
                          const Record *record)
{
    auto found = ledger.physical.find(key);
    fatal_if(found != ledger.physical.end() && found->second.count != count,
             "%s: static PPA inventory changed for '%s'", name(), key);
    ledger.physical[key] = {count, record ? record->area : Quantity{},
                            record ? record->leakage : Quantity{}};
    if (count && (!record || !record->area.available)) {
        missing.insert(key + ":area");
    }
    if (count && (!record || !record->leakage.available)) {
        missing.insert(key + ":leakage");
    }
}

void
SALAMPowerModel::configureFunctionalUnit(const std::string &alias,
                                         uint64_t count,
                                         const Operations &operations,
                                         double intervalSeconds,
                                         double computePeriodSeconds)
{
    if (!enabled) {
        return;
    }
    std::string key = "fu:" + alias;
    const Record *record = find(key);
    bool compatible = true;
    if (count && (!record || !record->timing.available)) {
        timingCompatibilityAvailable = false;
        missing.insert(key + ":timing_constraints_unavailable");
    }
    if (count && record && record->timing.available &&
        record->timing.pipelined != (intervalSeconds > 0)) {
        timingCompatible = false;
        markStaticIncomplete(key + ":timing:pipeline_organization");
        markIncomplete(key + ":timing:pipeline_organization");
    }
    // A positive explicit II is part of the configured physical unit,
    // including a positive-limit unit which is idle in this graph.
    if (count && record && record->timing.available && intervalSeconds > 0 &&
        std::max(computePeriodSeconds, intervalSeconds) <
            record->timing.minimumIssueInterval) {
        timingCompatible = false;
        markStaticIncomplete(key + ":timing:minimum_issue_interval");
        markIncomplete(key + ":timing:minimum_issue_interval");
    }
    for (const auto &op : operations) {
        std::string eventKey =
            "op:" + alias + ":" + op.opcode + ":" + op.signature;
        const Record *event = find(eventKey);
        if (record && record->timing.available) {
            const auto timing = record->timing.check(
                op.latencySeconds, intervalSeconds, computePeriodSeconds);
            const std::string operationKey =
                key + ":timing:" + op.opcode + ":" + op.signature;
            if (!timing.latency) {
                timingCompatible = false;
                markStaticIncomplete(operationKey + ":minimum_latency");
                markIncomplete(operationKey + ":minimum_latency");
            }
            if (intervalSeconds == 0 && !timing.interval) {
                timingCompatible = false;
                markStaticIncomplete(operationKey + ":minimum_issue_interval");
                markIncomplete(operationKey + ":minimum_issue_interval");
            }
        }
        if (!record || !allows(record->signatures, op.signature)) {
            compatible = false;
            missing.insert(key + ":signature:" + op.signature);
        }
        // A datatype whitelist alone does not identify the implemented
        // operation: a multiplier and divider may have identical types.
        // The event row binds this operation to the physical record even
        // when its dynamic-energy quantity is explicitly unavailable.
        if (!event) {
            compatible = false;
            missing.insert(eventKey + ":physical_binding");
        }
        if (event && record) {
            fatal_if(event->id != record->id,
                     "%s: event '%s' belongs to a different physical "
                     "implementation",
                     name(), eventKey);
            fatal_if(event->area.available && record->area.available &&
                         event->area.value != record->area.value,
                     "%s: contradictory physical area for '%s'", name(), key);
            fatal_if(event->leakage.available && record->leakage.available &&
                         event->leakage.value != record->leakage.value,
                     "%s: contradictory physical leakage for '%s'", name(),
                     key);
        }
        if (!event || !event->event.available) {
            ledger.activityCoverage = false;
            missing.insert(eventKey + ":energy");
        }
    }
    if (!compatible) {
        incompatibleUnits.insert(alias);
        ledger.activityCoverage = false;
    }
    physical(key, count, compatible ? record : nullptr);
}

const SALAMPowerModel::Record *
SALAMPowerModel::registerRecord(const std::string &type, unsigned &scale) const
{
    scale = 1;
    if (const auto *record = find("register:" + type)) {
        return record;
    }
    scale = typeBits(type);
    return find("register:bit");
}

void
SALAMPowerModel::configureRegisters(
    const std::map<std::string, uint64_t> &slots)
{
    if (!enabled) {
        return;
    }
    fatal_if(registersConfigured && registerSlots != slots,
             "%s: SSA storage inventory changed between invocations", name());
    registerSlots = slots;
    registersConfigured = true;
    inventoryConfigured = true;
    for (const auto &entry : slots) {
        unsigned scale;
        const Record *record = registerRecord(entry.first, scale);
        std::string key = "register:" + entry.first;
        physical(key, entry.second * scale, record);
        if (entry.second &&
            (!record || !record->read.available || !record->write.available)) {
            ledger.activityCoverage = false;
            missing.insert(key + ":access_energy");
        }
    }
}

void
SALAMPowerModel::configureStorage(const std::string &identity, uint64_t bytes,
                                  unsigned rw)
{
    if (!enabled) {
        return;
    }
    fatal_if(storageConfigured,
             "%s: a PPA model must belong to one physical memory", name());
    storageConfigured = true;
    inventoryConfigured = true;
    memoryWordGeometryKnown = params().word_bytes != 0;
    const auto *record = find("memory");
    if (record) {
        fatal_if(params().capacity_bytes != bytes ||
                     params().read_ports != 0 || params().write_ports != 0 ||
                     params().rw_ports != rw || !params().word_bytes ||
                     !params().banks,
                 "%s: memory characterization geometry does not match %s",
                 name(), identity);
    }
    physical("memory:" + identity, 1, record);
    if (record) {
        fatal_if(storageScope() != "whole_device" &&
                     storageScope() != "storage_cells",
                 "%s: explicit physical memory scope required", name());
        if (!memoryTimingAvailable()) {
            markIncomplete("memory:" + identity +
                           ":timing_constraints_unavailable");
        }
        if (storageScope() == "storage_cells") {
            markStaticIncomplete("memory:" + identity +
                                 ":selection_decoder_port_logic");
            markIncomplete("memory:" + identity +
                           ":selection_decoder_port_logic");
        }
    }
    if (!record || !record->read.available || !record->write.available) {
        ledger.activityCoverage = false;
        missing.insert("memory:" + identity + ":access_energy");
    }
}

void
SALAMPowerModel::beginInvocation(Tick tick)
{
    if (!enabled) {
        return;
    }
    fatal_if(ledger.active, "%s: overlapping PPA invocations", name());
    ledger.begin(tick);
    invocationStarted = true;
    checkComplete();
}

void
SALAMPowerModel::finishInvocation(Tick tick)
{
    if (enabled) {
        ledger.finish(tick);
        checkComplete();
    }
}

void
SALAMPowerModel::charge(const std::string &key, Quantity energy,
                        uint64_t count)
{
    if (!enabled || !count) {
        return;
    }
    ledger.event(energy, count, key);
    fatal_if(strict && !energy.available,
             "%s: missing dynamic-energy characterization for '%s'", name(),
             key);
}

void
SALAMPowerModel::operation(const std::string &alias, const std::string &opcode,
                           const std::string &signature)
{
    std::string key = "op:" + alias + ":" + opcode + ":" + signature;
    const auto *record = incompatibleUnits.count(alias) ? nullptr : find(key);
    charge(key, record ? record->event : Quantity{}, 1);
}

void
SALAMPowerModel::unboundOperation(const std::string &opcode,
                                  const std::string &signature)
{
    charge("op:unbound:" + opcode + ":" + signature, Quantity{}, 1);
}

void
SALAMPowerModel::registerRead(const std::string &type)
{
    if (!enabled) {
        return;
    }
    using Scope = SALAM::PPA::Ledger::Scope;
    ledger.observe(&Scope::operandReads, 1);
    ledger.observe(&Scope::operandReadBits, typeBits(type));
    unsigned scale;
    const auto *record = registerRecord(type, scale);
    charge("register:" + type + ":read", record ? record->read : Quantity{},
           scale);
}

void
SALAMPowerModel::registerWrite(const std::string &type)
{
    if (!enabled) {
        return;
    }
    using Scope = SALAM::PPA::Ledger::Scope;
    ledger.observe(&Scope::resultWrites, 1);
    ledger.observe(&Scope::resultWriteBits, typeBits(type));
    unsigned scale;
    const auto *record = registerRecord(type, scale);
    charge("register:" + type + ":write", record ? record->write : Quantity{},
           scale);
}

void
SALAMPowerModel::accountStorageAccess(const Packet &packet, Addr memory_start)
{
    if (!enabled || !SALAM::hasStorageTransfer(packet)) {
        return;
    }
    const bool write = packet.isWrite();
    const unsigned width = wordBytes();
    if (!width) {
        unknownStorageAccess(write);
        return;
    }
    const auto activity = SALAM::storageAccess(packet, memory_start, width);
    if (!write) {
        storageRead(activity.words, activity.bits);
        return;
    }
    if (activity.partialWords && maskModel() == "unsupported") {
        unknownStorageAccess(write, "memory:partial_write_physical_model");
        return;
    }
    const auto rmw =
        maskModel() == "read_modify_write" ? activity.partialWords : 0;
    storageWrite(activity.words, activity.bits, rmw);
}

void
SALAMPowerModel::storageRead(uint64_t words, uint64_t bits)
{
    if (!enabled) {
        return;
    }
    ledger.observe(&SALAM::PPA::Ledger::Scope::readWords, words);
    const auto *record = find("memory");
    charge("memory:read", record ? record->read : Quantity{},
           params().storage_physical == "flip_flop" ? bits : words);
}

void
SALAMPowerModel::storageWrite(uint64_t words, uint64_t bits, uint64_t rmwWords)
{
    if (!enabled) {
        return;
    }
    ledger.observe(&SALAM::PPA::Ledger::Scope::writeWords, words);
    ledger.observe(&SALAM::PPA::Ledger::Scope::readWords, rmwWords);
    const auto *record = find("memory");
    charge("memory:write", record ? record->write : Quantity{},
           params().storage_physical == "flip_flop" ? bits : words);
    charge("memory:read_modify_write", record ? record->read : Quantity{},
           rmwWords);
}

void
SALAMPowerModel::unknownStorageAccess(bool write, const std::string &reason)
{
    if (!enabled) {
        return;
    }
    const std::string identity = reason + (write ? ":write" : ":read");
    ledger.unknownStorageAccess(identity);
    fatal_if(strict, "%s: unknown physical storage access: %s", name(),
             identity);
}

SALAMPowerModel::Result
SALAMPowerModel::windowResult() const
{
    auto result =
        ledger.result(ledger.window, curTick(), sim_clock::as_float::s);
    if (!enabled || !inventoryConfigured) {
        result.areaComplete = false;
        result.dynamicComplete = false;
        result.leakageComplete = false;
    }
    return result;
}

SALAMPowerModel::Result
SALAMPowerModel::invocationResult() const
{
    return ledger.result(ledger.invocation, curTick(), sim_clock::as_float::s);
}

void
SALAMPowerModel::checkComplete() const
{
    if (!enabled || !strict) {
        return;
    }
    const auto result = windowResult();
    fatal_if(!result.areaComplete || !result.leakageComplete ||
                 !result.dynamicComplete,
             "%s: strict PPA coverage is incomplete; %d missing bindings",
             name(), missing.size());
}

void
SALAMPowerModel::markIncomplete(const std::string &reason)
{
    if (!enabled) {
        return;
    }
    missing.insert(reason);
    ledger.activityCoverage = false;
    fatal_if(strict, "%s: incomplete PPA coverage: %s", name(), reason);
}

void
SALAMPowerModel::markStaticIncomplete(const std::string &reason)
{
    if (!enabled) {
        return;
    }
    ledger.staticCoverage = false;
    if (reason.rfind("fu:unbound:", 0) == 0) {
        timingCompatibilityAvailable = false;
    }
    missing.insert(reason);
    fatal_if(strict, "%s: strict PPA coverage is incomplete: %s", name(),
             reason);
}

void
SALAMPowerModel::printInvocation(std::ostream &stream) const
{
    if (enabled && params().report_details && invocationStarted) {
        printResult(stream, invocationResult(), ledger.invocation,
                    "invocation: FU datapath and SSA storage");
    }
}

void
SALAMPowerModel::printWindow(std::ostream &stream) const
{
    if (enabled && params().report_details) {
        printResult(stream, windowResult(), ledger.window,
                    "device statistics window");
    }
}

void
SALAMPowerModel::printResult(std::ostream &stream, const Result &result,
                             const SALAM::PPA::Ledger::Scope &scope,
                             const std::string &label) const
{
    auto flags = stream.flags();
    auto precision = stream.precision();
    stream << std::setprecision(17);
    stream << "   PPA model: " << name() << '\n';
    stream << "   PPA scope: " << label << '\n';
    stream << "   PPA elapsed seconds: " << result.duration << '\n';
    stream << "   PPA known area (um^2): " << result.area << '\n';
    stream << "   PPA known dynamic energy (J): " << result.dynamic << '\n';
    stream << "   PPA known leakage energy (J): " << result.leakage << '\n';
    stream << "   PPA known total energy (J): " << result.total() << '\n';
    stream << "   PPA known average power (W): ";
    if (result.powerAvailable()) {
        stream << result.power() << '\n';
    } else {
        stream << "unavailable (zero duration)\n";
    }
    stream << "   PPA coverage area/dynamic/leakage: " << result.areaComplete
           << '/' << result.dynamicComplete << '/' << result.leakageComplete
           << "; approximate=" << result.approximate << '\n';
    stream << "   PPA FU timing compatibility: "
           << (!timingCompatible               ? "incompatible"
               : !timingCompatibilityAvailable ? "unavailable"
                                               : "compatible")
           << '\n';
    stream << "   PPA operand reads/bits: " << scope.operandReads << '/'
           << scope.operandReadBits << '\n';
    stream << "   PPA result writes/bits: " << scope.resultWrites << '/'
           << scope.resultWriteBits << '\n';
    stream << "   PPA memory read/write words: " << scope.readWords << '/'
           << scope.writeWords << '\n';
    stream << "   PPA memory word counts available: "
           << (memoryWordGeometryKnown && scope.wordCountsAvailable) << '\n';
    stream << "   PPA missing event count: " << scope.missingEvents << '\n';
    for (const auto &entry : ledger.physical) {
        stream << "   PPA physical inventory: " << entry.first << '='
               << entry.second.count << '\n';
    }
    for (const auto &entry : registerSlots) {
        stream << "   PPA SSA slots: " << entry.first << '=' << entry.second
               << '\n';
    }
    for (const auto &key : missing) {
        stream << "   PPA missing: " << key << '\n';
    }
    for (const auto &key : scope.missingEventKeys) {
        stream << "   PPA uncharacterized activity: " << key << '\n';
    }
    stream.flags(flags);
    stream.precision(precision);
    // Python and C++ share stdout but buffer it independently. Complete
    // this report before returning to Python or emitting other output.
    stream.flush();
}

SALAMPowerModel::ModelStats::ModelStats(SALAMPowerModel *owner)
    : statistics::Group(owner),
      ADD_STAT(knownAreaUm2, statistics::units::Unspecified::get(),
               "Known modeled area in square micrometres; see coverage"),
      ADD_STAT(knownDynamicEnergyJ, statistics::units::Joule::get(),
               "Known dynamic energy in the statistics window"),
      ADD_STAT(knownLeakageEnergyJ, statistics::units::Joule::get(),
               "Known leakage energy in the statistics window"),
      ADD_STAT(knownTotalEnergyJ, statistics::units::Joule::get(),
               "Known dynamic plus leakage energy; see coverage"),
      ADD_STAT(knownAveragePowerW, statistics::units::Watt::get(),
               "Known energy over window seconds; unavailable at zero time"),
      ADD_STAT(elapsedSeconds, statistics::units::Second::get(),
               "Duration of this statistics window"),
      ADD_STAT(areaComplete, statistics::units::Count::get(),
               "All modeled physical instances have area coefficients"),
      ADD_STAT(dynamicComplete, statistics::units::Count::get(),
               "All modeled activity has energy coefficients"),
      ADD_STAT(leakageComplete, statistics::units::Count::get(),
               "All modeled physical instances have leakage coefficients"),
      ADD_STAT(powerAvailable, statistics::units::Count::get(),
               "Average power has a nonzero observation interval"),
      ADD_STAT(approximate, statistics::units::Count::get(),
               "Known quantities include explicit proxies"),
      ADD_STAT(
          timingCompatibilityAvailable, statistics::units::Count::get(),
          "All present functional units have explicit timing constraints"),
      ADD_STAT(timingCompatible, statistics::units::Count::get(),
               "Known FU timing constraints match execution configuration"),
      ADD_STAT(missingEventCount, statistics::units::Count::get(),
               "Observed events without a valid physical energy mapping"),
      ADD_STAT(operandReads, statistics::units::Count::get(),
               "Materialized SSA operand transfers"),
      ADD_STAT(resultWrites, statistics::units::Count::get(),
               "Materialized SSA result and argument publications"),
      ADD_STAT(operandReadBits, statistics::units::Count::get(),
               "LLVM-typed bits captured from materialized SSA slots"),
      ADD_STAT(resultWriteBits, statistics::units::Count::get(),
               "LLVM-typed bits published into materialized SSA slots"),
      ADD_STAT(readWords, statistics::units::Count::get(),
               "Known physical memory words read including write RMW"),
      ADD_STAT(writeWords, statistics::units::Count::get(),
               "Known physical memory words written"),
      ADD_STAT(memoryWordCountsAvailable, statistics::units::Count::get(),
               "Every serviced access has a known physical word mapping")
{
    // Text statistics use fixed decimal places, not significant digits.
    // Retain sub-femtojoule energy and picosecond elapsed-time detail.
    knownAreaUm2.precision(17);
    knownDynamicEnergyJ.precision(30);
    knownLeakageEnergyJ.precision(30);
    knownTotalEnergyJ.precision(30);
    knownAveragePowerW.precision(24);
    elapsedSeconds.precision(24);
    knownAreaUm2.functor([owner] { return owner->windowResult().area; });
    knownDynamicEnergyJ.functor(
        [owner] { return owner->windowResult().dynamic; });
    knownLeakageEnergyJ.functor(
        [owner] { return owner->windowResult().leakage; });
    knownTotalEnergyJ.functor(
        [owner] { return owner->windowResult().total(); });
    knownAveragePowerW.functor(
        [owner] { return owner->windowResult().power(); });
    elapsedSeconds.functor([owner] { return owner->windowResult().duration; });
    areaComplete.functor([owner] {
        return owner->enabled && owner->windowResult().areaComplete;
    });
    dynamicComplete.functor([owner] {
        return owner->enabled && owner->windowResult().dynamicComplete;
    });
    leakageComplete.functor([owner] {
        return owner->enabled && owner->windowResult().leakageComplete;
    });
    powerAvailable.functor([owner] {
        return owner->enabled && owner->windowResult().powerAvailable();
    });
    approximate.functor([owner] { return owner->windowResult().approximate; });
    timingCompatibilityAvailable.functor([owner] {
        return owner->enabled && owner->timingCompatibilityAvailable;
    });
    timingCompatible.functor(
        [owner] { return owner->enabled && owner->timingCompatible; });
    missingEventCount.functor(
        [owner] { return owner->ledger.window.missingEvents; });
    operandReads.functor(
        [owner] { return owner->ledger.window.operandReads; });
    resultWrites.functor(
        [owner] { return owner->ledger.window.resultWrites; });
    operandReadBits.functor(
        [owner] { return owner->ledger.window.operandReadBits; });
    resultWriteBits.functor(
        [owner] { return owner->ledger.window.resultWriteBits; });
    readWords.functor([owner] { return owner->ledger.window.readWords; });
    writeWords.functor([owner] { return owner->ledger.window.writeWords; });
    memoryWordCountsAvailable.functor([owner] {
        return owner->enabled && owner->memoryWordGeometryKnown &&
               owner->ledger.window.wordCountsAvailable;
    });
}
