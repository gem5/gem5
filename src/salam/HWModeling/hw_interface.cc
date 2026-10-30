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

#include "hw_interface.hh"

HWInterface::HWInterface(const HWInterfaceParams &params)
    : SimObject(params),
      cycle_counts(params.cycle_counts),
      functional_units(params.functional_units),
      inst_config(params.inst_config),
      salam_power_model(params.salam_power_model),
      simulator_config(params.simulator_config)
{
    for (auto unit : functional_units->functionalUnitList) {
        unitStats[unit->getEnumValue()] =
            std::make_unique<UnitStats>(this, unit);
    }
}

HWInterface::UnitStats::UnitStats(HWInterface *hw, FunctionalUnit *unit)
    : statistics::Group(hw, unit->getAlias().c_str()),
      ADD_STAT(instances, statistics::units::Count::get(),
               "Instances of the unit"),
      ADD_STAT(admissions, statistics::units::Count::get(),
               "Operations admitted"),
      ADD_STAT(denials, statistics::units::Count::get(),
               "Times a ready operation found no instance free"),
      ADD_STAT(inFlight, statistics::units::Count::get(),
               "Time-weighted average of operations in flight"),
      ADD_STAT(busyInstances, statistics::units::Count::get(),
               "Time-weighted average of instances with an operation in "
               "flight"),
      ADD_STAT(utilization, statistics::units::Ratio::get(),
               "Busy-instance time over instances times the window; 0 "
               "with no instances")
{
    uint32_t id = unit->getEnumValue();
    instances.functor([hw, id] {
        return hw->fuPool.hasUnit(id) ? hw->fuPool.instances(id) : 0;
    });
    utilization.functor([this, hw, id] {
        double n = hw->fuPool.hasUnit(id) ? hw->fuPool.instances(id) : 0;
        return n ? busyInstances.result() / n : 0.0;
    });
}

void
HWInterface::sampleUnit(uint32_t unit)
{
    UnitStats &stats = *unitStats.at(unit);
    stats.inFlight = fuPool.inFlight(unit);
    stats.busyInstances = fuPool.busyInstances(unit);
}

void
HWInterface::fuAdmitted(uint32_t unit)
{
    unitStats.at(unit)->admissions++;
    sampleUnit(unit);
}

void
HWInterface::fuReleased(uint32_t unit)
{
    sampleUnit(unit);
}

void
HWInterface::fuDenied(uint32_t unit)
{
    unitStats.at(unit)->denials++;
}

FunctionalUnit *
HWInterface::findFunctionalUnit(uint64_t enum_value)
{
    for (auto unit : functional_units->functionalUnitList) {
        if (unit->getEnumValue() == enum_value) {
            return unit;
        }
    }
    return nullptr;
}

void
HWInterface::setInventory(const FUInventory &resolved)
{
    panic_if(inventoryResolved && !(resolved == inventory),
             "%s: functional unit inventory changed between invocations",
             name());
    inventory = resolved;
    inventoryResolved = true;
}
