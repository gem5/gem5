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

#ifndef __HWMODEL_HW_MODEL_HH__
#define __HWMODEL_HW_MODEL_HH__

#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "../common/macros.hh"
#include "base/statistics.hh"
#include "cycle_counts.hh"
#include "fu_pool.hh"
#include "opcodes.hh"
#include "params/HWInterface.hh"
#include "salam/HWModeling/functional_units.hh"
#include "salam/HWModeling/instruction_config.hh"
#include "salam_power_model.hh"
#include "sim/sim_object.hh"
#include "simulator_config.hh"

using namespace gem5;

class HWInterface : public SimObject
{
    friend class LLVMInterface;

  private:
  protected:
  public:
    CycleCounts *cycle_counts;
    FunctionalUnits *functional_units;
    InstConfig *inst_config;
    InstOpCodes *opcodes;
    SALAMPowerModel *salam_power_model;
    SimulatorConfig *simulator_config;

    HWInterface();
    HWInterface(const HWInterfaceParams &params);
    // The unit with this enum_value, or nullptr.
    FunctionalUnitBase *findFunctionalUnit(uint64_t enum_value);
    // Admission to this accelerator's functional units.
    SALAM::FUPool fuPool;

    // Record an admission, a release or a denial for a unit's
    // statistics.
    void fuAdmitted(uint32_t unit);
    void fuReleased(uint32_t unit);
    void fuDenied(uint32_t unit);

    // The functional units an accelerator has, resolved from the
    // hardware profile and the kernel's static graph.
    struct FUInventory
    {
        // Instances of each unit, by enum_value.
        std::map<uint32_t, uint32_t> instances;
        // (static instruction UID, enum_value) of each instruction
        // that uses a unit.
        std::vector<std::pair<uint64_t, uint32_t>> mapping;

        bool
        operator==(const FUInventory &other) const
        {
            return instances == other.instances && mapping == other.mapping;
        }
    };
    // Record the inventory. Every later invocation must resolve the
    // same one.
    void setInventory(const FUInventory &resolved);
    const FUInventory &
    getInventory() const
    {
        return inventory;
    }

  private:
    bool inventoryResolved = false;
    FUInventory inventory;

    // One functional unit's statistics, named by its alias. Instances
    // and utilization are computed from the pool when dumped, so a
    // statistics reset does not change them.
    struct UnitStats : public statistics::Group
    {
        UnitStats(HWInterface *hw, FunctionalUnitBase *unit);

        statistics::Value instances;
        statistics::Scalar admissions;
        statistics::Scalar denials;
        statistics::Average inFlight;
        statistics::Average busyInstances;
        statistics::Value utilization;
    };
    std::map<uint32_t, std::unique_ptr<UnitStats>> unitStats;
    void sampleUnit(uint32_t unit);
};

#endif //__HWMODEL_HW_MODEL_HH__
