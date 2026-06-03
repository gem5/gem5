/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __HWMODEL_HW_MODEL_HH__
#define __HWMODEL_HW_MODEL_HH__

#include <cstdlib>
#include <iostream>
#include <vector>

#include "../common/src/macros.hh"
#include "cycle_counts.hh"
#include "functional_units.hh"
#include "hw_statistics.hh"
#include "instruction_config.hh"
#include "opcodes.hh"
#include "params/HWInterface.hh"
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
    HWStatistics *hw_statistics;
    InstConfig *inst_config;
    InstOpCodes *opcodes;
    SALAMPowerModel *salam_power_model;
    SimulatorConfig *simulator_config;

    HWInterface();
    HWInterface(const HWInterfaceParams &params);
    bool availableFunctionalUnit(uint64_t functional_unit);
    void clearFunctionalUnit(uint64_t functional_unit);
};

#endif //__HWMODEL_HW_MODEL_HH__
