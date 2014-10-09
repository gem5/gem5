/*
 * Copyright (c) 2012-2014, TU Delft
 * Copyright (c) 2012-2014, TU Eindhoven
 * Copyright (c) 2012-2014, TU Kaiserslautern
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Karthik Chandrasekar, Matthias Jung, Omar Naji
 *
 */

#ifndef MEMORY_POWER_MODEL_H
#define MEMORY_POWER_MODEL_H

#include "MemorySpecification.h"
#include "CommandAnalysis.h"

namespace Data {
class MemoryPowerModel {
 public:
  // Calculate energy and average power consumption for the given memory
  // command trace
  void power_calc(MemorySpecification    memSpec,
                  const CommandAnalysis& counters,
                  int                    term);

  // Used to calculate self-refresh active energy
  static double engy_sref(double idd6,
                          double idd3n,
                          double idd5,
                          double vdd,
                          double sref_cycles,
                          double sref_ref_act_cycles,
                          double sref_ref_pre_cycles,
                          double spup_ref_act_cycles,
                          double spup_ref_pre_cycles,
                          double clk);

  int64_t total_cycles;

  struct Energy {
    // Total energy of all activates
    double act_energy;

    // Total energy of all precharges
    double pre_energy;

    // Total energy of all reads
    double read_energy;

    // Total energy of all writes
    double write_energy;

    // Total energy of all refreshes
    double ref_energy;

    // Total background energy of all active standby cycles
    double act_stdby_energy;

    // Total background energy of all precharge standby cycles
    double pre_stdby_energy;

    // Total energy of idle cycles in the active mode
    double idle_energy_act;

    // Total energy of idle cycles in the precharge mode
    double idle_energy_pre;

    // Total trace/pattern energy
    double total_energy;

    // Average Power
    double average_power;

    // Energy consumed in active/precharged fast/slow-exit modes
    double f_act_pd_energy;
    double f_pre_pd_energy;
    double s_act_pd_energy;
    double s_pre_pd_energy;

    // Energy consumed in self-refresh mode
    double sref_energy;

    // Energy consumed in auto-refresh during self-refresh mode
    double sref_ref_energy;
    double sref_ref_act_energy;
    double sref_ref_pre_energy;

    // Energy consumed in powering-up from self-refresh mode
    double spup_energy;

    // Energy consumed in auto-refresh during self-refresh power-up
    double spup_ref_energy;
    double spup_ref_act_energy;
    double spup_ref_pre_energy;

    // Energy consumed in powering-up from active/precharged power-down modes
    double pup_act_energy;
    double pup_pre_energy;

    // Energy consumed by IO and Termination
    double read_io_energy;     // Read IO Energy
    double write_term_energy;  // Write Termination Energy
    double read_oterm_energy;  // Read Termination Energy from idle rank
    double write_oterm_energy; // Write Termination Energy from idle rank
    // Total IO and Termination Energy
    double io_term_energy;
  };

  struct Power {
    // Power measures corresponding to IO and Termination
    double IO_power;     // Read IO Power
    double WR_ODT_power; // Write ODT Power
    double TermRD_power; // Read Termination in idle rank (in dual-rank systems)
    double TermWR_power; // Write Termination in idle rank (in dual-rank systems)

    // Average Power
    double average_power;
  };

  // Print the power and energy
  void power_print(MemorySpecification memSpec,
                   int                 term,
                   const CommandAnalysis& counters) const;

  // To derive IO and Termination Power measures using DRAM specification
  void io_term_power(MemorySpecification memSpec);

  Energy energy;
  Power  power;

 private:
  double calcIoTermEnergy(int64_t cycles, double period, double power, int64_t numBits) const;
};

class EnergyDomain {
 public:
  EnergyDomain(double voltage, double clkPeriod) :
    voltage(voltage),
    clkPeriod(clkPeriod)
  {}

  double calcTivEnergy(int64_t cycles, double current) const;
 private:
  const double voltage;
  const double clkPeriod;
};

}
#endif // ifndef MEMORY_POWER_MODEL_H
