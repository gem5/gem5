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
 * Authors: Karthik Chandrasekar
 *          Matthias Jung
 *          Omar Naji
 *          Subash Kannoth
 *          Éder F. Zulian
 *          Felipe S. Prado
 *
 */

#ifndef MEMORY_POWER_MODEL_H
#define MEMORY_POWER_MODEL_H

#include <numeric>
#include "MemorySpecification.h"
#include "MemBankWiseParams.h"
#include "CommandAnalysis.h"

namespace Data {
class MemoryPowerModel {
 public:

  MemoryPowerModel();

  // Calculate energy and average power consumption for the given memory
  // command trace
  void power_calc(const MemorySpecification& memSpec,
                  const CommandAnalysis& c,
                  int term,
                  const MemBankWiseParams& bwPowerParams);

  // Used to calculate self-refresh active energy
  static double engy_sref(double idd6,
                          double idd3n,
                          double idd5,
                          double vdd,
                          double sref_cycles_idd6,
                          double sref_ref_act_cycles,
                          double sref_ref_pre_cycles,
                          double spup_ref_act_cycles,
                          double spup_ref_pre_cycles,
                          double clk);
  static double engy_sref_banks(double idd6,
                                double idd3n,
                                double idd5,
                                double vdd,
                                double sref_cycles,
                                double sref_ref_act_cycles,
                                double sref_ref_pre_cycles,
                                double spup_ref_act_cycles,
                                double spup_ref_pre_cycles,
                                double clk,
                                double esharedPASR,
                                const MemBankWiseParams& bwPowerParams,
                                unsigned bnkIdx,
                                int64_t nbrofBanks);

  int64_t total_cycles;

  int64_t window_cycles;

  struct Energy {
    // Total energy of all activates
    double act_energy;
    std::vector<double> act_energy_banks;

    // Total energy of all precharges
    double pre_energy;
    std::vector<double> pre_energy_banks;

    // Total energy of all reads
    double read_energy;
    std::vector<double> read_energy_banks;

    // Total energy of all writes
    double write_energy;
    std::vector<double> write_energy_banks;

    // Total energy of all refreshes
    double ref_energy;
    std::vector<double> ref_energy_banks;

    // Bankwise refresh energy
    std::vector<double> refb_energy_banks;

    // Total background energy of all active standby cycles
    double act_stdby_energy;
    std::vector<double> act_stdby_energy_banks;

    // Total background energy of all precharge standby cycles
    double pre_stdby_energy;
    std::vector<double> pre_stdby_energy_banks;

    // Total energy of idle cycles in the active mode
    double idle_energy_act;
    std::vector<double> idle_energy_act_banks;

    // Total energy of idle cycles in the precharge mode
    double idle_energy_pre;
    std::vector<double> idle_energy_pre_banks;

    // Total trace/pattern energy
    double total_energy;
    std::vector<double> total_energy_banks;

    // Window energy
    double window_energy;

    // Average Power
    double average_power;

    // Energy consumed in active/precharged fast/slow-exit modes
    double f_act_pd_energy;
    std::vector<double> f_act_pd_energy_banks;

    double f_pre_pd_energy;
    std::vector<double> f_pre_pd_energy_banks;

    double s_act_pd_energy;
    std::vector<double> s_act_pd_energy_banks;

    double s_pre_pd_energy;
    std::vector<double> s_pre_pd_energy_banks;

    // Energy consumed in self-refresh mode
    double sref_energy;
    std::vector<double> sref_energy_banks;

    // Energy consumed in auto-refresh during self-refresh mode
    double sref_ref_energy;
    std::vector<double> sref_ref_energy_banks;

    double sref_ref_act_energy;
    std::vector<double> sref_ref_act_energy_banks;

    double sref_ref_pre_energy;
    std::vector<double> sref_ref_pre_energy_banks;

    // Energy consumed in powering-up from self-refresh mode
    double spup_energy;
    std::vector<double> spup_energy_banks;

    // Energy consumed in auto-refresh during self-refresh power-up
    double spup_ref_energy;
    std::vector<double> spup_ref_energy_banks;

    double spup_ref_act_energy;
    std::vector<double> spup_ref_act_energy_banks;

    double spup_ref_pre_energy;
    std::vector<double> spup_ref_pre_energy_banks;

    // Energy consumed in powering-up from active/precharged power-down modes
    double pup_act_energy;
    std::vector<double> pup_act_energy_banks;

    double pup_pre_energy;
    std::vector<double> pup_pre_energy_banks;

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

    // Window Average Power
    double window_average_power;
  };

  // Print the power and energy
  void power_print(const MemorySpecification& memSpec,
                   int                 term,
                   const CommandAnalysis& c,
                   bool bankwiseMode) const;

  // To derive IO and Termination Power measures using DRAM specification
  void io_term_power(const MemorySpecification& memSpec);

  Energy energy;
  Power  power;

 private:
  double calcIoTermEnergy(int64_t cycles, double period, double power, int64_t numBits) const;
  // Sum quantities (e.g., operations, energy, cycles) that are stored in a per bank basis returning the total amount.
  template <typename T> T sum(const std::vector<T> vec) const { return std::accumulate(vec.begin(), vec.end(), static_cast<T>(0)); }
};

class EnergyDomain {
 public:
  EnergyDomain(double voltage, double clkPeriod) :
    voltage(voltage),
    clkPeriod(clkPeriod)
  {}

  double calcTivEnergy(int64_t cycles, double current) const;
  double getVoltage() const{ return voltage; };
 private:
  const double voltage;
  const double clkPeriod;
};

}
#endif // ifndef MEMORY_POWER_MODEL_H
