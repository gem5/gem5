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

#include "MemoryPowerModel.h"

#include <stdint.h>

#include <cmath>  // For pow
#include <iostream>  // fmtflags


using namespace std;
using namespace Data;

// Calculate energy and average power consumption for the given command trace

void MemoryPowerModel::power_calc(const MemorySpecification& memSpec,
                                  const CommandAnalysis& c,
                                  int term)
{
  const MemTimingSpec& t                 = memSpec.memTimingSpec;
  const MemArchitectureSpec& memArchSpec = memSpec.memArchSpec;
  const MemPowerSpec&  mps               = memSpec.memPowerSpec;

  energy.act_energy          = 0.0;
  energy.pre_energy          = 0.0;
  energy.read_energy         = 0.0;
  energy.write_energy        = 0.0;
  energy.ref_energy          = 0.0;
  energy.act_stdby_energy    = 0.0;
  energy.pre_stdby_energy    = 0.0;
  energy.idle_energy_act     = 0.0;
  energy.idle_energy_pre     = 0.0;
  energy.total_energy        = 0.0;
  energy.f_act_pd_energy     = 0.0;
  energy.f_pre_pd_energy     = 0.0;
  energy.s_act_pd_energy     = 0.0;
  energy.s_pre_pd_energy     = 0.0;
  energy.sref_energy         = 0.0;
  energy.sref_ref_energy     = 0.0;
  energy.sref_ref_act_energy = 0.0;
  energy.sref_ref_pre_energy = 0.0;
  energy.spup_energy         = 0.0;
  energy.spup_ref_energy     = 0.0;
  energy.spup_ref_act_energy = 0.0;
  energy.spup_ref_pre_energy = 0.0;
  energy.pup_act_energy      = 0.0;
  energy.pup_pre_energy      = 0.0;
  power.IO_power             = 0.0;
  power.WR_ODT_power         = 0.0;
  power.TermRD_power         = 0.0;
  power.TermWR_power         = 0.0;
  energy.read_io_energy      = 0.0;
  energy.write_term_energy   = 0.0;
  energy.read_oterm_energy   = 0.0;
  energy.write_oterm_energy  = 0.0;
  energy.io_term_energy      = 0.0;

  // How long a single burst takes, measured in command-clock cycles.
  int64_t burstCc = memArchSpec.burstLength / memArchSpec.dataRate;

  // IO and Termination Power measures are included, if required.
  if (term) {
    io_term_power(memSpec);

    // memArchSpec.width represents the number of data (dq) pins.
    // 1 DQS pin is associated with every data byte
    int64_t dqPlusDqsBits = memArchSpec.width + memArchSpec.width / 8;
    // 1 DQS and 1 DM pin is associated with every data byte
    int64_t dqPlusDqsPlusMaskBits = memArchSpec.width + memArchSpec.width / 8 + memArchSpec.width / 8;
    // Size of one clock period for the data bus.
    double ddrPeriod = t.clkPeriod / static_cast<double>(memArchSpec.dataRate);

    // Read IO power is consumed by each DQ (data) and DQS (data strobe) pin
    energy.read_io_energy = calcIoTermEnergy(c.numberofreads * memArchSpec.burstLength,
                                             ddrPeriod,
                                             power.IO_power,
                                             dqPlusDqsBits);

    // Write ODT power is consumed by each DQ (data), DQS (data strobe) and DM
    energy.write_term_energy = calcIoTermEnergy(c.numberofwrites * memArchSpec.burstLength,
                                                ddrPeriod,
                                                power.WR_ODT_power,
                                                dqPlusDqsPlusMaskBits);

    if (memArchSpec.nbrOfRanks > 1) {
      // Termination power consumed in the idle rank during reads on the active
      // rank by each DQ (data) and DQS (data strobe) pin.
      energy.read_oterm_energy = calcIoTermEnergy(c.numberofreads * memArchSpec.burstLength,
                                                  ddrPeriod,
                                                  power.TermRD_power,
                                                  dqPlusDqsBits);

      // Termination power consumed in the idle rank during writes on the active
      // rank by each DQ (data), DQS (data strobe) and DM (data mask) pin.
      energy.write_oterm_energy = calcIoTermEnergy(c.numberofwrites * memArchSpec.burstLength,
                                                   ddrPeriod,
                                                   power.TermWR_power,
                                                   dqPlusDqsPlusMaskBits);
    }

    // Sum of all IO and termination energy
    energy.io_term_energy = energy.read_io_energy + energy.write_term_energy
                            + energy.read_oterm_energy + energy.write_oterm_energy;
  }

  total_cycles = c.actcycles + c.precycles +
                 c.f_act_pdcycles + c.f_pre_pdcycles +
                 c.s_act_pdcycles + c.s_pre_pdcycles + c.sref_cycles
                 + c.sref_ref_act_cycles + c.sref_ref_pre_cycles +
                 c.spup_ref_act_cycles + c.spup_ref_pre_cycles;

  EnergyDomain vdd0Domain(mps.vdd, t.clkPeriod);

  energy.act_energy       = vdd0Domain.calcTivEnergy(c.numberofacts   * t.RAS          , mps.idd0 - mps.idd3n);
  energy.pre_energy       = vdd0Domain.calcTivEnergy(c.numberofpres   * (t.RC - t.RAS) , mps.idd0 - mps.idd2n);
  energy.read_energy      = vdd0Domain.calcTivEnergy(c.numberofreads  * burstCc        , mps.idd4r - mps.idd3n);
  energy.write_energy     = vdd0Domain.calcTivEnergy(c.numberofwrites * burstCc        , mps.idd4w - mps.idd3n);
  energy.ref_energy       = vdd0Domain.calcTivEnergy(c.numberofrefs   * t.RFC          , mps.idd5 - mps.idd3n);
  energy.pre_stdby_energy = vdd0Domain.calcTivEnergy(c.precycles, mps.idd2n);
  energy.act_stdby_energy = vdd0Domain.calcTivEnergy(c.actcycles, mps.idd3n);
  // Idle energy in the active standby clock cycles
  energy.idle_energy_act  = vdd0Domain.calcTivEnergy(c.idlecycles_act, mps.idd3n);
  // Idle energy in the precharge standby clock cycles
  energy.idle_energy_pre  = vdd0Domain.calcTivEnergy(c.idlecycles_pre, mps.idd2n);
  // fast-exit active power-down cycles energy
  energy.f_act_pd_energy  = vdd0Domain.calcTivEnergy(c.f_act_pdcycles, mps.idd3p1);
  // fast-exit precharged power-down cycles energy
  energy.f_pre_pd_energy  = vdd0Domain.calcTivEnergy(c.f_pre_pdcycles, mps.idd2p1);
  // slow-exit active power-down cycles energy
  energy.s_act_pd_energy  = vdd0Domain.calcTivEnergy(c.s_act_pdcycles, mps.idd3p0);
  // slow-exit precharged power-down cycles energy
  energy.s_pre_pd_energy  = vdd0Domain.calcTivEnergy(c.s_pre_pdcycles, mps.idd2p0);

  // self-refresh cycles energy including a refresh per self-refresh entry
  energy.sref_energy = engy_sref(mps.idd6, mps.idd3n,
                                 mps.idd5, mps.vdd,
                                 static_cast<double>(c.sref_cycles), static_cast<double>(c.sref_ref_act_cycles),
                                 static_cast<double>(c.sref_ref_pre_cycles), static_cast<double>(c.spup_ref_act_cycles),
                                 static_cast<double>(c.spup_ref_pre_cycles), t.clkPeriod);

  // background energy during active auto-refresh cycles in self-refresh
  energy.sref_ref_act_energy = vdd0Domain.calcTivEnergy(c.sref_ref_act_cycles, mps.idd3p0);
  // background energy during precharged auto-refresh cycles in self-refresh
  energy.sref_ref_pre_energy = vdd0Domain.calcTivEnergy(c.sref_ref_pre_cycles, mps.idd2p0);
  // background energy during active auto-refresh cycles in self-refresh exit
  energy.spup_ref_act_energy = vdd0Domain.calcTivEnergy(c.spup_ref_act_cycles, mps.idd3n);
  // background energy during precharged auto-refresh cycles in self-refresh exit
  energy.spup_ref_pre_energy = vdd0Domain.calcTivEnergy(c.spup_ref_pre_cycles, mps.idd2n);
  // self-refresh power-up cycles energy -- included
  energy.spup_energy         = vdd0Domain.calcTivEnergy(c.spup_cycles, mps.idd2n);
  // active power-up cycles energy - same as active standby -- included
  energy.pup_act_energy      = vdd0Domain.calcTivEnergy(c.pup_act_cycles, mps.idd3n);
  // precharged power-up cycles energy - same as precharged standby -- included
  energy.pup_pre_energy      = vdd0Domain.calcTivEnergy(c.pup_pre_cycles, mps.idd2n);

  // similar equations as before to support multiple voltage domains in LPDDR2
  // and WIDEIO memories
  if (memArchSpec.twoVoltageDomains) {
    EnergyDomain vdd2Domain(mps.vdd2, t.clkPeriod);

    energy.act_energy       += vdd2Domain.calcTivEnergy(c.numberofacts   * t.RAS          , mps.idd02 - mps.idd3n2);
    energy.pre_energy       += vdd2Domain.calcTivEnergy(c.numberofpres   * (t.RC - t.RAS) , mps.idd02 - mps.idd2n2);
    energy.read_energy      += vdd2Domain.calcTivEnergy(c.numberofreads  * burstCc        , mps.idd4r2 - mps.idd3n2);
    energy.write_energy     += vdd2Domain.calcTivEnergy(c.numberofwrites * burstCc        , mps.idd4w2 - mps.idd3n2);
    energy.ref_energy       += vdd2Domain.calcTivEnergy(c.numberofrefs   * t.RFC          , mps.idd52 - mps.idd3n2);
    energy.pre_stdby_energy += vdd2Domain.calcTivEnergy(c.precycles, mps.idd2n2);
    energy.act_stdby_energy += vdd2Domain.calcTivEnergy(c.actcycles, mps.idd3n2);
    // Idle energy in the active standby clock cycles
    energy.idle_energy_act  += vdd2Domain.calcTivEnergy(c.idlecycles_act, mps.idd3n2);
    // Idle energy in the precharge standby clock cycles
    energy.idle_energy_pre  += vdd2Domain.calcTivEnergy(c.idlecycles_pre, mps.idd2n2);
    // fast-exit active power-down cycles energy
    energy.f_act_pd_energy  += vdd2Domain.calcTivEnergy(c.f_act_pdcycles, mps.idd3p12);
    // fast-exit precharged power-down cycles energy
    energy.f_pre_pd_energy  += vdd2Domain.calcTivEnergy(c.f_pre_pdcycles, mps.idd2p12);
    // slow-exit active power-down cycles energy
    energy.s_act_pd_energy  += vdd2Domain.calcTivEnergy(c.s_act_pdcycles, mps.idd3p02);
    // slow-exit precharged power-down cycles energy
    energy.s_pre_pd_energy  += vdd2Domain.calcTivEnergy(c.s_pre_pdcycles, mps.idd2p02);

    energy.sref_energy      += engy_sref(mps.idd62, mps.idd3n2,
                                         mps.idd52, mps.vdd2,
                                         static_cast<double>(c.sref_cycles), static_cast<double>(c.sref_ref_act_cycles),
                                         static_cast<double>(c.sref_ref_pre_cycles), static_cast<double>(c.spup_ref_act_cycles),
                                         static_cast<double>(c.spup_ref_pre_cycles), t.clkPeriod);

    // background energy during active auto-refresh cycles in self-refresh
    energy.sref_ref_act_energy += vdd2Domain.calcTivEnergy(c.sref_ref_act_cycles, mps.idd3p02);
    // background energy during precharged auto-refresh cycles in self-refresh
    energy.sref_ref_pre_energy += vdd2Domain.calcTivEnergy(c.sref_ref_pre_cycles, mps.idd2p02);
    // background energy during active auto-refresh cycles in self-refresh exit
    energy.spup_ref_act_energy += vdd2Domain.calcTivEnergy(c.spup_ref_act_cycles, mps.idd3n2);
    // background energy during precharged auto-refresh cycles in self-refresh exit
    energy.spup_ref_pre_energy += vdd2Domain.calcTivEnergy(c.spup_ref_pre_cycles, mps.idd2n2);
    // self-refresh power-up cycles energy -- included
    energy.spup_energy         += vdd2Domain.calcTivEnergy(c.spup_cycles, mps.idd2n2);
    // active power-up cycles energy - same as active standby -- included
    energy.pup_act_energy      += vdd2Domain.calcTivEnergy(c.pup_act_cycles, mps.idd3n2);
    // precharged power-up cycles energy - same as precharged standby -- included
    energy.pup_pre_energy      += vdd2Domain.calcTivEnergy(c.pup_pre_cycles, mps.idd2n2);
  }

  // auto-refresh energy during self-refresh cycles
  energy.sref_ref_energy = energy.sref_ref_act_energy + energy.sref_ref_pre_energy;

  // auto-refresh energy during self-refresh exit cycles
  energy.spup_ref_energy = energy.spup_ref_act_energy + energy.spup_ref_pre_energy;

  // adding all energy components for the active rank and all background and idle
  // energy components for both ranks (in a dual-rank system)
  energy.total_energy = energy.act_energy + energy.pre_energy + energy.read_energy +
                        energy.write_energy + energy.ref_energy + energy.io_term_energy +
                        static_cast<double>(memArchSpec.nbrOfRanks) * (energy.act_stdby_energy +
                                                  energy.pre_stdby_energy + energy.sref_energy +
                                                  energy.f_act_pd_energy + energy.f_pre_pd_energy + energy.s_act_pd_energy
                                                  + energy.s_pre_pd_energy + energy.sref_ref_energy + energy.spup_ref_energy);

  // Calculate the average power consumption
  power.average_power = energy.total_energy / (static_cast<double>(total_cycles) * t.clkPeriod);
} // MemoryPowerModel::power_calc

void MemoryPowerModel::power_print(const MemorySpecification& memSpec, int term, const CommandAnalysis& c) const
{
  const MemTimingSpec& memTimingSpec     = memSpec.memTimingSpec;
  const MemArchitectureSpec& memArchSpec = memSpec.memArchSpec;
  const uint64_t nRanks = static_cast<uint64_t>(memArchSpec.nbrOfRanks);
  const char eUnit[] = " pJ";

  ios_base::fmtflags flags = cout.flags();
  streamsize precision = cout.precision();
  cout.precision(0);
  cout << "* Trace Details:" << fixed << endl
       << endl << "#ACT commands: "                 << c.numberofacts
       << endl << "#RD + #RDA commands: "           << c.numberofreads
       << endl << "#WR + #WRA commands: "           << c.numberofwrites
  /* #PRE commands (precharge all counts a number of #PRE commands equal to the number of active banks) */
       << endl << "#PRE (+ PREA) commands: "        << c.numberofpres
       << endl << "#REF commands: "                 << c.numberofrefs
       << endl << "#Active Cycles: "                << c.actcycles
       << endl << "  #Active Idle Cycles: "         << c.idlecycles_act
       << endl << "  #Active Power-Up Cycles: "     << c.pup_act_cycles
       << endl << "    #Auto-Refresh Active cycles during Self-Refresh Power-Up: " << c.spup_ref_act_cycles
       << endl << "#Precharged Cycles: "            << c.precycles
       << endl << "  #Precharged Idle Cycles: "     << c.idlecycles_pre
       << endl << "  #Precharged Power-Up Cycles: " << c.pup_pre_cycles
       << endl << "    #Auto-Refresh Precharged cycles during Self-Refresh Power-Up: " << c.spup_ref_pre_cycles
       << endl << "  #Self-Refresh Power-Up Cycles: "                          << c.spup_cycles
       << endl << "Total Idle Cycles (Active + Precharged): "                  << c.idlecycles_act + c.idlecycles_pre
       << endl << "#Power-Downs: "                                             << c.f_act_pdns +  c.s_act_pdns + c.f_pre_pdns + c.s_pre_pdns
       << endl << "  #Active Fast-exit Power-Downs: "                          << c.f_act_pdns
       << endl << "  #Active Slow-exit Power-Downs: "                          << c.s_act_pdns
       << endl << "  #Precharged Fast-exit Power-Downs: "                      << c.f_pre_pdns
       << endl << "  #Precharged Slow-exit Power-Downs: "                      << c.s_pre_pdns
       << endl << "#Power-Down Cycles: "                                       << c.f_act_pdcycles + c.s_act_pdcycles + c.f_pre_pdcycles + c.s_pre_pdcycles
       << endl << "  #Active Fast-exit Power-Down Cycles: "                    << c.f_act_pdcycles
       << endl << "  #Active Slow-exit Power-Down Cycles: "                    << c.s_act_pdcycles
       << endl << "    #Auto-Refresh Active cycles during Self-Refresh: "      << c.sref_ref_act_cycles
       << endl << "  #Precharged Fast-exit Power-Down Cycles: "                << c.f_pre_pdcycles
       << endl << "  #Precharged Slow-exit Power-Down Cycles: "                << c.s_pre_pdcycles
       << endl << "    #Auto-Refresh Precharged cycles during Self-Refresh: "  << c.sref_ref_pre_cycles
       << endl << "#Auto-Refresh Cycles: "                                     << c.numberofrefs * memTimingSpec.RFC
       << endl << "#Self-Refreshes: "                                          << c.numberofsrefs
       << endl << "#Self-Refresh Cycles: "                                     << c.sref_cycles
       << endl << "----------------------------------------"
       << endl << "Total Trace Length (clock cycles): " << total_cycles
       << endl << "----------------------------------------" << endl;

  cout.precision(2);
  cout << endl << "* Trace Power and Energy Estimates:" << endl
       << endl << "ACT Cmd Energy: " << energy.act_energy   << eUnit
       << endl << "PRE Cmd Energy: " << energy.pre_energy   << eUnit
       << endl << "RD Cmd Energy: "  << energy.read_energy  << eUnit
       << endl << "WR Cmd Energy: "  << energy.write_energy << eUnit;

  if (term) {
    cout << "RD I/O Energy: " << energy.read_io_energy << eUnit << endl;
    // No Termination for LPDDR/2/3 and DDR memories
    if (memSpec.memArchSpec.termination) {
      cout << "WR Termination Energy: " << energy.write_term_energy << eUnit << endl;
    }

    if (nRanks > 1 && memSpec.memArchSpec.termination) {
      cout <<         "RD Termination Energy (Idle rank): " << energy.read_oterm_energy << eUnit
           << endl << "WR Termination Energy (Idle rank): " << energy.write_oterm_energy << eUnit << endl;
    }
  }

  double nRanksDouble = static_cast<double>(nRanks);

  cout <<         "ACT Stdby Energy: "                                                                      << nRanksDouble * energy.act_stdby_energy << eUnit
       << endl << "  Active Idle Energy: "                                                                  << nRanksDouble * energy.idle_energy_act << eUnit
       << endl << "  Active Power-Up Energy: "                                                              << nRanksDouble * energy.pup_act_energy << eUnit
       << endl << "    Active Stdby Energy during Auto-Refresh cycles in Self-Refresh Power-Up: "           << nRanksDouble * energy.spup_ref_act_energy << eUnit
       << endl << "PRE Stdby Energy: "                                                                      << nRanksDouble * energy.pre_stdby_energy << eUnit
       << endl << "  Precharge Idle Energy: "                                                               << nRanksDouble * energy.idle_energy_pre << eUnit
       << endl << "  Precharged Power-Up Energy: "                                                          << nRanksDouble * energy.pup_pre_energy << eUnit
       << endl << "    Precharge Stdby Energy during Auto-Refresh cycles in Self-Refresh Power-Up: "        << nRanksDouble * energy.spup_ref_pre_energy << eUnit
       << endl << "  Self-Refresh Power-Up Energy: "                                                        << nRanksDouble * energy.spup_energy << eUnit
       << endl << "Total Idle Energy (Active + Precharged): "                                               << nRanksDouble * (energy.idle_energy_act + energy.idle_energy_pre) << eUnit
       << endl << "Total Power-Down Energy: "                                                               << nRanksDouble * (energy.f_act_pd_energy + energy.f_pre_pd_energy + energy.s_act_pd_energy + energy.s_pre_pd_energy) << eUnit
       << endl << "  Fast-Exit Active Power-Down Energy: "                                                  << nRanksDouble * energy.f_act_pd_energy << eUnit
       << endl << "  Slow-Exit Active Power-Down Energy: "                                                  << nRanksDouble * energy.s_act_pd_energy << eUnit
       << endl << "    Slow-Exit Active Power-Down Energy during Auto-Refresh cycles in Self-Refresh: "     << nRanksDouble * energy.sref_ref_act_energy << eUnit
       << endl << "  Fast-Exit Precharged Power-Down Energy: "                                              << nRanksDouble * energy.f_pre_pd_energy << eUnit
       << endl << "  Slow-Exit Precharged Power-Down Energy: "                                              << nRanksDouble * energy.s_pre_pd_energy << eUnit
       << endl << "    Slow-Exit Precharged Power-Down Energy during Auto-Refresh cycles in Self-Refresh: " << nRanksDouble * energy.sref_ref_pre_energy << eUnit
       << endl << "Auto-Refresh Energy: "                                                                   << energy.ref_energy << eUnit
       << endl << "Self-Refresh Energy: "                                                                   << nRanksDouble * energy.sref_energy << eUnit
       << endl << "----------------------------------------"
       << endl << "Total Trace Energy: "                                                                    << energy.total_energy << eUnit
       << endl << "Average Power: "                                                                         << power.average_power << " mW"
       << endl << "----------------------------------------" << endl;

  cout.flags(flags);
  cout.precision(precision);
} // MemoryPowerModel::power_print

// Self-refresh active energy estimation (not including background energy)
double MemoryPowerModel::engy_sref(double idd6, double idd3n, double idd5,
                                   double vdd, double sref_cycles, double sref_ref_act_cycles,
                                   double sref_ref_pre_cycles, double spup_ref_act_cycles,
                                   double spup_ref_pre_cycles, double clk)
{
  double sref_energy;

  sref_energy = ((idd6 * sref_cycles) + ((idd5 - idd3n) * (sref_ref_act_cycles
                                                           + spup_ref_act_cycles + sref_ref_pre_cycles + spup_ref_pre_cycles)))
                * vdd * clk;
  return sref_energy;
}

// IO and Termination power calculation based on Micron Power Calculators
// Absolute power measures are obtained from Micron Power Calculator (mentioned in mW)
void MemoryPowerModel::io_term_power(const MemorySpecification& memSpec)
{
  const MemTimingSpec& memTimingSpec     = memSpec.memTimingSpec;
  const MemArchitectureSpec& memArchSpec = memSpec.memArchSpec;
  const MemPowerSpec&  memPowerSpec      = memSpec.memPowerSpec;

  power.IO_power     = memPowerSpec.ioPower;    // in mW
  power.WR_ODT_power = memPowerSpec.wrOdtPower; // in mW

  if (memArchSpec.nbrOfRanks > 1) {
    power.TermRD_power = memPowerSpec.termRdPower; // in mW
    power.TermWR_power = memPowerSpec.termWrPower; // in mW
  }

  if (memPowerSpec.capacitance != 0.0) {
    // If capacity is given, then IO Power depends on DRAM clock frequency.
    power.IO_power = memPowerSpec.capacitance * 0.5 * pow(memPowerSpec.vdd2, 2.0) * memTimingSpec.clkMhz * 1000000;
  }
} // MemoryPowerModel::io_term_power


double MemoryPowerModel::calcIoTermEnergy(int64_t cycles, double period, double power, int64_t numBits) const
{
  return static_cast<double>(cycles) * period * power * static_cast<double>(numBits);
}

// time (t) * current (I) * voltage (V) energy calculation
double EnergyDomain::calcTivEnergy(int64_t cycles, double current) const
{
  return static_cast<double>(cycles) * clkPeriod * current * voltage;
}
