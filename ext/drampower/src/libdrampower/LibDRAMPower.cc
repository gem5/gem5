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
 * Authors: Matthias Jung, Omar Naji
 *
 */

#include "LibDRAMPower.h"

using namespace Data;

libDRAMPower::libDRAMPower(const MemorySpecification& memSpec, bool includeIoAndTermination) :
  memSpec(memSpec),
  counters(CommandAnalysis(memSpec.memArchSpec.nbrOfBanks)),
  includeIoAndTermination(includeIoAndTermination)
{
}

libDRAMPower::~libDRAMPower()
{
}

void libDRAMPower::doCommand(MemCommand::cmds type, int bank, int64_t timestamp)
{
  MemCommand cmd(type, static_cast<unsigned>(bank), timestamp);
  cmdList.push_back(cmd);
}

void libDRAMPower::updateCounters(bool lastUpdate)
{
  counters.getCommands(memSpec, cmdList, lastUpdate);
  cmdList.clear();
}

void libDRAMPower::calcEnergy()
{
  mpm.power_calc(memSpec, counters, includeIoAndTermination);
}

void libDRAMPower::clearState()
{
  counters.clear();
}

void libDRAMPower::clearCounters(int64_t timestamp)
{
  counters.clearStats(timestamp);
}

const Data::MemoryPowerModel::Energy& libDRAMPower::getEnergy() const
{
  return mpm.energy;
}

const Data::MemoryPowerModel::Power& libDRAMPower::getPower() const
{
  return mpm.power;
}
