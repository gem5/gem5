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
 */

#include <cstdio>
#include <algorithm>  // std::count
#include <string>

#include "CommandAnalysis.h"

using std::cerr;
using std::endl;
using std::string;

using namespace Data;


// To get the time of completion of the issued command
// Derived based on JEDEC specifications

int64_t CommandAnalysis::timeToCompletion(MemCommand::cmds type)
{
  int64_t offset = 0;
  const MemTimingSpec& memTimingSpec     = memSpec.memTimingSpec;
  const MemArchitectureSpec& memArchSpec = memSpec.memArchSpec;

  if (type == MemCommand::RD) {
    offset = memTimingSpec.RL +
                              memTimingSpec.DQSCK + 1 + (memArchSpec.burstLength /
                                                         memArchSpec.dataRate);
  } else if (type == MemCommand::WR) {
    offset = memTimingSpec.WL +
                              (memArchSpec.burstLength / memArchSpec.dataRate) +
                              memTimingSpec.WR;
  } else if (type == MemCommand::ACT) {
    offset = memTimingSpec.RCD;
  } else if ((type == MemCommand::PRE) || (type == MemCommand::PREA)) {
    offset = memTimingSpec.RP;
  }
  return offset;
} // CommandAnalysis::timeToCompletion


// Returns the number of active banks based on the bank_state vector.
unsigned CommandAnalysis::get_num_active_banks(void)
{
  return bank_state.get_num_active_banks();
}

// Naming-standard compliant wrapper
unsigned CommandAnalysis::nActiveBanks(void)
{
  return get_num_active_banks();
}

bool CommandAnalysis::isPrecharged(unsigned bank)
{
    return bank_state.GetByIndex(bank) == BANK_PRECHARGED;
}

void CommandAnalysis::printWarningIfActive(const string& warning, int type, int64_t timestamp, unsigned bank)
{
  if (get_num_active_banks() != 0) {
    printWarning(warning, type, timestamp, bank);
  }
}

void CommandAnalysis::printWarningIfNotActive(const string& warning, int type, int64_t timestamp, unsigned bank)
{
  if (get_num_active_banks() == 0) {
    printWarning(warning, type, timestamp, bank);
  }
}

void CommandAnalysis::printWarningIfPoweredDown(const string& warning, int type, int64_t timestamp, unsigned bank)
{
  if (mem_state != 0) {
    printWarning(warning, type, timestamp, bank);
  }
}

void CommandAnalysis::printWarning(const string& warning, int type, int64_t timestamp, unsigned bank)
{
  cerr << "WARNING: " << warning << endl;
  cerr << "Command: " << type << ", Timestamp: " << timestamp <<
    ", Bank: " << bank << endl;
}
