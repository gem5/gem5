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
 * Authors: Karthik Chandrasekar,
 *          Matthias Jung,
 *          Omar Naji,
 *          Sven Goossens,
 *          Éder F. Zulian
 *          Subash Kannoth
 *          Felipe S. Prado
 *
 */

#include <fstream>
#include <algorithm>
#include <sstream>

#include "CommandAnalysis.h"
#include "CmdScheduler.h"

using namespace Data;
using namespace std;

bool commandSorter(const MemCommand& i, const MemCommand& j)
{
  if (i.getTimeInt64() == j.getTimeInt64()) {
    return i.getType() == MemCommand::PRE && j.getType() != MemCommand::PRE;
  } else {
    return i.getTimeInt64() < j.getTimeInt64();
  }
}

CommandAnalysis::CommandAnalysis(const Data::MemorySpecification& memSpec) :
  memSpec(memSpec)

{
  auto &nBanks = memSpec.memArchSpec.nbrOfBanks;
  // Initializing all counters and variables
  numberofactsBanks.assign(static_cast<size_t>(nBanks), 0);
  numberofpresBanks.assign(static_cast<size_t>(nBanks), 0);
  numberofreadsBanks.assign(static_cast<size_t>(nBanks), 0);
  numberofwritesBanks.assign(static_cast<size_t>(nBanks), 0);
  actcyclesBanks.assign(static_cast<size_t>(nBanks), 0);
  numberofrefbBanks.assign(static_cast<size_t>(nBanks), 0);

  first_act_cycle_banks.resize(static_cast<size_t>(nBanks), 0);

  clearStats(0);
  zero = 0;

  bank_state.resize(static_cast<size_t>(nBanks), BANK_PRECHARGED);
  last_bank_state.resize(static_cast<size_t>(nBanks), BANK_PRECHARGED);
  mem_state  = MS_NOT_IN_PD;

  cmd_list.clear();
  cached_cmd.clear();
  activation_cycle.resize(static_cast<size_t>(nBanks), 0);
  num_banks = nBanks;
}

// function to clear counters
void CommandAnalysis::clearStats(const int64_t timestamp)
{
  std::fill(numberofactsBanks.begin(), numberofactsBanks.end(), 0);
  std::fill(numberofpresBanks.begin(), numberofpresBanks.end(), 0);
  std::fill(numberofreadsBanks.begin(), numberofreadsBanks.end(), 0);
  std::fill(numberofwritesBanks.begin(), numberofwritesBanks.end(), 0);
  std::fill(actcyclesBanks.begin(), actcyclesBanks.end(), 0);

  numberofrefs        = 0;
  f_act_pdns          = 0;
  s_act_pdns          = 0;
  f_pre_pdns          = 0;
  s_pre_pdns          = 0;
  numberofsrefs       = 0;

  actcycles           = 0;
  precycles           = 0;
  f_act_pdcycles      = 0;
  s_act_pdcycles      = 0;
  f_pre_pdcycles      = 0;
  s_pre_pdcycles      = 0;
  pup_act_cycles      = 0;
  pup_pre_cycles      = 0;
  sref_cycles         = 0;
  spup_cycles         = 0;
  sref_ref_act_cycles = 0;
  sref_ref_pre_cycles = 0;
  spup_ref_act_cycles = 0;
  spup_ref_pre_cycles = 0;
  idlecycles_act      = 0;
  idlecycles_pre      = 0;

  // reset count references to timestamp so that they are moved
  // to start of next stats generation
  std::fill(first_act_cycle_banks.begin(), first_act_cycle_banks.end(), timestamp);
  first_act_cycle     = timestamp;

  pdn_cycle           = timestamp;
  sref_cycle_window   = timestamp;

  end_act_op          = timestamp;
  end_read_op         = timestamp;
  end_write_op        = timestamp;

  latest_read_cycle   = -1;
  latest_write_cycle  = -1;

  if (timestamp == 0) {
    latest_pre_cycle = -1;
    latest_act_cycle = -1;
    sref_cycle = 0;
    last_pre_cycle = 0;
    sref_ref_act_cycles_window = 0;
    sref_ref_pre_cycles_window = 0;
  } else {
    last_pre_cycle = max(timestamp,last_pre_cycle);

    latest_pre_cycle = max(timestamp, latest_pre_cycle);

    if (latest_act_cycle < timestamp)
        latest_act_cycle = -1;
  }
}

// function to clear all arrays
void CommandAnalysis::clear()
{
  cached_cmd.clear();
  cmd_list.clear();
  last_bank_state.clear();
  bank_state.clear();
}

// Reads through the trace file, identifies the timestamp, command and bank
// If the issued command includes an auto-precharge, adds an explicit
// precharge to a cached command list and computes the precharge offset from the
// issued command timestamp, when the auto-precharge would kick in

void CommandAnalysis::getCommands(std::vector<MemCommand>& list, bool lastupdate, int64_t timestamp)
{
  if (!next_window_cmd_list.empty()) {
    list.insert(list.begin(), next_window_cmd_list.begin(), next_window_cmd_list.end());
    next_window_cmd_list.clear();
  }
  for (size_t i = 0; i < list.size(); ++i) {
    MemCommand& cmd = list[i];
    MemCommand::cmds cmdType = cmd.getType();
    if (cmdType == MemCommand::ACT) {
      activation_cycle[cmd.getBank()] = cmd.getTimeInt64();
    } else if (cmdType == MemCommand::RDA || cmdType == MemCommand::WRA) {
      // Remove auto-precharge flag from command
      cmd.setType(cmd.typeWithoutAutoPrechargeFlag());

      // Add the auto precharge to the list of cached_cmds
      int64_t preTime = max(cmd.getTimeInt64() + cmd.getPrechargeOffset(memSpec, cmdType),
                           activation_cycle[cmd.getBank()] + memSpec.memTimingSpec.RAS);
      list.push_back(MemCommand(MemCommand::PRE, cmd.getBank(), preTime));
    }

    if (!lastupdate && timestamp > 0) {
      if(cmd.getTimeInt64() > timestamp)
      {
          MemCommand nextWindowCmd = list[i];
          next_window_cmd_list.push_back(nextWindowCmd);
          list.erase(find(list.begin(), list.end(), cmd));
      }
    }
  }
  sort(list.begin(), list.end(), commandSorter);

  if (lastupdate && list.empty() == false) {
    // Add cycles at the end of the list
    int64_t t = timeToCompletion(list.back().getType()) + list.back().getTimeInt64() - 1;
    list.push_back(MemCommand(MemCommand::NOP, 0, t));
  }

  evaluateCommands(list);
} // CommandAnalysis::getCommands


// Used to analyse a given list of commands and identify command timings
// and memory state transitions
void CommandAnalysis::evaluateCommands(vector<MemCommand>& cmd_list)
{
  // for each command identify timestamp, type and bank
  for (auto cmd : cmd_list) {
    // For command type
    int type = cmd.getType();
    // For command bank
    unsigned bank = cmd.getBank();
    // Command Issue timestamp in clock cycles (cc)
    int64_t timestamp = cmd.getTimeInt64();

    if (type == MemCommand::ACT) {
      handleAct(bank, timestamp);
    } else if (type == MemCommand::RD) {
      handleRd(bank, timestamp);
    } else if (type == MemCommand::WR) {
      handleWr(bank, timestamp);
    } else if (type == MemCommand::REF) {
      handleRef(bank, timestamp);
    } else if (type == MemCommand::REFB) {
      handleRefB(bank, timestamp);
    } else if (type == MemCommand::PRE) {
      handlePre(bank, timestamp);
    } else if (type == MemCommand::PREA) {
      handlePreA(bank, timestamp);
    } else if (type == MemCommand::PDN_F_ACT) {
      handlePdnFAct(bank, timestamp);
    } else if (type == MemCommand::PDN_S_ACT) {
      handlePdnSAct(bank, timestamp);
    } else if (type == MemCommand::PDN_F_PRE) {
      handlePdnFPre(bank, timestamp);
    } else if (type == MemCommand::PDN_S_PRE) {
      handlePdnSPre(bank, timestamp);
    } else if (type == MemCommand::PUP_ACT) {
      handlePupAct(timestamp);
    } else if (type == MemCommand::PUP_PRE) {
      handlePupPre(timestamp);
    } else if (type == MemCommand::SREN) {
      handleSREn(bank, timestamp);
    } else if (type == MemCommand::SREX) {
      handleSREx(bank, timestamp);
    } else if (type == MemCommand::END || type == MemCommand::NOP) {
      handleNopEnd(timestamp);
    } else {
      printWarning("Unknown command given, exiting.", type, timestamp, bank);
      exit(-1);
    }
  }
} // CommandAnalysis::evaluateCommands

// To update idle period information whenever active cycles may be idle
void CommandAnalysis::idle_act_update(int64_t latest_read_cycle, int64_t latest_write_cycle,
                                      int64_t latest_act_cycle, int64_t timestamp)
{
  if (latest_read_cycle >= 0) {
    end_read_op = latest_read_cycle + timeToCompletion(MemCommand::RD) - 1;
  }

  if (latest_write_cycle >= 0) {
    end_write_op = latest_write_cycle + timeToCompletion(MemCommand::WR) - 1;
  }

  if (latest_act_cycle >= 0) {
    end_act_op = latest_act_cycle + timeToCompletion(MemCommand::ACT) - 1;
  }

  idlecycles_act += max(zero, timestamp - max(max(end_read_op, end_write_op),
                                              end_act_op));
} // CommandAnalysis::idle_act_update

// To update idle period information whenever precharged cycles may be idle
void CommandAnalysis::idle_pre_update(int64_t timestamp, int64_t latest_pre_cycle)
{
  if (latest_pre_cycle > 0) {
    idlecycles_pre += max(zero, timestamp - latest_pre_cycle -
                          memSpec.memTimingSpec.RP);
  } else if (latest_pre_cycle == 0) {
    idlecycles_pre += max(zero, timestamp - latest_pre_cycle);
  }
}

