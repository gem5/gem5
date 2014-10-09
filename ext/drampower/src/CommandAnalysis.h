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

#ifndef COMMAND_TIMINGS_H
#define COMMAND_TIMINGS_H

#include <stdint.h>

#include <vector>
#include <iostream>
#include <deque>
#include <string>

#include "MemCommand.h"
#include "MemorySpecification.h"
#include "Utils.h"

namespace Data {
class CommandAnalysis {
 public:
  // Power-Down and Self-refresh related memory states
  enum memstate {
    MS_PDN_F_ACT = 10, MS_PDN_S_ACT = 11, MS_PDN_F_PRE = 12,
    MS_PDN_S_PRE = 13, MS_SREF = 14
  };

  CommandAnalysis();

  // Returns number of reads, writes, acts, pres and refs in the trace
  CommandAnalysis(const int nbrofBanks);

  // Number of activate commands
  int64_t numberofacts;
  // Number of precharge commands
  int64_t numberofpres;
  // Number of reads commands
  int64_t numberofreads;
  // Number of writes commands
  int64_t numberofwrites;
  // Number of refresh commands
  int64_t numberofrefs;
  // Number of precharge cycles
  int64_t precycles;
  // Number of active cycles
  int64_t actcycles;
  // Number of Idle cycles in the active state
  int64_t idlecycles_act;
  // Number of Idle cycles in the precharge state
  int64_t idlecycles_pre;
  // Number of fast-exit activate power-downs
  int64_t f_act_pdns;
  // Number of slow-exit activate power-downs
  int64_t s_act_pdns;
  // Number of fast-exit precharged power-downs
  int64_t f_pre_pdns;
  // Number of slow-exit activate power-downs
  int64_t s_pre_pdns;
  // Number of self-refresh commands
  int64_t numberofsrefs;
  // Number of clock cycles in fast-exit activate power-down mode
  int64_t f_act_pdcycles;
  // Number of clock cycles in slow-exit activate power-down mode
  int64_t s_act_pdcycles;
  // Number of clock cycles in fast-exit precharged power-down mode
  int64_t f_pre_pdcycles;
  // Number of clock cycles in slow-exit precharged power-down mode
  int64_t s_pre_pdcycles;
  // Number of clock cycles in self-refresh mode
  int64_t sref_cycles;
  // Number of clock cycles in activate power-up mode
  int64_t pup_act_cycles;
  // Number of clock cycles in precharged power-up mode
  int64_t pup_pre_cycles;
  // Number of clock cycles in self-refresh power-up mode
  int64_t spup_cycles;

  // Number of active auto-refresh cycles in self-refresh mode
  int64_t sref_ref_act_cycles;
  // Number of precharged auto-refresh cycles in self-refresh mode
  int64_t sref_ref_pre_cycles;
  // Number of active auto-refresh cycles during self-refresh exit
  int64_t spup_ref_act_cycles;
  // Number of precharged auto-refresh cycles during self-refresh exit
  int64_t spup_ref_pre_cycles;

  // function for clearing arrays
  void clear();

  // To identify auto-precharges
  void getCommands(const MemorySpecification& memSpec,
                   const int
                   nbrofBanks,
                   std::vector<MemCommand>&   list,
                   bool                       lastupdate);

 private:
  unsigned init;
  int64_t  zero;
  unsigned pop;
  // Cached last read command from the file
  std::vector<MemCommand> cached_cmd;

  // Stores the memory commands for analysis
  std::vector<MemCommand> cmd_list;

  // Stores all memory commands for analysis
  std::vector<MemCommand> full_cmd_list;

  // To save states of the different banks, before entering active
  // power-down mode (slow/fast-exit).
  std::vector<int> last_states;
  // Bank state vector
  std::vector<int> bankstate;

  std::vector<int64_t> activation_cycle;
  // To keep track of the last ACT cycle
  int64_t latest_act_cycle;
  // To keep track of the last PRE cycle
  int64_t latest_pre_cycle;
  // To keep track of the last READ cycle
  int64_t latest_read_cycle;
  // To keep track of the last WRITE cycle
  int64_t latest_write_cycle;

  // To calculate end of READ operation
  int64_t end_read_op;
  // To calculate end of WRITE operation
  int64_t end_write_op;
  // To calculate end of ACT operation
  int64_t end_act_op;

  // Clock cycle when self-refresh was issued
  int64_t sref_cycle;

  // Clock cycle when the latest power-down was issued
  int64_t pdn_cycle;

  // Memory State
  unsigned mem_state;

  // Clock cycle of first activate command when memory state changes to ACT
  int64_t first_act_cycle;

  // Clock cycle of last precharge command when memory state changes to PRE
  int64_t last_pre_cycle;
  // To collect and analyse all commands including auto-precharges
  void analyse_commands(const int nbrofBanks,
                        Data::MemorySpecification
                        memSpec,
                        int64_t    nCommands,
                        int64_t    nCached,
                        bool      lastupdate);
  // To perform timing analysis of a given set of commands and update command counters
  void evaluate(const MemorySpecification& memSpec,
                std::vector<MemCommand>&   cmd_list,
                int                        nbrofBanks);

  // To calculate time of completion of any issued command
  int timeToCompletion(const MemorySpecification& memSpec,
                       MemCommand::cmds           type);

  // To update idle period information whenever active cycles may be idle
  void idle_act_update(const MemorySpecification& memSpec,
                       int64_t                     latest_read_cycle,
                       int64_t                     latest_write_cycle,
                       int64_t                     latest_act_cycle,
                       int64_t                     timestamp);

  // To update idle period information whenever precharged cycles may be idle
  void idle_pre_update(const MemorySpecification& memSpec,
                       int64_t                     timestamp,
                       int64_t                     latest_pre_cycle);

  void printWarningIfActive(const std::string& warning, int type, int64_t timestamp, int bank);
  void printWarningIfNotActive(const std::string& warning, int type, int64_t timestamp, int bank);
  void printWarning(const std::string& warning, int type, int64_t timestamp, int bank);
};
}
#endif // ifndef COMMAND_TIMINGS_H
