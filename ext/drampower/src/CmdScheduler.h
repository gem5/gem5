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
 *
 */

#ifndef CMDSCHEDULER_H
#define CMDSCHEDULER_H

#include <string>
#include <vector>
#include <functional>  // for binary_function<>
#include <fstream>

#include "MemorySpecification.h"
#include "Utils.h"

namespace Data {
class cmdScheduler {
 public:
        #define READ            0
        #define WRITE           1
        #define ACTIVATE        2
        #define PRECHARGE       3
        #define POWER_DOWN      1
        #define SELF_REFRESH    2

  // the format of a transaction.
  class trans {
   public:
    int64_t type;
    int64_t timeStamp;
    uint64_t logicalAddress;
  };

  std::vector<trans> transTrace; // to store the transactions.

  // the format of physical address.
  class physicalAddr {
   public:
    uint64_t rowAddr;
    uint64_t bankAddr;
    uint64_t bankGroupAddr;
    uint64_t colAddr;
  };

  // the format of a command.
  class commandItem {
   public:
    int64_t Type;
    int64_t bank;
    int64_t time;
    std::string  name;
    physicalAddr PhysicalAddr;
    // sorting the commands according to their scheduling time.
    struct commandItemSorter : public std::binary_function<commandItem&,
                                                           commandItem&, bool>{
      bool operator()(const commandItem& lhs,
                      const commandItem& rhs) const
      {
        return lhs.time < rhs.time;
      }
    };
  };

  commandItem cmd;
  commandItem transFinish; // the last scheduled command for a transaction.
  commandItem PreRDWR;     // the latest scheduled READ or WRITE command.
  // the scheduled ACTIVATE commands are stored in ACT.
  std::vector<commandItem> ACT;
  // PRE is sued to keep recording the time when a precharge occurs.
  std::vector<commandItem> PRE;
  // the scheduled READ or WRITE commands are stored in RDWR.
  std::vector<std::vector<commandItem> > RDWR;
  // all the scheduled commands for a transaction is stored by cmdScheduling.
  std::vector<commandItem> cmdScheduling;
  std::vector<commandItem> cmdList;
  unsigned elements;
  int64_t BI, BC, BGI;

  // the function used to translate a transaction into a sequence of
  // commands which are scheduled to the memory.
  void transTranslation(const MemorySpecification& memSpec,
                        std::ifstream&            trans_trace,
                        int                       grouping,
                        int                       interleaving,
                        int                       burst,
                        int                       powerdown);
  // get the transactions by reading the traces.
  void getTrans(std::ifstream&      pwr_trace,
                const MemorySpecification& memSpec);
  // the initialization function for scheduling.
  void schedulingInitialization(const MemorySpecification& memSpec);
  // the function used to schedule commands according to the timing constraints.
  void analyticalScheduling(const MemorySpecification& memSpec);
  // translate the logical address into physical address.
  physicalAddr memoryMap(trans               Trans,
                         const MemorySpecification& memSpec);
  // the power down and power up are scheduled by pdScheduling
  void pdScheduling(int64_t endTime,
                    int64_t timer,
                    const MemorySpecification& memSpec);
  // get the timings for scheduling a precharge since a read or write command
  // is scheduled.
  int64_t getRWTP(int64_t transType,
              const MemorySpecification& memSpec);
  // get different kind of timing constraints according to the used memory.
  void getTimingConstraints(bool                BGSwitch,
                            const MemorySpecification& memSpec,
                            int64_t                 PreType,
                            int64_t                 CurrentType);

  uint64_t uintLog2(uint64_t in);

  int64_t transTime;
  // the flag for power down.
  int64_t power_down;
  int64_t Inselfrefresh;
  int64_t tRRD_init;
  int64_t tCCD_init;
  int64_t tWTR_init;
  int64_t tREF;
  int64_t tSwitch_init;
  int64_t tRWTP;
  int64_t bankaccess;
  int64_t nBanks;
  int64_t nColumns;
  int64_t burstLength;
  int64_t nbrOfBankGroups;
  bool timingsGet;
  int64_t startTime;

  // the scheduling results for all the transactions are written into
  // commands which will be used by the power analysis part.
  std::ofstream commands;
};
}

#endif // ifndef CMDSCHEDULER_H
