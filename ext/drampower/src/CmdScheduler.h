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
    int type;
    double timeStamp;
    unsigned logicalAddress;
  };

  std::vector<trans> transTrace; // to store the transactions.

  // the format of physical address.
  class physicalAddr {
   public:
    unsigned rowAddr;
    unsigned bankAddr;
    unsigned bankGroupAddr;
    unsigned colAddr;
  };

  // the format of a command.
  class commandItem {
   public:
    int Type;
    int bank;
    double time;
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
  int BI, BC, BGI;

  // the function used to translate a transaction into a sequence of
  // commands which are scheduled to the memory.
  void transTranslation(Data::MemorySpecification memSpec,
                        std::ifstream&            trans_trace,
                        int                       grouping,
                        int                       interleaving,
                        int                       burst,
                        int                       powerdown);
  // get the transactions by reading the traces.
  void getTrans(std::ifstream&      pwr_trace,
                MemorySpecification memSpec);
  // the initialization function for scheduling.
  void schedulingInitialization(MemorySpecification memSpec);
  // the function used to schedule commands according to the timing constraints.
  void analyticalScheduling(MemorySpecification memSpec);
  // translate the logical address into physical address.
  physicalAddr memoryMap(trans               Trans,
                         MemorySpecification memSpec);
  // the power down and power up are scheduled by pdScheduling
  void pdScheduling(double              endTime,
                    double              timer,
                    MemorySpecification memSpec);
  // get the timings for scheduling a precharge since a read or write command
  // is scheduled.
  int getRWTP(int                 transType,
              MemorySpecification memSpec);
  // get different kind of timing constraints according to the used memory.
  void getTimingConstraints(bool                BGSwitch,
                            MemorySpecification memSpec,
                            int                 PreType,
                            int                 CurrentType);

  double transTime;
  // the flag for power down.
  int    power_down;
  int    Inselfrefresh;
  int    tRRD_init;
  int    tCCD_init;
  int    tWTR_init;
  double tREF;
  double tSwitch_init;
  double tRWTP;
  int    bankaccess;
  unsigned nBanks;
  unsigned nColumns;
  unsigned burstLength;
  unsigned nbrOfBankGroups;
  bool timingsGet;
  double   startTime;

  // the scheduling results for all the transactions are written into
  // commands which will be used by the power analysis part.
  std::ofstream commands;
};
}

#endif // ifndef CMDSCHEDULER_H
