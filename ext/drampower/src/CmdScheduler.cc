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
#include "CmdScheduler.h"

#include <cassert>
#include <cmath>  // For log2

#include <algorithm>  // For max


using namespace std;
using namespace Data;

// Read the traces and get the transaction. Each transaction is executed by
// scheduling a number of commands to the memory. Hence, the transactions are
// translated into a sequence of commands which will be used for power analysis.
void cmdScheduler::transTranslation(MemorySpecification memSpec,
                                    ifstream& trans_trace, int grouping, int interleaving, int burst, int powerdown)
{
  commands.open("commands.trace", ifstream::out);
  MemArchitectureSpec& memArchSpec = memSpec.memArchSpec;
  nBanks          = memArchSpec.nbrOfBanks;
  nColumns        = memArchSpec.nbrOfColumns;
  burstLength     = memArchSpec.burstLength;
  nbrOfBankGroups = memArchSpec.nbrOfBankGroups;

  BGI             = grouping;
  BI = interleaving;
  BC = burst;
  power_down      = powerdown;

  schedulingInitialization(memSpec);
  getTrans(trans_trace, memSpec);

  trans_trace.close();
  commands.close();
  ACT.erase(ACT.begin(), ACT.end());
  PRE.erase(PRE.begin(), PRE.end());
  RDWR.erase(RDWR.begin(), RDWR.end());
  cmdScheduling.erase(cmdScheduling.begin(), cmdScheduling.end());
  cmdList.erase(cmdList.begin(), cmdList.end());
  transTrace.erase(transTrace.begin(), transTrace.end());
} // cmdScheduler::transTranslation

// initialize the variables and vectors for starting command scheduling.
void cmdScheduler::schedulingInitialization(MemorySpecification memSpec)
{
  MemTimingSpec& memTimingSpec = memSpec.memTimingSpec;

  ACT.resize(2 * memSpec.memArchSpec.nbrOfBanks);
  RDWR.resize(2 * memSpec.memArchSpec.nbrOfBanks);
  PRE.resize(memSpec.memArchSpec.nbrOfBanks);
  bankaccess = memSpec.memArchSpec.nbrOfBanks;
  if (!ACT.empty()) {
    ACT.erase(ACT.begin(), ACT.end());
  }
  if (!PRE.empty()) {
    PRE.erase(PRE.begin(), PRE.end());
  }
  if (!RDWR.empty()) {
    RDWR.erase(RDWR.begin(), RDWR.end());
  }

  ///////////////initialization//////////////
  for (unsigned i = 0; i < memSpec.memArchSpec.nbrOfBanks; i++) {
    cmd.Type = PRECHARGE;
    cmd.bank = i;
    cmd.name = "PRE";
    if (memSpec.id == "WIDEIO_SDR")
      cmd.time = 1 - static_cast<double>(memSpec.memTimingSpec.TAW);
    else
      cmd.time = 1 - static_cast<double>(memSpec.memTimingSpec.FAW);

    PRE.push_back(cmd);

    cmd.Type = ACTIVATE;
    cmd.name = "ACT";
    ACT.push_back(cmd);

    cmd.Type = WRITE;
    cmd.name = "WRITE";
    cmd.time = -1;
    RDWR[i].push_back(cmd);
  }
  tREF             = memTimingSpec.REFI;
  transFinish.time = 0;
  transFinish.bank = 0;

  PreRDWR.bank     = -1;
  PreRDWR.Type     = READ;
  PreRDWR.name     = "RD";
  PreRDWR.time     = -1;
  startTime        = 0;
} // cmdScheduler::schedulingInitialization

// transactions are generated according to the information read from the traces.
// Then the command scheduling function is triggered to generate commands and
// schedule them to the memory according to the timing constraints.
void cmdScheduler::getTrans(std::ifstream& trans_trace, MemorySpecification memSpec)
{
  std::string line;

  transTime = 0;
  unsigned newtranstime;
  unsigned transAddr;
  unsigned transType = 1;
  trans    TransItem;

  if (!transTrace.empty()) {
    transTrace.erase(transTrace.begin(), transTrace.end());
  }

  while (getline(trans_trace, line)) {
    istringstream linestream(line);
    string item;
    unsigned itemnum = 0;
    while (getline(linestream, item, ',')) {
      if (itemnum == 0) {
        stringstream timestamp(item);
        timestamp >> newtranstime;
        transTime = transTime + newtranstime;
      } else if (itemnum == 1) {
        if (item  == "write" || item == "WRITE") {
          transType = WRITE;
        } else   {
          transType = READ;
        }
      } else if (itemnum == 2) {
        stringstream timestamp(item);
        timestamp >> std::hex >> transAddr;
      }
      itemnum++;
    }
    // generate a transaction
    TransItem.timeStamp      = transTime;
    TransItem.logicalAddress = transAddr;
    TransItem.type           = transType;

    transTrace.push_back(TransItem);

    if (transTrace.size() == MILLION) {
      // The scheduling is implemented for every MILLION transactions.
      // It is used to reduce the used memory during the running of this tool.
      analyticalScheduling(memSpec);
      transTrace.erase(transTrace.begin(), transTrace.end());
    }
  }

  if ((transTrace.size() < MILLION) && (!transTrace.empty())) {
    analyticalScheduling(memSpec);
    transTrace.erase(transTrace.begin(), transTrace.end());
  }
} // cmdScheduler::getTrans

// Transactions are executed individually and the command scheduling is
// independent between transactions. The commands for a new transaction cannot
// be scheduled until all the commands for the current one are scheduled.
// After the scheduling, a sequence of commands are obtained and they are written
// into commands.txt which will be used for power analysis.
void cmdScheduler::analyticalScheduling(MemorySpecification memSpec)
{
  int  Bs               = -1;
  int  transType        = -1;
  double timer          = 0;
  int  bankGroupPointer = 0;
  int  bankGroupAddr    = 0;
  bool collisionFound;
  physicalAddr PhysicalAddress;
  bool bankGroupSwitch  = false;
  std::vector<unsigned> bankPointer(nbrOfBankGroups, 0);
  std::vector<int>  bankAccessNum(nBanks, -1);
  std::vector<bool> ACTSchedule(nBanks, false);
  int bankAddr       = -1;
  double endTime     = 0;
  double tComing_REF = 0;

  Inselfrefresh = 0;

  MemTimingSpec& memTimingSpec = memSpec.memTimingSpec;

  for (unsigned t = 0; t < transTrace.size(); t++) {
    cmdScheduling.erase(cmdScheduling.begin(), cmdScheduling.end());

    for (unsigned i = 0; i < nBanks; i++) {
      ACTSchedule[i]   = false;
      bankAccessNum[i] = -1;
    }

    timingsGet      = false;
    timer           = transTrace[t].timeStamp;

    PhysicalAddress = memoryMap(transTrace[t], memSpec);

    for (unsigned i = 0; i < nbrOfBankGroups; i++) {
      bankPointer[i] = PhysicalAddress.bankAddr; // the bank pointer per group.
    }
    bankGroupPointer = PhysicalAddress.bankGroupAddr;

    endTime          = max(transFinish.time, PRE[transFinish.bank].time +
                           static_cast<int>(memTimingSpec.RP));

    // Before starting the scheduling for the next transaction, it has to
    // check whether it is necessary for implementing power down.
    if (power_down == SELF_REFRESH)
      pdScheduling(endTime, timer, memSpec);
    else if (power_down == POWER_DOWN)
      pdScheduling(endTime, min(timer, tREF), memSpec);

    tComing_REF = tREF;

    ///////////////Scheduling Refresh////////////////////////
    if (((transFinish.time >= tREF) || (timer >= tREF))) {
      for (double i = 0; i <= ((timer - tComing_REF) > 0 ? (timer - tComing_REF) /
                               memTimingSpec.REFI : 0); i++) {
        cmd.bank = 0;
        cmd.name = "REF";
        cmd.time = max(max(max(transFinish.time, PRE[transFinish.bank].time
                               + static_cast<int>(memTimingSpec.RP)), tREF), startTime);
        if (((power_down == SELF_REFRESH) && !Inselfrefresh) ||
            (power_down != SELF_REFRESH)) {
          cmdScheduling.push_back(cmd);
          startTime = cmd.time + memTimingSpec.RFC;
        }
        tREF = tREF + memTimingSpec.REFI;
        // during the refreshing, power down should be taken into account.
        if (!Inselfrefresh)
          pdScheduling(endTime, min(timer, tREF), memSpec);
      }
    }
    ///////////////Execution Transactions///////////////////
    Bs        = PhysicalAddress.bankAddr;
    transType = transTrace[t].type;

    tRWTP     = getRWTP(transType, memSpec);

    for (int i = 0; i < BI; i++) {
      for (int k = 0; k < BC; k++) {
        if (memSpec.memoryType == MemoryType::DDR4) {
          bankGroupPointer = PhysicalAddress.bankGroupAddr;
        }

        for (int j = 0; j < BGI; j++) {
          bankGroupSwitch = false;
          if (memSpec.memoryType == MemoryType::DDR4) {
            if (bankGroupPointer != bankGroupAddr) {
              bankGroupSwitch = true;
            }
            // update to the current bank group address.
            bankGroupAddr = PhysicalAddress.bankGroupAddr + j;
            bankAddr      = bankGroupAddr * nBanks / nbrOfBankGroups +
                            bankPointer[bankGroupAddr];
          } else   {
            bankAddr = Bs + i;
          }

          if (!timingsGet) {
            getTimingConstraints(bankGroupSwitch, memSpec,
                                 PreRDWR.Type, transType);
          }

          ////////////////ACT Scheduling///////////////////
          if (!ACTSchedule[bankAddr]) {
            cmd.bank                  = bankAddr;
            cmd.PhysicalAddr.bankAddr = cmd.bank;
            cmd.PhysicalAddr.rowAddr  = PhysicalAddress.rowAddr;
            cmd.Type                  = ACTIVATE;
            cmd.name                  = "ACT";
            Inselfrefresh             = 0;
            cmd.time                  = max(max(ACT[bankaccess - 1].time + tRRD_init,
                                                PRE[cmd.bank].time + static_cast<int>(memTimingSpec.RP)),
                                            ACT[bankaccess - 4].time +
                                            static_cast<int>(memTimingSpec.FAW));

            if (memSpec.memoryType == MemoryType::WIDEIO_SDR) {
              cmd.time = max(max(ACT[bankaccess - 1].time + tRRD_init,
                                 PRE[cmd.bank].time + static_cast<int>(memTimingSpec.RP)),
                             ACT[bankaccess - 2].time +
                             static_cast<int>(memTimingSpec.TAW));
            }

            if ((i == 0) && (j == 0)) {
              cmd.time = max(cmd.time, PreRDWR.time + 1);
              cmd.time = max(cmd.time, timer);
              cmd.time = max(startTime, cmd.time);
            }

            //////////collision detection////////////////////
            for (int n = 1; n <= i * BGI + j; n++) {
              collisionFound = false;
              for (unsigned m = 0; m < RDWR[bankaccess - n].size(); m++) {
                if (RDWR[bankaccess - n][m].time == cmd.time) {
                  cmd.time      += 1; // ACT is shifted
                  collisionFound = true;
                  break;
                }
              }
              if (collisionFound) {
                break;
              }
            }

            ACT.push_back(cmd);
            cmdScheduling.push_back(cmd);

            ACTSchedule[bankAddr]   = true;
            bankAccessNum[bankAddr] = bankaccess;
            bankaccess++;
          }

          /////RDWR Scheduling//////
          cmd.bank = bankAddr;
          cmd.PhysicalAddr.bankAddr = cmd.bank;
          cmd.PhysicalAddr.rowAddr  = PhysicalAddress.rowAddr;
          cmd.PhysicalAddr.colAddr  = PhysicalAddress.colAddr + k * burstLength;
          cmd.Type = transType;
          switch (transType) {
          case READ:
            cmd.name = "RD";
            break;

          case WRITE:
            cmd.name = "WR";
            break;
          }
          for (int ACTBank = static_cast<int>(ACT.size() - 1);
               ACTBank >= 0; ACTBank--) {
            if (ACT[ACTBank].bank == bankAddr) {
              cmd.time = max(PreRDWR.time + tSwitch_init, ACT.back().time
                             + static_cast<int>(memTimingSpec.RCD));
              break;
            }
          }

          if ((i == BI - 1) && (k == BC - 1) && (j == BGI - 1)) {
            transFinish.time = cmd.time + 1;
            transFinish.bank = bankAddr;
          }
          if (k == BC - 1) {
            switch (transType) {
            case READ:
              cmd.name = "RDA";
              break;

            case WRITE:
              cmd.name = "WRA";
              break;
            }
          }
          PreRDWR = cmd;

          RDWR[bankAccessNum[bankAddr]].push_back(cmd);
          cmdScheduling.push_back(cmd);

          ////////////////PRE Scheduling////////////////////
          if (k == BC - 1) {
            PRE[bankAddr].bank = bankAddr;
            PRE[bankAddr].Type = PRECHARGE;
            PRE[bankAddr].name = "PRE";
            for (int ACTBank = static_cast<int>(ACT.size() - 1);
                 ACTBank >= 0; ACTBank--) {
              if (ACT[ACTBank].bank == bankAddr) {
                PRE[bankAddr].time = max(ACT.back().time +
                                         static_cast<int>(memTimingSpec.RAS),
                                         PreRDWR.time + tRWTP);
                break;
              }
            }
            bankPointer[bankGroupAddr] = bankPointer[bankGroupAddr] + 1;
          }

          bankGroupPointer++;
        }
      }
    }

    // make sure the scheduled commands are stored with an ascending scheduling time
    sort(cmdScheduling.begin(), cmdScheduling.end(),
         commandItem::commandItemSorter());

    // write the scheduled commands into commands.txt.
    for (unsigned i = 0; i < cmdScheduling.size(); i++) {
      cmdList.push_back(cmdScheduling[i]);
    }

    /////////////Update Vector Length/////////////////
    // the vector length is reduced so that less memory is used for running
    // this tool.
    if (ACT.size() >= memSpec.memArchSpec.nbrOfBanks) {
      for (int m = 0; m < BI * BGI; m++) {
        ACT.erase(ACT.begin());
        RDWR[0].erase(RDWR[0].begin(), RDWR[0].end());
        for (int h = 0; h < bankaccess - 1 - m; h++) {
          RDWR[h].insert(RDWR[h].begin(), RDWR[h + 1].begin(), RDWR[h + 1].end());
          RDWR[h + 1].resize(0);
        }
      }
      bankaccess = bankaccess - (BI * BGI);
    }
  }

  for (unsigned j = 0; j < cmdList.size(); j++) {
    commands.precision(0);
    commands << fixed << cmdList[j].time << "," << cmdList[j].name << "," <<
      cmdList[j].bank << endl;
  }
  cmdList.erase(cmdList.begin(), cmdList.end());
} // cmdScheduler::analyticalScheduling

// to add the power down/up during the command scheduling for transactions.
// It is called when the command scheduling for a transaction is finished, and it
// is also called if there is a refresh.
void cmdScheduler::pdScheduling(double endTime, double timer,
                                MemorySpecification memSpec)
{
  double ZERO = 0;
  MemTimingSpec& memTimingSpec = memSpec.memTimingSpec;

  endTime = max(endTime, startTime);
  double pdTime = max(ZERO, timer - endTime);

  if ((timer > (endTime + memTimingSpec.CKE)) && (power_down == POWER_DOWN)) {
    cmd.bank = 0;
    cmd.name = "PDN_S_PRE";
    cmd.time = endTime;
    cmdScheduling.push_back(cmd);
    cmd.name = "PUP_PRE";

    if (pdTime > memTimingSpec.REFI)
      cmd.time = cmd.time + memTimingSpec.REFI;
    else
      cmd.time = cmd.time + pdTime;

    if (memSpec.memoryType.isLPDDRFamily())
      startTime = cmd.time + memTimingSpec.XP;
    else
      startTime = cmd.time + memTimingSpec.XPDLL - memTimingSpec.RCD;

    cmdScheduling.push_back(cmd);
  } else if ((timer > (endTime + memTimingSpec.CKESR)) && (power_down == SELF_REFRESH))    {
    cmd.bank      = 0;
    cmd.name      = "SREN";
    cmd.time      = endTime;
    cmdScheduling.push_back(cmd);
    Inselfrefresh = 1;
    cmd.name      = "SREX";
    cmd.time      = cmd.time + pdTime;

    if (memSpec.memoryType.isLPDDRFamily())
      startTime = cmd.time + memTimingSpec.XS;
    else
      startTime = cmd.time + memTimingSpec.XSDLL - memTimingSpec.RCD;

    cmdScheduling.push_back(cmd);
  }
} // cmdScheduler::pdScheduling

// get the time when a precharge occurs after a read/write command is scheduled.
// In addition, it copes with different kind of memories.
int cmdScheduler::getRWTP(int transType, MemorySpecification memSpec)
{
  int tRWTP_init = 0;
  MemTimingSpec& memTimingSpec     = memSpec.memTimingSpec;
  MemArchitectureSpec& memArchSpec = memSpec.memArchSpec;

  if (transType == READ) {
    switch (memSpec.memoryType) {
    case MemoryType::LPDDR:
    case MemoryType::WIDEIO_SDR:
      tRWTP_init = memArchSpec.burstLength / memArchSpec.dataRate;
      break;

    case MemoryType::LPDDR2:
    case MemoryType::LPDDR3:
      tRWTP_init = memArchSpec.burstLength / memArchSpec.dataRate +
                   max(0, static_cast<int>(memTimingSpec.RTP - 2));
      break;

    case MemoryType::DDR2:
      tRWTP_init = memTimingSpec.AL + memArchSpec.burstLength /
                   memArchSpec.dataRate +
                   max(static_cast<int>(memTimingSpec.RTP), 2) - 2;
      break;

    case MemoryType::DDR3:
    case MemoryType::DDR4:
      tRWTP_init = memTimingSpec.RTP;
      break;
    default:
      assert("Unknown memory type" && false);
    } // switch
  } else if (transType == WRITE)    {
    if (memSpec.memoryType == MemoryType::WIDEIO_SDR) {
      tRWTP_init = memTimingSpec.WL + memArchSpec.burstLength /
                   memArchSpec.dataRate - 1 + memSpec.memTimingSpec.WR;
    } else   {
      tRWTP_init = memTimingSpec.WL + memArchSpec.burstLength /
                   memArchSpec.dataRate + memSpec.memTimingSpec.WR;
    }
    if ((memSpec.memoryType == MemoryType::LPDDR2) ||
        (memSpec.memoryType == MemoryType::LPDDR3)) {
      tRWTP_init = tRWTP_init + 1;
    }
  }

  return tRWTP_init;
} // cmdScheduler::getRWTP

// get the timings for command scheduling according to different memories.
// In particular, tSwitch_init is generally used to provide the timings for
// scheduling a read/write command after a read/write command which have been
// scheduled to any possible banks within any possible bank groups (DDR4).
void cmdScheduler::getTimingConstraints(bool BGSwitch, MemorySpecification memSpec,
                                        int PreType, int CurrentType)
{
  MemTimingSpec& memTimingSpec     = memSpec.memTimingSpec;
  MemArchitectureSpec& memArchSpec = memSpec.memArchSpec;

  if (memSpec.memoryType != MemoryType::DDR4) {
    tRRD_init = memTimingSpec.RRD;
    if (PreType == CurrentType) {
      tSwitch_init = memTimingSpec.CCD;
      timingsGet   = true;
    }

    if ((PreType == WRITE) && (CurrentType == READ)) {
      if (memSpec.memoryType == MemoryType::WIDEIO_SDR) {
        tSwitch_init = memTimingSpec.WL + memArchSpec.burstLength /
                       memArchSpec.dataRate - 1 + memTimingSpec.WTR;
      } else   {
        tSwitch_init = memTimingSpec.WL + memArchSpec.burstLength /
                       memArchSpec.dataRate + memTimingSpec.WTR;
      }

      if ((memSpec.memoryType == MemoryType::LPDDR2) ||
          (memSpec.memoryType == MemoryType::LPDDR3)) {
        tSwitch_init = tSwitch_init + 1;
      }
    }
  }

  if (memSpec.memoryType == MemoryType::DDR4) {
    if (BGSwitch) {
      tCCD_init = memTimingSpec.CCD_S;
      tRRD_init = memTimingSpec.RRD_S;
      tWTR_init = memTimingSpec.WTR_S;
    } else   {
      tCCD_init = memTimingSpec.CCD_L;
      tRRD_init = memTimingSpec.RRD_L;
      tWTR_init = memTimingSpec.WTR_L;
    }

    if (PreType == CurrentType) {
      tSwitch_init = tCCD_init;
      timingsGet   = true;
    } else if ((PreType == WRITE) && (CurrentType == READ)) {
      tSwitch_init = memTimingSpec.WL + memArchSpec.burstLength /
                     memArchSpec.dataRate + tWTR_init;
    }
  }

  if ((PreType == READ) && (CurrentType == WRITE)) {
    tSwitch_init = memTimingSpec.RL + memArchSpec.burstLength /
                   memArchSpec.dataRate + 2 - memTimingSpec.WL;
  }
} // cmdScheduler::getTimingConstraints

// The logical address of each transaction is translated into a physical address
// which consists of bank group (for DDR4), bank, row and column addresses.
cmdScheduler::physicalAddr cmdScheduler::memoryMap(trans               Trans,
                                                   MemorySpecification memSpec)
{
  int DecLogic;
  physicalAddr PhysicalAddr;

  DecLogic = Trans.logicalAddress;

  // row-bank-column-BI-BC-BGI-BL
  if ((BGI > 1) && (memSpec.memoryType == MemoryType::DDR4)) {
    unsigned colBits   = static_cast<unsigned>(log2(nColumns));
    unsigned bankShift = static_cast<unsigned>(colBits + ((BI > 1) ? log2(BI) : 0)
                                               + ((BGI > 1) ? log2(BGI) : 0));
    unsigned bankMask  = static_cast<unsigned>(nBanks / (BI * nbrOfBankGroups) - 1)
                        << bankShift;
    unsigned bankAddr  = (DecLogic & bankMask) >>
                         static_cast<unsigned>(colBits + ((BGI > 1) ? log2(BGI) : 0));
    PhysicalAddr.bankAddr = bankAddr;

    unsigned bankGroupShift = static_cast<unsigned>(log2(burstLength));
    unsigned bankGroupMask  = (nbrOfBankGroups / BGI - 1) << bankGroupShift;
    unsigned bankGroupAddr  = (DecLogic & bankGroupMask) >> bankGroupShift;
    PhysicalAddr.bankGroupAddr = bankGroupAddr;

    unsigned colShift       = static_cast<unsigned>(log2(BC * burstLength) +
                                                    ((BI > 1) ? log2(BI) : 0) + ((BGI > 1) ? log2(BGI) : 0));
    unsigned colMask        = static_cast<unsigned>(nColumns / (BC * burstLength) - 1)
                       << colShift;
    unsigned colAddr        = (DecLogic & colMask) >>
                              static_cast<unsigned>((colShift - log2(static_cast<unsigned>(BC) * burstLength)));
    PhysicalAddr.colAddr = colAddr;
  } else   {
    unsigned colBits   = static_cast<unsigned>(log2(nColumns));
    unsigned bankShift = static_cast<unsigned>(colBits + ((BI > 1) ? log2(BI) : 0));
    unsigned bankMask  = static_cast<unsigned>(nBanks / BI - 1) << bankShift;
    unsigned bankAddr  = (DecLogic & bankMask) >> colBits;
    PhysicalAddr.bankAddr = bankAddr;

    unsigned colShift  = static_cast<unsigned>(log2(BC * burstLength) +
                                               ((BI > 1) ? log2(BI) : 0));
    unsigned colMask   = static_cast<unsigned>(nColumns / (BC * burstLength) - 1)
                       << colShift;
    unsigned colAddr   = (DecLogic & colMask) >>
                         static_cast<unsigned>((colShift - log2(static_cast<unsigned>(BC) * burstLength)));
    PhysicalAddr.colAddr       = colAddr;

    PhysicalAddr.bankGroupAddr = 0;
  }

  unsigned rowShift = static_cast<unsigned>(log2(nColumns * nBanks));
  unsigned rowMask  = static_cast<unsigned>(memSpec.memArchSpec.nbrOfRows - 1)
                     << rowShift;
  unsigned rowAddr  = (DecLogic & rowMask) >> rowShift;
  PhysicalAddr.rowAddr = rowAddr;

  return PhysicalAddr;
} // cmdScheduler::memoryMap
