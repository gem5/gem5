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

#include "MemCommand.h"

#include <algorithm>  // for max

#include "MemorySpecification.h"

using namespace Data;
using namespace std;


MemCommand::MemCommand(MemCommand::cmds type,
                       unsigned bank, int64_t timestamp) :
  type(type),
  bank(bank),
  timestamp(timestamp)
{
}

void MemCommand::setType(MemCommand::cmds _type)
{
  type = _type;
}

MemCommand::cmds MemCommand::getType() const
{
  return type;
}

void MemCommand::setBank(unsigned _bank)
{
  bank = _bank;
}

unsigned MemCommand::getBank() const
{
  return bank;
}

// For auto-precharge with read or write - to calculate cycle of precharge
int64_t MemCommand::getPrechargeOffset(const MemorySpecification& memSpec,
                                   MemCommand::cmds           type) const
{
  int64_t precharge_offset = 0;

  int64_t BL = memSpec.memArchSpec.burstLength;
  int64_t RTP = memSpec.memTimingSpec.RTP;
  int64_t dataRate = memSpec.memArchSpec.dataRate;
  int64_t AL = memSpec.memTimingSpec.AL;
  int64_t WL = memSpec.memTimingSpec.WL;
  int64_t WR = memSpec.memTimingSpec.WR;
  int64_t B = BL/dataRate;

  const MemoryType::MemoryType_t& memType = memSpec.memoryType;

  // Read with auto-precharge
  if (type == MemCommand::RDA) {
    if (memType == MemoryType::DDR2) {
      precharge_offset = B + AL - 2 + max(RTP, int64_t(2));
    } else if (memType == MemoryType::DDR3) {
      precharge_offset = AL + max(RTP, int64_t(4));
    } else if (memType == MemoryType::DDR4) {
      precharge_offset = AL + RTP;
    } else if (memType == MemoryType::LPDDR) {
      precharge_offset = B;
    } else if (memType == MemoryType::LPDDR2) {
      precharge_offset = B + max(int64_t(0), RTP - 2);
    } else if (memType == MemoryType::LPDDR3) {
      precharge_offset = B + max(int64_t(0), RTP - 4);
    } else if (memType == MemoryType::WIDEIO_SDR) {
      precharge_offset = B;
    }
  } else if (type == MemCommand::WRA) { // Write with auto-precharge
    if (memType == MemoryType::DDR2) {
      precharge_offset = B + WL + WR;
    } else if (memType == MemoryType::DDR3) {
      precharge_offset = B + WL + WR;
    } else if (memType == MemoryType::DDR4) {
      precharge_offset = B + WL + WR;
    } else if (memType == MemoryType::LPDDR) {
      precharge_offset = B + WR;  // + DQSS actually, but we don't have that parameter.
    } else if (memType == MemoryType::LPDDR2) {
      precharge_offset = B +  WL + WR + 1;
    } else if (memType == MemoryType::LPDDR3) {
      precharge_offset = B +  WL + WR + 1;
    } else if (memType == MemoryType::WIDEIO_SDR) {
      precharge_offset = B + WL + WR - 1;
    }
  }

  return precharge_offset;
} // MemCommand::getPrechargeOffset

void MemCommand::setTime(int64_t _timestamp)
{
  timestamp = _timestamp;
}

int64_t MemCommand::getTimeInt64() const
{
  return timestamp;
}

MemCommand::cmds MemCommand::typeWithoutAutoPrechargeFlag() const
{
  if (type == MemCommand::RDA) {
    return MemCommand::RD;
  } else if (type == MemCommand::WRA) {
    return MemCommand::WR;
  }
  return type;
}
