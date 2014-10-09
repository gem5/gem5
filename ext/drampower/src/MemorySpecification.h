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

#ifndef TOOLS_MEMORY_SPECIFICATION_H
#define TOOLS_MEMORY_SPECIFICATION_H

#include <cassert>
#include <string>

#include "MemArchitectureSpec.h"
#include "MemTimingSpec.h"
#include "MemPowerSpec.h"
#include "Parametrisable.h"


namespace Data {
// Supported memory types
class MemoryType {
 public:
  enum MemoryType_t {
    DDR2 = 0,
    DDR3,
    DDR4,
    LPDDR,
    LPDDR2,
    LPDDR3,
    WIDEIO_SDR,
    MEMORY_TYPE_INVALID
  };

  MemoryType(MemoryType_t _val) :
    val(_val)
  {
  }

  MemoryType() :
    val(MEMORY_TYPE_INVALID)
  {
  }

  MemoryType(const std::string& _val) :
    val(MEMORY_TYPE_INVALID)
  {
    if (_val == "DDR2") {
      val = DDR2;
    } else if (_val == "DDR3") {
      val = DDR3;
    } else if (_val == "DDR4") {
      val = DDR4;
    } else if (_val == "LPDDR") {
      val = LPDDR;
    } else if (_val == "LPDDR2") {
      val = LPDDR2;
    } else if (_val == "LPDDR3") {
      val = LPDDR3;
    } else if (_val == "WIDEIO_SDR") {
      val = WIDEIO_SDR;
    }
    assert("Unknown memory type." && val != MEMORY_TYPE_INVALID);
  }

  bool isLPDDRFamily() const
  {
    return val == LPDDR ||
           val == LPDDR2 ||
           val == LPDDR3 ||
           val == WIDEIO_SDR;
  }

  bool hasTwoVoltageDomains() const
  {
    return val == LPDDR ||
           val == LPDDR2 ||
           val == LPDDR3 ||
           val == WIDEIO_SDR;
  }

  bool isDDRFamily() const
  {
    return val == DDR2 ||
           val == DDR3 ||
           val == DDR4;
  }

  bool hasDll() const
  {
    return val == DDR2 ||
           val == DDR3 ||
           val == DDR4;
  }

  bool hasTermination() const
  {
    return val == DDR2 ||
           val == DDR3 ||
           val == DDR4;
  }

  double getCapacitance() const
  {
    // LPDDR/2/3 and DDR memories only have IO Power (no ODT)
    // Conservative estimates based on Micron Mobile LPDDR2 Power Calculator
      // LPDDR/2/3 IO Capacitance in mF
    if (val == LPDDR) {
        return 0.0000000045;
    } else if (val == LPDDR2) {
        return 0.0000000025;
    } else if (val == LPDDR3) {
        return 0.0000000018;
    } else {
        return 0.0;
    }
  }

  double getIoPower() const
  {
    if (val == DDR2) {
        // Conservative estimates based on Micron DDR2 Power Calculator
        return 1.5;    // in mW
    } else if (val == DDR3) {
        // Conservative estimates based on Micron DDR3 Power Calculator
        return 4.6;    // in mW
    } else if (val == DDR4) {
        // Conservative estimates based on Micron DDR3 Power Calculator
        // using available termination resistance values from Micron DDR4 Datasheets
        return 3.7;    // in mW
    } else {
        return 0.0;
    }
  }

  double getWrOdtPower() const
  {
    if (val == DDR2) {
        // Conservative estimates based on Micron DDR2 Power Calculator
        return 8.2;    // in mW
    } else if (val == DDR3) {
        // Conservative estimates based on Micron DDR3 Power Calculator
        return 21.2;    // in mW
    } else if (val == DDR4) {
        // Conservative estimates based on Micron DDR3 Power Calculator
        // using available termination resistance values from Micron DDR4 Datasheets
        return 17.0;    // in mW
    } else {
        return 0.0;
    }
  }

  double getTermRdPower() const
  {
    if (val == DDR2) {
        // Conservative estimates based on Micron DDR2 Power Calculator
        return 13.1;    // in mW
    } else if (val == DDR3) {
        // Conservative estimates based on Micron DDR3 Power Calculator
        return 15.5;    // in mW
    } else if (val == DDR4) {
        // Conservative estimates based on Micron DDR3 Power Calculator
        // using available termination resistance values from Micron DDR4 Datasheets
        return 12.4;    // in mW
    } else {
        return 0.0;
    }
  }

  double getTermWrPower() const
  {
    if (val == DDR2) {
        // Conservative estimates based on Micron DDR2 Power Calculator
        return 14.6;    // in mW
    } else if (val == DDR3) {
        // Conservative estimates based on Micron DDR3 Power Calculator
        return 15.4;    // in mW
    } else if (val == DDR4) {
        // Conservative estimates based on Micron DDR3 Power Calculator
        // using available termination resistance values from Micron DDR4 Datasheets
        return 12.3;    // in mW
    } else {
        return 0.0;
    }
  }

  operator MemoryType_t() const {
    return val;
  }

 private:
  MemoryType_t val;
};

class MemorySpecification : public virtual Parametrisable {
 public:
  std::string id;
  MemoryType  memoryType;

  MemArchitectureSpec memArchSpec;
  MemTimingSpec memTimingSpec;
  MemPowerSpec  memPowerSpec;

  void processParameters();

  static MemorySpecification getMemSpecFromXML(const std::string& id);
};
}  // namespace Data
#endif // ifndef TOOLS_MEMORY_SPECIFICATION_H
