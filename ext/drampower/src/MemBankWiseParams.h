/*
 * Copyright (c) 2012-2014, TU Delft
 * Copyright (c) 2012-2014, TU Eindhoven
 * Copyright (c) 2012-2016, TU Kaiserslautern
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
 * Authors: Subash Kannoth, Matthias Jung, Eder Zulian
 *
 */
#ifndef MEMBANKWISEPARAMS_H
#define MEMBANKWISEPARAMS_H

#include <stdint.h>
#include <vector>
#include <algorithm>
#include <numeric>

namespace Data {
  class MemBankWiseParams {
    public:
      // Set of possible PASR modes
      enum pasrModes{
        PASR_0,
        PASR_1,
        PASR_2,
        PASR_3,
        PASR_4,
        PASR_5,
        PASR_6,
        PASR_7
      };
      // List of active banks under the specified PASR mode
      std::vector<unsigned> activeBanks;
      // ACT Standby power factor
      int64_t bwPowerFactRho;
      // Self-Refresh power factor( true : Bankwise mode)
      int64_t bwPowerFactSigma;
      // Bankwise or Normal mode
      bool bwMode;
      // Wherther PASR is enabled ( true : enabled )
      bool flgPASR;
      //Default constructor
      MemBankWiseParams();
      MemBankWiseParams(int64_t factRho, int64_t factSigma,
                        bool hasPASR, int64_t pasrMode,
                        bool opMode, unsigned nbrofBanks);

      bool isBankActiveInPasr(const unsigned bankIdx) const;
  };
}

#endif // MEMBANKWISEPARAMS_H
