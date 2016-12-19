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
 * Authors: Subash Kannoth, Matthias Jung, Éder F. Zulian
 *
 */

#include "MemBankWiseParams.h"

using namespace Data;
/**
 * Sets the default bankwise configurations.
 */
MemBankWiseParams::MemBankWiseParams():
  bwPowerFactRho(100),
  bwPowerFactSigma(100),
  bwMode(false),
  flgPASR(false)
{
}
/**
 * Sets all the bankwise parameters required in bankwise mode
 */
MemBankWiseParams::MemBankWiseParams(int64_t factRho, int64_t factSigma,
                                      bool hasPASR, int64_t pasrMode,
                                      bool opMode, unsigned nbrofBanks)
{

  bwPowerFactRho = factRho;
  bwPowerFactSigma = factSigma;
  bwMode = opMode;
  flgPASR = hasPASR;
  ///////////////////////////////////////////////////////////
  // Activate banks for self refresh based on the PASR mode
  // ACTIVE     - X
  // NOT ACTIVE - 0
  ///////////////////////////////////////////////////////////
  switch(pasrMode){

    case(PASR_0):{
                   // PASR MODE 0
                   // FULL ARRAY
                   // |X X X X |
                   // |X X X X |
                   activeBanks.resize(nbrofBanks);
                   std::iota(activeBanks.begin(), activeBanks.end(), 0);
                   break;
                 }
    case(PASR_1):{
                   // PASR MODE 1
                   // (1/2) ARRAY
                   // |X X X X |
                   // |0 0 0 0 |
                   activeBanks.resize(nbrofBanks - 4);
                   std::iota(activeBanks.begin(), activeBanks.end(), 0);
                   break;
                 }
    case(PASR_2):{
                   // PASR MODE 2
                   // (1/4) ARRAY
                   // |X X 0 0 |
                   // |0 0 0 0 |
                   activeBanks.resize(nbrofBanks - 6);
                   std::iota(activeBanks.begin(), activeBanks.end(), 0);
                   break;
                 }
    case(PASR_3):{
                   // PASR MODE 3
                   // (1/8) ARRAY
                   // |X 0 0 0 |
                   // |0 0 0 0 |
                   activeBanks.resize(nbrofBanks - 7);
                   std::iota(activeBanks.begin(), activeBanks.end(), 0);
                   break;
                 }
    case(PASR_4):{
                   // PASR MODE 4
                   // (3/4) ARRAY
                   // |0 0 X X |
                   // |X X X X |
                   activeBanks.resize(nbrofBanks - 2);
                   std::iota(activeBanks.begin(), activeBanks.end(), 2);
                   break;
                 }
    case(PASR_5):{
                   // PASR MODE 5
                   // (1/2) ARRAY
                   // |0 0 0 0 |
                   // |X X X X |
                   activeBanks.resize(nbrofBanks - 4);
                   std::iota(activeBanks.begin(), activeBanks.end(), 4);
                   break;
                 }
    case(PASR_6):{
                   // PASR MODE 6
                   // (1/4) ARRAY
                   // |0 0 0 0 |
                   // |0 0 X X |
                   activeBanks.resize(nbrofBanks - 6);
                   std::iota(activeBanks.begin(), activeBanks.end(), 6);
                   break;
                 }
    case(PASR_7):{
                   // PASR MODE 7
                   // (1/8) ARRAY
                   // |0 0 0 0 |
                   // |0 0 0 X |
                   activeBanks.resize(nbrofBanks - 7);
                   std::iota(activeBanks.begin(), activeBanks.end(), 7);
                   break;
                 }
    default:{
                   // PASR MODE 0
                   // FULL ARRAY
                   // |X X X X |
                   // |X X X X |
                   activeBanks.resize(nbrofBanks);
                   std::iota(activeBanks.begin(), activeBanks.end(), 0);
                   break;
    }
  }
}

/**
 * Returns true if the given bank is active under the current PASR mode.
 */
bool MemBankWiseParams::isBankActiveInPasr(const unsigned bankIdx) const
{
  return (std::find(activeBanks.begin(), activeBanks.end(), bankIdx)
      != activeBanks.end());
}
