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
 * Authors: Karthik Chandrasekar, Sven Goossens
 *
 */

#include <stdint.h>

#include "Parametrisable.h"

namespace Data {
class MemTimingSpec : public virtual Parametrisable {
 public:
  MemTimingSpec();
  void processParameters();

  double clkMhz;
  int64_t RC;
  int64_t RCD;
  int64_t CCD;
  int64_t CCD_S;
  int64_t CCD_L;
  int64_t RRD;
  int64_t RRD_S;
  int64_t RRD_L;
  int64_t FAW;
  int64_t TAW;
  int64_t WTR;
  int64_t WTR_S;
  int64_t WTR_L;
  int64_t REFI;
  int64_t RL;
  int64_t RP;
  int64_t RFC;
  int64_t RAS;
  int64_t WL;
  int64_t AL;
  int64_t DQSCK;
  int64_t RTP;
  int64_t WR;
  int64_t XP;
  int64_t XPDLL;
  int64_t XS;
  int64_t XSDLL;
  int64_t CKE;
  int64_t CKESR;
  double   clkPeriod;
};
}
