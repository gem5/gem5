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

#include "MemTimingSpec.h"

using namespace Data;

MemTimingSpec::MemTimingSpec() :
  clkMhz(0.0),
  RC(0),
  RCD(0),
  CCD(0),
  CCD_S(0),
  CCD_L(0),
  RRD(0),
  RRD_S(0),
  RRD_L(0),
  FAW(0),
  TAW(0),
  WTR(0),
  WTR_S(0),
  WTR_L(0),
  REFI(0),
  RL(0),
  RP(0),
  RFC(0),
  REFB(0),
  RAS(0),
  WL(0),
  AL(0),
  DQSCK(0),
  RTP(0),
  WR(0),
  XP(0),
  XPDLL(0),
  XS(0),
  XSDLL(0),
  CKE(0),
  CKESR(0),
  clkPeriod(0.0)
{
}

void MemTimingSpec::processParameters()
{
  clkMhz    = getParamValWithDefault("clkMhz", 0.0);
  RC        = getParamValWithDefault("RC", 0);
  RCD       = getParamValWithDefault("RCD", 0);
  CCD       = getParamValWithDefault("CCD", 0);
  RRD       = getParamValWithDefault("RRD", 0);
  WTR       = getParamValWithDefault("WTR", 0);
  CCD_S     = getParamValWithDefault("CCD_S", 0);
  CCD_L     = getParamValWithDefault("CCD_L", 0);
  RRD_S     = getParamValWithDefault("RRD_S", 0);
  RRD_L     = getParamValWithDefault("RRD_L", 0);
  WTR_S     = getParamValWithDefault("WTR_S", 0);
  WTR_L     = getParamValWithDefault("WTR_L", 0);
  TAW       = getParamValWithDefault("TAW", 0);
  FAW       = getParamValWithDefault("FAW", 0);
  REFI      = getParamValWithDefault("REFI", 0);
  RL        = getParamValWithDefault("RL", 0);
  RP        = getParamValWithDefault("RP", 0);
  RFC       = getParamValWithDefault("RFC", 0);
  REFB      = getParamValWithDefault("REFB", 0);
  RAS       = getParamValWithDefault("RAS", 0);
  WL        = getParamValWithDefault("WL", 0);
  AL        = getParamValWithDefault("AL", 0);
  DQSCK     = getParamValWithDefault("DQSCK", 0);
  RTP       = getParamValWithDefault("RTP", 0);
  WR        = getParamValWithDefault("WR", 0);
  XP        = getParamValWithDefault("XP", 0);
  XPDLL     = getParamValWithDefault("XPDLL", 0);
  XS        = getParamValWithDefault("XS", 0);
  XSDLL     = getParamValWithDefault("XSDLL", 0);
  CKE       = getParamValWithDefault("CKE", 0);
  CKESR     = getParamValWithDefault("CKESR", 0);
  clkPeriod = 1000.0 / clkMhz;
} // MemTimingSpec::processParameters
