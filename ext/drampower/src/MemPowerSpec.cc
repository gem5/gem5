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

#include "MemPowerSpec.h"

using namespace Data;

MemPowerSpec::MemPowerSpec() :
  idd0(0.0),
  idd02(0.0),
  idd2p0(0.0),
  idd2p02(0.0),
  idd2p1(0.0),
  idd2p12(0.0),
  idd2n(0.0),
  idd2n2(0.0),
  idd3p0(0.0),
  idd3p02(0.0),
  idd3p1(0.0),
  idd3p12(0.0),
  idd3n(0.0),
  idd3n2(0.0),
  idd4r(0.0),
  idd4r2(0.0),
  idd4w(0.0),
  idd4w2(0.0),
  idd5(0.0),
  idd52(0.0),
  idd6(0.0),
  idd62(0.0),
  vdd(0.0),
  vdd2(0.0),
  capacitance(0.0),
  ioPower(0.0),
  wrOdtPower(0.0),
  termRdPower(0.0),
  termWrPower(0.0)
{
}

void MemPowerSpec::processParameters()
{
  idd0    = getParamValWithDefault("idd0", 0.0);
  idd02   = getParamValWithDefault("idd02", 0.0);
  idd2p0  = getParamValWithDefault("idd2p0", 0.0);
  idd2p02 = getParamValWithDefault("idd2p02", 0.0);
  idd2p1  = getParamValWithDefault("idd2p1", 0.0);
  idd2p12 = getParamValWithDefault("idd2p12", 0.0);
  idd2n   = getParamValWithDefault("idd2n", 0.0);
  idd2n2  = getParamValWithDefault("idd2n2", 0.0);
  idd3p0  = getParamValWithDefault("idd3p0", 0.0);
  idd3p02 = getParamValWithDefault("idd3p02", 0.0);
  idd3p1  = getParamValWithDefault("idd3p1", 0.0);
  idd3p12 = getParamValWithDefault("idd3p12", 0.0);
  idd3n   = getParamValWithDefault("idd3n", 0.0);
  idd3n2  = getParamValWithDefault("idd3n2", 0.0);
  idd4r   = getParamValWithDefault("idd4r", 0.0);
  idd4r2  = getParamValWithDefault("idd4r2", 0.0);
  idd4w   = getParamValWithDefault("idd4w", 0.0);
  idd4w2  = getParamValWithDefault("idd4w2", 0.0);
  idd5    = getParamValWithDefault("idd5", 0.0);
  idd52   = getParamValWithDefault("idd52", 0.0);
  idd6    = getParamValWithDefault("idd6", 0.0);
  idd62   = getParamValWithDefault("idd62", 0.0);
  vdd     = getParamValWithDefault("vdd", 0.0);
  vdd2    = getParamValWithDefault("vdd2", 0.0);

  capacitance = getParamValWithDefault("capacitance", 0.0);
  ioPower     = getParamValWithDefault("ioPower", 0.0);
  wrOdtPower  = getParamValWithDefault("wrOdtPower", 0.0);
  termRdPower = getParamValWithDefault("termRdPower", 0.0);
  termWrPower = getParamValWithDefault("termWrPower", 0.0);
} // MemPowerSpec::processParameters
