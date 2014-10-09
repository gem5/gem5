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

#include "MemArchitectureSpec.h"

#include <cassert>

using namespace Data;

MemArchitectureSpec::MemArchitectureSpec() :
  burstLength(0),
  nbrOfBanks(0),
  nbrOfRanks(0),
  dataRate(0),
  nbrOfColumns(0),
  nbrOfRows(0),
  width(0),
  nbrOfBankGroups(0),
  dll(false),
  twoVoltageDomains(false),
  termination(false)
{
}

void MemArchitectureSpec::processParameters()
{
  // Load all parameters in our member variables
  nbrOfBanks      = getParamValWithDefault("nbrOfBanks", 1);
  nbrOfRanks      = getParamValWithDefault("nbrOfRanks", 1);
  nbrOfBankGroups = getParamValWithDefault("nbrOfBankGroups", 1);
  dataRate        = getParamValWithDefault("dataRate", 1);
  burstLength     = getParamValWithDefault("burstLength", 1);
  nbrOfColumns    = getParamValWithDefault("nbrOfColumns", 1);
  nbrOfRows       = getParamValWithDefault("nbrOfRows", 1);
  width           = getParamValWithDefault("width", 1);
  assert("memory width should be a multiple of 8" && (width % 8) == 0);
  dll             = getParamValWithDefault("dll", false);
  twoVoltageDomains = getParamValWithDefault("twoVoltageDomains", false);
  termination       = getParamValWithDefault("termination", false);
}
