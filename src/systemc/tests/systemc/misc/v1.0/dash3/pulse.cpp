/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************
 
  pulse.cpp -- Implementation of the pulse generator.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: pulse.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:26:24  acg
// systemc_tests-2.3
//
// Revision 1.3  2006/01/19 00:48:19  acg
// Andy Goodrich: Changes for the fact signal write checking is enabled.
//
// Revision 1.2  2006/01/18 00:23:50  acg
// Change over from SC_NO_WRITE_CHECK to sc_write_check_enable() call.
//

#define SC_NO_WRITE_CHECK
#include "systemc.h"
#include "const.h"
#include "pulse.h"

// Find the pulse period to produce speed.
// This function also rounds the period to the nearest integer.
int
gen_pulse_mod::find_period(int speed)
{
  if (speed <= 0)
    return 1;

  const double num = DIST_BETWEEN_TWO_PULSES * PERIODS_PER_HOUR / 2;

  double dp = num / speed;
  int    ip = int(dp);

  ip = ip + ((dp - ip) >= double(0.5) ? 1 : 0);

  return ip;
}

// Generate pulses for speedometer and odometers.
void
gen_pulse_mod::gen_pulse_proc()
{
  wait();

  speed_pulse = false;
  dist_pulse = false;

  // Wait until the car is started.
  do {
    wait();
  } while (start.read() == false);

  while (true) {

    speed_pulse = true;
    dist_pulse = true;
    AWAIT(find_period(speed));

    speed_pulse = false;
    dist_pulse = false;
    AWAIT(find_period(speed));
  }
}

// End of file
