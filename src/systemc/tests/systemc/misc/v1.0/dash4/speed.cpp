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
 
  speed.cpp -- Definition of the speedometer.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: speed.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:26:24  acg
// systemc_tests-2.3
//
// Revision 1.3  2006/01/19 00:48:20  acg
// Andy Goodrich: Changes for the fact signal write checking is enabled.
//
// Revision 1.2  2006/01/18 00:23:50  acg
// Change over from SC_NO_WRITE_CHECK to sc_write_check_enable() call.
//

#define SC_NO_WRITE_CHECK
#include "systemc.h"
#include "const.h"
#include "speed.h"

// Find the elapsed_time between NUM_PULSES_FOR_SPEED pulses.
void speed_mod::find_time_proc()
{
  if (start)
    elapsed_time = elapsed_time + 1;
  else
    elapsed_time = 0;
}

// Compute speed.
void
speed_read_mod::read_speed_proc()
{
  wait();

  while (true) {

    // More than one pulse is needed to compute a distance and 
    // consequently, speed. This function collects NUM_PULSES_FOR_SPEED
    // pulses for that purpose.
    AWAIT(NUM_PULSES_FOR_SPEED);

    if (start)
      raw_speed = DIST_BETWEEN_TWO_PULSES * PERIODS_PER_HOUR / elapsed_time;
    else
      raw_speed = 0.0;

    // Reset timer.
    elapsed_time = 0;
  }
}

// Filter speed.
void 
speed_read_mod::filter_speed_proc()
{
  if (start)
    filtered_speed = raw_speed * 1.0;  // Ambiguous overload w/o 1.0.
  else
    filtered_speed = 0.0;
}

// Compute needle angle and drive the speedometer.
void 
speed_pwm_mod::pwm_driver_proc()
{
  if (start) {
    speed = filtered_speed * 1.0;  // Ambiguous overload w/o 1.0.
    angle = filtered_speed * MAX_ANGLE / MAX_SPEED;
  }
  else {
    speed = 0.0;
    angle = 0.0;
  }
}


// End of file
