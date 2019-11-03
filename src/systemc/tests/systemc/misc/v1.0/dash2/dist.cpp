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
 
  dist.cpp -- Implementation of the odometers.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: dist.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:26:24  acg
// systemc_tests-2.3
//
// Revision 1.3  2006/01/19 00:48:14  acg
// Andy Goodrich: Changes for the fact signal write checking is enabled.
//
// Revision 1.2  2006/01/18 00:23:50  acg
// Change over from SC_NO_WRITE_CHECK to sc_write_check_enable() call.
//

#define SC_NO_WRITE_CHECK
#include "systemc.h"
#include "const.h"
#include "dist.h"

bool dist_mod::prev_reset;

// Get the pulses for one distance increment.
void
dist_mod::get_dist_proc()
{
  wait();

  while (true) {

    // More than one pulse is needed for a distance increment.  This
    // function collects NUM_PULSES_FOR_DIST_INCR pulses for that
    // purpose.
    AWAIT(NUM_PULSES_FOR_DIST_INCR);

    if (start)
      ok_for_incr = (ok_for_incr ? false : true);
    else
      ok_for_incr = false;
  }
}

// Compute total distance.
void
dist_mod::compute_total_proc()
{
  if (start)
    total_dist = total_dist + 1.0;
  else
    total_dist = 0.0;
}

// Compute partial distance.
void
dist_mod::compute_partial_proc()
{
  if (start) {

    // Implement reset.event():
    if (prev_reset != (bool) reset)
      partial_dist = 0.0;
    else
      partial_dist = partial_dist + 1.0;

    prev_reset = reset;

  }
  else
    partial_dist = 0.0;
}

// LCD display driver.
void
dist_mod::lcd_driver_proc()
{
  if (start) {

    if (total_dist.event())
      total = total_dist * DIST_INCR;

    if (partial_dist.event())
      partial = partial_dist * DIST_INCR;

  }
  else {
    total = 0.0;
    partial = 0.0;
  }
}

// End of file
