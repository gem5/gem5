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

  const.h -- Constants for the dashboard controller.
 
  Original Author: Ali Dasdan, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef CONST_H
#define CONST_H

const int MAX_SPEED = 130; // Car speed in km/h.
const int MAX_ANGLE = 270; // Needle angle in the speedometer in degrees.

const double DIST_BETWEEN_TWO_PULSES = 0.00066 / 2; // 1/2 of a tire in km.
const double DIST_INCR               = 0.01; // One distance increment in km.

// Two pulses are needed to compute speed.
const int NUM_PULSES_FOR_SPEED     = 2;   

// DIST_INCR / DIST_BETWEEN_TWO_PULSES
const int NUM_PULSES_FOR_DIST_INCR = 61;  

const int SLOW_CLOCK_PERIOD0    = 10000;    // in 10 x milliseconds.
const int FAST_CLOCK_PERIOD1    = 50;       // in 10 x milliseconds.
const int ONE_HOUR              = 36000000; // in 10 x milliseconds.
const int PERIODS_PER_HOUR      = ONE_HOUR / FAST_CLOCK_PERIOD1;

#define AWAIT(N) \
  for (register int i = 0; i < N; ++i)  \
    wait();

#endif
