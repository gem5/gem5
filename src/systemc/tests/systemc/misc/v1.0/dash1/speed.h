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
 
  speed.h -- Definition of the speedometer.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef SPEED_H
#define SPEED_H

SC_MODULE( speed_mod )
{
  // Input ports:
  sc_in_clk clk; // Clock to measure the time, needed to compute the speed.
  sc_in<bool> start; // Becomes true if the car's started.
  sc_in<bool> pulse; // Pulse coming from the pulse generator.

  // Output ports:
  sc_out<double> speed; // Displayed speed.
  sc_out<double> angle; // Displayed angle.

  // Internal signals:
  sc_signal<int> elapsed_time;

  // Find the elapsed_time between NUM_PULSES_FOR_SPEED pulses.
  void find_time_proc();

  // Compute speed.
  void read_speed_proc();

  SC_CTOR( speed_mod )
  {
    SC_METHOD( find_time_proc );
    sensitive << clk.pos();

    SC_THREAD( read_speed_proc );
    sensitive << pulse.pos();

    elapsed_time = 0;
  }

};

#endif

