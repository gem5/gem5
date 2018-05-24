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
 
  pulse.h -- Definition of the pulse generator.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef PULSE_H
#define PULSE_H

SC_MODULE( gen_pulse_mod )
{
  // Ports:
  sc_in_clk    clk;          // Clock for the pulse generator.
  sc_in<bool>  start;        // Becomes true if the car's started.
  sc_in<int>   speed;        // Speed of the car set by the driver.
  sc_out<bool> speed_pulse;  // Pulses for the speedometer.
  sc_out<bool> dist_pulse;   // Pulses for the odometers.

  // Find the pulse period to produce speed.
  int find_period(int speed);

  // Generate pulses for speedometer and odometers.
  void gen_pulse_proc();

  SC_CTOR( gen_pulse_mod )
  {
    SC_THREAD( gen_pulse_proc );
    sensitive << clk << start;
  }

};

#endif

