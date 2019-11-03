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

SC_MODULE( speed_read_mod )
{
  // Input ports:
  sc_in<bool> start; // Becomes true if the car's started.
  sc_in<bool> pulse; // Pulse coming from the pulse generator.
 
  // Output ports:
  sc_out<double> filtered_speed; // Filtered speed.  

  // Inout ports:
  sc_inout<int>  elapsed_time;

  // Internal signals:
  sc_signal<double> raw_speed;

  // Compute speed.
  void read_speed_proc();

  // Filter speed.
  void filter_speed_proc();

  SC_CTOR( speed_read_mod )
  {
    SC_THREAD( read_speed_proc );
    sensitive << pulse.pos();

    SC_METHOD( filter_speed_proc );
    sensitive << raw_speed;

    raw_speed = 0.0;
  }  
};

SC_MODULE( speed_pwm_mod )
{
  // Input ports:
  sc_in<bool> start; // Becomes true if the car's started.  
  sc_in<double> filtered_speed; 

  // Output ports:
  sc_out<double> speed; // Displayed speed.
  sc_out<double> angle; // Displayed angle.

  // Compute needle angle and drive the speedometer.
  void pwm_driver_proc();

  SC_CTOR( speed_pwm_mod )
  {
    SC_METHOD( pwm_driver_proc );
    sensitive << filtered_speed;    
  }  
};

SC_MODULE( speed_mod )
{
  // Input ports:
  sc_in_clk   clk; // Clock to measure the time, needed to compute the speed.
  sc_in<bool> start; // Becomes true if the car's started.
  sc_in<bool> pulse; // Pulse coming from the pulse generator.
 
  // Output ports:
  sc_out<double> speed; // Displayed speed.
  sc_out<double> angle; // Displayed angle.

  // Internal signals:
  sc_signal<int>    elapsed_time;
  sc_signal<double> filtered_speed;

  // Internal models:
  speed_read_mod *read_mod;
  speed_pwm_mod  *pwm_mod;

  // Find the elapsed_time between NUM_PULSES_FOR_SPEED pulses.
  void find_time_proc();

  SC_CTOR( speed_mod )
  {
    SC_METHOD( find_time_proc );
    sensitive << clk.pos();

    read_mod = new speed_read_mod("read_mod");
    pwm_mod = new speed_pwm_mod("pwm_mod");

    // read_mod->start.bind(start);
    // read_mod->pulse.bind(pulse);
    // read_mod->elapsed_time.bind(elapsed_time);
    // read_mod->filtered_speed.bind(filtered_speed);
    read_mod->start(start);
    read_mod->pulse(pulse);
    read_mod->elapsed_time(elapsed_time);
    read_mod->filtered_speed(filtered_speed);

    // pwm_mod->start.bind(start);
    // pwm_mod->filtered_speed.bind(filtered_speed);
    // pwm_mod->speed.bind(speed);
    // pwm_mod->angle.bind(angle);
    pwm_mod->start(start);
    pwm_mod->filtered_speed.bind(filtered_speed);
    pwm_mod->speed.bind(speed);
    pwm_mod->angle(angle);

    elapsed_time = 0;
    filtered_speed = 0.0;
  }
};

#endif

