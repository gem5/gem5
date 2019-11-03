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
 
  driver.h -- Definition of the driver.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef DRIVER_H
#define DRIVER_H

SC_MODULE( driver_mod )
{
  // Input ports:
  sc_in_clk     clk;   // Clock for the actions of the driver.
  sc_in<double> speed;
  sc_in<double> angle;
  sc_in<double> total;
  sc_in<double> partial;

  // Output ports:
  sc_out<bool>  reset; // Set if the driver wants to reset the partial
                       // distance odometer.
  sc_out<int>   speed_set; // Speed of the car as set by the driver.
  sc_out<bool>  start;     // Set if the driver starts the car.

  // Driver's actions.
  void driver_out_proc();
  void driver_in_proc();

  SC_CTOR( driver_mod )
  {
    SC_CTHREAD( driver_out_proc, clk.pos() );

    SC_METHOD( driver_in_proc );
    sensitive << speed << angle << total << partial;
  }
};

#endif
