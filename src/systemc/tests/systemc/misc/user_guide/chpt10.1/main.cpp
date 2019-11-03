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

  main.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Main file for parametric adder simulation */

#include "paramadd.h"
#include "stim.h"

int sc_main(int ac, char *av[])
{
  // Parameter Settings
  int width = 4;

  // Signal Instantiation
  sc_signal<bool> reset;
  signal_bool_vector a ("Operand1");
  signal_bool_vector b ("Operand2");
  sc_signal<bool> cin;
  sc_signal<bool> ready;
  signal_bool_vector sum ("Sum");
  sc_signal<bool> co;
  sc_signal<bool> done;

  // Clock Instantiation
  sc_clock clk ("Clock", 10, SC_NS, 0.5, 0, SC_NS);

  // Process Instantiation
  paramadd D1("D1", clk, reset, a, b, cin, ready, sum, 
	      co, done, width);
  stim T1("T1", clk, done, reset, a, b, cin, 
	  ready, width);

  // Simulation Run Control
  sc_start();
  return 0;
}
