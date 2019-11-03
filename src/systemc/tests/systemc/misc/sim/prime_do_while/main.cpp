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

                /***************************************/
                /* Main Filename: 	main.cc        */
                /***************************************/

#include "reset.h"
#include "display.h"
#include "prime_numgen.h"

int
sc_main(int ac, char *av[])
{

// Signal Instantiation
  sc_signal<bool>         reset	("reset");
  signal_bool_vector      prime	("prime");

// Clock Instantiation
  sc_clock 	clk ("CLK", 6, SC_NS, 0.5, 10, SC_NS, false);	// 167 Mhz

// Process Instantiation
  prime_numgen	D1 ("D1", clk, reset, prime);

  resetp	T1 ("T1", clk, reset);
 
  displayp	T2 ("T2", clk, prime);
 
// Simulation Run Control
  sc_start();
 return 0;
}
