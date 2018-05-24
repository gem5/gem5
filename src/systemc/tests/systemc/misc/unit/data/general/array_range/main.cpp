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
                /* Main Filename:       main.cc        */
                /***************************************/
 
#include "array_range.h" 	
#include "stimgen.h" 	

int sc_main(int ac, char *av[])
{

// Signal Instantiation
  signal_bool_vector8  	  in1 		("in1");
  signal_bool_vector4  	  o1		("o1");
  signal_bool_vector4  	  o2		("o2");
  signal_bool_vector8  	  o3		("o3");
  signal_bool_vector8  	  o4		("o4");
  signal_bool_vector8  	  o5		("o5");

// Clock Instantiation
  sc_clock clk( "clock", 10, SC_NS, 0.5, 0, SC_NS); 

// Process Instantiation
  array_range	D1 ("D1", clk, in1, o1, o2, o3, o4, o5); 

  stimgen	T1 ("T1", clk, o1, o2, o3, o4, o5, in1); 

// Simulation Run Control
  sc_start(); 
  return 0;
}
