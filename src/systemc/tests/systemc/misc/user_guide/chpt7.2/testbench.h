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

  testbench.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename testbench.h */
/* This is the interface file for synchronous process `testbench' */

#include "systemc.h"

SC_MODULE( testbench )
{
  SC_HAS_PROCESS( testbench );

  sc_in_clk clk;

  const sc_signal<int>& Ssum; //input
  const sc_signal<int>& Sdiff; //input
  const sc_signal<bool>& adder_sub_done; //input
  sc_signal<int>& Sa; //output
  sc_signal<int>& Sb; //output
  sc_signal<int>& Sc; //output
  sc_signal<bool>& adder_sub_ready; //output

  //Constructor 
  testbench(sc_module_name NAME,
	    sc_clock& CLK,
	    const sc_signal<int>& SSUM,
	    const sc_signal<int>& SDIFF,
	    const sc_signal<bool>& ADDER_SUB_DONE,
	    sc_signal<int>& SA,
	    sc_signal<int>& SB,
	    sc_signal<int>& SC,
	    sc_signal<bool>& ADDER_SUB_READY)
    : Ssum(SSUM), Sdiff(SDIFF), 
      adder_sub_done(ADDER_SUB_DONE),
      Sa(SA), Sb(SB), Sc(SC), adder_sub_ready(ADDER_SUB_READY)
      
  {
    clk(CLK);
	SC_CTHREAD( entry, clk.pos() );
  }

  // Process functionality in member function below
  void entry();
};


