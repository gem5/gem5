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

  stim.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename stim.h */
/* This is the interface file for synchronous process `stim' */

#include "common.h"

SC_MODULE( stim )
{
  SC_HAS_PROCESS( stim );

  sc_in_clk clk;

  // Inputs
  const sc_signal<bool>& done;
  // Outputs
  sc_signal<bool>& reset;
  signal_bool_vector& a;
  signal_bool_vector& b;
  sc_signal<bool>& cin;
  sc_signal<bool>& ready;
  // Parameters
  const int data_width;
 
  // Constructor
  stim (sc_module_name NAME,
	sc_clock& TICK,
	const sc_signal<bool>& DONE,
	sc_signal<bool>& RESET,
	signal_bool_vector& A,
	signal_bool_vector& B,
	sc_signal<bool>& CIN,
	sc_signal<bool>& READY,
        const int DATA_WIDTH = 8)
    : done(DONE), reset(RESET), 
      a(A),b(B), cin(CIN),
      ready(READY),
      data_width(DATA_WIDTH)
  { 
    clk(TICK);
	SC_CTHREAD( entry, clk.neg() );
  }

  // Process functionality in member function below
  void entry();
};
