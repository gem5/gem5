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

  fsmr.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Header file for FSM recognizer */

#include "systemc.h"

SC_MODULE( fsm_recognizer )
{
  SC_HAS_PROCESS( fsm_recognizer );

  sc_in_clk clk;

  // The input signals
  const sc_signal<char>& input_char; // The input character
  const sc_signal<bool>& data_ready; // The data ready signal
  // The output signals
  sc_signal<bool>& found;            // The indicator that sequence found

  // The internal variables
  char pattern[4];   // This string will hold the pattern to match against
  
  // The constructor
  fsm_recognizer(sc_module_name NAME,
		 sc_clock& TICK,
		 const sc_signal<char>& INPUT_CHAR,
		 const sc_signal<bool>& DATA_READY,
		 sc_signal<bool>& FOUND)
    : input_char(INPUT_CHAR), data_ready(DATA_READY), 
      found(FOUND) {
        clk(TICK);
		SC_CTHREAD( entry, clk.pos() );
	pattern[0] = 'S';
	pattern[1] = 'c';
	pattern[2] = 'e';
	pattern[3] = 'n';
  }

  // The functionality of the process
  void entry();
};
