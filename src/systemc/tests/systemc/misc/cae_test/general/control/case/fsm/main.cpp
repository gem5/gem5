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

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-30

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "fsm.h"
#include "stimulus.h"
#include "display.h"

int sc_main (int argc , char *argv[]) {
  sc_clock        	clock;
  sc_signal<bool> 	reset;
  sc_signal_bool_vector stim1;
  sc_signal_bool_vector stim2;
  sc_signal_bool_vector stim3;
  sc_signal<bool>       input_valid;
  sc_signal_bool_vector result1;
  sc_signal_bool_vector result2;
  sc_signal_bool_vector result3;
  sc_signal<bool>       output_valid1;
  sc_signal<bool>       output_valid2;
  sc_signal<bool>       output_valid3;


  fsm  fsm1   ( "process_body",
			    clock, 
			    reset,
			    stim1,
			    stim2,
			    stim3,
			    input_valid,
			    result1,
			    result2,
			    result3,
			    output_valid1,
			    output_valid2,
			    output_valid3
			    ); 

  stimulus stimulus1   ("stimulus",
			clock,
			reset,
			stim1,
			stim2,
			stim3,
			input_valid);

  display display1   ("display",
		      clock,
		      result1,
		      result2,
		      result3,
		      output_valid1,
		      output_valid2,
		      output_valid3
		      );

  sc_start();
  return 0;
}

// EOF
