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

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-05-13

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "stimulus.h"
#include "display.h"
#include "modulo.h"

int sc_main (int argc , char *argv[]) {
  sc_clock        clock;
  sc_signal<bool> reset;
  sc_signal<int>              stimulus_line1;
  sc_signal_bool_vector4      stimulus_line2;
  sc_signal_bool_vector4      stimulus_line3;
  sc_signal_bool_vector8      stimulus_line4;
  sc_signal_bool_vector8      stimulus_line5;
  sc_signal<bool>             input_valid;
  sc_signal<bool>             output_valid;
  sc_signal<int>              result_line1;
  sc_signal_bool_vector4      result_line2;
  sc_signal_bool_vector4      result_line3;
  sc_signal_bool_vector8      result_line4;
  sc_signal_bool_vector8      result_line5;

  stimulus stimulus1("stimulus_block",
                      clock,
		      reset,
                      stimulus_line1,
                      stimulus_line2,
                      stimulus_line3,
                      stimulus_line4,
                      stimulus_line5,
		      input_valid);

  modulo   modulo1    ( "process_body",
                       clock, 
		       reset,
                       stimulus_line1,
                       stimulus_line2,
                       stimulus_line3,
                       stimulus_line4,
                       stimulus_line5,
		       input_valid,
                       result_line1, 
                       result_line2, 
                       result_line3, 
                       result_line4, 
                       result_line5, 
		       output_valid);

  display  display1 ( "display",
                       clock,
		       result_line1,
		       result_line2,
		       result_line3,
		       result_line4,
		       result_line5,
		       output_valid);

  sc_start();
  return 0;
}

// EOF
