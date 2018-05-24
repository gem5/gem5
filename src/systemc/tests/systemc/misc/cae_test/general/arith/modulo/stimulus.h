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

  stimulus.h -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-05-13

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "common.h"

SC_MODULE( stimulus )
{
  SC_HAS_PROCESS( stimulus );

  sc_in_clk clk;

  sc_signal<bool>&  reset;
  sc_signal<int>&              out_stimulus1;
  sc_signal_bool_vector4&      out_stimulus2;
  sc_signal_bool_vector4&      out_stimulus3;
  sc_signal_bool_vector8&      out_stimulus4;
  sc_signal_bool_vector8&      out_stimulus5;
  sc_signal<bool>&  out_valid;

  stimulus(sc_module_name NAME,
	   sc_clock&  CLK,
	   sc_signal<bool>& RESET,
	   sc_signal<int>& OUT_STIMULUS1,
	   sc_signal_bool_vector4&            OUT_STIMULUS2,      
	   sc_signal_bool_vector4&            OUT_STIMULUS3,     
	   sc_signal_bool_vector8&            OUT_STIMULUS4,      
	   sc_signal_bool_vector8&            OUT_STIMULUS5,     
	   sc_signal<bool>&                   OUT_VALID
          )
          : 
            reset(RESET),
            out_stimulus1(OUT_STIMULUS1),
            out_stimulus2(OUT_STIMULUS2),
            out_stimulus3(OUT_STIMULUS3),
            out_stimulus4(OUT_STIMULUS4),
            out_stimulus5(OUT_STIMULUS5),
            out_valid(OUT_VALID)
  {
    clk (CLK);
	SC_CTHREAD( entry, clk.pos() );
  }
  
  void entry();
};

// EOF
