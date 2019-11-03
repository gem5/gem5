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

  for_datatypes.h -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-27

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "systemc.h"

SC_MODULE( for_datatypes )
{
    SC_HAS_PROCESS( for_datatypes );

    sc_in_clk clk;

    const sc_signal<bool>&      reset;
    const sc_signal<bool>&      in_valid;
    const sc_signal<int>&       in_value;
          sc_signal<bool>&      out_valid;
          sc_signal<int>&       result;
 
    for_datatypes (
              sc_module_name            NAME,        // referense name
              sc_clock&                 CLK,          // clock
	const sc_signal<bool>&          RESET,
	const sc_signal<bool>&          IN_VALID,
	const sc_signal<int>&           IN_VALUE,
	      sc_signal<bool>&          OUT_VALID,
              sc_signal<int>&           RESULT 
        )
        : 
          reset          (RESET),
          in_valid       (IN_VALID),
          in_value       (IN_VALUE),
          out_valid      (OUT_VALID),
          result         (RESULT)
    {
      clk            (CLK);
	  SC_CTHREAD( entry, clk.pos() );
      reset_signal_is(reset,true);
    };
    void entry ();
};
