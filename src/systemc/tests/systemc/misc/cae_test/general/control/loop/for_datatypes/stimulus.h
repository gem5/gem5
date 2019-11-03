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

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-27

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "systemc.h"

SC_MODULE( stimulus )
{
        SC_HAS_PROCESS( stimulus );

        sc_in_clk clk;

        sc_signal<bool>&      reset;
        sc_signal<int>&       in_value;
        sc_signal<bool>&      in_valid;

  stimulus(sc_module_name        NAME,
	   sc_clock&             CLK,
           sc_signal<bool>&      RESET,
           sc_signal<int>&       IN_VALUE,
           sc_signal<bool>&      IN_VALID
          )
          : 
            reset          (RESET),
            in_value       (IN_VALUE),
            in_valid       (IN_VALID)
  {
    clk            (CLK);
	SC_CTHREAD( entry, clk.pos() );
  }
  
  void entry();
};

// EOF
