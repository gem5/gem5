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

  tb.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/******************************************************************************/
/*************************** testbench Class Definition    ********************/
/******************************************************************************/

#include "common.h"

SC_MODULE( TESTBENCH )
{
    sc_signal<bool> 	reset;
    sc_signal<bool>    	in_ok;
    sc_signal<bool> 	out_ok;
    sc_signal<bool> 	instrb;
    sc_signal<bool> 	outstrb;
    signal_bool_vector  a1,a2,a3,a4,a5,a6,a7,a8; 	// -128 to 127
    signal_bool_vector  d1,d2,d3,d4,d5,d6,d7,d8; 	// -128 to 127
    STIM		st1;
    BUBBLE		bubble;
    DISPLAY		disp1;
 
    TESTBENCH( 	sc_module_name 	NAME,
		      sc_clock&	TICK  ) 
        
       : st1	("ST1", TICK, reset, in_ok, out_ok, instrb, outstrb,
		 a1, a2, a3, a4, a5, a6, a7, a8,
		 d1, d2, d3, d4, d5, d6, d7, d8) ,

	 bubble	("B1", TICK, reset, in_ok, out_ok, instrb, outstrb,
		 a1, a2, a3, a4, a5, a6, a7, a8, 
		 d1, d2, d3, d4, d5, d6, d7, d8) ,

	 disp1 	("D1", reset, in_ok, out_ok, instrb, outstrb,
		 a1, a2, a3, a4, a5, a6, a7, a8, 
		 d1, d2, d3, d4, d5, d6, d7, d8)
    {}
};
