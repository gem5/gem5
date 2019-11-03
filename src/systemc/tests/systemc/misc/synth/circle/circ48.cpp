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

  circ48.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "tb.h"
#include "circ48.h" 	

int
sc_main( int, char *[] )
{
    sc_clock clk( "clk", 20, SC_NS, 0.5, 10, SC_NS );

    sc_signal<bool>    I_reset;
    sc_signal<bool>    I_x_ok;
    sc_signal<bool>    I_y_ok;
    sc_signal<bool>    O_out_wr;
    sc_signal<bool>    O_out_sel;
    signal_bool_vector O_out_xy;
    signal_bool_vector O_diffs;

    testbench tb( "TB", clk, I_reset, I_x_ok, I_y_ok,
		  O_out_wr, O_out_sel, O_out_xy );

    circ48 c1( "C1", clk, I_reset, I_x_ok, I_y_ok,
	       O_out_wr, O_out_sel, O_out_xy, O_diffs );

    sc_start();

    return 0;
}
