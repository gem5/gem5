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

/* Filename tb.h */
/* This is the interface file for the testbench module */

#include "systemc.h"

struct testbench : public sc_module {
  sc_signal<bool> out_clk_pos;
  sc_signal<bool> out_clk_neg;
  sc_signal<bool> out_clk2_pos;
  sc_signal<bool> out_clk2_neg;
  clk_pos clkp;
  clk_neg clkn;
  clk2_pos clkp2;
  clk2_neg clkn2;

  // Constructor
  testbench(sc_module_name NAME,
	    sc_clock&    TICK,
	    sc_clock&    TICK2)
    : sc_module (NAME),
      out_clk_pos	("out_clk_pos"),
      out_clk_neg	("out_clk_neg"),
      out_clk2_pos	("out_clk2_pos"),
      out_clk2_neg	("out_clk2_neg"),
      clkp 		("CLKP", TICK, out_clk_pos),
      clkn 		("CLKN", TICK, out_clk_neg),
      clkp2 		("CLKP2", TICK2, out_clk2_pos),
      clkn2 		("CLKN2", TICK2, out_clk2_neg)
    {
    }
};
