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

  clocks.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename clocks.h */
/* This is an integrated interface & implementation file for */
/* all clock processes that react to the clock edges */

#include "systemc.h"

// First process
SC_MODULE( clk_pos )
{
  SC_HAS_PROCESS( clk_pos );

  sc_in_clk clk;

  sc_signal<bool>& out_clk_pos; 
  
  clk_pos(sc_module_name NAME,
	  sc_clock& TICK_P,
	  sc_signal<bool>& OUT_CLK_POS)
    : 
      out_clk_pos(OUT_CLK_POS)
  {
    clk(TICK_P);
	SC_CTHREAD( entry, clk.pos() );
  }
  
  void entry();
};

void
clk_pos::entry()
{
  out_clk_pos.write(1); cout << "Clk Pos 1\n";
  wait();
  out_clk_pos.write(0); cout << "Clk Pos 0\n";
  wait();
}

// Second process
SC_MODULE( clk_neg )
{
  SC_HAS_PROCESS( clk_neg );

  sc_in_clk clk;

  sc_signal<bool>& out_clk_neg; 
  
  clk_neg(sc_module_name NAME,
	  sc_clock& TICK_N,
	  sc_signal<bool>& OUT_CLK_NEG)
    : 
      out_clk_neg(OUT_CLK_NEG)
  {
    clk(TICK_N);
	SC_CTHREAD( entry, clk.neg() );
  }
 
  void entry();
};

void
clk_neg::entry()
{
  out_clk_neg.write(1); cout << "Clk Neg 1\n";
  wait();
  out_clk_neg.write(0); cout << "Clk Neg 0\n";
  wait();
}

// Third process
SC_MODULE( clk2_pos )
{
  SC_HAS_PROCESS( clk2_pos );

  sc_in_clk clk;

  sc_signal<bool>& out_clk2_pos; 
 
  clk2_pos(sc_module_name NAME,
	   sc_clock& TICK2_P,
	   sc_signal<bool>& OUT_CLK2_POS)
    : 
      out_clk2_pos(OUT_CLK2_POS)
  {
    clk(TICK2_P);
	SC_CTHREAD( entry, clk.pos() );
  }
 
  void entry();
};

void
clk2_pos::entry()
{
  out_clk2_pos.write(1); cout << "Clk2 Pos 1\n";
  wait();
  out_clk2_pos.write(0); cout << "Clk2 Pos 0\n";
  wait();
}

// Fourth process
SC_MODULE( clk2_neg )
{
  SC_HAS_PROCESS( clk2_neg );

  sc_in_clk clk;

  sc_signal<bool>& out_clk2_neg; 
 
  clk2_neg(sc_module_name NAME,
	   sc_clock& TICK2_N,
	   sc_signal<bool>& OUT_CLK2_NEG)
    : 
      out_clk2_neg (OUT_CLK2_NEG)
  {
    clk(TICK2_N);
	SC_CTHREAD( entry, clk.neg() );
  }
 
  void entry();
};

void
clk2_neg::entry()
{
  out_clk2_neg.write(1); cout << "Clk2 Neg 1\n";
  wait();
  out_clk2_neg.write(0); cout << "Clk2 Neg 0\n";
  wait();
}
