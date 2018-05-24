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

  pr-2.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE( foo )
{
    SC_HAS_PROCESS( foo );

    sc_in_clk clk;

    const sc_signal<bool>& a;
    const sc_signal<bool>& b;
          sc_signal<bool>& c;

    sc_signed x;
    sc_unsigned y;
//    sc_logic_vector z;
//    sc_bool_vector v;

    foo( sc_module_name name,
         sc_clock& CLK,
         const sc_signal<bool>& A,
         const sc_signal<bool>& B,
               sc_signal<bool>& C )
        : 
          a(A), b(B), c(C),
          x(13), y(15) // , z(8), v(11)
    {
      clk(CLK);
	  SC_CTHREAD( entry, clk.pos() );
    }
    void entry();
};

void
foo::entry()
{
    sc_signed x2(13);
    sc_unsigned y2(15);

    x2 = x; // should have no converts here
    y2 = y; // should have no converts here
}

int sc_main(int argc, char* argv[] )
{
  return 0;
}
