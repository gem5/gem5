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

  pr213.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE( abc )
{
    SC_HAS_PROCESS( abc );

    sc_in_clk clk;

    const sc_signal<bool>& a;
    const sc_signal<bool>& b;
          sc_signal<bool>& c;

    abc( sc_module_name NAME,
         sc_clock& CLK,

         const sc_signal<bool>& A,
         const sc_signal<bool>& B,
               sc_signal<bool>& C )

        : 
          a(A), b(B), c(C)
    {
      clk(CLK);
	  SC_CTHREAD( entry, clk.pos() );
    }
    void entry();
};

void
abc::entry()
{
    int i = 0, j = 0;
    do {
        i++;
        if (i == 5) continue;
        if (i == 7) break;
        j = j + 2;
        wait();
    } while (a.read() && b.read());
}

int sc_main( int, char** ) { return 0; }
