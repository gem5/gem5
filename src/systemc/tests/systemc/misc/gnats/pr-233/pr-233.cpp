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

  pr-233.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int*
bar(int* x, int* y, int q)
{
    return q ? x : y;
}

SC_MODULE( pr233 )
{
    SC_HAS_PROCESS( pr233 );

    sc_in_clk clk;

    // const sc_signal<int*>& x;
    // const sc_signal<int*>& y;
    // const sc_signal<int>&  q;
    //       sc_signal<int*>& z;
    sc_in<int*>  x;
    sc_in<int*>  y;
    sc_in<int>   q;
    sc_out<int*> z;

    pr233( sc_module_name         NAME,
           sc_clock&              CLK,
           const sc_signal<int*>& X,
           const sc_signal<int*>& Y,
           const sc_signal<int>&  Q,
           sc_signal<int*>&       Z )
        : 
          x(X), y(Y), q(Q), z(Z)
    {
      clk( CLK );
	  SC_CTHREAD( entry, clk.pos() );
    }
    void entry();

};

void
pr233::entry()
{
    z.write( bar(x, y, q) );
    wait();
}

int sc_main(int,char**) { return 0; }
