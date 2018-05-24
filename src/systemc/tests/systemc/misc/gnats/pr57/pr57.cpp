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

  pr57.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE( pr57 )
{
    SC_HAS_PROCESS( pr57 );

    sc_in_clk clk;

    const sc_signal<bool>& a;
          sc_signal<bool>& b;

    bool c;
    bool d;

    pr57( sc_module_name NAME,
          sc_clock& CLK,
          const sc_signal<bool>& A,
                sc_signal<bool>& B )
        : 
          a(A), b(B)
    {
      clk(CLK);
	  SC_CTHREAD( entry, clk.pos() );
    }
    void entry();
    bool pres1(bool x, bool y);
    bool pres2(bool x);
    void pres3(bool z);
};

bool
pr57::pres1(bool x, bool y)
{
    bool u = x | c;
    bool v = y & d;
    bool t = d;
    d = y;
    c = t;
    return u ^ v;
}

bool
pr57::pres2(bool x)
{
    bool u = x & c & d;
    d = x ^ u;
    return u;
}

void
pr57::pres3(bool z)
{
    c = z;
}

void
pr57::entry()
{
    c = a;
    d = ! a.read();
    b = pres1(1, 0);
    wait();
    c = ! a.read();
    d = a;
    b = pres1(0, 1);
    wait();
    c = a;
    d = ! a.read();
    b = pres2(0);
    wait();
    pres3(!a.read());
    d = a;
    b = pres2(1);
    wait();
}

int
sc_main( int, char*[] )
{
    return 0;
}
