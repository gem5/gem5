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

  test01.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: test01.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:25:56  acg
// systemc_tests-2.3
//
// Revision 1.2  2006/01/24 19:11:29  acg
// Andy Goodrich: Changed sc_simulation_time() usage to sc_time_stamp().
//

// test of sc_buffer<T> -- general test

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in<bool> clk;
    sc_out<int> out;

    void main_action()
    {
        while( true ) {
            wait();
            out = 3;
        }
    }

    SC_CTOR( mod_a )
    {
        SC_THREAD( main_action );
        sensitive << clk.pos();
    }
};

SC_MODULE( mod_b )
{
    sc_in<int> in;

    void main_action()
    {
        cout << sc_time_stamp() << " " << in.read() << endl;
    }

    SC_CTOR( mod_b )
    {
        SC_METHOD( main_action );
        sensitive << in;
    }
};

int
sc_main( int, char*[] )
{
    mod_a a( "a" );
    mod_b b( "b" );

    sc_clock clk;
    sc_buffer<int> buf;

    a.clk( clk );
    a.out( buf );
    b.in( buf );

    sc_start( 100, SC_NS);

    return 0;
}
