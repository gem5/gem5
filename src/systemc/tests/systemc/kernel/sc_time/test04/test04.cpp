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

  test04.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of function sc_set_time_resolution

#include "systemc.h"

SC_MODULE( source )
{
    sc_in_clk   clk;
    sc_out<int> out;

    void main_action()
    {
        sc_set_time_resolution( 10, SC_PS );
        int a = 0;
        while( true ) {
            wait();
            out = ++ a;
        }
    }

    SC_CTOR( source )
    {
        SC_THREAD( main_action );
        sensitive << clk.pos();
    }
};

int
sc_main( int, char*[] )
{
    sc_clock clk( "clk" );
    sc_signal<int> sig( "sig" );

    source src( "src" );
    src.clk( clk );
    src.out( sig );

    sc_start();

    return 0;
}
