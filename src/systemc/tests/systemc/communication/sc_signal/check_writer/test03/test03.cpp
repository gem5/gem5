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

  test03.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of dynamic design rule checking in signals.

#define DEBUG_SYSTEMC
#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in_clk clk;

    sc_out<int>      out_int;
    sc_out<bool>     out_bool;
    sc_out<sc_logic> out_logic;
    sc_out<int>      out_int2;
    sc_out_resolved  out_resolved;
    sc_out_rv<1>     out_rv1;

    void main_action1()
    {
        out_int = 42;
        out_bool = true;
        out_logic = SC_LOGIC_1;
        out_int2 = 1;
        out_resolved = SC_LOGIC_1;
        out_rv1 = sc_lv<1>( SC_LOGIC_1 );
    }

    void main_action2()
    {
        out_int = 0;
        out_bool = false;
        out_logic = SC_LOGIC_0;
        out_int2 = 0;
        out_resolved = SC_LOGIC_0;
        out_rv1 = sc_lv<1>( SC_LOGIC_0 );
    }

    SC_CTOR( mod_a )
    {
        SC_METHOD( main_action1 );
        sensitive << clk.pos();
        dont_initialize();
        SC_METHOD( main_action2 );
        sensitive << clk.neg();
        dont_initialize();
    }
};

int
sc_main( int, char*[] )
{
    sc_clock clk;

    sc_signal<int> sig_int("sig_int");
    sc_signal<bool> sig_bool("sig_bool");
    sc_signal<sc_logic> sig_logic("sig_logic");
    sc_buffer<int> buf_int("buf_int");
    sc_signal_resolved sig_resolved("sig_resolved");
    sc_signal_rv<1> sig_rv1("sig_rv1");

    mod_a a("a");
    a(clk, sig_int, sig_bool, sig_logic, buf_int, sig_resolved, sig_rv1);

    sc_start( 20, SC_NS );

    return 0;
}
