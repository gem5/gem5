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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of signal port tracing.

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in_clk clk;

    sc_in<int>       in_int;
    sc_in<bool>      in_bool;
    sc_in<sc_logic>  in_logic;
    sc_in_resolved   in_resolved;
    sc_in_rv<1>      in_rv1;

    sc_out<int>      out_int;
    sc_out<bool>     out_bool;
    sc_out<sc_logic> out_logic;
    sc_out_resolved  out_resolved;
    sc_out_rv<1>     out_rv1;

    void main_action()
    {
        int      a_int      = 0;
        bool     a_bool     = false;
        sc_logic a_logic    = SC_LOGIC_X;
        sc_logic a_resolved = SC_LOGIC_X;
        sc_lv<1> a_rv1      = sc_lv<1>( SC_LOGIC_X );

        wait();

        while( true ) {
            out_int      = a_int;
            out_bool     = a_bool;
            out_logic    = a_logic;
            out_resolved = a_resolved;
            out_rv1      = a_rv1;

            a_int ++;
            a_bool     = ! a_bool;
            a_logic    = sc_dt::sc_logic_value_t( a_int % 4 );
            a_resolved = a_logic;
            a_rv1      = sc_lv<1>( a_logic );

            wait();
        }
    }

    SC_CTOR( mod_a )
    {
        SC_THREAD( main_action );
        sensitive << clk.pos();
    }
};

int
sc_main( int, char*[] )
{
    sc_clock clk;

    sc_signal<int>      sig_int;
    sc_signal<bool>     sig_bool;
    sc_signal<sc_logic> sig_logic;
    sc_signal_resolved  sig_resolved;
    sc_signal_rv<1>     sig_rv1;

    mod_a a( "a" );

    a.clk( clk );

    a.in_int( sig_int );
    a.in_bool( sig_bool );
    a.in_logic( sig_logic );
    a.in_resolved( sig_resolved );
    a.in_rv1( sig_rv1 );

    a.out_int( sig_int );
    a.out_bool( sig_bool );
    a.out_logic( sig_logic );
    a.out_resolved( sig_resolved );
    a.out_rv1( sig_rv1 );

    sc_trace_file* tf = sc_create_vcd_trace_file( "test" );

    sc_trace( tf, sig_int,      "sig_int" );
    sc_trace( tf, sig_bool,     "sig_bool" );
    sc_trace( tf, sig_logic,    "sig_logic" );
    sc_trace( tf, sig_resolved, "sig_resolved" );
    sc_trace( tf, sig_rv1,      "sig_rv1" );

    sc_trace( tf, a.in_int,      "a.in_int" );
    sc_trace( tf, a.in_bool,     "a.in_bool" );
    sc_trace( tf, a.in_logic,    "a.in_logic" );
    sc_trace( tf, a.in_resolved, "a.in_resolved" );
    sc_trace( tf, a.in_rv1,      "a.in_rv1" );

    sc_trace( tf, a.out_int,      "a.out_int" );
    sc_trace( tf, a.out_bool,     "a.out_bool" );
    sc_trace( tf, a.out_logic,    "a.out_logic" );
    sc_trace( tf, a.out_resolved, "a.out_resolved" );
    sc_trace( tf, a.out_rv1,      "a.out_rv1" );

    sc_start( 10, SC_NS );

    sc_close_vcd_trace_file( tf );

    return 0;
}
