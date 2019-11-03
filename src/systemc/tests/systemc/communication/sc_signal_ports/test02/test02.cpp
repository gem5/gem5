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

  test02.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of signal port event methods

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in<int>         in_int;
    sc_in<bool>        in_bool;
    sc_in<sc_logic>    in_logic;

    void main_action()
    {
        wait( in_int.default_event() );
        cout << "in_int       default_event()" << endl;
        wait( in_int.value_changed_event() );
        cout << "in_int       value_changed_event()" << endl;

        wait( in_bool.default_event() );
        cout << "in_bool      default_event()" << endl;
        wait( in_bool.value_changed_event() );
        cout << "in_bool      value_changed_event()" << endl;
        wait( in_bool.posedge_event() );
        cout << "in_bool      posedge_event()" << endl;
        wait( in_bool.negedge_event() );
        cout << "in_bool      negedge_event()" << endl;

        wait( in_logic.default_event() );
        cout << "in_logic     default_event()" << endl;
        wait( in_logic.value_changed_event() );
        cout << "in_logic     value_changed_event()" << endl;
        wait( in_logic.posedge_event() );
        cout << "in_logic     posedge_event()" << endl;
        wait( in_logic.negedge_event() );
        cout << "in_logic     negedge_event()" << endl;

        sc_stop();
    }

    SC_CTOR( mod_a )
    {
        SC_THREAD( main_action );
    }
};

SC_MODULE( mod_b )
{
    sc_in_clk clk;

    sc_out<bool>     out_bool;
    sc_out<int>      out_int;
    sc_out<sc_logic> out_logic;

    void main_action()
    {
        int i = 0;
        bool b = false;
        sc_logic l = SC_LOGIC_0;
        while( true ) {
            wait();
            out_int.write( i );
            i ++;
            out_bool.write( b );
            b = !b;
            out_logic.write( l );
            l = ~l;
        }
    }

    SC_CTOR( mod_b )
    {
        SC_THREAD( main_action );
        sensitive << clk.pos();
    }
};

int
sc_main( int, char*[] )
{
    sc_clock clk( "clk", 10, SC_NS );

    mod_a a( "a" );
    mod_b b( "b" );

    sc_signal<int> sig_int;
    sc_signal<bool> sig_bool;
    sc_signal<sc_logic> sig_logic;

    b.clk( clk );
    b.out_bool( sig_bool );
    b.out_int( sig_int );
    b.out_logic( sig_logic );

    a.in_int( sig_int );
    a.in_bool( sig_bool );
    a.in_logic( sig_logic );

    sc_start();

    return 0;
}
