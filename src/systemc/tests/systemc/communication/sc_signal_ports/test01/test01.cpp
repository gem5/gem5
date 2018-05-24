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

// test of the initialize() method in output signal ports

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_out<int>      out_int;
    sc_out<bool>     out_bool;
    sc_out<sc_logic> out_logic;
    sc_out_rv<4>     out_rv4;

    SC_CTOR( mod_a )
    {
        out_int.initialize( 1 );
        out_int.initialize( 2 );
        out_bool.initialize( true );
        out_logic.initialize( sc_dt::Log_Z );
        out_logic.initialize( sc_dt::Log_1 );
        out_rv4.initialize( sc_lv<4>( "ZZZZ" ) );
        out_rv4.initialize( sc_lv<4>( "1111" ) );
    }
};

int
sc_main( int, char*[] )
{
    sc_signal<int> sig_int;
    sc_signal<bool> sig_bool;
    sc_signal<sc_logic> sig_logic;
    sc_signal_rv<4> sig_rv4;

    mod_a a( "a" );

    a.out_int( sig_int );
    a.out_bool( sig_bool );
    a.out_logic( sig_logic );
    a.out_rv4( sig_rv4 );

    cout << sig_int << endl;
    cout << sig_bool << endl;
    cout << sig_logic << endl;
    cout << sig_rv4 << endl;

    sc_start(0, SC_NS);

    cout << sig_int << endl;
    cout << sig_bool << endl;
    cout << sig_logic << endl;
    cout << sig_rv4 << endl;

    return 0;
}
