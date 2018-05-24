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

// test of sc_signal_resolved

#include "systemc.h"

SC_MODULE( mod_a )
{
    // ports
    sc_out_resolved out1;
    sc_out_resolved out2;
    sc_in_resolved  in;

    // variables
    sc_logic l1;
    sc_logic l2;

    // events
    sc_event ready1;
    sc_event ready2;

    void out_action1()
    {
        for( int i = 0; i < 4; ++ i ) {
            l1 = sc_dt::sc_logic_value_t( i );
            for( int j = 0; j < 4; ++j ) {
                out1.write( l1 );
                wait( 1, SC_NS );
                ready1.notify();
                wait( SC_ZERO_TIME );
            }
        }
    }

    void out_action2()
    {
        for( int i = 0; i < 4; ++ i ) {
            for( int j = 0; j < 4; ++ j ) {
                l2 = sc_dt::sc_logic_value_t( j );
                out2.write( l2 );
                wait( 1, SC_NS );
                ready2.notify();
                wait( SC_ZERO_TIME );
            }
        }
    }

    void in_action()
    {
        for( int i = 0; i < 16; ++ i ) {
            wait( ready1 & ready2 );
            cout << l1 << " " << l2 << " -> " << in.read() << endl;
        }
    }

    SC_CTOR( mod_a )
    {
        SC_THREAD( out_action1 );
        SC_THREAD( out_action2 );
        SC_THREAD( in_action );
    }
};

int
sc_main( int, char*[] )
{
    sc_signal_resolved sig;

    mod_a a( "a" );

    a.out1( sig );
    a.out2( sig );
    a.in( sig );

    sc_start();

    return 0;
}
