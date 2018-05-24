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

  Original Author: Martin Janssen, Synopsys, Inc., 2002-03-13

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: test01.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:26:10  acg
// systemc_tests-2.3
//
// Revision 1.2  2006/01/24 21:05:13  acg
//  Andy Goodrich: replacement of deprecated features with their non-deprecated
//  counterparts.
//

// test of former asserts (should not be reachable from public APIs) now errors

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in_clk clk;

    SC_CTOR( mod_a )
    {
        clk.pos().find_event();
    }
};

SC_MODULE( mod_b )
{
    sc_in_clk clk;

    SC_CTOR( mod_b )
    {
        clk->read();
    }
};

SC_MODULE( mod_c )
{
    const sc_in_clk clk;

    SC_CTOR( mod_c )
    {
        clk->read();
    }
};

int
sc_main( int, char*[] )
{
    // sc_clock error(s)

    try {
        sc_clock clk1( "clk1", 0, SC_PS );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }

    try {
        sc_clock clk2( "clk2", 1, SC_PS, 0.1 );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }

    try {
        sc_clock clk3( "clk3", 1, SC_PS, 0.9 );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }


    // sc_event_finder error(s)

    try {
        mod_a a( "a" );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }


    // sc_port error(s)

    try {
        mod_b b( "b" );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }

    try {
        mod_c c( "c" );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }


    // sc_semaphore error(s)

    try {
        sc_semaphore sem1( -1 );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }

    try {
        sc_semaphore sem2( "sem2", -1 );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }


    // sc_event error(s)

    try {
        sc_event e1;
        e1.notify( 10, SC_MS );
        e1.notify_delayed();
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }

    try {
        sc_event e2;
        e2.notify( 10, SC_MS );
        e2.notify_delayed( SC_ZERO_TIME );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }


    // sc_name_gen error(s)

    try {
        sc_gen_unique_name( 0 );
    } catch( sc_report x ) {
        cout << "\nException caught" << endl;
        cout << x.what() << endl;
    }

    return 0;
}
