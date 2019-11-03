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

// test of the sc_clock constructors

#include "systemc.h"

#define CLOCK_INFO(clk) \
    cout << endl; \
    cout << "name       = " << clk.name() << endl; \
    cout << "period     = " << clk.period() << endl; \
    cout << "duty_cycle = " << clk.duty_cycle() << endl;

int
sc_main( int, char*[] )
{
    sc_time t1( 8, SC_NS );
    sc_time t2( 2, SC_NS );

    sc_clock c1;
    CLOCK_INFO( c1 );

    sc_clock c2( "c2" );
    CLOCK_INFO( c2 );

    sc_clock c3( "c3", t1 );
    CLOCK_INFO( c3 );
    sc_clock c4( "c4", t1, 0.1 );
    CLOCK_INFO( c4 );
    sc_clock c5( "c5", t1, 0.1, t2 );
    CLOCK_INFO( c5 );
    sc_clock c6( "c6", t1, 0.1, t2, false );
    CLOCK_INFO( c6 );

    sc_clock c7( "c7", 8, SC_NS );
    CLOCK_INFO( c7 );
    sc_clock c8( "c8", 8, SC_NS, 0.1 );
    CLOCK_INFO( c8 );

    sc_clock c9( "c9", 8, SC_NS, 0.1, 2, SC_NS );
    CLOCK_INFO( c9 );
    sc_clock cA( "cA", 8, SC_NS, 0.1, 2, SC_NS, false );
    CLOCK_INFO( cA );

    sc_clock cB( "cB", 8 );
    CLOCK_INFO( cB );
    sc_clock cC( "cC", 8, 0.1 );
    CLOCK_INFO( cC );
    sc_clock cD( "cD", 8, 0.1, 2 );
    CLOCK_INFO( cD );
    sc_clock cE( "cE", 8, 0.1, 2, false );
    CLOCK_INFO( cE );

    return 0;
}
