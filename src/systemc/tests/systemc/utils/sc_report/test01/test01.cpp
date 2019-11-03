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

// test of reporting -- API

#include "systemc.h"

int
sc_main( int, char*[] )
{
    const int MY_ID = 9999;

    sc_report::register_id( MY_ID, "bogus message" );
    cout << sc_report::get_message( MY_ID ) << endl;
    cout << sc_report::is_suppressed( MY_ID ) << endl;
    sc_report::suppress_id( MY_ID, true );
    cout << sc_report::is_suppressed( MY_ID ) << endl;
    sc_report::suppress_id( MY_ID, false );
    cout << sc_report::is_suppressed( MY_ID ) << endl;

    SC_REPORT_INFO( MY_ID, "infos ..." );
    sc_report::suppress_infos( true );
    SC_REPORT_INFO( MY_ID, "infos suppressed" );
    sc_report::suppress_infos( false );
    SC_REPORT_INFO( MY_ID, "infos not suppressed" );
    sc_report::suppress_id( MY_ID, true );
    SC_REPORT_INFO( MY_ID, "suppressed" );
    sc_report::suppress_id( MY_ID, false );
    SC_REPORT_INFO( MY_ID, "not suppressed" );

    SC_REPORT_WARNING( MY_ID, "warnings ..." );
    sc_report::suppress_warnings( true );
    SC_REPORT_WARNING( MY_ID, "warnings suppressed" );
    sc_report::suppress_warnings( false );
    SC_REPORT_WARNING( MY_ID, "warnings not suppressed" );
    sc_report::suppress_id( MY_ID, true );
    SC_REPORT_WARNING( MY_ID, "suppressed" );
    sc_report::suppress_id( MY_ID, false );
    SC_REPORT_WARNING( MY_ID, "not suppressed" );

    sc_report::make_warnings_errors( true );
    try {
        SC_REPORT_WARNING( MY_ID, "do make warnings errors" );
    }
    catch( sc_report x ) {
        cout << "\ncaught exception" << endl;
        cout << x.what() << endl;
    }
    sc_report::make_warnings_errors( false );
    try {
        SC_REPORT_WARNING( MY_ID, "do not make warnings errors" );
    }
    catch( sc_report x ) {
        cout << "\ncaught exception" << endl;
        cout << x.what() << endl;
    }

    try {
        SC_REPORT_ERROR( MY_ID, "errors ..." );
    }
    catch( sc_report x ) {
        cout << "\ncaught exception" << endl;
        cout << x.what() << endl;
    }
    sc_report::suppress_id( MY_ID, true );
    try {
        SC_REPORT_ERROR( MY_ID, "cannot be suppressed" );
    }
    catch( sc_report x ) {
        cout << "\ncaught exception" << endl;
        cout << x.what() << endl;
    }

    return 0;
}
