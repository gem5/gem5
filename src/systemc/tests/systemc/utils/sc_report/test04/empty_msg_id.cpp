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

  empty_msg_id.cpp -- Test for empty message IDs

  Original Author: Philipp A. Hartmann, OFFIS, 2013-09-17

 *****************************************************************************/

#include <systemc>

using std::cout;
using std::endl;
using sc_core::sc_report_handler;

static const char * null_msg  = NULL;
static const char * empty_msg = "";

int sc_main(int,char*[])
{
  SC_REPORT_INFO( empty_msg, "empty msg id" );
    cout << sc_report_handler::get_count(empty_msg) << endl;
    sc_assert( sc_report_handler::get_count(empty_msg) == 1 );

  SC_REPORT_INFO( 1, "empty msg id" ); // integer ID
    cout << sc_report_handler::get_count("") << endl;
    sc_assert( sc_report_handler::get_count(empty_msg) == 2 );


  SC_REPORT_INFO( null_msg,  "null msg id" );
    cout << sc_report_handler::get_count(null_msg) << endl;
    sc_assert( sc_report_handler::get_count(null_msg) == 1 );

  SC_REPORT_INFO( 0,  "another (integer) null msg id" );
    cout << sc_report_handler::get_count(null_msg) << endl;
    sc_assert( sc_report_handler::get_count(null_msg) == 2 );

  return 0;
}
