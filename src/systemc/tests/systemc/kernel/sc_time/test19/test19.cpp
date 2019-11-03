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

  test19.cpp -- Test sc_time string/tuple conversions

  Original Author: Philipp A. Hartmann, OFFIS

 *****************************************************************************/


#include "systemc.h"

void check_time( const sc_time& t, sc_time_unit tu, const std::string & str )
{
    sc_time_tuple tp = t;

    std::cout << t.to_string() << ", value=" << t.value() << std::endl;
    std::cout << "  ";
    if( tp.has_value() )
        std::cout << "t.value=" << tp.value();
    else
        std::cout << "t.double=" << tp.to_double();
    std::cout << ", t.unit="  << tp.unit_symbol()
              << std::endl;

    sc_assert( tp.has_value() );
    sc_assert( t.to_string() == str );
    sc_assert( tp.to_string() == t.to_string() );
    sc_assert( tp.unit() == tu );

    sc_time u = sc_time::from_string( str.c_str() );
    sc_assert( t == u );
    sc_assert( u == tp );
    sc_assert( tp.unit() == sc_time_tuple(u).unit() );

    u = sc_time( tp.to_double(), tp.unit_symbol() );
    sc_assert( t == u );
    sc_assert( u == tp );
    sc_assert( tp.unit() == sc_time_tuple(u).unit() );
}

int sc_main( int, char*[] )
{
    sc_report_handler::set_actions( SC_ID_SET_TIME_RESOLUTION_, SC_DO_NOTHING );
    sc_report_handler::set_actions( SC_ID_TIME_CONVERSION_FAILED_, SC_DISPLAY );

    unsigned resolutions[] = { 100, 10, 1 };
    sc_time_unit resunit   = SC_FS;
    unsigned* res = resolutions;

    while( true )
    {
        std::cout << "\nResolution = " << sc_get_time_resolution() << std::endl;

        check_time( sc_time(  10,   SC_NS),  SC_NS,  "10 ns" );
        check_time( sc_time( 100,   SC_NS),  SC_NS, "100 ns" );
        check_time( sc_time(1000,   SC_NS),  SC_US,   "1 us" );
        check_time( sc_time(   0.1, SC_US),  SC_NS, "100 ns" );
        check_time( sc_time(   1,   SC_US),  SC_US,   "1 us" );
        check_time( sc_time(  10,   SC_US),  SC_US,  "10 us" );
        check_time( sc_time( 100,   SC_US),  SC_US, "100 us" );
        check_time( sc_time(1000,   SC_US),  SC_MS,   "1 ms" );
        check_time( sc_time( 100,  SC_SEC), SC_SEC, "100 s" );

        // exit loop before final resolution update
        if (res == resolutions + (sizeof(resolutions)/sizeof(*resolutions)))
            break;

        sc_set_time_resolution( *res, resunit );
        res++;
    }

    cout << "\nProgram completed" << endl;
    return 0;
}
