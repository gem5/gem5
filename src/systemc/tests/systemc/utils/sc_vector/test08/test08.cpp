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

  test08.cpp -- sc_vector of objects not derived from sc_object
                (OSCI extension to IEEE 1666-2011)

  Original Author: Philipp A. Hartmann, OFFIS, 2011-10-01

 *****************************************************************************/

#include "systemc.h"

struct foo
{
  explicit
  foo( const char* nm )
    : name(nm)
  {}
  std::string name;
};

int sc_main( int, char*[] )
{
  sc_report_handler::set_actions( SC_ERROR, SC_DISPLAY );

  sc_vector< sc_event > ev_vec ( "evs", 1 );

  sc_assert( ev_vec.size() == 1 );
  // should print an error
  sc_assert( ev_vec.get_elements().size() == 0 );

  sc_vector< foo >      foo_vec( "foo", 1 );

  sc_assert( foo_vec.size() == 1 );
  sc_assert( sc_assemble_vector( foo_vec, &foo::name ).size() == 1 );
  // should print an error
  sc_assert( sc_assemble_vector( foo_vec, &foo::name )
               .get_elements().size() == 0 );

  cout << "\nSuccess" << endl;
  return 0;
}
