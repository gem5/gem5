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

  test06.cpp -- Test sc_vector::get_elements

  Original Author: Philipp A. Hartmann, OFFIS, 2010-11-04

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc"

#include "sysc/utils/sc_vector.h"

using sc_core::sc_object;
using sc_core::sc_vector;
using sc_core::sc_mutex;
using sc_core::sc_in;

void print_vector( const char * header,
                   const std::vector<sc_object*> & vec )
{
  std::cout << "\n-->-- " << header << " -->--\n";

  std::cout << " - size: " << vec.size() << "\n";

  for (size_t i=0; i<vec.size(); ++i )
    std::cout << " - "
              << vec[i]->name() << " - "
              << vec[i]->kind() << "\n";

  std::cout << "--<-- " << header << " --<--"
            << std::endl;
}

#define PRINT_VECTOR( Vector ) \
  print_vector( #Vector, Vector )

SC_MODULE( sub_module )
{
  sc_in<bool> in;
  SC_CTOR(sub_module) {}
};


SC_MODULE( module )
{
  // vector of sub-modules
  sc_vector< sub_module > sub_vec;

  // vector of ports
  sc_vector< sc_in<bool> > in_vec;

  SC_CTOR(module)
    : sub_vec( "sub_modules" )
    , in_vec( "in_vec" )
  {}

  void init( unsigned n_sub )
  {
    sub_vec.init(n_sub);
    in_vec.init(n_sub);
    // in_vec.init(n_sub); // second call fails

    // bind ports of sub-modules -- no dereference
    sc_assemble_vector(  sub_vec, &sub_module::in ).bind( in_vec );
  }

};

int sc_main(int, char* [])
{
  module m("dut");
  m.init(4); // calls from external context

  PRINT_VECTOR( m.get_child_objects() );

  PRINT_VECTOR( m.sub_vec.get_child_objects() );

  PRINT_VECTOR( m.sub_vec.get_elements() );

  PRINT_VECTOR( sc_assemble_vector( m.sub_vec, &sub_module::in ).get_elements() );

  std::cout << "\nProgram completed" << std::endl;
  return 0;
}
