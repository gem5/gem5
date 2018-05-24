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

  test01.cpp -- Test sc_vector

  Original Author: Philipp A. Hartmann, OFFIS, 2010-01-10

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

#include "sysc/utils/sc_vector.h"
using sc_core::sc_vector;

SC_MODULE( sub_module )
{
  sc_in<bool> in;
  SC_CTOR(sub_module) {}
};

SC_MODULE( module )
{
  // vector of sub-modules
  sc_vector< sub_module > m_sub_vec;

  // vector of ports
  sc_vector< sc_in<bool> > in_vec;

  module( sc_core::sc_module_name, unsigned n_sub )
    : m_sub_vec( "sub_modules", n_sub ) // set name prefix, and create sub-modules
    // , in_vec()                       // use default constructor
    // , in_vec( "in_vec" )             // set name prefix
  {
    // delayed initialisation of port vector
    // here with default prefix sc_core::sc_gen_unique_name("vector")
    in_vec.init( n_sub );

    // bind ports of sub-modules -- sc_assemble_vector
    sc_assemble_vector( m_sub_vec, &sub_module::in ).bind( in_vec );
  }
};

int sc_main(int , char* [])
{
  module m("dut", 4);

  std::vector<sc_object*> children = m.get_child_objects();

  for (size_t i=0; i<children.size(); ++i )
    cout << children[i]->name() << " - "
         << children[i]->kind()
         << endl;

  cout << "Program completed" << endl;
  return 0;
}
