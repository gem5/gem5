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

  test04.cpp -- Test sc_vector -- build a two dimensional mesh

  Original Author: Philipp A. Hartmann, OFFIS, 2010-03-01

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"
#include "sysc/utils/sc_vector.h"


SC_MODULE( mesh_node )
{
  // constructor with additional parameters
  mesh_node( sc_core::sc_module_name, int x, int y );
};

typedef sc_vector< mesh_node > row_type;
typedef sc_vector< row_type  > mesh_type;

SC_MODULE(mesh)
{
  mesh_type nodes;

  mesh( sc_module_name, int n, int m );

  struct node_creator
  {
    node_creator( int row ) : x(row) {}

    mesh_node* operator()(const char * name, size_t idx )
    {
      return new mesh_node( name, x, idx );
    }
    int x;
  };

  struct row_creator
  {
    row_creator( int n_cols ) : cols_( n_cols ) {}

    row_type* operator()( const char* name, size_t idx )
    {
      return new row_type( name, cols_, node_creator(idx) );
    }
    int cols_;
  };

  const unsigned rows;
  const unsigned cols;

};

mesh_node::mesh_node( sc_module_name, int x, int y )
{
  std::cout << name() << " created @ "
            << x << "x" << y << std::endl;
}

mesh::mesh( sc_module_name, int n, int m )
  : nodes("nodes")
  , rows( n )
  , cols( m )
{
  nodes.init( n, row_creator(m) );
}

int sc_main(int argc, char* argv[])
{
  mesh dut("dut", 4, 5);

  for (size_t j=0; j<dut.cols; ++j )
    for (size_t i=0; i<dut.rows; ++i )
      cout << dut.nodes[i][j].name() << " - "
           << dut.nodes[i][j].kind()
           << endl;

  cout << "Program completed" << endl;
  return 0;
}
