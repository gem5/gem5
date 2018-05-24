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

  test05.cpp -- Test sc_vector (and its binding functions)

  Original Author: Philipp A. Hartmann, OFFIS, 2010-03-05

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>

#include "sysc/utils/sc_vector.h"
using sc_core::sc_vector;

typedef sc_vector< sc_fifo_out<int> > port_vec;

static void
dump_port_array( const char* from, const port_vec& in )
{
    std::cout << "\n" << from << "\n";
    for( unsigned i=0; i<in.size(); ++i )
    {
      std::cout
        << "  "
        << in[i].name()
        << ".size() == " << in[i].size();

      for( int j=0; j<in[i].size(); ++j)
      {
        std::cout
          << " - "
          << dynamic_cast<const sc_core::sc_object*>(in[i][j])->name()
          << " @ " << j;
      }
      std::cout << std::endl;
    }
}

SC_MODULE(sub_module)
{
  port_vec in;

  sub_module( sc_core::sc_module_name, unsigned n_sub )
    : in("in", n_sub ) {}

  void before_end_of_elaboration()
    { dump_port_array( "sub_module::before_end_of_elaboration", in ); }
  void end_of_elaboration()
    { dump_port_array( "sub_module::end_of_elaboration", in ); }
};


SC_MODULE(module)
{
  sub_module sub;
  port_vec   in;

  SC_CTOR(module)
    : sub("sub", 4), in("in",4)
  {
    // vector to vector binding
    sub.in( in );
  }

  void before_end_of_elaboration()
    { dump_port_array( "module::before_end_of_elaboration", in ); }
  void end_of_elaboration()
    { dump_port_array( "module::end_of_elaboration", in ); }
};

int sc_main( int, char*[] )
{

  module top("top");

  const unsigned size = 4;
  const unsigned half = 2;

  sc_assert( top.in.size() == size );

  sc_vector< sc_fifo<int> >
    fifo_1( "src_1", half ),
    fifo_2( "src_2", size );

  // bind full vector (smaller than target)
  port_vec::iterator mid = top.in( fifo_1 );
  sc_assert( ( mid - top.in.begin() ) - half == 0 );

  // bind range, starting from last position
  mid = top.in.bind( fifo_2.begin(), fifo_2.end(), mid );
  sc_assert( mid == top.in.end() );

  // bind a plain C array of channels
  sc_fifo<int> fifo_a[ half ];
  top.sub.in( fifo_a, fifo_a + half );

  sc_start();
  return 0;
}
