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

// test03.cpp -- test for delayed before end of elaboration.
//
//  Original Author: Philipp A. Hartmann, OFFIS Institute for Information 
//                                        Technology
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: test03.cpp,v $
// Revision 1.2  2011/07/24 13:05:30  acg
//  Alan Fitch: added return 0 to sc_main that was missing.
//
// Revision 1.1  2011/05/08 17:55:36  acg
//  Philipp A. Hartmann: first check in of test.
//

#include <systemc>

using namespace sc_core;

#define PRINT_CALL( CallBack, Condition ) \
    std::cout << this->name() \
              << "->" #CallBack "()" \
              << ( ( Condition ) ? " " : " NOT " ) \
              << "called" \
              << std::endl

SC_MODULE(in_port_module)
{
  SC_CTOR(in_port_module) : beoe_called(false) {}

  void before_end_of_elaboration()
    { beoe_called = true; }

  void end_of_elaboration()
    { PRINT_CALL(before_end_of_elaboration,beoe_called); }

  bool beoe_called;
};

struct my_port : sc_in<bool>
{
  typedef sc_in<bool> base_type;

  explicit my_port( const char* nm )
    : base_type(nm)
    , direct_mod( (std::string(nm)+"_direct_mod").c_str() )
    , beoe_called(false) {}

  void before_end_of_elaboration()
  {
    beoe_called = true;
    std::string nm  = std::string(basename()) + "_delayed_mod";
    delayed_mod = new in_port_module( nm.c_str()  );
  }

  void end_of_elaboration()
    { PRINT_CALL(before_end_of_elaboration,beoe_called); }

  in_port_module  direct_mod;
  in_port_module* delayed_mod;
  bool            beoe_called;
};

SC_MODULE(sub_module)
{
  my_port  direct_port;
  my_port* delayed_port;

  SC_CTOR(sub_module)
    : direct_port("direct_port")
    , delayed_port(0)
    , beoe_called(false)
  {}

  void before_end_of_elaboration()
  {
    delayed_port = new my_port( "delayed_port" );
    (*delayed_port)( direct_port );
    beoe_called = true;
  }

  void end_of_elaboration()
    { PRINT_CALL(before_end_of_elaboration,beoe_called); }

  bool beoe_called;
};

SC_MODULE(module)
{
  sub_module  direct_mod;
  my_port     direct_port;
  sub_module* delayed_mod;
  my_port*    delayed_port;

  SC_CTOR(module)
    : direct_mod("direct_mod")
    , direct_port("direct_port")
    , delayed_mod(0)
    , delayed_port(0)
    , beoe_called(false)
  {
    direct_mod.direct_port( direct_port );
  }

  void before_end_of_elaboration()
  {
    delayed_port = new my_port( "delayed_port" );
    (*delayed_port)( direct_port );

    delayed_mod  = new sub_module( "delayed_mod" );
    delayed_mod->direct_port( *delayed_port );
    beoe_called = true;
  }

  void end_of_elaboration()
    { PRINT_CALL(before_end_of_elaboration,beoe_called); }

  bool beoe_called;
};

int sc_main( int, char*[] )
{
  module          mod("top");
  sc_signal<bool> sig("sig");
  mod.direct_port( sig );

  sc_start();
  return 0;
}
