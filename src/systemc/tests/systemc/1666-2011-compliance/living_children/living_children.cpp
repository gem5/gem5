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

// living_children.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: living_children.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Not invaliding handles to processes with living children

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    t1 = sc_spawn(sc_bind(&Top::T1, this));
  }
  
  sc_process_handle t1, t2a, t2b;
  
  void T1()
  {
    wait(1, SC_NS);
    t2a = sc_spawn(sc_bind(&Top::T2, this));
    t2b = sc_spawn(sc_bind(&Top::T2, this));
    wait(1, SC_NS);
    sc_assert( t2a.valid() );
    sc_assert( t2b.valid() );
    
    std::vector<sc_object*> children = t1.get_child_objects();
    sc_assert( children.size() == 2);
    
    sc_event_and_list and_list = t2a.terminated_event() & t2b.terminated_event();
    wait( and_list );

    sc_assert( t2a.valid() );
    sc_assert( t2a.terminated() );
    sc_assert( t2b.valid() );
    sc_assert( t2b.terminated() );
    sc_assert( sc_time_stamp() == sc_time(3, SC_NS) );
  }
  
  void T2()
  {
    wait(2, SC_NS);
    sc_process_handle t3 = sc_spawn(sc_bind(&Top::T3, this));
    wait(SC_ZERO_TIME);
    sc_assert( t3.valid() );

    sc_process_handle me = sc_get_current_process_handle();
    sc_assert( me.get_parent_object() ); // Parent handle shall still be valid
    std::vector<sc_object*> children = me.get_child_objects();
    sc_assert( children.size() == 1);
    
    std::vector<sc_event*> my_events = me.get_child_events();
    sc_assert( my_events.size() == 0 );
  }
  
  void T3()
  {
    wait(3, SC_NS);
    sc_process_handle me = sc_get_current_process_handle();
    sc_assert( me.get_parent_object() ); // Parent handle shall still be valid
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
  
