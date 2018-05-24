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

// named_events.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: named_events.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Hierarchically named events

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Object: sc_object
{
  Object(const char* n) 
  : sc_object(n) 
  , ev((std::string(n) + "_ev").c_str()) {}
  
  sc_event ev; // Parent of event is containing module or process instance
};

struct Top: sc_module
{
  sc_event ev1, ev2; 
  Object *obj;
  sc_signal<int> sig; // Kernel events should not be hierarchically named
  sc_fifo<int> fifo;  // Kernel events should not be hierarchically named
  sc_semaphore sema;  // Kernel events should not be hierarchically named   
  sc_mutex mut;       // Kernel events should not be hierarchically named
  
  bool T_done;

  Top(sc_module_name _name)
  : ev2("ev2")
  , sig("sig")
  , sema(1)
  , T_done(false)
  {
    
    sc_assert( ev1.in_hierarchy() );
    sc_assert( ev1.get_parent_object() == this );
    sc_assert( std::string(ev1.name()).substr(0,9) == "top.event" );
    sc_assert( std::string(ev1.basename()).substr(0,5) == "event" );

    sc_assert( ev2.in_hierarchy() );
    sc_assert( ev2.get_parent_object() == this );
    sc_assert( std::string(ev2.name()) == "top.ev2" );
    sc_assert( std::string(ev2.basename()) == "ev2" );
    
    sc_assert( sc_find_event("top.ev2") == &ev2 );
    
    std::vector<sc_event*> vec = this->get_child_events();
    sc_assert( vec.size() == 2 );
    
    sc_process_handle dummy_handle;
    sc_assert( dummy_handle.get_child_events().size() == 0 );
    sc_assert( std::string(dummy_handle.name()) == "");
    
    obj = new Object("obj");
    vec = obj->get_child_events(); // Should return empty vector
    sc_assert( vec.size() == 0 );
    
    vec = this->get_child_events();
    sc_assert( vec.size() == 3 );
    
    sc_assert( sc_find_event("top.obj_ev") == &obj->ev );
    
    Object obj2("ev2");  // Name clash
    
    vec = this->get_child_events();
    sc_assert( vec.size() == 4 );
    
    SC_THREAD(T);
  }
  
  void T()
  {
    sc_event ev1("local1");
    sc_event ev2("local2");
    
    sc_process_handle handle = sc_get_current_process_handle();
    std::string proc_name = handle.name();
    
    sc_assert( ev1.in_hierarchy() );
    sc_assert( ev1.get_parent_object() == handle.get_process_object() );
    sc_assert( std::string(ev1.name()) == proc_name + ".local1" );
    sc_assert( std::string(ev1.basename()) == "local1" );
    sc_assert( sc_hierarchical_name_exists(ev1.name()) );

    sc_assert( ev2.in_hierarchy() );
    sc_assert( ev2.get_parent_object() == handle.get_process_object() );
    sc_assert( std::string(ev2.name()) == proc_name + ".local2" );
    sc_assert( std::string(ev2.basename()) == "local2" );
    sc_assert( sc_hierarchical_name_exists(ev2.name()) );

    std::vector<sc_event*> vec = handle.get_child_events();
    sc_assert( vec.size() == 2 );
    sc_assert( vec[0] == &ev1 );
    sc_assert( vec[1] == &ev2 );
    
    sc_assert( sc_find_event(ev1.name()) == &ev1 );
    sc_assert( sc_find_event(ev2.name()) == &ev2 );
    
    T_done = true;
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  std::vector<sc_object*> vec_o;
  vec_o = sc_get_top_level_objects();
  sc_assert( vec_o.size() == 0 );

  std::vector<sc_event*> vec_e;
  vec_e = sc_get_top_level_events();
  sc_assert( vec_e.size() == 0 );
  
  sc_event ev("foo");
  
  sc_assert( ev.in_hierarchy() );
  sc_assert( ev.get_parent_object() == 0 );
  sc_assert( std::string(ev.name()) == "foo" );
  sc_assert( std::string(ev.basename()) == "foo" );
  sc_assert( sc_hierarchical_name_exists("foo") );
  
  vec_o = sc_get_top_level_objects();
  sc_assert( vec_o.size() == 0 );  
  vec_e = sc_get_top_level_events();
  sc_assert( vec_e.size() == 1 );
  sc_assert( vec_e[0] == &ev );
  sc_assert( std::string(vec_e[0]->name()) == "foo" );
  
  sc_assert( sc_find_event("foo") == &ev );
  
  Top top("top");
  sc_assert( sc_hierarchical_name_exists("top") );
  sc_assert( sc_hierarchical_name_exists("top.ev2") );
  sc_assert( sc_hierarchical_name_exists("top.sig") );
  sc_assert( sc_hierarchical_name_exists("top.obj") );
  sc_assert( sc_hierarchical_name_exists("top.obj_ev") );
  sc_assert( !sc_hierarchical_name_exists("woowoo") );
  sc_assert( !sc_hierarchical_name_exists("top.woowoo") );
    
  sc_event ev2;
  
  sc_assert( ev2.in_hierarchy() );
  sc_assert( ev2.get_parent_object() == 0 );
  sc_assert( std::string(ev2.name()).substr(0,5) == "event" );
  sc_assert( std::string(ev2.basename()).substr(0,5) == "event" );
  sc_assert( sc_hierarchical_name_exists(ev2.name()) );
  
  vec_e = sc_get_top_level_events();
  sc_assert( vec_e.size() == 2);
  
  sc_event ev3("top"); // Name clash
  vec_e = sc_get_top_level_events();
  sc_assert( vec_e.size() == 3);
  vec_o = sc_get_top_level_objects();
  sc_assert( vec_o.size() == 1 ); 
  
  sc_assert( sc_find_event(ev3.name()) == &ev3 );
  
  sc_signal<bool> sig; // Kernel events should not be hierarchically named
  vec_e = sc_get_top_level_events();
  sc_assert( vec_e.size() == 3 );
  vec_o = sc_get_top_level_objects();
  sc_assert( vec_o.size() == 2 );
  
  sc_start();
  
  sc_assert( top.T_done );
  
  cout << endl << "There should be two name clashes reported above" << endl;
  cout << endl << "Success" << endl;
  return 0;
}
  
