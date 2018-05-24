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

// immed_self_notif.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: immed_self_notif.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Change to the semantics of immediate self-notification to match Verilog semantics

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    SC_THREAD(T1);
      sensitive << ev1;
      
    SC_THREAD(T2);
      sensitive << ev1;
      
    SC_THREAD(T3);
      
    SC_THREAD(T4);
      sensitive << ev3;
      
    SC_METHOD(M1);
      sensitive << ev5;
      
    SC_METHOD(M2);
    
    SC_METHOD(M3);
    
    SC_THREAD(yield_test);
      
    end_of_t2   = false;
    m1_run      = false;
    m2_run      = false;
    m3_run      = false;
    first_yield = true;
    yield_count = 0;
    yield_test_run = false;
  }
  
  sc_event ev1, ev2, ev3, ev4, ev5, ev6, ev7;
  bool end_of_t2;
  bool m1_run;
  bool m2_run;
  bool m3_run;
  
  sc_event yield_event_1, yield_event_2;
  bool first_yield;
  int yield_count;
  bool yield_test_run;
  
  void T1()
  {
    wait(SC_ZERO_TIME);
    ev1.notify();
    sc_assert( sc_delta_count() == 1 );
    wait(ev1);
    sc_assert( false );
  }
  
  void T2()
  {
    wait(ev1);
    sc_assert( sc_delta_count() == 1 );
    end_of_t2 = true;
  }

  void T3()
  {
    wait(SC_ZERO_TIME);
    ev2.notify();
    sc_assert( sc_delta_count() == 1 );
    wait(ev2);
    sc_assert( false );
  }
  
  void T4()
  {
    wait(SC_ZERO_TIME);
    ev3.notify();
    sc_assert( sc_delta_count() == 1 );
    wait(ev4);
    sc_assert( false );
  }
  
  void M1()
  {
    sc_assert( !m1_run );
    ev5.notify();
    m1_run = true;
  }
  
  void M2()
  {
    sc_assert( !m2_run );
    ev6.notify();
    next_trigger(ev6);
    m2_run = true;
  }

  void M3()
  {
    sc_assert( !m3_run );
    next_trigger(ev7);
    ev7.notify();
    m3_run = true;
  }
  
  void yield_test()
  {
    sc_assert( sc_delta_count() == 0 );
    wait(SC_ZERO_TIME);
    sc_assert( sc_delta_count() == 1 );
    
    yield();
    sc_spawn(sc_bind( &Top::yield_test_child, this));
    yield();
    sc_spawn(sc_bind( &Top::yield_test_child, this));
    yield();
    
    sc_assert( sc_delta_count() == 1 );
    wait(1, SC_MS);
    unsigned int delta_count = sc_delta_count();
    
    yield();
    sc_spawn(sc_bind( &Top::yield_test_child, this));
    yield();
    sc_spawn(sc_bind( &Top::yield_test_child, this));
    yield();

    sc_assert( sc_delta_count() == delta_count );
    yield_test_run = true;
  }
  
  void yield_test_child()
  {
    yield();
  }
  
  void yield()
  {
    ++yield_count;
    if (first_yield)
    {
      sc_spawn(sc_bind(&Top::yield_helper, this));
      first_yield = false;
    }
    else
      yield_event_1.notify();
    wait(yield_event_2);
  }
  
  void yield_helper()
  {
    yield_event_2.notify();
    while (true)
    {
      wait(yield_event_1);
      yield_event_2.notify();
    }
  }

  SC_HAS_PROCESS(Top);
};


int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();
  
  sc_assert( top.end_of_t2 );
  sc_assert( top.m1_run );
  sc_assert( top.m2_run );
  sc_assert( top.m3_run );
  sc_assert( top.first_yield == false );
  sc_assert( top.yield_count == 10 );
  sc_assert( top.yield_test_run );
  
  cout << endl << "Success" << endl;
  return 0;
}
  
