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

// include_descendants.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: include_descendants.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Process control methods include_descendants argument

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    SC_THREAD(calling);
    
    SC_THREAD(target1);
      t1 = sc_get_current_process_handle();
      
    SC_THREAD(target4);
      t4 = sc_get_current_process_handle();
      
    SC_THREAD(target7);
      t7 = sc_get_current_process_handle();
      
    count = 0;
    f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = f10 = 0;
    f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = 0;
  }
  
  sc_process_handle t1, ch2, ch3, t4, ch5, ch6, t7, ch8, ch9;
  sc_process_handle gch10, gch11, gch12, gch13;
  sc_event ev;
  int count;
  int f1, f2, f3, f4, f5, f6, f7, f8, f9, f10;
  int f11, f12, f13, f14, f15, f16, f17, f18;
  
  std::exception ex;
  
  void calling()
  {
    wait(SC_ZERO_TIME);

    count = 1;
    t1.suspend(SC_INCLUDE_DESCENDANTS);
    ev.notify(5, SC_NS);
    wait(10, SC_NS);
    
    count = 2;
    t1.resume(SC_INCLUDE_DESCENDANTS);
    wait(10, SC_NS);
    
    count = 3;
    t1.disable(SC_INCLUDE_DESCENDANTS);
    ev.notify(5, SC_NS);
    wait(10, SC_NS);
    
    count = 4;
    t1.enable(SC_INCLUDE_DESCENDANTS);
    wait(10, SC_NS);
    
    count = 5;
    ev.notify();
    wait(10, SC_NS);
    
    count = 6;
    t1.sync_reset_on(SC_INCLUDE_DESCENDANTS);
    wait(10, SC_NS);
    
    count = 7;
    ev.notify();
    wait(10, SC_NS);

    count = 8;
    t1.sync_reset_off(SC_INCLUDE_DESCENDANTS);
    wait(sc_time(110, SC_NS) - sc_time_stamp());
    
    count = 10;
    t4.reset(SC_INCLUDE_DESCENDANTS);
    wait(sc_time(210, SC_NS) - sc_time_stamp());
    
    t7.throw_it(ex, SC_INCLUDE_DESCENDANTS);
  }

  void target1()
  {
    sc_assert(count == 0);
    ch2 = sc_spawn(sc_bind(&Top::child2, this));
    ch3 = sc_spawn(sc_bind(&Top::child3, this));
    wait(ch2.terminated_event() & ch3.terminated_event());
    f5 = 1;
  }
  
  void child2()
  {
    if (count == 0) // Initialization
    {
      wait(ev);
      sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
      wait(ev);
      sc_assert(count == 5);
      sc_assert( sc_time_stamp() == sc_time(40, SC_NS) );
      f1 = 1;
      wait(ev);
    }
    else if (count == 7) // Sync reset
    {
      sc_assert( sc_time_stamp() == sc_time(60, SC_NS) );
      f3 = 1;
      wait(20, SC_NS);
    }
  }
  
  void child3()
  {
    if (count == 0) // Initialization
    {
      wait(ev);
      sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
      wait(ev);
      sc_assert(count == 5);
      sc_assert( sc_time_stamp() == sc_time(40, SC_NS) );
      f2 = 1;
      wait(ev);
    }
    else if (count == 7) // Sync reset
    {
      sc_assert( sc_time_stamp() == sc_time(60, SC_NS) );
      f4 = 1;
      wait(20, SC_NS);
    }
  }

  void target4()
  {
    if (count == 0)
    {
      wait(100, SC_NS);
      count = 9;
      ch5 = sc_spawn(sc_bind(&Top::child5, this));
      ch6 = sc_spawn(sc_bind(&Top::child6, this));
    }
    else // Hard reset
    {
      sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );
      f11 = 1;
    }
    wait(ch5.terminated_event() & ch6.terminated_event());
    f6 = 1;
  }
  
  void child5()
  {
    switch (count)
    {
      case  9: sc_assert( sc_time_stamp() == sc_time(100, SC_NS) ); f7=1; break;
      case 10: sc_assert( sc_time_stamp() == sc_time(110, SC_NS) ); f8=1; break;
      default: sc_assert( false ); break;
    }
    wait(20, SC_NS);
  }
  
  void child6()
  {
    switch (count)
    {
      case  9: sc_assert( sc_time_stamp() == sc_time(100, SC_NS) ); f9=1; break;
      case 10: sc_assert( sc_time_stamp() == sc_time(110, SC_NS) ); f10=1; break;
      default: sc_assert( false ); break;
    }
    wait(20, SC_NS);
  }
  
  void target7()
  {
    wait(200, SC_NS);
    count = 11;
    ch8 = sc_spawn(sc_bind(&Top::child8, this));
    ch9 = sc_spawn(sc_bind(&Top::child9, this));
    try {
      wait(20, SC_NS);
    }
    catch (std::exception e) {
      sc_assert( sc_time_stamp() == sc_time(210, SC_NS) );
    }
    wait(ch8.terminated_event() & ch9.terminated_event());
    sc_assert( sc_time_stamp() == sc_time(214, SC_NS) );
    f12 = 1;
  }
  
  void child8()
  {
    gch10 = sc_spawn(sc_bind(&Top::grandchild10, this));
    gch11 = sc_spawn(sc_bind(&Top::grandchild11, this));
    try {
      wait(20, SC_NS);
    }
    catch (std::exception e) {
      f13 = 1;
      sc_assert( sc_time_stamp() == sc_time(210, SC_NS) );
    }
    wait(gch10.terminated_event() & gch11.terminated_event());
  }
  
  void child9()
  {
    gch12 = sc_spawn(sc_bind(&Top::grandchild12, this));
    gch13 = sc_spawn(sc_bind(&Top::grandchild13, this));
    try {
      wait(20, SC_NS);
    }
    catch (std::exception e) {
      f14 = 1;
      sc_assert( sc_time_stamp() == sc_time(210, SC_NS) );
    }
    wait(gch12.terminated_event() & gch13.terminated_event());
  }
  
  void grandchild10()
  {
    try {
      wait(20, SC_NS);
    }
    catch (std::exception e) {
      f15 = 1;
      sc_assert( sc_time_stamp() == sc_time(210, SC_NS) );
    }
    wait(1, SC_NS);
  }
  
  void grandchild11()
  {
    try {
      wait(20, SC_NS);
    }
    catch (std::exception e) {
      f16 = 1;
      sc_assert( sc_time_stamp() == sc_time(210, SC_NS) );
    }
    wait(4, SC_NS);
  }
  
  void grandchild12()
  {
    try {
      wait(20, SC_NS);
    }
    catch (std::exception e) {
      f17 = 1;
      sc_assert( sc_time_stamp() == sc_time(210, SC_NS) );
    }
    wait(2, SC_NS);
  }
  
  void grandchild13()
  {
    try {
      wait(20, SC_NS);
    }
    catch (std::exception e) {
      f18 = 1;
      sc_assert( sc_time_stamp() == sc_time(210, SC_NS) );
    }
    wait(3, SC_NS);
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();

  sc_assert( top.f1 );
  sc_assert( top.f2 );
  sc_assert( top.f3 );
  sc_assert( top.f4 );
  sc_assert( top.f5 );
  sc_assert( top.f6 );
  sc_assert( top.f7 );
  sc_assert( top.f8 );
  sc_assert( top.f9 );
  sc_assert( top.f10 );
  sc_assert( top.f11 );
  sc_assert( top.f12 );
  sc_assert( top.f13 );
  sc_assert( top.f14 );
  sc_assert( top.f15 );
  sc_assert( top.f16 );
  sc_assert( top.f17 );
  sc_assert( top.f18 );
  
  cout << endl << "Success" << endl;
  return 0;
}
  
