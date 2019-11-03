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

// odds_and_ends.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: odds_and_ends.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Quick test of new features in 1666-2011

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;

SC_MODULE(Top)
{
  SC_CTOR(Top)
  {
    SC_THREAD(gen);
    SC_THREAD(T1);
      h1 = sc_get_current_process_handle();
    SC_THREAD(T2);
      h2 = sc_get_current_process_handle();
      
    // Complete for mutex
    SC_THREAD(task1);
    SC_THREAD(task2);

    SC_METHOD(reset_handler);
      dont_initialize();
      sensitive << h2.reset_event();

    SC_METHOD(kill_handler);
      dont_initialize();
      sensitive << h2.terminated_event();
      
    SC_THREAD(T3);
    
    end_of_T1 = end_of_T3 = T3A_called = T3B_called = false;
  }
  
  sc_event ev;
  
  sc_process_handle h1, h2;
  bool end_of_T1, end_of_T3;
  
  void gen()
  {
    for (;;)
    {
      wait(10, SC_NS);
      ev.notify();
    }
  }
  
  void T1()
  {
    wait(25, SC_NS);
    cout << "suspend at " << sc_time_stamp() << endl;
    h2.suspend();
    wait(20, SC_NS);
    cout << "resume at " << sc_time_stamp() << endl;
    h2.resume();
    wait(20, SC_NS);

    cout << "disable at " << sc_time_stamp() << endl;
    h2.disable();
    wait(20, SC_NS);
    cout << "enable at " << sc_time_stamp() << endl;
    h2.enable();
    wait(20, SC_NS);
    
    h2.reset();
    wait(20, SC_NS);
    
    h2.kill();
    wait(20, SC_NS);

    sc_pause();
    wait(50, SC_NS);
    sc_stop();
    end_of_T1 = true;
  }
  
  void T2()
  {
    for (;;)
    {
      wait(ev);
      cout << "T2 at " << sc_time_stamp() << endl;
    }
  }
  
  void task1()
  {
    resource();
    sc_assert( sc_time_stamp() == sc_time(10, SC_NS) || sc_time_stamp() == sc_time(20, SC_NS) );
    cout << "task1 or task2 completed" << endl;
  }
  
  void task2()
  {
    resource();
    sc_assert( sc_time_stamp() == sc_time(10, SC_NS) || sc_time_stamp() == sc_time(20, SC_NS) );
    cout << "task1 or task2 completed" << endl;
  }
  
  void resource()
  {
    sc_mutex mut;
    mut.lock();
    wait(10, SC_NS);
    mut.unlock();
  }
  
  void reset_handler()
  {
    cout << "reset_handler() called at " << sc_time_stamp() << endl;
    sc_assert( sc_time_stamp() == sc_time(105, SC_NS) );
    sc_assert( !sc_is_unwinding() );
  }
  
  void kill_handler()
  {
    cout << "kill_handler() called at " << sc_time_stamp() << endl;
    sc_assert( sc_time_stamp() == sc_time(125, SC_NS) );
    sc_assert( !sc_is_unwinding() );
  }
  
  void T3()
  {
    wait(10, SC_NS);
    SC_FORK
      t3a = sc_spawn(sc_bind( &Top::T3A, this)),   
      t3b = sc_spawn(sc_bind( &Top::T3B, this))   
    SC_JOIN
    if (t3a.valid()) sc_assert( t3a.terminated() );
    if (t3b.valid()) sc_assert( t3b.terminated() );
    end_of_T3 = true;
  }
  
  sc_process_handle t3a, t3b;
  bool T3A_called;
  bool T3B_called;
  
  void T3A()
  {
    sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
    wait(5, SC_NS);
    T3A_called = true;
  }
  
  void T3B()
  {
    sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
    wait(7, SC_NS);
    T3B_called = true;
  }
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  sc_start();
  
  while (sc_pending_activity() && sc_get_status() != SC_STOPPED)
  {
    cout << "Reentering sc_start at " << sc_time_stamp() << endl;
    sc_start(sc_time_to_pending_activity());
  }
    
  cout << "sc_max_time() = " << sc_max_time() << endl;
  sc_assert( sc_get_status() == SC_STOPPED );

  sc_assert( top.end_of_T1 );
  sc_assert( top.end_of_T3 );
  sc_assert( top.T3A_called );
  sc_assert( top.T3B_called );
  
  cout << endl << "Success" << endl;
  return 0;
}
