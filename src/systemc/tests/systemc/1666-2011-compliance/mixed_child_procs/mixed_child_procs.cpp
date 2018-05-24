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

// mixed_child_procs.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: mixed_child_procs.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Process control methods applied where child process tree contains
// a mixture of method and thread processes

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

#include <string>
using std::string;

struct Top: sc_module
{
  Top(sc_module_name _name)
  : count(0)
  , index(0)
  , n(30)
  {
    SC_THREAD(ctrl);

    opt.spawn_method();
    opt.set_sensitivity( &ev );
    
    given_birth = new bool[n];

    f0 = new int[n];
    f1 = new int[n];
    f2 = new int[n];
    f3 = new int[n];
    f4 = new int[n];
    f5 = new int[n];

    for (int i = 0; i < n; i++)
    {
      given_birth[i] = false;

      f0[i] = 0;
      f1[i] = 0;
      f2[i] = 0;
      f3[i] = 0;
      f4[i] = 0;
      f5[i] = 0;
    }

    t = sc_spawn(sc_bind(&Top::child_thread, this, index++, 3));
    m = sc_spawn(sc_bind(&Top::child_method, this, index++, 3), "m", &opt);
  }

  sc_spawn_options opt;
  sc_process_handle t, m;  
  std::exception ex;
  int count;
  int index;
  const int n;
  int *f0, *f1, *f2, *f3, *f4, *f5;
  bool *given_birth;

  sc_event ev;
  
  void ctrl()
  {
    wait(10, SC_NS);

    count = 1;
    ev.notify();
    wait(10, SC_NS);
    
    count = 2;
    t.throw_it(ex, SC_INCLUDE_DESCENDANTS);
    m.throw_it(ex, SC_INCLUDE_DESCENDANTS);
    wait(10, SC_NS);
    
    count = 3;
    t.sync_reset_on(SC_INCLUDE_DESCENDANTS);
    m.sync_reset_on(SC_INCLUDE_DESCENDANTS);
    wait(10, SC_NS);
    
    count = 4;
    ev.notify();
    wait(10, SC_NS);
    
    count = 5;
    t.sync_reset_off(SC_INCLUDE_DESCENDANTS);
    m.sync_reset_off(SC_INCLUDE_DESCENDANTS);
    wait(10, SC_NS);
    
    count = 6;
    ev.notify();
    wait(10, SC_NS);
   }
  
  void child_thread(int i, int level)
  {
    //cout << "child_thread " << i << " at level " << level << " called at " << sc_time_stamp() << endl;
    if (level > 0 && !given_birth[i])
    {
      sc_spawn(sc_bind(&Top::child_thread, this, index++, level-1));
      sc_spawn(sc_bind(&Top::child_method, this, index++, level-1), "h", &opt);
      given_birth[i] = true;
    }
    switch(count)
    {
      case  0: f0[i]=1; break;
      case  4: f4[i]=1; break;
      default: sc_assert(false); break;
    }
    while(true)
    {
      try {
        wait(ev);
        //cout << "child_thread " << i << " at level " << level << " awoke at " << sc_time_stamp() << endl;
        switch(count)
        {
          case  1: f1[i]=1; break;
          case  6: f5[i]=1; break;
          default: sc_assert(false); break;
        }
      }
      catch (const std::exception& e) {
        //cout << "child_thread " << i << " at level " << level << " caught at " << sc_time_stamp() << endl;
        switch(count)
        {
          case  2: f2[i]=1; break;
          case  4: f3[i]=1; throw static_cast<const sc_unwind_exception&>(e);
          default: sc_assert(false); break;
        }
      }
    }
  }
  
  void child_method(int i, int level)
  {
    //cout << "child_method " << i << " at level " << level << " at " << sc_time_stamp() << endl;
    if (level > 0 && !given_birth[i])
    {
      sc_spawn(sc_bind(&Top::child_thread, this, index++, level-1));
      sc_spawn(sc_bind(&Top::child_method, this, index++, level-1), "m", &opt);
      given_birth[i] = true;
    }
    switch(count)
    {
      case  0: f0[i]=2; break;
      case  1: sc_assert(sc_time_stamp() == sc_time(10, SC_NS)); f1[i]=1; break;
      case  4: f4[i]=1; break;
      case  6: f5[i]=1; break;
      default: sc_assert(false); break;
    }
    next_trigger(ev);
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  sc_report_handler::set_actions(SC_WARNING, SC_DO_NOTHING);

  Top top("top");
  
  sc_start();
  
  sc_assert( top.index == top.n );
  
  for (int i = 0; i < top.n; i++)
  {
    sc_assert( top.f0[i] );
    sc_assert( top.f1[i] );
    
    if (top.f0[i] == 1)  // i.e. a thread process
    {
      sc_assert( top.f2[i] );
      sc_assert( top.f3[i] );
    }
    sc_assert( top.f4[i] );
    sc_assert( top.f5[i] );
  }

  cout << endl << "Success" << endl;
  return 0;
}
  
