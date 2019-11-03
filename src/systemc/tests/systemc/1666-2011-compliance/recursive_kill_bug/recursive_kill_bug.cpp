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

// recursive_kill_bug.cpp -- test for 
//
//  Original Author: John Aynsley, Doulus
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//

// 

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    SC_THREAD(control);
    
    SC_METHOD(caller);
      sensitive << ev;
      dont_initialize();
      caller_handle = sc_get_current_process_handle();

    SC_THREAD(target);
      target_handle = sc_get_current_process_handle();

    count = 0;
    f0 = f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
  }
  
  struct bomb
  {
    sc_process_handle h;
    
    bomb(sc_process_handle _h)
    {
      h = _h;
    }
    
    ~bomb()
    {
      h.kill();
    }
  };

  sc_process_handle caller_handle;
  sc_process_handle target_handle;
  int count;
  int f0, f1, f2, f3, f4, f5, f6, f7, f8, f9;
  sc_event ev;
  
  void control()
  {
    count = 0;
    wait(10, SC_NS);
    
    count = 1;
    ev.notify();
  }

  void caller()
  {
    f0 = 1;
    target_handle.kill();
    sc_assert( false );  // FAILS !!!!!!
  }

  void target()
  {
    bomb local_obj(caller_handle);
    sc_assert( count == 0 );
    f1 = 1;
    try {
      wait(20, SC_NS);
    }
    catch (const sc_unwind_exception& e) {
      sc_assert( count == 1 );
      sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
      f2 = 1;
      throw e;
    }
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();

  sc_assert( top.f0 ); 
  sc_assert( top.f1 ); 
  sc_assert( top.f2 ); 
  
  cout << endl << "Success" << endl;
  return 0;
}
  
