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

// living_dead_bug.cpp -- test for 
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
    
    SC_THREAD(target);
      dont_initialize();
      target_handle = sc_get_current_process_handle();

    f0 = f1 = 0;
  }
  
  sc_process_handle target_handle;
  int f0, f1;
  
  void control()
  {
    // create another (orphaned) instance of "target"
    {
      sc_spawn_options opt;
      opt.dont_initialize();
      sc_spawn( sc_bind( &Top::target, this ), "dyn_target", &opt );
    }

    wait(10, SC_NS);
    f0 = 1;
    target_handle.kill();
    f1 = 1;
  }

  void target()
  {
    sc_assert( false );  // FAILS!!!!!!
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();

  sc_assert( top.f0 ); 
  sc_assert( top.f1 ); 
  
  cout << endl << "Success" << endl;
  return 0;
}
  
