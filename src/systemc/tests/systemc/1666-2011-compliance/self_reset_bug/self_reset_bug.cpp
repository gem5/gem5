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

// self_reset_bug.cpp -- test for 
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
    SC_METHOD(target);
      target_handle = sc_get_current_process_handle();

    count = 0;
    f0 = f1 = f2 = f3 = 0;
  }
  
  sc_process_handle target_handle;
  int count;
  int f0, f1, f2, f3;

  void target()
  {
    switch (count)
    {
      case 0:
        f0 = 1;
        count = 1;
        next_trigger(10, SC_NS);
        break;

      case 1:
        f1 = 1;
        count = 2;
        target_handle.reset();
        break;

      case 2:
        f2 = 1;
        count = 3;
        target_handle.reset();
        break;

      case 3:
        f3 = 1;
        count = 4;
        break;
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
  sc_assert( top.f3 ); 
  
  cout << endl << "Success" << endl;
  return 0;
}
  
