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

// test03.cpp -- Quick Test Of kill() And reset() sc_process_handle Methods.
//
//  Original Author: John Aynsley, Doulos
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: test03.cpp,v $
// Revision 1.1  2011/02/05 21:13:26  acg
//  Andy Goodrich: move of tests John Aynsley will replace.
//
// Revision 1.1  2011/01/20 16:55:01  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M3: sc_module
{
  M3(sc_module_name _name)
  {
    SC_THREAD(ticker);
    SC_THREAD(calling);
    SC_THREAD(target);
    t = sc_get_current_process_handle();
  }
  
  sc_process_handle t;
  sc_event ev;
  int count;

  void ticker()
  {
    for (;;)
    {
      wait(10, SC_NS);
      ev.notify();
    }
  }
   
  void calling()
  {
    wait(15, SC_NS);
    // Target runs at time 10 NS due to notification
    sc_assert( count == 1 );
 
    wait(10, SC_NS);
    // Target runs again at time 20 NS due to notification
    sc_assert( count == 2 );
    
    t.reset();
    // Target reset immediately at time 25 NS
    sc_assert( count == 0 );
 
    wait(10, SC_NS);
    // Target runs again at time 30 NS due to notification
    sc_assert( count == 1 );
    
    t.kill();
    // Target killed immediately at time 35 NS
    sc_assert( t.terminated() );
      
    sc_stop();
  }

  void target()
  {
    cout << "Target called/reset at " << sc_time_stamp() << endl;
    count = 0;
    for (;;)
    {
      wait(ev);
      cout << "Target awoke at " << sc_time_stamp() << endl;
      ++count;
    }
  }
  
  SC_HAS_PROCESS(M3);
};

int sc_main(int argc, char* argv[])
{
  M3 m("m");
  
  sc_start();
  
  return 0;
}
  
