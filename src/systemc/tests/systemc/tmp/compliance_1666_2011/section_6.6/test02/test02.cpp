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

// test02.cpp -- Test Method Suspending Itself
//
//  Original Author: John Aynsley, Doulos
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: test02.cpp,v $
// Revision 1.3  2011/03/07 19:32:19  acg
//  Andy Goodrich: addition to set sc_core::sc_allow_process_control_corners
//  to true so that this test avoids corner case error messages.
//
// Revision 1.2  2011/02/20 13:44:06  acg
//  Andy Goodrich: updates for IEEE 1666 2011.
//
// Revision 1.1  2011/02/05 21:13:26  acg
//  Andy Goodrich: move of tests John Aynsley will replace.
//
// Revision 1.1  2011/01/20 16:54:54  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M5: sc_module
{
  M5(sc_module_name _name)
  {
    SC_THREAD(ticker);
    SC_THREAD(calling);
    SC_METHOD(target);
      sensitive << ev;
      dont_initialize();
      t = sc_get_current_process_handle();
    suspend_target = false;
    resume_target = false;
  }
  
  sc_process_handle t;
  sc_event ev;
  bool suspend_target;
  bool resume_target;

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
    // Target runs at 10 NS
    
    suspend_target = true;
    wait(10, SC_NS);
    // Target runs at 20 NS and suspends itself
    
    wait(10, SC_NS);
    // Target does not run at 30 NS
    
    suspend_target = false;
    t.resume();
    // Target runs at 35 NS
    
    wait(10, SC_NS);
    // Target runs at 40 NS  

    suspend_target = true;
    resume_target = true;
    wait(10, SC_NS);
    // Target runs at 50 NS  
    
    sc_stop();
  }

  void target()
  {
    cout << "Target called at " << sc_time_stamp() << endl;
    if (suspend_target)
      t.suspend();
    if (resume_target)
    {
      t.resume();
      suspend_target = false;
    }
  }
  
  SC_HAS_PROCESS(M5);
};

int sc_main(int argc, char* argv[])
{
  M5 m("m");
  
  sc_core::sc_allow_process_control_corners = true;
  sc_start();
  
  return 0;
}
  
