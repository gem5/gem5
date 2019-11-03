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

// suspend_resume.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: suspend_resume.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M1: sc_module
{
  M1(sc_module_name _name)
  {
    SC_THREAD(ticker);
    SC_THREAD(calling);
    SC_THREAD(target);
      t = sc_get_current_process_handle();
  }
  
  sc_process_handle t;
  sc_event ev;

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
    
    t.suspend();
    wait(10, SC_NS);
    // Target does not run at time 20 NS while suspended
    
    t.resume();
    // Target runs at time 25 NS when resume is called
    
    wait(10, SC_NS);
    // Target runs at time 30 NS due to notification
    
    t.disable();
    wait(10, SC_NS);
    // Target does not run at time 40 NS while disabled
    
    t.enable();
    // Target does not run at time 45 NS when enable is called
    
    wait(10, SC_NS);
    // Target runs at time 50 NS due to notification
    
    sc_stop();
  }

  void target()
  {
    for (;;)
    {
      wait(ev);
      cout << "Target awoke at " << sc_time_stamp() << endl;
    }
  }
  
  SC_HAS_PROCESS(M1);
};

int sc_main(int argc, char* argv[])
{
  M1 m("m");
  
  sc_start();
  
  cout << endl << "Success" << endl;
  return 0;
}
  
