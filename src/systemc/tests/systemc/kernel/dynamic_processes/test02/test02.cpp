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

/*****************************************************************************

  Original Author: Bishnupriya Bhattacharya, Cadence Design Systems, 
                   September 5, 2003

 *****************************************************************************/

// test dynamic method processes and hierarchical dynamic process naming

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: test02.cpp,v $
// Revision 1.3  2011/03/06 06:55:19  acg
//  Andy Goodrich: removed carriage returns.
//
// Revision 1.2  2011/02/01 17:17:40  acg
//  Andy Goodrich: update of copyright notice, added visible CVS logging.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>

int function_method(double d)
{
  cout << endl << sc_time_stamp() << ", " 
       << sc_get_current_process_handle().name() 
       << ": function_method sees " << d << endl;
  return int(d);
}

void function_thread(double d)
{
  cout << endl << sc_time_stamp() << ", " 
       << sc_get_current_process_handle().name() 
       << ": function_thread sees " << d << endl;
  sc_core::wait(10, SC_NS);
  cout << sc_time_stamp() << ", "
       << sc_get_current_process_handle().name()
       << ": ending thread" << endl;
}

class module1 : public sc_module
{
private:
  sc_event& ev;
  int method_count;
  int r;
public:

  SC_HAS_PROCESS(module1);

  module1(sc_module_name name, sc_event& event) : sc_module(name), 
    ev(event), method_count(0), r(0)
  {
    SC_THREAD(main);
    cout << endl << sc_time_stamp() << ": CTOR, Before spawning function_method " << endl;
    sc_spawn_options o1;
    o1.spawn_method();
    o1.dont_initialize();
    o1.set_sensitivity(&ev);
    sc_process_handle h4 = sc_spawn(&r, sc_bind(&function_method, 1.2345), "event_sensitive_method", &o1);

  }

  void main()
  {
    int r;
    sc_event e1, e2, e3, e4;
    sc_spawn_options o1, o2, o3, o4;

    cout << endl << sc_time_stamp() << ", " 
    << sc_get_current_process_handle().name() 
    << ": main thread, Before spawning round robin methods " 
    << endl << endl;

    e1.notify(15, SC_NS);
    o1.spawn_method();
    o2.spawn_method();
    o3.spawn_method();
    o4.spawn_method();

    // Spawn several methods that co-operatively execute in round robin order

    sc_spawn(
      sc_bind(&module1::round_robin, this, "1", sc_ref(e1), sc_ref(e2), 1), "method1", &o1);
    sc_spawn(
      sc_bind(&module1::round_robin, this, "2", sc_ref(e2), sc_ref(e3), 1), "method2", &o2);
    sc_spawn(
      sc_bind(&module1::round_robin, this, "3", sc_ref(e3), sc_ref(e4), 1), "method3", &o3);
    sc_spawn(
      sc_bind(&module1::round_robin, this, "4", sc_ref(e4), sc_ref(e1), 1), "method4", &o4);


    cout << endl << sc_time_stamp() << ", " 
	 << sc_get_current_process_handle().name() 
	 << ": main thread, Issuing wait(60, SC_NS)" << endl;

    sc_core::wait(60, SC_NS);

    cout << endl << sc_time_stamp() << ", " 
	 << sc_get_current_process_handle().name() 
	 << ": Done main thread." << endl;
  }

  void round_robin(const char *str, sc_event& receive, sc_event& send, int cnt)
  {
    cout << sc_time_stamp() << ", " 
	 << sc_get_current_process_handle().name() 
	 << ": In Round robin method " << str; 
      
    if (method_count < 4) {
      method_count++;
      next_trigger(receive);
      cout << ". Issued next_trigger. " << endl;
    } else {
      send.notify(10, SC_NS);
      cout << ". Notified. Ending Round robin method " << str << endl;
    }

  }

};

int sc_main (int argc , char *argv[]) 
{
  sc_event event1;
  event1.notify(55, SC_NS);

  module1 mod1("mod1", event1);
  sc_start(100, SC_NS);
  cout << endl << sc_time_stamp() << ": sc_main, Before spawning function_thread " << endl;
  sc_process_handle h4 = sc_spawn(sc_bind(&function_thread, 6.789));
  sc_start(100, SC_NS);
  return 0;
}
